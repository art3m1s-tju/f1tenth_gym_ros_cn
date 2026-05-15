# LQR 控制器调参指南

## 核心参数

| 参数 | 作用 | 默认值 | 范围建议 |
|------|------|--------|---------|
| `lqr_q_lateral` | 横向误差权重，Q 矩阵对角元素 | 3.0 | 1.0 ~ 10.0 |
| `lqr_q_heading` | 航向误差权重，Q 矩阵对角元素 | 1.2 | 0.5 ~ 5.0 |
| `lqr_r_steering` | 转向输入代价，R 矩阵 | 8.0 | 2.0 ~ 20.0 |
| `lqr_feedforward_gain` | 曲率前馈增益 | 1.0 | 0.5 ~ 1.5 |
| `target_speed` | 目标巡航速度 (m/s) | 1.0 | 0.3 ~ 3.0 |
| `min_speed` | 最低速度下限 (m/s) | 0.4 | 0.2 ~ 1.0 |
| `max_lateral_accel` | 弯道限速的横向加速度上限 (m/s^2) | 4.0 | 2.0 ~ 6.0 |
| `max_steering_angle` | 最大转向角 (rad) | 0.4189 | 与仿真器物理一致 |

## 参数含义

LQR 通过最小化代价函数 `J = x'Qx + u'Ru` 求解最优反馈增益：
- **Q 矩阵** = diag(q_lateral, q_heading)：状态误差的惩罚权重
- **R 矩阵** = [r_steering]：控制输入（转向角）的惩罚权重
- **Q/R 比值**决定跟踪紧度：比值大 → 跟踪紧但转向激进；比值小 → 平滑但误差大

前馈项 `delta_ff = feedforward_gain * atan(wheelbase * curvature)` 根据参考曲率提前打方向，减少反馈项的负担。

## 调参流程

### Step 1：低速验证基本跟踪

```
target_speed: 0.5
lqr_q_lateral: 3.0
lqr_q_heading: 1.2
lqr_r_steering: 8.0
```

目标：车能稳定跑完一圈不飞出赛道。如果振荡，增大 `r_steering`。

### Step 2：调横向跟踪精度

固定 `r_steering=8.0`，逐步增大 `q_lateral`：

- 3.0 → 5.0 → 8.0 → 10.0

每次跑一圈后用 `tracker_evaluate.py` 检查 `mean_abs_e_y`。目标：< 5cm。如果出现振荡（横向误差在正负间快速切换），说明 `q_lateral` 过大，回退一档。

### Step 3：调航向响应

如果入弯时车头跟不上（航向误差大），增大 `q_heading`：

- 1.2 → 2.0 → 3.0

观察 `mean_abs_e_psi_deg`，目标：< 3°。

### Step 4：提速

横向和航向误差满意后，逐步提高 `target_speed`：

- 0.5 → 0.8 → 1.0 → 1.5 → 2.0

每次提速后观察：
- 弯道是否超出 `max_lateral_accel` 限制（自动减速是否足够）
- 横向误差是否明显增大
- 是否出现转向饱和（`delta_cmd` 达到 `max_steering_angle`）

如果高速弯道误差大，可以：
- 降低 `max_lateral_accel`（更早减速）
- 增大 `q_lateral`（更紧跟踪）
- 降低 `lqr_feedforward_gain` 到 0.8（减少前馈过冲）

### Step 5：精调前馈

`lqr_feedforward_gain` 控制弯道前馈量：
- 1.0：标准前馈，适合大多数情况
- \> 1.0：弯道提前打更多方向，适合高速
- < 1.0：减少前馈，更依赖反馈修正，适合低速或曲率估计不准时

## 评估命令

```bash
python3 /sim_ws/src/f1tenth_gym_ros/code/tracker_evaluate.py \
  --log /sim_ws/src/f1tenth_gym_ros/code/outputs/logs/lqr_tracking_log.csv \
  --reference-trajectory-csv /sim_ws/src/f1tenth_gym_ros/code/outputs/csv/global_trajectory.csv \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/evaluation
```

## 关键指标参考

| 指标 | 优秀 | 合格 | 需要调整 |
|------|------|------|---------|
| mean_abs_e_y | < 3 cm | < 5 cm | > 8 cm |
| p95_abs_e_y | < 5 cm | < 8 cm | > 12 cm |
| max_abs_e_y | < 10 cm | < 15 cm | > 20 cm |
| mean_abs_e_psi | < 2° | < 4° | > 6° |
| p95_abs_e_psi | < 4° | < 6° | > 10° |

## 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| 车辆振荡（左右摇摆） | Q/R 比值过大 | 增大 `r_steering` 或减小 `q_lateral` |
| 弯道大幅偏离 | 前馈不足或速度过高 | 增大 `feedforward_gain` 或降低 `target_speed` |
| 入弯响应慢 | 航向权重不够 | 增大 `q_heading` |
| 转向饱和（打满方向盘） | 速度过高或曲率过大 | 降低 `target_speed` 或 `max_lateral_accel` |
| 直道误差大弯道误差小 | 横向权重不够 | 增大 `q_lateral` |
