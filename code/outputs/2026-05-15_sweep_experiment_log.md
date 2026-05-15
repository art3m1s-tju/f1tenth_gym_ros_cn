# LQR 参数扫描实验记录

日期：2026-05-15

## 1. 实验目标

使用离线参数扫描工具 (`lqr_sweep`) 自动搜索不同速度下的最优 LQR 参数组合。

- 扫描参数：`q_lateral`, `q_heading`, `r_steering`, `feedforward_gain`
- 速度点：0.5, 1.0, 1.5, 2.0, 2.5, 3.0 m/s
- 网格规模：5×4×5×3 = 300 粗网格 + top-3 局部细化（每点 243 次评估）

## 2. 第一次扫描结果（原始目标函数和搜索空间）

### 搜索空间

| 参数 | 下界 | 上界 |
|------|------|------|
| q_lateral | 0.5 | 20.0 |
| q_heading | 0.3 | 10.0 |
| r_steering | 1.0 | 50.0 |
| feedforward_gain | 0.5 | 1.5 |

### 目标函数权重

| 指标 | 权重 |
|------|------|
| lateral_mean | 0.30 |
| lateral_p95 | 0.25 |
| heading | 0.15 |
| steer_rate | 0.15 |
| steer_rms | 0.10 |
| speed | 0.05 |

### 结果

- `q_lateral` 全部撞上界 20.0
- `r_steering` 高速时撞下界 1.0
- `q_heading` 撞下界 0.3

### 问题分析

目标函数对 lateral 权重过高（合计 0.55），steering 惩罚不够（合计 0.25）；搜索空间边界设置不合理，优化器在边界处无法继续探索。

## 3. 第二次扫描（调整目标函数权重和搜索空间后）

### 调整内容

- lateral 总权重降到 0.45，steering 总权重升到 0.35
- steering 归一化尺度收紧：steer_rate 50→30 deg/s，steer_rms 10→6 deg
- 搜索空间收窄：

| 参数 | 下界 | 上界 |
|------|------|------|
| q_lateral | 1.0 | 12.0 |
| q_heading | 0.5 | 6.0 |
| r_steering | 3.0 | 30.0 |
| feedforward_gain | 0.6 | 1.4 |

- `max_steering_angle` 从 0.4189 改为 0.36 rad（匹配实车物理限位）

### 结果

```
v=0.5: Q=(12.0, 1.83) R=3.0  ff=1.0   score=0.29
v=1.0: Q=(10.2, 0.65) R=9.49 ff=0.6   score=0.55
v=1.5: Q=(8.4,  0.58) R=3.73 ff=0.6   score=0.59
v=2.0: Q=(12.0, 1.49) R=3.0  ff=0.6   score=0.64
v=2.5: Q=(12.0, 1.83) R=3.0  ff=0.69  score=0.70
v=3.0: Q=(10.2, 0.80) R=3.0  ff=0.78  score=0.75
```

### 仍存在的问题

- `q_lateral` 仍撞上界 12.0（4/6 个速度点）
- `r_steering` 5/6 个速度点撞下界 3.0
- `q_heading` 不单调且偏低（0.58–1.83）
- `feedforward_gain` 偏低（多数为 0.6），不符合理论预期

## 4. 问题根因分析

纯仿真环境无噪声、无延迟，激进参数（高 Q 低 R）永远在仿真中表现最好。

**目标函数灵敏度分析：**

横向误差改善 1cm 的收益约 0.05，约等于转向率恶化 7.5 deg/s 的代价。优化器理性选择暴力拉大 Q、压低 R 来换取横向误差的微小改善。

**q_heading 被压低的原因：**

`q_lateral` 已经很高时，再加高 `q_heading` 会导致控制器"双重激进"，steering_rate 飙升，在目标函数中被惩罚。优化器选择牺牲 heading 跟踪来换取 steering 平滑。

**feedforward_gain 被压低的原因：**

高 Q 低 R 的反馈增益已经很大，ff=1.0 会导致弯道过冲（反馈 + 前馈叠加超调），优化器将 ff 压低来补偿过高的反馈增益。

**(q_lat, q_head, r_steer) 冗余自由度：**

不同参数组合可产生相同的增益 K，导致结果在等价面上漂移，无法收敛到唯一解。

## 5. 关于 Q/R 冗余和插值的讨论

DARE 只依赖 Q/R 比值。例如 `(q_lat=6, q_head=1.2, r=3)` 和 `(q_lat=2, q_head=0.4, r=1)` 产生完全相同的 K。

### 曾考虑的替代方案

1. **直接扫 K**：不保证 LQR 稳定性和鲁棒裕度
2. **固定 R=1 只扫 Q**：可消除冗余，但对 K 做速度插值不保证插值结果稳定

### 最终结论

保持现有框架：扫 Q/R → 存 Q/R → 插值在 Q/R 空间做 → 运行时实时解 DARE。

理由：DARE 只要 Q≥0、R>0 就保证闭环稳定，Q/R 空间中任意凸组合（线性插值）仍满足半正定/正定条件，解出的 K 必然稳定。

## 6. 解决方案：仿真加执行延迟和传感器噪声

在 `sim_harness.py` 中加入真实性扰动：

| 扰动类型 | 参数 | 说明 |
|----------|------|------|
| 转向执行延迟 | 2 步（20ms） | 模拟 CAN 总线 + 舵机响应 |
| 位置噪声 | σ = 5mm | 模拟定位抖动 |
| 航向噪声 | σ = 0.005 rad (≈0.3°) | 模拟 IMU/定位融合噪声 |

### 预期效果

- 高 Q / 低 R 组合因延迟放大而振荡崩溃，优化器自然选择更保守参数
- Q/R 比值预期随速度单调递减（高速更保守）
- `feedforward_gain` 收窄到 [0.8, 1.2]（理论值应接近 1.0）

## 7. 第三次扫描（加延迟 2 步 + 噪声，physics_dt=0.01s）

### 配置

- `physics_dt = 0.01`（100Hz 控制频率）
- `steering_delay_steps = 2`（2 步 × 0.01s = 20ms）
- `position_noise_std = 0.005`（5mm）
- `heading_noise_std = 0.005`（≈0.3°）
- `ff_gain_range = [0.8, 1.2]`

### 结果

```
v=0.5: Q=(1.0, 0.5)  R=30.0  ff=1.0  score=0.45
v=1.0: Q=(1.15, 1.83) R=25.5  ff=0.6  score=0.55  (原始结果)
v=1.5: Q=(2.42, 0.80) R=3.73  ff=0.6  score=0.59
v=2.0: Q=(4.51, 5.1)  R=21.0  ff=0.6  score=0.64
v=2.5: Q=(1.15, 0.5)  R=25.5  ff=1.0  score=0.91
v=3.0: Q=(1.3, 0.65)  R=30.0  ff=1.0  score=0.94
```

### 问题

矫枉过正——从之前的"全撞下界（R 太小）"变成了"全撞上界（R=30）"。Q/R 比值极低（0.03~0.21），控制器几乎没有反馈增益，纯靠前馈跑。

原因：2 步延迟在 100Hz 控制频率下对 2 阶系统惩罚过重，任何有意义的反馈增益都会引起振荡。

## 8. 第四次扫描（延迟降为 1 步，physics_dt 改为 0.02s）

### 配置调整

发现实车控制频率为 50Hz（`controller.py` 中 `control_dt = 0.05`，实际由 odom 回调驱动约 50Hz）。将仿真参数对齐实车：

- `physics_dt = 0.02`（50Hz，匹配实车控制频率）
- `steering_delay_steps = 1`（1 步 × 0.02s = 20ms，匹配实车 CAN + 舵机延迟）
- 噪声保持不变

### 结果

```
v=0.5: Q=(1.0, 0.5)   R=30.0   ff=1.0  score=0.379
v=1.0: Q=(1.3, 1.49)  R=25.5   ff=0.8  score=0.676
v=1.5: Q=(1.3, 0.80)  R=12.33  ff=0.8  score=0.680
v=2.0: Q=(1.86, 6.0)  R=16.87  ff=0.8  score=0.715
v=2.5: Q=(1.0, 0.65)  R=30.0   ff=1.0  score=0.786
v=3.0: Q=(1.0, 0.65)  R=30.0   ff=1.0  score=0.819
```

### 问题

仍然不理想：
- `q_lateral` 撞下界 1.0（4/6 个速度点）
- `r_steering` 撞上界 30.0（4/6 个速度点）
- Q/R 比值仍极低（0.033~0.110）
- 参数不单调（v=2.0 突然激进，v=2.5 又回到最保守）

### 分析：双重惩罚问题

延迟本身已经在做筛选——激进参数因振荡而 collision/timeout，直接被淘汰（score=inf）。在存活参数中，目标函数的 steering 权重（合计 0.35）又进一步把优化器推向"尽量别转向"。两层惩罚叠加，"几乎不转向"成了最优。

## 9. 关于两层"目标函数"的澄清

系统中存在两个不同层面的"目标函数"：

### LQR 内部代价函数

```
J = Σ_{k=0}^{∞} [e_lat² × q_lat + e_head² × q_head + delta² × r_steer]
```

这是控制器内部用来求解最优增益 K 的。给定 (Q, R)，DARE 解出使 J 最小的 K。Q/R 是人为给定的"偏好"，LQR 不判断哪组偏好更好。

### Sweep 外部评分函数 (`objective.py`)

评估一组 (Q, R) 跑完一圈后的实际表现（横向误差、航向误差、转向平滑度、速度跟踪），给出一个可比较的分数。这个分数决定"哪组偏好让车实际跑得最好"。

LQR 的 J 在不同 (Q, R) 之间不可比（尺子不同），所以需要外部评分来选参数。

## 10. 下一步方向

核心思路：**让延迟做筛选（淘汰不稳定参数），目标函数回归跟踪精度。**

建议将目标函数权重调整为：

| 项 | 当前权重 | 建议权重 |
|---|---|---|
| lateral_mean | 0.25 | 0.30 |
| lateral_p95 | 0.20 | 0.25 |
| heading_mean | 0.15 | 0.20 |
| steering_rate | 0.20 | 0.10 |
| steering_rms | 0.15 | 0.10 |
| speed_tracking | 0.05 | 0.05 |

逻辑：延迟已经替代了 steering 惩罚的角色（淘汰不稳定的激进参数），存活的参数都是"延迟下不崩溃的"，在这些参数中应优先选跟踪精度最好的。

## 11. 其他改进

- 添加了 `tqdm` 进度条（写入 Dockerfile 依赖和 `sweep_engine.py`）
- 屏蔽了 f1tenth_gym 的 "Chosen integrator is RK4" 警告（避免日志刷屏）
- 单次完整扫描耗时约 260–380 秒（6 速度点，每点 300 + 243 = 543 次评估）

## 12. 2026-05-15 代码修正：延迟语义、评分函数和真实误差评估

### 发现的问题

1. `steering_delay_steps` 存在 off-by-one 语义错误。

   原实现为：

   ```
   steering_buffer.append(delta_cmd)
   delta_delayed = steering_buffer[0]
   ```

   当 `steering_delay_steps = 1` 时，`deque` 长度为 1，append 后旧值立即被挤掉，`steering_buffer[0]` 就是当前刚算出的 `delta_cmd`。因此日志中写的“1 步延迟 = 20ms”实际没有生效；`steering_delay_steps = 2` 才近似表现为 1 个控制周期延迟。

2. 外部评分函数过于复杂且 steering 惩罚偏强。

   原 `objective.py` 同时评价平均横向误差、95 分位横向误差、平均航向误差、转向角 RMS、转向角速度 RMS 和速度误差。加入噪声和延迟后，激进参数已经会因振荡、碰撞或超时被淘汰；存活参数又被 steering 项二次惩罚，导致优化器偏向“尽量少转向”的低 Q / 高 R 参数。

3. 噪声仿真下，控制输入和评分使用了同一组 noisy projection。

   控制器应该看到带噪观测，这是模拟真实传感器；但 objective 的目的应该是评价“车实际跑得离真实参考轨迹有多远”。如果评分也使用 noisy projection，就会把观测噪声本身混入评价，使优化器更偏向降低观测抖动导致的转向，而不是单纯优化真实跟踪精度。

### 修改方式

1. 修复转向延迟队列顺序。

   新实现为：

   ```
   delta_delayed = steering_buffer[0]
   steering_buffer.append(delta_cmd)
   ```

   因此 `steering_delay_steps = 1` 表示当前仿真步执行上一控制周期的命令；`steering_delay_steps = 0` 表示无延迟。

2. 简化 `objective.py`。

   新评分只保留两个核心跟踪指标：

   ```
   score = 0.5 * (mean_abs_e_y / 0.05 + mean_abs_e_psi_deg / 3.0)
   ```

   其中 0.05 m 和 3 deg 只是归一化参考尺度，用来避免“米”和“度”直接相加。分数仍然越小越好。碰撞、超时、DARE 求解失败仍由 sweep 引擎直接记为 `inf`。

3. 拆分控制用误差和评分用误差。

   - 控制器输入：继续使用带噪声的 `(x_noisy, y_noisy, theta_noisy)` 投影，模拟真实定位/传感器噪声。
   - objective 指标：改为使用仿真器真实状态 `(x, y, theta)` 投影，评价车辆真实轨迹相对参考轨迹的横向误差和航向误差。

### 预期影响

- 延迟参数的物理含义与日志描述一致，后续扫描结果更容易解释。
- 评分函数更可控：先只回答“哪组 Q/R 跟踪更准”，不再混入平滑性和速度误差偏好。
- 噪声仍然会影响控制器行为，但不会直接污染评分指标；如果某组参数因为噪声导致真实轨迹变差，objective 仍会惩罚它。

### 后续建议

下一轮建议先使用当前简化 objective 重新跑一次 full sweep，观察是否仍出现 Q 撞下界、R 撞上界。如果参数重新变得过激，再考虑逐步加入一个很轻的 steering_rate 约束或失败惩罚，而不是一开始就把多个指标混在同一个复杂目标函数里。

## 13. 第五次扫描（修复延迟 + 简化 objective 后）

### 配置

- `steering_delay_steps` 修复为真实 N 步延迟语义。
- objective 只使用真实状态下的平均横向误差和平均航向误差：

  ```
  score = 0.5 * (mean_abs_e_y / 0.05 + mean_abs_e_psi_deg / 3.0)
  ```

- 控制器仍使用带噪声观测，评分改为使用仿真真实状态。

### 结果

```
v=0.5: Q=(1.0,  6.0)   R=3.0   ff=1.0  score=0.019
v=1.0: Q=(1.0,  1.14)  R=16.87 ff=0.8  score=0.721
v=1.5: Q=(4.51, 0.65)  R=21.0  ff=0.8  score=0.585
v=2.0: Q=(12.0, 0.5)   R=16.87 ff=0.8  score=0.469
v=2.5: Q=(12.0, 0.575) R=3.45  ff=0.8  score=0.413
v=3.0: Q=(12.0, 0.575) R=3.45  ff=0.8  score=0.416
```

对应 Q/R 比值：

```
v=0.5: q_lat/R=0.333, q_head/R=2.000
v=1.0: q_lat/R=0.059, q_head/R=0.068
v=1.5: q_lat/R=0.215, q_head/R=0.031
v=2.0: q_lat/R=0.711, q_head/R=0.030
v=2.5: q_lat/R=3.478, q_head/R=0.167
v=3.0: q_lat/R=3.478, q_head/R=0.167
```

### 现象分析

1. 过度保守问题基本解除。

   第四次扫描中 `q_lateral` 大量撞下界、`r_steering` 大量撞上界，说明 steering 惩罚把优化器推向“少转向”。本轮取消 steering 项后，高速段转为 `q_lateral` 撞上界、`r_steering` 接近下界，说明优化器重新开始优先追求跟踪精度。

2. 高速段重新出现激进边界。

   `v=2.0~3.0` 的 `q_lateral` 均撞到上界 12.0，`v=2.5/3.0` 的 `R=3.45` 接近下界。这说明在当前搜索空间和简化 objective 下，横向误差改善仍然有收益，优化器还想要更大的横向反馈。

3. `q_heading` 在高速段偏低。

   高速段 `q_heading/q_lateral` 约为 0.04~0.05，说明最优解主要依赖横向误差反馈，而不是强航向误差反馈。可能原因是：航向误差反馈过强会在带噪观测和延迟下增加摆动；横向误差项已经足够把车拉回轨迹。

4. `feedforward_gain` 多数撞下界 0.8。

   即使移除了 steering 惩罚，优化器仍选择较低前馈，说明当前曲率前馈 `atan(L*kappa)` 可能略偏大，或反馈项已经承担了一部分弯道修正，`ff=1.0` 时更容易产生过冲。

5. `v=0.5` 的 score 极低。

   低速下控制难度很小，且 `R=3.0`、`q_heading=6.0` 使航向收敛很强，因此分数接近 0。这个点不一定能代表高速趋势，后续可以单独检查该速度点的实际轨迹和日志，确认是否存在完成圈判据过早或路径误差过于理想化的问题。

### 结论

简化 objective 的方向是有效的：它消除了“为了少转向而牺牲跟踪”的偏置。但当前目标函数已经变成纯跟踪精度目标，因此高速段自然会重新向激进参数边界移动。下一步应先用该表跑完整 ROS 验证；如果出现抖动或实车不可接受的转向，再加入轻量的约束或惩罚，例如只惩罚超过阈值的 steering_rate，而不是重新把 steering_rms 和 steering_rate 大权重加入主目标。

## 14. 2026-05-15 多圈稳定性问题与修正

### 发现的问题

3.0 m/s 使用第五次扫描得到的新表做完整 ROS 验证时，第一圈比较贴线，但连续跑几圈后开始蛇形并最终撞墙。日志显示：

- 前两次经过起点附近时横向误差仍在厘米级。
- 后续某圈在高速弯附近出现转向长时间饱和。
- `closest_idx` 在非正常位置发生大跳变，随后横向误差扩大到 1 m 以上。
- `delta_cmd` 长时间卡在限幅，控制器已经没有足够余量把车拉回轨迹。

这说明“单圈平均误差很小”不足以代表参数鲁棒。高 Q / 低 R 参数可能第一圈非常贴线，但连续运行后会因为相位滞后、转向饱和和路径投影跳变逐渐进入蛇形。

### 环境不一致

离线 sweep 中 `SimConfig.max_steering_angle = 0.36 rad`，但 ROS launch 中 `max_steering_angle` 默认仍为 `0.4189 rad`。这会让同一张 LQR 表在离线 sweep 和完整 ROS 验证中面对不同的执行限幅，导致结果不可直接比较。

已修正：

- `launch/pnc_sim_launch.py` 默认 `max_steering_angle` 改为 `0.36`。
- `validate_ros.py` 会把 `lqr_gain_table_path` 传入 LQR 控制器，确保完整 ROS 验证真的使用扫描得到的速度分段表。

### 多圈 sweep

原离线 sweep 每组参数只要求完成 1 圈，难以暴露“第二圈以后开始蛇形”的问题。现已加入 `lap_count`：

- `SimConfig.lap_count = 5`
- `run_sweep.py` 新增 `--laps` 参数，默认 5 圈。
- README 中的 full/coarse/single/validate 命令均加入 `--laps 5`。

这样每组候选参数必须连续完成 5 圈才进入评分，能更早淘汰单圈好看但多圈不稳定的参数。

### 新 objective：硬约束 + 简单评分

保留简单 tracking score，但加入硬约束筛选：

```
if max_abs_e_y > 0.25 m: score = inf
if steering_saturation_ratio > 0.10: score = inf
if steering_rate_rms_deg_s > 100 deg/s: score = inf

score = 0.5 * (mean_abs_e_y / 0.05 + mean_abs_e_psi_deg / 3.0)
```

含义：

- `max_abs_e_y` 约束淘汰明显偏离轨迹的参数。
- `steering_saturation_ratio` 约束淘汰长时间接近转向限幅的参数。
- `steering_rate_rms_deg_s` 约束淘汰高频蛇形参数。
- 幸存参数仍按平均横向误差和平均航向误差排序。

### steering_rate 解释

`steering_rate` 是转向命令变化速度：

```
steering_rate[k] = abs(delta_cmd[k] - delta_cmd[k-1]) / dt
```

单位这里用 `deg/s`。它不是车的 yaw rate，而是“方向盘/前轮转角命令变化得有多快”。如果 `steering_rate_rms` 很大，通常说明控制器在左右快速修正，也就是肉眼看到的蛇形或高频抖动。

### 后续建议

下一轮重新运行 full sweep 时使用多圈默认配置：

```
python3 -m lqr_sweep.run_sweep --mode full \
  --speeds 0.5 1.0 1.5 2.0 2.5 3.0 \
  --coarse-grid 5,4,5,3 \
  --laps 5 \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep
```

如果 5 圈 sweep 后仍出现高速撞边界，可以再收紧：

- `steering_saturation_ratio` 阈值从 0.10 降到 0.05。
- `steering_rate_rms_deg_s` 阈值从 100 降到 80。
- 或者把高速段 `r_steering_range` 下界从 3.0 提高到 5.0。

## 15. 关于是否将 saturation 和 steering_rate 加入评分函数

### 判断

应该加入，但不应回到早期那种大权重 steering 惩罚。更合理的形式是：

1. 先用硬约束淘汰明显不稳定参数。
2. 再对幸存参数加入轻量软惩罚。

原因：只有硬约束时，`steering_rate_rms=20 deg/s` 和 `95 deg/s` 在阈值 100 以内会被视为同等可接受，最终排序仍完全由 tracking error 决定，可能选出“还没超过阈值但已经明显抖”的参数。

### 推荐形式

保留当前硬约束：

```
if max_abs_e_y > 0.25 m: score = inf
if steering_saturation_ratio > 0.10: score = inf
if steering_rate_rms_deg_s > 100 deg/s: score = inf
```

然后将幸存参数评分改为：

```
score =
  0.45 * mean_abs_e_y / 0.05
+ 0.45 * mean_abs_e_psi_deg / 3.0
+ 0.05 * steering_rate_rms_deg_s / 100.0
+ 0.05 * steering_saturation_ratio / 0.10
```

含义：

- tracking 精度仍占 90%，保持主要优化目标。
- steering_rate 和 saturation 合计只占 10%，用于在跟踪性能接近时优先选择更平滑、转向余量更大的参数。
- 不加入 `steering_rms`，避免重新奖励“少转向”并把优化器推回低 Q / 高 R 的保守角落。

### 测试建议

先用少量速度点和较小网格做 smoke test，确认目标函数能正常淘汰蛇形参数：

```
python3 -m lqr_sweep.run_sweep --mode coarse-only \
  --speeds 2.5 3.0 \
  --coarse-grid 4,3,4,3 \
  --laps 5 \
  --max-workers 4 \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_smoke
```

如果 smoke test 结果合理，再跑完整 5 圈 sweep。

## 16. Smoke test 全部 `inf` 的问题

### 现象

运行高速 smoke test 后，`sweep_smoke/sweep_summary.json` 中 2.5 m/s 和 3.0 m/s 的结果均为：

```
score = Infinity
Q=(1.0, 0.5), R=3.0, ff=1.2
```

这不是“这组参数最优”，而是所有候选参数都被判为 `inf` 后，扫描器仍然把排序列表里的第一项写入了表。因此生成的 `lqr_gain_table.yaml` 没有实际意义。

### 修正

已修改 `sweep_engine.py`：

- 粗搜索阶段只对有限分数的候选做局部细化。
- 如果某个速度点所有候选都是 `inf`，直接抛出错误，不再生成伪最优表。

### 下一步诊断

需要先用 `single` 模式检查代表性参数的 metrics，确认到底是哪一条约束把候选淘汰：

```
python3 -m lqr_sweep.run_sweep --mode single \
  --speed 2.5 \
  --laps 5 \
  --q-lateral 12.0 --q-heading 0.575 --r-steering 3.45 --feedforward-gain 0.8
```

重点看：

- `max_abs_e_y`
- `steering_saturation_ratio`
- `steering_rate_rms_deg_s`

如果多数参数只是略微超过阈值，说明约束过严，需要放宽。若大幅超过阈值，说明高速参数空间需要更保守，例如提高 `r_steering` 下界或降低 `q_lateral` 上界。

## 17. 离线 sweep 加入纵向速度斜坡

### 背景

完整 ROS LQR 控制器中，纵向速度命令不是一步跳变，而是经过 `_apply_speed_ramp()` 做加减速限幅：

```
加速: current_speed_cmd += max_accel * dt
减速: current_speed_cmd -= max_decel * dt
```

默认参数：

```
max_accel = 1.0 m/s^2
max_decel = 2.0 m/s^2
```

但离线 `sim_harness.py` 之前直接将曲率限速结果作为 `speed_cmd` 发送给 gym，速度命令可以瞬时变化。这是又一个离线 sweep 与完整 ROS 验证之间的不一致。

### 修改

已在 `SimConfig` 中加入：

```
enable_speed_ramp = True
max_accel = 1.0
max_decel = 2.0
```

并在 `run_sweep.py` 中暴露命令行参数：

```
--max-lateral-accel
--max-accel
--max-decel
--disable-speed-ramp
```

离线 sweep 现在默认使用与 ROS 控制器一致的速度斜坡。这样多圈 sweep 更接近真实 ROS 闭环，也能更真实地暴露“弯前降速不够早”的问题。

### 注意

速度斜坡只解决“速度命令是否平滑”的一致性问题，不等于前方曲率预瞄限速。当前限速仍主要基于当前投影点曲率和当前转角；若高速弯仍撞，下一步应加入前方曲率 preview，用前方一段距离内的最大曲率提前计算 `speed_cmd`。

## 18. 生成 3.0m/s 物理可行标准赛道

### 动机

原赛道含大曲率弯，不适合直接用 2.5/3.0m/s 全程固定速度做 LQR 参数标定。否则 sweep 混入了“速度物理不可行”的问题，得到的结果不再单纯反映 Q/R/ff 的好坏。

更合理的分层实验：

1. 在低曲率、3.0m/s 物理可行的标准赛道上标定 LQR 的速度查找表。
2. 在原赛道上使用曲率限速/预瞄限速，根据当前速度查表验证完整系统。

### 生成内容

新增脚本：

```
code/lqr_sweep/generate_feasible_track.py
```

默认生成 stadium 赛道：

- 两条直道 + 两个半圆弯。
- 半圆半径 `4.0 m`。
- 最大曲率 `0.25 1/m`。
- 3.0m/s 下横向加速度约 `2.25 m/s^2`。
- 赛道宽度 `2.0 m`。

输出文件：

```
maps/stadium_3ms_open.pgm
maps/stadium_3ms_open.yaml
code/outputs/generated_tracks/stadium_3ms_trajectory.csv
code/outputs/generated_tracks/stadium_3ms_processed_track.csv
code/outputs/generated_tracks/stadium_3ms_open_preview.png
```

推荐起始位姿：

```
sx=0.0, sy=4.0, stheta=3.1416
```

### 使用方式

在标准赛道上生成 LQR lookup table：

```
cd /sim_ws/src/f1tenth_gym_ros/code
python3 -m lqr_sweep.run_sweep --mode full \
  --map-path /sim_ws/src/f1tenth_gym_ros/maps/stadium_3ms_open \
  --trajectory-csv /sim_ws/src/f1tenth_gym_ros/code/outputs/generated_tracks/stadium_3ms_trajectory.csv \
  --speeds 0.5 1.0 1.5 2.0 2.5 3.0 \
  --coarse-grid 5,4,5,3 \
  --laps 3 \
  --max-sim-time 420 \
  --disable-curvature-speed-limit \
  --disable-speed-ramp \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_stadium
```

标准赛道使用开放空白地图，只用于标定 LQR 表；虚拟左右边界只参与可视化和误差理解，不参与碰撞。扫表时必须关闭曲率限速和速度斜坡，保证全程定速。生成的表用于控制器速度调度；原赛道则用于验证“速度规划/曲率限速 + LQR 表”的完整闭环表现。

## 19. 标准赛道 sweep 增加定速模式

### 需求

标准赛道的目标是纯粹标定不同固定速度下的 LQR `Q/R/ff`，因此 sweep 时不能让曲率限速或速度斜坡改变速度。否则 `speed=3.0` 仍可能在局部因为转向命令触发限速，破坏“固定速度标定”的实验假设。

### 修改

在 `SimConfig` 和 `run_sweep.py` 中加入：

```
enable_curvature_speed_limit = True
--disable-curvature-speed-limit
```

关闭后，离线仿真直接使用：

```
speed_cmd = target_speed
```

标准赛道 sweep 命令同步改为：

```
--disable-curvature-speed-limit
--disable-speed-ramp
```

这样 stadium 赛道上的 LQR 表标定就是严格的全程定速测试。原赛道完整验证仍应开启曲率限速和速度斜坡。

## 20. 低速多圈 sweep 超时修正

### 问题

标准赛道一圈约 `41 m`，0.5m/s 跑 3 圈理论耗时：

```
41 * 3 / 0.5 ≈ 247 s
```

但 `SimConfig.max_sim_time` 默认只有 `120 s`，因此 0.5m/s 的所有候选都会 timeout，导致该速度点没有任何有限分数。

### 修改

`run_sweep.py` 新增：

```
--max-sim-time
```

如果不手动指定，脚本会根据轨迹长度、圈数和最低测试速度自动估算：

```
max_sim_time = max(120, 1.5 * path_length * laps / min_speed + 20)
```

启动 sweep 时会打印每个候选允许的最大仿真时间。stadium 标准赛道示例命令显式使用：

```
--max-sim-time 420
```

避免低速 5 圈标定被错误 timeout。

### 进一步诊断

stadium 定速 sweep 在 0.5m/s 仍可能出现“所有候选都是 `inf`”。为避免再次只看到伪最优或笼统错误，已增强 `sweep_engine.py`：

- `_evaluate_single()` 返回失败原因和 metrics。
- 若某速度点没有任何有限分数，错误信息会打印：
  - collision / timeout / objective_constraints 数量。
  - `max_abs_e_y`、`steering_saturation_ratio`、`steering_rate_rms_deg_s` 等指标范围。

这样可以区分是地图/碰撞问题、低速超时问题，还是 objective 阈值过严。

### 开放地图像素语义修正

stadium 开放地图最初使用 `255` 表示 free space，但原始可运行地图 `my_map.pgm` 的 free space 像素为 `254`，障碍为 `0`。f1tenth_gym 的地图/距离场处理对 `255` 可能存在特殊语义，导致全开放地图仍被判 `collision`。

已将 `generate_feasible_track.py` 中的 free-space 像素从 `255` 改为 `254`，与原始地图保持一致：

```
free = 254
occupied = 0
```

同时 collision 失败原因中加入仿真时间、位置、横向误差、航向误差和路径索引，便于继续诊断。

### `done` 误判修正

开放地图重新测试后，所有候选仍返回 `collision`，但诊断信息显示：

```
t≈327s, x≈0.32, y≈4.0, e_y≈0, idx≈510
```

这不是偏离轨迹撞墙，而是车辆贴着轨迹回到起点附近时 gym 返回了 `done=True`。原 `sim_harness.py` 将所有 `done` 都解释为 collision，因此把成功完成目标圈数误判为碰撞。

已修正：当 `done=True` 且车辆位于起点附近、累计距离已超过目标圈数阈值、横向误差很小时，视为成功完成目标圈数并正常结束仿真；只有其他 `done` 情况才按 collision 处理。

补充修正：gym 返回 `done` 时可能发生在 harness 的圈数计数更新之前，因此仅依赖累计距离阈值仍会误判。现改为：当 `done=True` 且车辆在起点附近、横向误差很小、仿真时间已达到目标圈数理论耗时的 75% 以上时，视为成功完成目标圈数。

## 21. ROS 原地图可视化启动失败修正

### 问题

使用原地图和 0.5m/s sweep 参数启动完整 ROS 可视化时，车辆不动。终端日志显示 LQR 节点直接退出：

```
Couldn't parse parameter override rule: '-p lqr_gain_table_path:='
```

原因是 `pnc_sim_launch.py` 无论是否提供增益表路径，都会向 `run_lqr.py` 传入空参数：

```
-p lqr_gain_table_path:=
```

ROS Foxy 不能解析这种空字符串参数覆盖，因此 LQR controller 没有启动，`/drive` 不会发布，车辆自然不动。

### 修改

`launch/pnc_sim_launch.py` 改为只在 `lqr_gain_table_path` 非空时传入该参数。

同时新增两个可从命令行覆盖的 launch 参数：

```
use_tf_pose
enable_curvature_speed_limit
```

这样原地图 ROS 可视化可以先用：

```
use_tf_pose:=false
```

让 LQR 直接使用 `/ego_racecar/odom` 中的位姿，避免 TF 链路异常时控制器无法得到车辆位姿。

## 22. Stadium full sweep 表与插值点验证

### Full sweep 输出

使用开放 stadium 标准赛道、3 圈、全程定速、关闭曲率限速和速度 ramp 完成 full sweep：

```
python3 -m lqr_sweep.run_sweep --mode full \
  --map-path /sim_ws/src/f1tenth_gym_ros/maps/stadium_3ms_open \
  --trajectory-csv /sim_ws/src/f1tenth_gym_ros/code/outputs/generated_tracks/stadium_3ms_trajectory.csv \
  --speeds 0.5 1.0 1.5 2.0 2.5 3.0 \
  --coarse-grid 5,4,5,3 \
  --laps 3 \
  --max-sim-time 420 \
  --disable-curvature-speed-limit \
  --disable-speed-ramp \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_stadium
```

生成：

```
/sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_stadium/lqr_gain_table.yaml
/sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_stadium/sweep_summary.json
```

表项：

| speed | q_lateral | q_heading | r_steering | ff | score |
|---:|---:|---:|---:|---:|---:|
| 0.5 | 8.3816 | 6.0000 | 3.9000 | 1.0 | 0.0148 |
| 1.0 | 10.2000 | 0.5750 | 30.0000 | 0.8 | 0.2664 |
| 1.5 | 1.0000 | 0.5000 | 3.0000 | 0.8 | 0.2106 |
| 2.0 | 12.0000 | 1.4881 | 9.4868 | 0.8 | 0.1693 |
| 2.5 | 4.5033 | 5.1000 | 3.0000 | 0.8 | 0.1181 |
| 3.0 | 8.3816 | 5.1000 | 3.0000 | 1.0 | 0.0790 |

### 离散速度 validate

对表内 6 个速度点重新 validate，全部完成 3 圈：

| speed | score | mean_abs_e_y | lap time |
|---:|---:|---:|---:|
| 0.5 | 0.0148 | 0.04 cm | 328.2 s |
| 1.0 | 0.2664 | 0.36 cm | 164.2 s |
| 1.5 | 0.2106 | 0.11 cm | 109.7 s |
| 2.0 | 0.1693 | 0.14 cm | 82.4 s |
| 2.5 | 0.1181 | 0.16 cm | 66.0 s |
| 3.0 | 0.0790 | 0.21 cm | 55.1 s |

结论：表内离散速度点稳定，full sweep 结果可以复现。

### 参数跳变观察

表中存在明显相邻速度参数跳变：

- `0.5 -> 1.0`：`q_heading 6.0 -> 0.575`，`R 3.9 -> 30.0`
- `1.0 -> 1.5`：`q_lateral 10.2 -> 1.0`，`R 30.0 -> 3.0`
- `1.5 -> 2.0`：`q_lateral 1.0 -> 12.0`
- `2.0 -> 2.5`：`q_heading 1.488 -> 5.1`

因此需要额外验证 lookup table 线性插值后的中间速度点，而不是只验证表内离散速度。

### 插值速度 1.25m/s 验证

`1.25m/s` 使用 `1.0` 和 `1.5` 两个表项线性插值得到：

```
q_lateral=5.6
q_heading=0.5375
r_steering=16.5
feedforward_gain=0.8
```

单点测试命令：

```
python3 -m lqr_sweep.run_sweep --mode single \
  --map-path /sim_ws/src/f1tenth_gym_ros/maps/stadium_3ms_open \
  --trajectory-csv /sim_ws/src/f1tenth_gym_ros/code/outputs/generated_tracks/stadium_3ms_trajectory.csv \
  --speed 1.25 \
  --q-lateral 5.6 \
  --q-heading 0.5375 \
  --r-steering 16.5 \
  --feedforward-gain 0.8 \
  --laps 3 \
  --max-sim-time 420 \
  --disable-curvature-speed-limit \
  --disable-speed-ramp
```

结果：

| metric | value |
|---|---:|
| completed_laps | 3 |
| lap_time | 131.54 s |
| score | 0.2464 |
| mean_abs_e_y | 0.002809 m |
| p95_abs_e_y | 0.012142 m |
| max_abs_e_y | 0.018970 m |
| mean_abs_e_psi_deg | 1.309715 deg |
| p95_abs_e_psi_deg | 2.380825 deg |
| steering_rms_deg | 3.980086 deg |
| steering_rate_rms_deg_s | 17.847090 deg/s |
| steering_saturation_count | 0 |
| steering_saturation_ratio | 0 |
| speed_rms_error | 0.052422 m/s |

结论：`1.25m/s` 插值点健康。虽然表项在 `1.0 -> 1.5` 之间参数跳变明显，但线性插值后的中间参数没有出现蛇形、转角饱和或横向误差放大。

后续建议继续补测：

```
0.75, 1.75, 2.25, 2.75 m/s
```

如果这些插值点也稳定，可以进入原地图 ROS 验证阶段。

## 23. 阶段 1：原地图 ROS 验证脚本增强

### 目标

将 stadium 标定得到的 `lqr_gain_table.yaml` 放回原地图完整 ROS 链路中验证，检查：

- LQR controller 是否正常启动并发布 `/drive`。
- 原地图轨迹上是否能完成多圈。
- 曲率限速和 speed ramp 开启后是否能避免急弯硬冲。
- 生成的 tracking log 是否可以用 `tracker_evaluate.py` 评估。

### 修改

`code/lqr_sweep/validate_ros.py` 增强 CLI 参数透传能力：

```
--track-csv
--trajectory-csv
--log-path
--min-speed
--max-lateral-accel
--max-steering-angle
--use-tf-pose
--disable-curvature-speed-limit
--disable-speed-ramp
--max-accel
--max-decel
```

默认行为用于原地图安全验证：

- 开启曲率限速。
- 开启速度斜坡。
- 默认 `use_tf_pose=false`，直接使用 `/ego_racecar/odom` 位姿，避免 TF 链路不完整导致 LQR 不发控制。

### 原地图验证命令

```
cd /sim_ws/src/f1tenth_gym_ros/code
python3 -m lqr_sweep.validate_ros \
  --table /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_stadium/lqr_gain_table.yaml \
  --speed 1.5 \
  --laps 3 \
  --timeout 240 \
  --track-csv /sim_ws/src/f1tenth_gym_ros/code/outputs/csv/processed_track.csv \
  --trajectory-csv /sim_ws/src/f1tenth_gym_ros/code/outputs/csv/global_trajectory.csv \
  --log-path /sim_ws/src/f1tenth_gym_ros/code/outputs/logs/ros_validate_original_map_1p5.csv \
  --min-speed 0.4 \
  --max-lateral-accel 4.0 \
  --max-accel 1.0 \
  --max-decel 2.0 \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/evaluation_ros/original_map_table_1p5
```

### 成功标准

- 终端出现 `LQR controller started`。
- `/drive` 有稳定发布。
- `completed laps >= 3`。
- `steering_saturation_ratio` 接近 0。
- `max_abs_e_y < 0.25 m`，`mean_abs_e_y < 0.05 m`。
