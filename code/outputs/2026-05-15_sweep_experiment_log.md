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

### 阶段一批量 RViz 验证补充

单独验证 `1.5m/s` 不足以覆盖原地图风险，因此 `validate_ros.py` 增加批量模式：

```
--mode batch
```

批量模式用于从 `0.5m/s` 到 `3.0m/s` 每隔 `0.5m/s` 做一组原地图 RViz 验证。每组默认可以跑满 `300s`，并将该组实验单独归档：

```
evaluation_ros/<batch_name>/
  manifest.csv
  v0p5_table_stadium_curv1_ramp1_alat4p0_<timestamp>/
    logs/
      *_launch.log
      *_tracking.csv
    evaluation/
      lookahead_summary.csv
      lookahead_summary.json
      *_lateral_error.png
      *_heading_error.png
      *_path_overlay.png
      *_timing.png
```

推荐命令：

```
cd /sim_ws/src/f1tenth_gym_ros/code
python3 -m lqr_sweep.validate_ros \
  --mode batch \
  --table /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_stadium/lqr_gain_table.yaml \
  --speeds 0.5 1.0 1.5 2.0 2.5 3.0 \
  --timeout 300 \
  --laps 99 \
  --track-csv /sim_ws/src/f1tenth_gym_ros/code/outputs/csv/processed_track.csv \
  --trajectory-csv /sim_ws/src/f1tenth_gym_ros/code/outputs/csv/global_trajectory.csv \
  --min-speed 0.4 \
  --max-lateral-accel 4.0 \
  --max-accel 1.0 \
  --max-decel 2.0 \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/evaluation_ros \
  --batch-name original_map_rviz_table_curvlimit_ramp_0p5_to_3p0_300s
```

其中 `--laps 99` 用于避免完成几圈后提前停止，让每组尽量跑满 `300s`。如果某组撞车或 ROS 节点退出，仍会保留 launch log 和已有 tracking log，便于定位问题。

### Worktree 使用注意

`--mode batch` 是阶段一分支 `stage/original-map-validate` 的新功能。如果容器仍从主目录启动：

```
/home/art3m1s/f1tenth_gym_ros
```

则容器内挂载的是 main 工作区，旧版 `validate_ros.py` 不包含批量参数，会出现：

```
unrecognized arguments: --mode batch --speeds ...
```

阶段一测试必须从阶段一 worktree 启动容器：

```
cd /home/art3m1s/f1tenth_stage1_original_map_validate
rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros
```

另外，如果将单轮测试时间从 `300s` 改为 `150s`，`--batch-name` 也应同步改成包含 `150s`，避免结果目录命名和真实测试时长不一致。

### Headless 批量测试修正

RViz 只用于人工观察；批量统计时可以关闭，减少图形渲染和窗口状态对 ROS 进程的干扰。`pnc_sim_launch.py` 新增：

```
enable_rviz:=false
```

`validate_ros.py` 对应新增：

```
--disable-rviz
```

后续自动批量测试建议默认加上 `--disable-rviz`。

同时修正两个归档问题：

- 速度标签从一位小数改为保留必要小数，避免 `0.75m/s` 被命名成 `v0p8`。
- 每组测试新增 `logs/*_evaluator.log`。如果 `tracker_evaluate.py` 失败，会在 `evaluation/` 下写入 `evaluation_failed.txt`，避免出现空 evaluation 文件夹却没有错误原因。

### 阶段一噪声/延迟验证选项

为避免只在理想 odom 下验证参数，阶段一 ROS 验证新增可选的位姿噪声和延迟注入。默认仍然是干净测试，不影响已有命令：

```
--noise-profile clean
```

新增轻量鲁棒性测试 preset：

```
--noise-profile light
```

该 preset 会让 LQR 控制器“看到”的位姿带有 `2cm` 位置高斯噪声、`1deg` 航向高斯噪声和 `60ms` 位姿延迟。控制使用带噪声/延迟的位姿，但 tracking CSV 仍记录真实 odom 相对参考轨迹的误差，因此评估图和 summary 反映的是噪声控制后真实车辆轨迹是否变差。

如需单独调节强度，可覆盖：

```
--position-noise-std 0.02
--heading-noise-std-deg 1.0
--pose-delay-ms 60
--noise-seed 42
```

归档结构同步调整为按噪声条件分文件夹，避免 clean 和 noisy 结果混在一起：

```
evaluation_ros/<batch_name>/clean/
evaluation_ros/<batch_name>/noisy_light_pos2cm_yaw1deg_delay60ms/
```

推荐先跑 `clean` 作为基线，再跑 `light` 看 `mean_abs_e_y`、`p95_abs_e_y`、`max_abs_e_y`、`mean_abs_e_psi`、`steering_rate_rms` 是否明显恶化。若 clean 通过但 light 下误差或转向变化率明显放大，说明当前表对感知/定位扰动比较敏感，后续再考虑增大 `R`、降低高速度段激进程度或调速度规划。

### 低速航向误差优化

低速段补扫 `0.5, 0.625, 0.75, 0.875, 1.0m/s` 后，横向误差明显改善，但 `0.62~1.0m/s` 的航向误差仍偏大：

- `0.62m/s`: `mean_abs_e_y=2.19cm`, `p95_abs_e_y=3.83cm`, `mean_abs_e_psi=4.21deg`, `p95_abs_e_psi=8.28deg`
- `0.75m/s`: `mean_abs_e_y=1.86cm`, `p95_abs_e_y=3.26cm`, `mean_abs_e_psi=4.06deg`, `p95_abs_e_psi=8.05deg`
- `1.0m/s`: `mean_abs_e_y=1.57cm`, `p95_abs_e_y=2.55cm`, `mean_abs_e_psi=3.87deg`, `p95_abs_e_psi=7.55deg`

原因判断：原 sweep 目标函数主要由平均横向误差和平均航向误差组成，且低速 sweep 允许 `q_heading=0.5`、`R=30`。结果低速段容易选到“横向贴线但航向修正很弱”的保守控制器。

解决方式：新增 `run_sweep.py` 的低速航向优化 profile：

```
--objective-profile low-speed-heading
```

该 profile 会：

- 在目标函数中加入 `p95_abs_e_y` 和 `p95_abs_e_psi_deg`；
- 提高 `mean/p95` 航向误差的排序权重；
- 默认将 `q_heading` 搜索范围改为 `2.0~12.0`；
- 默认将 `R` 搜索范围收窄为 `3.0~15.0`，避免候选长期顶到高 `R`。

推荐下一轮低速航向优化命令：

```
cd /sim_ws/src/f1tenth_gym_ros/code
python3 -m lqr_sweep.run_sweep --mode full \
  --map-path /sim_ws/src/f1tenth_gym_ros/maps/stadium_3ms_open \
  --trajectory-csv /sim_ws/src/f1tenth_gym_ros/code/outputs/generated_tracks/stadium_3ms_trajectory.csv \
  --speeds 0.5 0.625 0.75 0.875 1.0 \
  --coarse-grid 5,4,5,3 \
  --laps 3 \
  --max-sim-time 420 \
  --disable-curvature-speed-limit \
  --disable-speed-ramp \
  --objective-profile low-speed-heading \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_stadium_low_speed_heading
```

2026-05-17 复扫过程中发现该 profile 仍然会在 `0.6~0.9m/s` 选到 `q_heading` 下界和 `R` 上界，例如 `q_heading=1.0`、`R=25.0`。因此进一步加强该 profile：

- 权重从 `lateral_mean=0.30, heading_mean=0.35, lateral_p95=0.10, heading_p95=0.25` 调整为 `0.15, 0.45, 0.05, 0.35`；
- 默认 `q_heading_range` 从 `1.0~6.0` 改为 `2.0~12.0`；
- 默认 `r_steering_range` 从 `3.0~25.0` 改为 `3.0~15.0`。

若复扫后仍然撞 `q_heading` 下界或 `R` 上界，说明单纯调权重还不够，下一步应直接加硬约束或固定一组低速候选范围，例如 `q_heading=3~12`、`R=3~10`。

### 第一阶段收尾与进入第二阶段结论

2026-05-17 将低速补点表合并进主表后，完成原地图 clean 验证。该轮验证使用曲率限速和速度斜坡，运行时长实际为 `60s`，结果显示：

- `0.5~2.0m/s` 横向误差整体可控，适合作为后续低速/中速实车候选；
- `2.5m/s` 横向误差仍可接受，但转向变化率已经升高，需要谨慎；
- `3.0m/s` 虽然仿真横向误差不算爆炸，但控制动作过激，实车风险较高。

关键 `3.0m/s` 指标：

- `mean_abs_e_y=2.47cm`
- `p95_abs_e_y=5.33cm`
- `max_abs_e_y=6.07cm`
- `steering_rms=10.71deg`
- `steering_rate_rms=49.9deg/s`
- `delta_cmd p95_abs=0.299rad`
- `delta_cmd max_abs=0.36rad`，已触及转角限幅
- `steering saturation ratio≈2.8%`
- `delta_rate p95≈118deg/s`
- `delta_rate max≈743deg/s`

判断：第一阶段已经完成“LQR 参数表生成 + 原地图 ROS 验证 + 低速补点 + 噪声/延迟压力测试 + 高速风险识别”。当前主要瓶颈不再是继续微调 `Q/R`，而是高速弯道前没有足够提前、平滑的速度规划，以及控制器缺少实车友好的转角速率限制。

因此第一阶段可以合并入 `main`，进入第二阶段。第二阶段目标：

1. 实现前方曲率预瞄速度规划，而不是只看当前点曲率；
2. 对速度 profile 做加速度/减速度平滑，确保入弯前提前降速；
3. 给 LQR 输出增加 steering rate limit，避免实车舵机收到过大的瞬时转角变化；
4. 先在原地图 clean 验证，再做 light noisy 压力测试；
5. 实车测试从 `0.5 -> 1.0 -> 1.5m/s` 递进，不直接上 `3.0m/s`。

### 第二阶段实施计划：曲率预瞄限速与转角速率限制

第二阶段先不继续修改 `Q/R`，重点解决 `3.0m/s` 下转向命令变化过激的问题。计划分三步实现：

1. 前方曲率预瞄限速；
2. 速度命令平滑；
3. steering rate limit。

#### 1.0m 曲率预瞄限速逻辑

预瞄距离先固定为 `1.0m`。每个控制周期已经有车辆当前位置投影到参考路径的 `segment_idx` 和 `segment_t`，因此不需要重新找一个“单点 lookahead target”，而是从当前投影位置沿参考轨迹向前扫描一段弧长。

具体逻辑：

1. 从当前 `segment_idx` 开始；
2. 累加后续轨迹段长度；
3. 直到累计弧长达到 `1.0m`；
4. 收集这段窗口内所有轨迹点或轨迹段的 `abs(curvature)`；
5. 取窗口内最大值：

```
kappa_preview = max(abs(curvature[i]) for s_i in [0, 1.0m])
```

然后用横向加速度约束计算安全速度：

```
v_curve = sqrt(max_lateral_accel / max(kappa_preview, eps))
v_target = clamp(v_curve, min_speed, target_speed)
```

如果路径是闭环，扫描到末尾后从开头继续；如果路径是开环，则扫描到终点停止。这样做的好处是车会在弯道前提前看到高曲率，而不是等当前位置曲率变大后才降速。

初始参数建议：

- `curvature_speed_lookahead_m = 1.0`
- `max_lateral_accel = 3.5~4.0m/s^2`
- `min_speed = 0.4m/s`

验证时重点画 `curvature_ref`、`kappa_preview`、`v_cmd`、`delta_cmd`。理想现象是：进入急弯前 `kappa_preview` 先升高，`v_cmd` 提前下降，`delta_cmd` 峰值和变化率下降。

#### 速度命令平滑：先用 ramp，不先上 PID

当前问题不是“车辆实际速度跟不上速度命令”的闭环纵向控制问题，而是“速度命令本身是否提前、平滑”。因此第二阶段先保留当前 ramp：

```
v_cmd[k] = v_cmd[k-1] + clamp(v_desired - v_cmd[k-1],
                             -max_decel * dt,
                              max_accel * dt)
```

初始建议：

- `max_accel = 1.0m/s^2`
- `max_decel = 2.0~3.0m/s^2`

暂时不引入 PID 的原因：

- F1TENTH gym/ROS bridge 对 `AckermannDrive.speed` 已经有底层速度跟踪模型，外层再加 PID 可能和底层模型叠加；
- 当前最需要调的是速度参考 profile，而不是电机闭环；
- PID 需要真实速度反馈、积分限幅、抗饱和和低速死区处理，会引入新的调参维度。

如果后续日志显示 `v_actual` 长期跟不上 `v_cmd`，例如 `speed_rms_error` 很大，才进入纵向 PID 或速度前馈/反馈控制。

#### Steering rate limit 风险与应对

转角速率限制形式：

```
delta_limited = delta_prev + clamp(delta_raw - delta_prev,
                                  -max_steering_rate * dt,
                                   max_steering_rate * dt)
```

初始建议：

- `max_steering_rate = 2.0rad/s`
- 如果跟踪明显变差，再试 `3.0rad/s`

它确实等价于给转向执行增加低通/阻尼，风险是急弯入口可能“转不过来”，表现为：

- `p95_abs_e_y` 或 `max_abs_e_y` 明显增大；
- 弯道入口外侧偏差变大；
- `delta_raw` 和 `delta_limited` 长时间差距很大；
- steering saturation ratio 降低了，但横向误差上升。

遇到这种情况的处理顺序：

1. 先降低速度规划的 `max_lateral_accel`，让车更早更慢入弯；
2. 增大曲率预瞄距离，例如从 `1.0m` 调到 `1.5m`；
3. 将 `max_steering_rate` 从 `2.0rad/s` 放宽到 `3.0rad/s`；
4. 如果仍然转不过来，再考虑局部调整高速段 `Q/R`。

关键原则：实车安全优先。宁愿通过速度规划让车慢一点，也不要允许控制器用极大的转角变化率硬追轨迹。

#### 第二阶段验收标准

先在原地图 clean 验证 `1.5, 2.0, 2.5, 3.0m/s`：

- `steering_rate_rms` 相比第一阶段明显下降；
- `delta_rate p95` 明显下降；
- `steering saturation ratio` 下降或保持较低；
- `p95_abs_e_y` 不明显恶化；
- `3.0m/s` 若仍不安全，则将实车上限暂定为 `2.0~2.5m/s`。

clean 通过后再做 `light` noisy 压力测试。noisy 结果只作为鲁棒性参考，不直接用于决定是否上实车高速。

### 2026-05-17 第二阶段实现记录：预瞄限速与转角速率限制

已在 `stage/speed-planning-steering-rate` worktree 开始实现第二阶段功能，目标是降低原地图
`3.0m/s` 下过大的 `delta_cmd` 高频变化和转角饱和风险。

本次代码改动：

- `pnc_rc/lqr/controller.py`
  - 新增 `curvature_speed_lookahead_m`，默认 `1.0m`；
  - 新增路径段长度缓存 `segment_lengths`；
  - 每个控制周期从当前投影点沿路径向前扫描 `1.0m`，取窗口内最大绝对曲率作为
    `curvature_preview`；
  - 曲率限速从原来的当前点曲率改为预瞄曲率：
    `v_curve=sqrt(max_lateral_accel / max(curvature_preview, eps))`；
  - 新增 `enable_steering_rate_limit` 和 `max_steering_rate`，默认启用，
    初始值 `2.0rad/s`；
  - 转角命令先由 LQR 计算并按 `max_steering_angle` 裁剪，再按每周期最大变化量
    `max_steering_rate * dt` 进行限幅；
  - 日志新增 `curvature_preview`、`delta_raw`、`delta_rate_limited` 三列，用于区分
    LQR 原始转角、最终执行转角以及是否被速率限制命中。

- `launch/pnc_sim_launch.py`
  - 新增 launch 参数：`curvature_speed_lookahead_m`、`enable_steering_rate_limit`、
    `max_steering_rate`；
  - ROS validate 可以直接通过命令行切换预瞄距离和转角速率限制。

- `lqr_sweep/validate_ros.py`
  - batch/single 验证支持 `--curvature-speed-lookahead-m`、
    `--disable-steering-rate-limit`、`--max-steering-rate`；
  - manifest 会记录预瞄距离、是否开启转角速率限制、最大转角速率；
  - launch 输出中打印速度处理和转角处理配置，方便回看实验条件。

第一轮建议测试：

```bash
python3 -m lqr_sweep.validate_ros \
  --mode batch \
  --table /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_stadium/lqr_gain_table.yaml \
  --speeds 1.5 2.0 2.5 3.0 \
  --timeout 100 \
  --laps 99 \
  --min-speed 0.4 \
  --max-lateral-accel 3.5 \
  --curvature-speed-lookahead-m 1.0 \
  --max-accel 1.0 \
  --max-decel 3.0 \
  --max-steering-rate 2.0 \
  --noise-profile clean \
  --disable-rviz \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/evaluation_ros \
  --batch-name stage2_preview1p0_rate2p0_clean_100s
```

验收重点：

- `3.0m/s` 的 `delta_rate p95/max` 和 `steering_rate_rms` 应明显下降；
- `delta_cmd` 不应长时间贴着 `±0.36rad`；
- `p95_abs_e_y` 和 `max_abs_e_y` 不能明显恶化；
- 如果出现转不过弯，先降低 `max_lateral_accel` 到 `3.0~3.2` 或将预瞄距离增大到
  `1.5m`，再考虑把 `max_steering_rate` 放宽到 `3.0rad/s`。

### 2026-05-17 Stage2 第一轮 clean 验证结果

运行目录：

```text
code/outputs/evaluation_ros/stage2_preview1p0_rate2p0_clean_100s/clean
```

验证配置：

- 速度点：`1.5, 2.0, 2.5, 3.0m/s`
- 地图：原地图
- lookup table：`outputs/sweep_stadium/lqr_gain_table.yaml`
- 曲率限速：开启
- 曲率预瞄：`1.0m`
- `max_lateral_accel=3.5m/s^2`
- 速度 ramp：开启，`max_accel=1.0m/s^2`, `max_decel=3.0m/s^2`
- 转角速率限制：开启，`max_steering_rate=2.0rad/s`
- 噪声：clean，无额外位姿噪声和延迟
- RViz：关闭，自动日志和评估

评估摘要：

| speed | mean e_y | p95 e_y | max e_y | mean e_psi | p95 e_psi | steering_rms | steering_rate_rms |
|------:|---------:|--------:|--------:|-----------:|----------:|-------------:|------------------:|
| 1.5 | 0.50 cm | 1.08 cm | 1.56 cm | 3.31 deg | 7.02 deg | 9.81 deg | 87.7 deg/s |
| 2.0 | 0.72 cm | 1.62 cm | 3.20 cm | 2.64 deg | 5.99 deg | 10.17 deg | 99.5 deg/s |
| 2.5 | 1.60 cm | 3.59 cm | 4.34 cm | 2.52 deg | 6.12 deg | 10.69 deg | 101.1 deg/s |
| 3.0 | 2.92 cm | 5.79 cm | 7.16 cm | 2.52 deg | 6.25 deg | 10.74 deg | 104.6 deg/s |

3.0m/s 详细观察：

- `curvature_preview` 正常工作，预瞄曲率 `p95≈0.91 1/m`；
- `v_cmd` 被限速到约 `1.75~2.99m/s`，平均约 `2.23m/s`；
- `v_actual` 平均约 `2.18m/s`，能跟随速度命令变化；
- `delta_raw` 的 `p95_abs≈0.386rad`，仍经常超过物理转角上限；
- `delta_cmd` 仍会触碰 `±0.36rad`，饱和比例约 `5.7%`；
- `delta_rate_limited` 命中比例约 `20.8%`，说明转角速率限制经常介入；
- 但 `steering_rate_rms≈104.6deg/s`，比第一阶段 3.0m/s 的约 `49.9deg/s` 更差。

结论：

1. 预瞄限速已经生效，车辆在高曲率区域会提前降到约 `1.75~2.0m/s`。
2. 当前 `max_steering_rate=2.0rad/s` 的硬限幅没有让转角更平滑，反而让 `delta_cmd`
   频繁以最大允许斜率追赶 `delta_raw`，形成连续斜坡/锯齿波形。
3. 3.0m/s 横向误差仍可接受但比第一阶段变差，且转角饱和和转角变化率仍不适合直接上实车。
4. 这一轮不能认为 Stage2 已经通过，高速实车上限仍建议暂定 `2.0~2.5m/s`。

下一步建议：

1. 跑一组 `--disable-steering-rate-limit`，保持 `lookahead=1.0m` 和
   `max_lateral_accel=3.5` 不变，用来分离“预瞄限速”和“转角速率限制”的影响。
2. 跑一组更保守速度规划：`max_lateral_accel=3.0`，必要时再试 `2.5`；
   目标是降低 `delta_raw` 本身，而不是靠 rate limit 硬拦。
3. 若仍需要限制转角变化，应把单纯 slew-rate limiter 改成更温和的转向一阶低通/执行器模型，
   或先对 `delta_raw` 做滤波再限幅，避免当前这种持续顶着最大斜率追踪的锯齿行为。
4. 后续评价中需要额外输出 `delta_rate p95/max` 和 steering saturation ratio，当前 summary
   只保留 `steering_rate_rms`，不够直观。

### 2026-05-17 Stage2 评估脚本补强与下一轮测试

根据第一轮 clean 验证，先补强评估工具，再继续做控制参数对照。原因是第一轮 summary 中
`speed_rms_error`、`steering_saturation_ratio` 为空，且没有直接输出 `delta_rate p95/max`，
导致只能手动解析 tracking CSV。

本次改动：

- `tracker_evaluate.py`
  - LQR 日志中的 `v_cmd` 会自动映射为评估用 `v_ref`，因此下一轮 summary 会正常输出
    `mean_abs_speed_error_mps` 和 `speed_rms_error_mps`；
  - LQR 日志中的 `yaw` 会映射为 `yaw_vehicle`，便于 yaw-rate 统计；
  - 新增 `--steering-limit-rad`，当日志没有 `steering_limit` 列时用该值计算转角饱和比例；
  - summary 新增 `steering_rate_p95_deg_s`、`steering_rate_max_deg_s`、
    `p95_abs_delta_cmd_deg`、`max_abs_delta_cmd_deg`；
  - summary 新增 `delta_rate_limited_count` 和 `delta_rate_limited_ratio`；
  - terminal recommendation 中也显示 `steering_rate_p95`、`steering_sat_ratio` 和
    `delta_rate_limited_ratio`。

- `validate_ros.py`
  - 自动调用 `tracker_evaluate.py --steering-limit-rad <max_steering_angle>`，
    因此后续 ROS batch 评估会自动带出饱和比例。

下一轮对照测试顺序：

1. 保持 `lookahead=1.0m`、`max_lateral_accel=3.5`，关闭 steering rate limit：

```bash
python3 -m lqr_sweep.validate_ros \
  --mode batch \
  --table /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_stadium/lqr_gain_table.yaml \
  --speeds 1.5 2.0 2.5 3.0 \
  --timeout 100 \
  --laps 99 \
  --min-speed 0.4 \
  --max-lateral-accel 3.5 \
  --curvature-speed-lookahead-m 1.0 \
  --max-accel 1.0 \
  --max-decel 3.0 \
  --disable-steering-rate-limit \
  --noise-profile clean \
  --disable-rviz \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/evaluation_ros \
  --batch-name stage2_preview1p0_no_rate_limit_clean_100s
```

2. 若关掉 rate limit 后 `steering_rate` 更好，则进一步降低速度规划横向加速度上限：

```bash
python3 -m lqr_sweep.validate_ros \
  --mode batch \
  --table /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_stadium/lqr_gain_table.yaml \
  --speeds 2.0 2.5 3.0 \
  --timeout 100 \
  --laps 99 \
  --min-speed 0.4 \
  --max-lateral-accel 3.0 \
  --curvature-speed-lookahead-m 1.0 \
  --max-accel 1.0 \
  --max-decel 3.0 \
  --disable-steering-rate-limit \
  --noise-profile clean \
  --disable-rviz \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/evaluation_ros \
  --batch-name stage2_preview1p0_no_rate_limit_alat3p0_clean_100s
```

判断标准：

- 如果 `steering_rate_p95/max` 和 `steering_saturation_ratio` 明显下降，同时
  `p95_abs_e_y` 不明显变差，则说明应该优先靠速度规划降低转角需求；
- 如果关掉 rate limit 后转角也很糟，说明根因主要是轨迹/QR/速度上限本身太激进；
- 只有当速度规划已足够保守但转角仍有高频抖动时，再考虑把硬 slew-rate limiter 改成
  一阶转向执行器模型或低通滤波器。

### 2026-05-17 Stage2 no-rate-limit 对照验证结果

运行目录：

```text
code/outputs/evaluation_ros/stage2_preview1p0_no_rate_limit_clean_100s/clean
```

验证配置：

- 速度点：`1.5, 2.0, 2.5, 3.0m/s`
- 曲率预瞄：`1.0m`
- `max_lateral_accel=3.5m/s^2`
- 速度 ramp：开启，`max_accel=1.0m/s^2`, `max_decel=3.0m/s^2`
- steering rate limit：关闭
- 噪声：clean

评估摘要：

| speed | mean e_y | p95 e_y | max e_y | mean e_psi | p95 e_psi | steering_rms | steering_rate_rms | steering_rate_p95 | max delta | sat ratio |
|------:|---------:|--------:|--------:|-----------:|----------:|-------------:|------------------:|------------------:|----------:|----------:|
| 1.5 | 1.44 cm | 2.39 cm | 2.66 cm | 3.31 deg | 6.21 deg | 9.94 deg | 18.2 deg/s | 40.2 deg/s | 17.45 deg | 0.0% |
| 2.0 | 0.84 cm | 2.04 cm | 2.49 cm | 2.60 deg | 4.89 deg | 10.24 deg | 25.6 deg/s | 57.7 deg/s | 18.44 deg | 0.0% |
| 2.5 | 2.17 cm | 4.32 cm | 4.99 cm | 2.41 deg | 4.37 deg | 10.68 deg | 33.6 deg/s | 79.0 deg/s | 19.39 deg | 0.0% |
| 3.0 | 3.45 cm | 6.59 cm | 7.21 cm | 2.35 deg | 4.33 deg | 10.77 deg | 39.9 deg/s | 97.1 deg/s | 19.60 deg | 0.0% |

和上一轮 `max_steering_rate=2.0rad/s` 硬限幅对比：

- 3.0m/s 的 `steering_rate_rms` 从约 `104.6deg/s` 降到 `39.9deg/s`；
- 3.0m/s 的 steering saturation ratio 从约 `5.7%` 降到 `0.0%`；
- 3.0m/s 的航向误差改善：`p95 e_psi` 从约 `6.25deg` 降到 `4.33deg`；
- 但 3.0m/s 横向误差变差：`p95 e_y` 从约 `5.79cm` 上升到 `6.59cm`，
  `max e_y` 仍约 `7.2cm`；
- 2.5m/s 基本可接受但已接近边界：`max e_y=4.99cm`。

结论：

1. 硬 steering rate limiter 是上一轮锯齿和高 `steering_rate_rms` 的主要来源；
2. 当前阶段不建议继续使用简单 slew-rate limiter；
3. 关闭 rate limit 后转角波形明显健康，且没有转角饱和，但 3.0m/s 横向误差仍偏大；
4. `2.5m/s` 可以作为目前较稳的上限候选，`3.0m/s` 还不能直接上实车；
5. 下一步应优先调速度规划，而不是调 Q/R 或继续加硬转角限幅。

下一步执行：

- 先跑 `max_lateral_accel=3.0`、rate limit 关闭；
- 如果 3.0m/s 的 `p95 e_y/max e_y` 仍不满足，再跑 `max_lateral_accel=2.5`；
- 目标是让 3.0m/s 在不触发转角饱和的前提下，把 `p95 e_y` 压回 5cm 左右；
- 如果速度规划压低后仍不够，再考虑增大曲率预瞄距离到 `1.5m`。
