# 当前规划控制相对基础 Frenet 手写稿的改动说明

本文对照 `/home/art3m1s/references/paper_cn.pdf` 中 P8、P9、P22 的手写稿，说明当前仓库里的 Frenet 局部规划器和 LQR 控制器相比“最基本 Frenet 规划器”额外做了什么。

结论先写在前面：当前实现已经不是单纯的“`d_f`、`T`、`v_f` 三层采样 + 多项式 + 点障碍距离 + cost 排序”。它在基础 Frenet 外面加了一层 ROS 状态机，并在候选生成内部加入了局部栅格、车身扫掠碰撞、硬约束过滤、轨迹连续性、路径裁剪、限速发布和 LQR 闭环控制。这些逻辑提升了仿真稳定性，但也确实让代码看起来复杂。

## 基础手写稿做了什么

P8 的基础流程：

1. 横向 `d(t)` 使用五次多项式。
   - 起点约束：`d(0)=d0`，`d_dot(0)=d_dot0`，`d_ddot(0)=d_ddot0`。
   - 终点约束：`d(T)=d_f`，`d_dot(T)=0`，`d_ddot(T)=0`。
   - 目的：到达目标横向偏移后，不再继续横向运动。

2. 纵向 `s(t)` 使用四次多项式。
   - 起点约束：`s(0)=s0`，`s_dot(0)=s_dot0`，`s_ddot(0)=s_ddot0`。
   - 终点约束：`s_dot(T)=v_f`，`s_ddot(T)=0`。
   - 不约束 `s(T)`，只约束终点速度和终点加速度。

P9 的基础流程：

1. 第一层采样横向终点 `d_f`：`range(Dmin, Dmax, Ds)`。
2. 第二层采样规划时长 `T`：`range(Tmin, Tmax, Ts)`。
3. 第三层采样纵向目标速度 `v_f`：`range(Vmin, Vmax, Vs)`。
4. 对每组 `(d_f, T, v_f)` 生成一条候选轨迹。
5. 把候选轨迹从 Frenet 坐标转换到世界坐标。
6. 做碰撞检测：候选点和障碍物距离小于安全阈值 `R` 就认为碰撞。

P22 的基础目标函数：

1. 横向 jerk 代价：`J_d = sum(d'''(p)^2)`。
2. 纵向 jerk 代价：`J_s = sum(s'''(p)^2)`。
3. 速度误差：`d_s = (v_target - s_dot_f)^2`。
4. 横向代价：`C_d = k_j * J_d + k_t * T + k_d * (d_f - d)^2`。
5. 纵向代价：`C_s = k_j * J_s + k_t * T + k_d * d_s`。
6. 总代价：`C = k_lat * C_d + k_lon * C_s`。

## 当前实现的主流程

当前 Frenet 核心仍然保留基础三层采样结构，入口是：

- `code/pnc_rc/frenet/planner.py::plan_frenet_path()`

当前 ROS 节点主流程在：

- `code/pnc_rc/frenet/node.py::plan_once()`

简化后流程是：

1. 接收全局参考线 `/global_trajectory`，构造 `ReferencePath`。
2. 从 odom 估计车辆世界位置、yaw 和速度。
3. 把车辆状态投影到参考线，得到 Frenet 初始状态 `(s, d, s_dot, d_dot, s_ddot, d_ddot)`。
4. 由 LaserScan 和静态地图构造车辆局部占用栅格。
5. 先检测中心线前方是否有威胁。
6. 无威胁时发布中心线或回中路径。
7. 有远处威胁时进入 approach，发布中心线但同步低速限速。
8. 威胁进入激活距离后才真正生成 Frenet 候选。
9. 若上一条避障轨迹仍安全，优先复用。
10. 若需要重规划，三层采样 `(d_f, T, v_f)`，生成候选。
11. 对候选做硬过滤：前进性、航向跳变、车身扫掠碰撞、最小 clearance、曲率。
12. 对安全候选计算扩展 cost。
13. 在原始最低 cost 和上一帧通道连续性之间做二次选择。
14. 将选中路径裁剪/锚定到当前车辆附近，限制发布长度。
15. 发布 `/local_trajectory`，同时发布 `/local_trajectory_speed_limit`。
16. LQR 控制器跟踪局部路径，结合曲率限速、局部限速、速度斜坡和转向速率限制发 `/drive`。

## 额外改动清单

### 1. 参考线从简单数组升级为 `ReferencePath`

基础手写稿默认有一条参考线，可以做 Frenet 与世界坐标转换。

当前实现封装了 `ReferencePath`：

- 预计算每段线段长度 `segment_lengths`。
- 预计算累计弧长 `cumulative_s`。
- 预计算参考线航向 `headings`。
- 预计算曲率 `curvatures`。
- 用 KDTree 做最近点查询。
- 支持闭环赛道 `closed_loop=True`。
- 支持 `project_near(position, s_hint, window)`，只在上一帧 `s` 附近投影，降低闭环赛道投影跳点。

这比基础版多了“闭环连续性”和“局部投影防跳点”。

### 2. 初始状态不是直接给定，而是从 odom 估计

基础版通常直接给 `s0/d0/s_dot0/d_dot0/s_ddot0/d_ddot0`。

当前实现：

- 从 odom 四元数提取 yaw。
- 位置来自 odom pose。
- 速度优先由连续 odom 位置差分估计。
- 如果差分不可用，回退到 odom twist。
- 用参考线切向量投影速度得到 `s_dot`。
- 用参考线法向量投影速度得到 `d_dot`。
- `s_ddot` 由相邻规划周期 `s_dot` 差分估计。
- 对观测速度做异常过滤，默认 `max_observed_speed_mps <= 0` 时自动设为 `max(3.0, 1.5 * cruise_speed + 0.75)`。

这部分是为了适配仿真 odom/twist 的坐标语义和时间戳抖动。

### 3. 对纵向初始加速度做可信范围处理

基础版会直接使用 `s_ddot0` 作为四次多项式初始条件。

当前实现会先调用：

- `_forward_progress_planning_state(state, config)`

当前逻辑是：

- 非有限值：置 `0`。
- `abs(s_ddot) > max_reliable_initial_s_accel_mps2`：认为是离群估计，置 `0`。
- 否则按 `[-max_initial_s_accel_mps2, +max_initial_s_accel_mps2]` 限幅。
- 可信范围内的正、负加速度都会保留。

当前参数：

- `max_initial_s_accel_mps2 = 2.0`
- `max_reliable_initial_s_accel_mps2 = 3.0`

这相对基础版多了“估计加速度防离群”的保护。注意：当前版本已经不是“一律拒绝负加速度”，可信负加速度会保留。

### 4. 障碍物表示从点距离升级为局部占用栅格

基础 P9 是候选点和障碍物点算欧氏距离，小于半径 `R` 判碰撞。

当前实现构造局部栅格：

- LaserScan 有效 range 转成车体局部点。
- 雷达点加 `scan_offset_x_m` 转到 `base_link`/控制点坐标。
- 过滤 `0.0m` 或近似 0 的无效回波。
- 静态地图 `/map` 裁剪/旋转到车辆局部栅格后与 LaserScan 合并。
- 用 `distance_transform_edt()` 生成距离场。
- 用 `grid_inflation_radius_m` 对障碍膨胀。
- `distance_to_obstacle_m` 表示到膨胀障碍边界的 clearance。

这比基础版多了地图融合、栅格化、障碍膨胀和距离场。

### 5. 碰撞检测从路径点升级为车身扫掠区域

基础版一般检查候选轨迹点和障碍物距离。

当前实现检查整条轨迹的近似车身 footprint：

- `path_collision_sample_step_m`：沿路径加密采样。
- `corridor_radius_m`：路径左右扫出走廊半径。
- `corridor_sample_step_m`：走廊和 footprint 内部采样间隔。
- `footprint_front_m`：轨迹控制点向车头方向扫掠长度。
- `footprint_rear_m`：轨迹控制点向车尾方向扫掠长度。

生成的采样点会转换到局部栅格索引，若任一点落在 `occupied=True` 的 cell 中，就判定碰撞。

如果当前轨迹控制点是后轴中心，那么：

- `footprint_front_m` 应近似为后轴中心到车头最前端距离。
- `footprint_rear_m` 应近似为后轴中心到车尾最后端距离。

当前默认 `front=0.38m`、`rear=0.05m`，大体符合后轴中心/靠近后轴的控制点假设。若车长、后轴位置或 `base_link` 定义改变，需要重新量这两个参数。

### 6. 增加了多个硬过滤条件

基础版主要是碰撞过滤。

当前安全候选必须依次通过：

1. 前进性：
   - `s_dot_values` 不能为负。
   - `s_values` 不能倒退。
   - 总进度不能小于 `min_progress_step_m`。

2. 航向跳变：
   - 相邻路径点 heading jump 不能超过 `max_heading_jump`。

3. 车身扫掠碰撞：
   - footprint/corridor 查询局部栅格，不能碰撞。

4. 最小 clearance：
   - 即使没碰撞，`min_clearance` 也不能小于 `min_clearance_m`。

5. 曲率：
   - `max_curvature` 不能超过车辆可跟踪阈值。

这些硬过滤是为了避免 LQR 跟踪尖角、倒退、贴障碍或曲率过大的候选。

### 7. cost 函数比 P22 多了多项

基础 P22 cost 是：

- 横向 jerk。
- 纵向 jerk。
- 时间。
- 横向终点偏移。
- 终点速度误差。

当前 `score_candidate()` 仍包含这些基础项，但增加了：

- clearance 不足惩罚：低于 `safe_clearance` 但高于硬阈值时加 soft penalty。
- 最大曲率惩罚：避免太急的弯。
- 曲率变化率惩罚：主要抑制局部弯弯绕绕和曲率突变。
- 横向采样跳变惩罚：抑制 `d_values` 局部突变。

当前 cost 只在已经通过硬安全约束的候选之间排序，不负责碰撞兜底。

### 8. 候选选择增加了时间连续性

基础版通常直接取最低 cost。

当前流程是：

1. `plan_frenet_path()` 先按原始 cost 选 raw best。
2. node 层再调用 `_select_consistent_candidate()`。
3. 如果上一帧有 `d(s)` profile，会在当前所有 hard-safe candidates 中做二次选择。
4. 通过 `candidate_profile_consistency_weight` 惩罚与上一条通道差异大的候选。
5. 如果 raw best 的 clearance 明显更好，可以打破通道连续性锁定。

目的：减少左右两侧候选在相邻帧之间跳变，降低蛇形。

### 9. 增加了中心线威胁检测和激活状态机

基础版通常每帧都生成 Frenet 候选。

当前 node 先检查中心线前方是否有威胁：

- 无威胁：发布中心线。
- 威胁很远：继续中心线。
- 进入 slowdown/approach 区间：发布中心线但下发低速限速。
- 进入 activation 区间：才生成 Frenet 避障候选。
- 无候选：复用上一条安全轨迹或发布 stop path。

激活距离由速度相关公式计算：

```text
base + speed * reaction_time + speed^2 / (2 * decel)
```

然后又被限制到“当前发布路径长度 + margin”附近，避免太早进入 avoidance。

### 10. 增加了上一条安全轨迹复用

基础版每帧独立选一条新轨迹。

当前实现会缓存上一条安全避障轨迹：

- 若还未碰撞。
- clearance 仍高于阈值。
- 车辆到轨迹的横向误差不过大。
- heading 误差不过大。
- 剩余路径长度足够。

则直接裁剪后继续发布，不马上重新规划。

目的：减少频繁重规划带来的抖动。

### 11. 发布路径前会裁剪和限长

基础版生成完整候选后通常直接使用。

当前发布前会：

- 用当前车辆位置重锚定路径。
- 丢掉已经过时的前段。
- 保留一点 `published_path_lookahead_m`，避免路径从车身后方开始。
- 限制最小发布长度。
- 限制最大发布长度。
- 按发布间隔和路径变化做节流。

这部分是为了让下游 LQR 看到的 `/local_trajectory` 更稳定、更短、更贴近当前车辆。

### 12. 增加了 centerline return path

基础版没有单独的回中路径。

当前在无威胁或 approach 时，如果车辆当前 `d` 偏离中心线较大，不会直接发布 `d=0` 中心线，而是发布从当前 `d` 平滑回到 `0` 的 return path。

目的：避免局部路径突然横跳到中心线。

### 13. 增加了局部速度限制 topic

基础 Frenet 只输出路径。

当前 Frenet node 同时发布：

- `/local_trajectory`
- `/local_trajectory_speed_limit`

模式对应速度限制：

- `centerline`：`centerline_speed_limit_mps`，`-1.0` 表示清除局部限速。
- `approach`：`avoidance_speed_limit_mps`。
- `avoidance`：`avoidance_speed_limit_mps`。
- `stop`：`stop_speed_limit_mps`。

LQR 控制器订阅该限速 topic，并把速度指令裁剪到该限制以下。

### 14. 控制器不是基础 Frenet 的一部分，而是 LQR 跟踪局部路径

P8/P9/P22 主要讲规划，不含当前控制闭环。

当前控制器额外做：

- 投影车辆到当前跟踪路径。
- 计算横向误差和航向误差。
- 离散 LQR 反馈：`delta_fb = -K [e_y, e_psi]`。
- 曲率前馈：`delta_ff = feedforward_gain * atan(wheelbase * curvature_ref)`。
- 转向角限幅。
- 转向速率限制。
- 曲率限速。
- 接收 Frenet 局部限速。
- 速度斜坡限制加速度/减速度。
- 开放路径终点停车。
- 可选误差滤波、噪声/延迟验证、CSV 日志。

这让“规划”和“控制”耦合得更强：Frenet 不只需要路径安全，还要生成 LQR 可跟踪的形状。

### 15. ROS 运行层增加了 QoS、debug marker 和日志统计

当前实现还包含：

- `TRANSIENT_LOCAL` QoS，用于全局路径、局部路径、速度限制和地图等晚启动可接收最后一帧的 topic。
- RViz debug marker 显示 safe candidates 和最终候选。
- `FrenetPlanStats` 统计候选被哪类硬约束拒绝。
- 无解日志输出 `collision_rejections/progress_rejections/clearance_rejections` 等。

这些不改变基本数学，但显著增加了工程逻辑。

## 当前主要参数设置

下面按 `launch/pnc_sim_launch.py` 启动并启用 Frenet planner 时的典型生效值整理。注意：`enable_frenet_planner` 在 launch 默认是 `false`；只有显式启用时，LQR 才会跟踪 `/local_trajectory`，否则 LQR 默认跟踪 `/global_trajectory`。

### Frenet 采样参数

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `publish_rate_hz` | `20.0` | Frenet node 规划/发布频率 |
| `target_speed` | `1.5` | 候选速度误差项目标速度 |
| `d_min` | `-1.8` | 横向终点采样下界 |
| `d_max` | `1.8` | 横向终点采样上界 |
| `d_step` | `0.1` | 横向采样间隔 |
| `t_min` | `2.0` | 最短规划时长 |
| `t_max` | `3.0` | 最长规划时长 |
| `t_step` | `0.5` | 时长采样间隔 |
| `v_min` | `0.6` | 终点速度采样下界 |
| `v_max` | `2.5` | 终点速度采样上界 |
| `v_step` | `0.3` | 终点速度采样间隔 |
| `trajectory_dt` | `0.1` | 多项式离散采样时间间隔 |

### Frenet 硬约束和 cost 参数

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `max_curvature` | `1.14` | 候选最大曲率硬上限 |
| `max_heading_jump` | `0.85` | 相邻路径点 heading jump 硬上限 |
| `min_progress_step_m` | `0.20` | 候选总前进距离下限 |
| `max_initial_s_accel_mps2` | `2.0` | 初始 `s_ddot` 限幅 |
| `max_reliable_initial_s_accel_mps2` | `3.0` | 初始 `s_ddot` 离群拒绝阈值 |
| `safe_clearance_m` | `0.35` | 期望安全裕度，低于该值进入 soft cost |
| `min_clearance_m` | `0.05` | 最小 clearance 硬阈值 |
| `weight_lateral_jerk` | `0.15` | 横向 jerk 权重 |
| `weight_longitudinal_jerk` | `0.08` | 纵向 jerk 权重 |
| `weight_time` | `0.15` | 规划时长权重 |
| `weight_lateral_offset` | `0.6` | 终点横向偏移权重 |
| `weight_speed_error` | `0.7` | 终点速度误差权重 |
| `weight_obstacle_clearance` | `24.0` | clearance 不足惩罚权重 |
| `weight_curvature` | `0.4` | 最大曲率惩罚权重 |
| `weight_curvature_rate` | `8.0` | 曲率变化率惩罚权重 |
| `weight_lateral_shift` | `1.2` | 横向局部跳变惩罚权重 |

### 局部栅格和碰撞参数

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `grid_forward_m` | `10.0` | 局部栅格向车前覆盖距离 |
| `grid_rear_m` | `1.0` | 局部栅格向车后覆盖距离 |
| `grid_half_width_m` | `3.0` | 局部栅格左右半宽 |
| `grid_resolution_m` | `0.05` | 栅格分辨率 |
| `grid_inflation_radius_m` | `0.28` | 障碍膨胀半径 |
| `scan_offset_x_m` | `0.275` | 雷达到控制点/base_link 的前向偏移 |
| `corridor_radius_m` | `0.20` | 轨迹中心线左右扫掠半径 |
| `corridor_sample_step_m` | `0.10` | footprint/corridor 内采样间隔 |
| `path_collision_sample_step_m` | `0.05` | 沿路径加密采样最大间隔 |
| `footprint_front_m` | `0.38` | 控制点向车头扫掠长度 |
| `footprint_rear_m` | `0.05` | 控制点向车尾扫掠长度 |

### 状态机、复用和发布参数

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `reference_closed_loop` | `true` | 参考线是否闭环 |
| `projection_search_window_m` | `6.0` | `project_near` 局部投影窗口 |
| `debug_max_safe_candidates` | `12` | RViz 显示的最多安全候选 |
| `reuse_last_candidate_timeout_s` | `1.0` | 上一条候选复用时间窗口 |
| `held_path_replan_clearance_m` | `0.22` | 复用轨迹低于该 clearance 时重规划 |
| `held_path_min_remaining_m` | `2.0` | 复用轨迹最小剩余长度 |
| `held_path_max_lateral_error_m` | `0.45` | 复用轨迹允许最大横向误差 |
| `held_path_max_heading_error_rad` | `0.85` | 复用轨迹允许最大航向误差 |
| `candidate_profile_consistency_weight` | `20.0` | 候选通道连续性权重 |
| `candidate_profile_max_jump_m` | `0.35` | profile 最大跳变阈值 |
| `candidate_profile_lookahead_m` | `4.0` | profile 连续性比较前瞻距离 |
| `candidate_profile_unlock_clearance_gain_m` | `0.12` | raw best 多出多少 clearance 可打破连续性 |
| `candidate_channel_memory_timeout_s` | `1.5` | 通道记忆保留时间 |
| `stop_path_length_m` | `2.0` | stop path 长度 |
| `min_published_path_length_m` | `0.75` | 发布路径最小长度 |
| `published_path_lookahead_m` | `0.25` | 发布路径锚定前瞻 |
| `max_published_path_length_m` | `4.0` | 发布路径最大长度 |
| `min_path_publish_interval_s` | `0.25` | 路径最小重发布间隔 |
| `path_republish_distance_m` | `0.50` | 路径变化超过该距离才重发 |
| `path_republish_min_remaining_m` | `2.0` | 剩余路径低于该值允许重发 |

### 中心线威胁和激活参数

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `centerline_return_lookahead_m` | `5.0` | 中心线/回中路径长度 |
| `centerline_return_step_m` | `0.08` | 中心线采样间隔 |
| `centerline_return_direct_d_threshold_m` | `0.20` | 当前 `d` 超过该值时平滑回中 |
| `centerline_threat_corridor_radius_m` | `0.32` | 中心线威胁检测走廊半径 |
| `centerline_threat_lookahead_m` | `6.0` | 中心线威胁检测前瞻 |
| `cruise_speed_mps` | launch `target_speed`，默认 `1.0` | 激活距离计算用速度 |
| `activation_min_lookahead_m` | `3.0` | 激活距离下限 |
| `activation_max_lookahead_m` | `8.0` | 激活距离上限 |
| `activation_base_lookahead_m` | `2.2` | 激活基础距离 |
| `activation_reaction_time_s` | `1.0` | 反应时间项 |
| `activation_decel_mps2` | `2.0` | 制动距离项使用的减速度 |
| `activation_path_margin_m` | `0.75` | 发布路径长度外的激活余量 |
| `approach_slowdown_extra_m` | `-1.0` | 小于 0 时自动用制动距离额外量 |
| `centerline_speed_limit_mps` | `-1.0` | 中心线模式清除局部限速 |
| `avoidance_speed_limit_mps` | `0.75` | approach/avoidance 局部限速 |
| `stop_speed_limit_mps` | `0.0` | stop 模式局部限速 |

### LQR 控制参数

以下为 `pnc_sim_launch.py` 的默认启动参数：

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `path_topic` | Frenet 开启时 `/local_trajectory` | LQR 跟踪路径 |
| `path_closed_loop` | Frenet 开启时 `false` | 局部路径按开放路径处理 |
| `open_loop_endpoint_stop_max_path_length` | Frenet 开启时 `0.3` | 局部路径接近终点停车阈值 |
| `wheelbase` | `0.3302` | 控制模型轴距 |
| `target_speed` | `1.0` | LQR 基础目标速度 |
| `min_speed` | `0.4` | 曲率限速最低速度 |
| `max_steering_angle` | `0.36` | 转向角限幅 |
| `max_lateral_accel` | `4.0` | 曲率限速横向加速度上限 |
| `enable_curvature_speed_limit` | `true` | 是否按曲率限速 |
| `speed_limit_topic` | Frenet 开启时 `/local_trajectory_speed_limit` | 局部限速输入 |
| `speed_limit_timeout_s` | `1.0` | 局部限速超时 |
| `curvature_speed_lookahead_m` | `1.0` | 预览曲率限速距离 |
| `enable_speed_ramp` | `true` | 启用速度斜坡 |
| `max_accel` | `1.0` | 最大加速度 |
| `max_decel` | `2.0` | 最大减速度 |
| `enable_steering_rate_limit` | `true` | 启用转向速率限制 |
| `max_steering_rate` | `2.0` | 最大转向速率 |
| `lqr_lookahead_distance_m` | `0.0` | LQR 控制预瞄距离，launch 默认关闭 |
| `lqr_q_lateral` | `3.0` | LQR 横向误差权重 |
| `lqr_q_heading` | `1.2` | LQR 航向误差权重 |
| `lqr_r_steering` | `8.0` | LQR 转向输入权重 |
| `lqr_feedforward_gain` | `1.0` | 曲率前馈增益 |
| `control_dt` | `0.05` | 控制离散化周期 |
| `lqr_min_model_speed` | `0.25` | LQR 模型最低速度 |
| `enable_error_filter` | `false` | 默认不滤波跟踪误差 |
| `error_filter_alpha_y` | `0.30` | 横向误差滤波系数 |
| `error_filter_alpha_psi` | `0.25` | 航向误差滤波系数 |

## 哪些逻辑最容易让人觉得混乱

1. `plan_frenet_path()` 是纯候选生成和硬过滤，但 node 层又会做复用、连续性选择、裁剪和 stop fallback。看代码时要分清“规划核心”和“ROS 状态机”。

2. `safe_clearance_m` 和 `min_clearance_m` 不是一回事：
   - `min_clearance_m` 是硬过滤阈值。
   - `safe_clearance_m` 是 cost 惩罚阈值。

3. `corridor_radius_m` 和 `grid_inflation_radius_m` 是叠加保守性的两个来源：
   - `grid_inflation_radius_m` 先膨胀障碍。
   - `corridor_radius_m` 再扩大车辆路径扫掠宽度。

4. Frenet 的 `target_speed=1.5` 和 LQR launch 的 `target_speed=1.0` 不是同一个参数：
   - Frenet 的 `target_speed` 只影响候选 cost。
   - LQR 的 `target_speed` 是实际速度控制基准。
   - approach/avoidance 时 LQR 还会被 `avoidance_speed_limit_mps=0.75` 进一步限制。

5. `footprint_front_m/rear_m` 必须跟轨迹控制点一致。如果控制点是后轴中心，就不能按车体中心对称设置。

6. `enable_frenet_planner` launch 默认是 `false`。如果没有显式启用，LQR 不会使用这里的 Frenet 局部避障路径。

## 建议的理解顺序

建议按下面顺序读代码，避免被状态机绕进去：

1. `ReferencePath.sample()` 和 `ReferencePath.project_near()`：弄清 `(x,y)` 与 `(s,d)` 怎么互转。
2. `solve_quintic_lateral()` 和 `solve_quartic_longitudinal()`：对应 P8 多项式。
3. `plan_frenet_path()`：对应 P9 三层采样、硬过滤和 P22 cost。
4. `OccupancyGrid.query_path()`：理解当前碰撞检测为什么比手写稿复杂。
5. `FrenetStaticObstaclePlanner.plan_once()`：理解什么时候中心线、什么时候 approach、什么时候 Frenet、什么时候 stop。
6. `LqrController.control_once()` 和 `compute_lqr_steering()`：理解规划出的路径最后怎么变成转向和速度。

