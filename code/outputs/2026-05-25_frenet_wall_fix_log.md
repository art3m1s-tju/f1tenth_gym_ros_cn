# 2026-05-25 Frenet 撞墙问题修复记录

## 结论

本次撞墙不是单纯的 LQR 调参问题，主因在 Frenet 局部轨迹质量失控，随后被 LQR 放大。

最关键的两个症状来自已有运行日志 `code/outputs/logs/lqr_tracking_log.csv`：

- `curvature_ref` 出现 `-442.85 ~ 370.12 1/m` 的异常值。
- `e_psi` 最大接近 `178.4 deg`。

这说明 LQR 曾经收到过几何上接近反向、折返或尖角的局部路径，而不是“正常但略难跟踪”的局部路径。

## 发现的问题

### 1. Frenet 初始横向速度 `d_dot` 被重复计算

位置：[code/pnc_rc/frenet/planner.py](/home/art3m1s/f1tenth_frenet_static_avoidance/code/pnc_rc/frenet/planner.py:297)

旧实现先用法向投影计算一次 `d_dot`，随后又叠加了一次 `s_dot * sin(yaw - heading)`。这会把横向初始条件人为放大，使横向五次多项式更激进，更容易生成尖角轨迹。

### 2. Frenet 候选只有软惩罚，没有硬约束

位置：[code/pnc_rc/frenet/planner.py](/home/art3m1s/f1tenth_frenet_static_avoidance/code/pnc_rc/frenet/planner.py:318)

旧实现对以下风险都没有直接拒绝：

- 纵向采样点推进不足。
- 轨迹局部折返。
- 相邻轨迹段航向突变。
- 曲率虽然远超控制器能力，但只是在 cost 里加罚分。

结果是“代价高但仍可入选”的坏轨迹会继续流向 LQR。

### 3. LQR 每次收到新的局部路径都会重置跟踪内部状态

位置：[code/pnc_rc/lqr/controller.py](/home/art3m1s/f1tenth_frenet_static_avoidance/code/pnc_rc/lqr/controller.py:392)

Frenet 以 20Hz 连续发布滚动局部路径，但 LQR 在每次 `path_callback()` 中都会重置：

- `previous_delta_cmd`
- `filtered_lateral_error`
- `filtered_heading_error`

这会削弱转向速率限幅和误差滤波的连续性。只要局部路径本身稍有抖动，控制器就更容易出现突兀转向。

### 4. 缺少面向“可跟踪性”的单元测试

位置：[test/test_frenet_planner.py](/home/art3m1s/f1tenth_frenet_static_avoidance/test/test_frenet_planner.py:1)

旧测试覆盖了多项式边界条件，但没有覆盖：

- `d_dot` 是否被正确初始化。
- 候选路径是否存在明显折返。
- 纵向推进是否足够。

### 5. 默认仿真起始位姿与当前全局轨迹不一致

位置：[config/sim.yaml](/home/art3m1s/f1tenth_frenet_static_avoidance/config/sim.yaml:52)

验证时发现 `sim.yaml` 仍使用默认起点 `sx=0.0, sy=0.0, stheta=0.0`，但当前 `global_trajectory.csv` 的起点约为 `(-1.998, -0.925, 1.682)`。这会导致车辆一启动就不在参考轨迹附近，进而放大 Frenet 和 LQR 的异常表现。

### 6. `LaserScan` 中的零值回波被当成真实障碍

位置：[code/pnc_rc/frenet/planner.py](/home/art3m1s/f1tenth_frenet_static_avoidance/code/pnc_rc/frenet/planner.py:197)

后续在新设计的 Frenet 专用赛道上做离线对比时，发现了一个更直接的线上失效根因：

- 只用静态地图占用栅格时，Frenet 可以找到安全候选。
- 把同一帧扫描替换成全零 `scan.ranges` 后，Frenet 立即变成“全部碰撞”。

这说明问题不在候选轨迹本身，而在局部占用栅格构建：某些仿真帧里的 `0.0m` range 实际表示“无有效返回”，但旧实现把它们当成了贴近车体的真实障碍，直接把局部空间污染掉了。

## 代码修改

### Frenet 规划器

修改文件：

- [code/pnc_rc/frenet/planner.py](/home/art3m1s/f1tenth_frenet_static_avoidance/code/pnc_rc/frenet/planner.py:1)
- [code/pnc_rc/frenet/node.py](/home/art3m1s/f1tenth_frenet_static_avoidance/code/pnc_rc/frenet/node.py:1)

改动要点：

- 删除 `d_dot` 的重复叠加，只保留一次法向速度投影。
- 过滤 `0.0m` 和近似 `0.0m` 的无效 `LaserScan` range，避免把无回波误标为近距离障碍。
- 将候选轨迹曲率估计从“按等时间采样直接做数值微分”改成“先按最小空间间距压缩采样点，再按弧长求导”。这样静止起步时前几个几乎不动的点不会再把曲率数值放大。
- 为候选轨迹增加硬约束：
  - 相邻 `s` 采样点必须持续前进。
  - 单条候选轨迹总前向推进量默认不低于 `0.20m`。
  - 相邻轨迹段航向跳变默认不超过 `0.65rad`。
  - 最大曲率超过 `config.max_curvature` 直接剔除。
- 新增更细的拒绝统计：
  - `progress_reject`
  - `heading_reject`
  - `curvature_reject`
  - `collision_reject`
- 规划器启动后额外发布 `/frenet/debug/candidates`，用于 RViz 调试。
- 节点新增“短时复用上一条安全 Frenet 轨迹”的退化模式，避免单帧无解就直接掉成停车路径。

### LQR 控制器

修改文件：

- [code/pnc_rc/lqr/controller.py](/home/art3m1s/f1tenth_frenet_static_avoidance/code/pnc_rc/lqr/controller.py:392)

改动要点：

- 对闭环全局路径仍保持原有重置行为。
- 对 Frenet 滚动局部路径，不再在每次路径更新时重置转向记忆和误差滤波状态。

这样做的目的不是“掩盖上游问题”，而是让控制器对连续重规划的局部路径保持连续控制。

### Launch / RViz / 依赖

修改文件：

- [launch/pnc_sim_launch.py](/home/art3m1s/f1tenth_frenet_static_avoidance/launch/pnc_sim_launch.py:1)
- [launch/gym_bridge.rviz](/home/art3m1s/f1tenth_frenet_static_avoidance/launch/gym_bridge.rviz:1)
- [package.xml](/home/art3m1s/f1tenth_frenet_static_avoidance/package.xml:1)
- [docs/frenet_static_obstacle_avoidance.md](/home/art3m1s/f1tenth_frenet_static_avoidance/docs/frenet_static_obstacle_avoidance.md:1)

改动要点：

- Launch 新增两个 Frenet 约束参数：
  - `frenet_max_heading_jump`
  - `frenet_min_progress_step_m`
- Launch 新增 Frenet 纵向采样参数，与 LQR 巡航速度解耦：
  - `frenet_target_speed`
  - `frenet_v_min`
  - `frenet_v_max`
- Launch 新增 `frenet_reuse_last_candidate_timeout_s`，控制无解帧时复用上一条安全路径的最长时长。
- Launch 新增 `sx` / `sy` / `stheta` 覆盖参数，便于切换地图时直接对齐起始位姿。
- RViz 新增 `FrenetCandidates` MarkerArray 显示层，订阅 `/frenet/debug/candidates`。
- 补充 `visualization_msgs` 运行依赖。
- 文档新增 RViz 调试说明。
- 将 `config/sim.yaml` 的默认主车起始位姿修正到与当前全局轨迹起点一致。

## RViz 使用方式

启动：

```bash
ros2 launch f1tenth_gym_ros pnc_sim_launch.py enable_frenet_planner:=true
```

在 RViz 中重点看两层：

- `FrenetLocalTrajectory`：LQR 当前实际跟踪的 `/local_trajectory`
- `FrenetCandidates`：Frenet 当前认为安全的候选轨迹集合

判读建议：

- 如果 `FrenetCandidates` 本身已经贴墙或折返，问题在局部规划候选生成/筛选。
- 如果候选看起来合理，但 `FrenetLocalTrajectory` 频繁跳变，问题更偏向代价函数或重规划时序。
- 如果两者都平滑，但车体仍明显切墙，再回头查 LQR 或车辆动力学限制。

## 验证情况

本地完成了以下验证：

- 代码差异检查完成。
- `docs` / `launch` / `package.xml` 已同步更新。
- 使用项目镜像 `f1tenth_gym_ros:latest` 完成 `colcon build --symlink-install`。
- 容器内 `pytest /sim_ws/src/f1tenth_gym_ros/test/test_frenet_planner.py -q` 通过，结果为 `9 passed`。
- 容器内完成一次 `enable_frenet_planner:=true` 的 headless `ros2 launch` 启动验证，确认 Frenet 节点、LQR、map_server 和新增调试 topic 能正常启动。

后续又补了一条回归测试，专门保证零值 `LaserScan` 不会再次被当成障碍：

- [test/test_frenet_planner.py](/home/art3m1s/f1tenth_frenet_static_avoidance/test/test_frenet_planner.py:1) 新增 `test_occupancy_grid_ignores_zero_range_returns()`

### 启动验证中发现并修正的二次问题

第一次 headless `ros2 launch` 验证时，Frenet 规划器连续输出：

```text
No safe Frenet candidate; publishing stop path (total=42, safe=0, progress_reject=42, ...)
```

定位结果：

- 初版硬约束把 `min_progress_step_m` 实现成了“每个相邻采样点都至少前进固定距离”。
- 车辆静止起步时，纵向四次多项式前几帧推进量天然很小，导致全部候选被误杀。

修正后：

- 约束语义改成“整条候选轨迹的总前向推进量至少达到阈值”。
- 默认值同步调整为 `0.20m`。
- 增补了从静止起步但总推进量足够时应当放行的单测。

### 离线/在线不一致的进一步定位

在新赛道 `frenet_test_open_two_blocks` 上做离线复核时，得到的现象是：

- `empty_scan`：有安全候选，`best_clearance` 约 `2.43m`
- `zero_scan`：无安全候选，`best_clearance` 约 `0.03m`

这组对比非常关键，因为它把问题从“Frenet 是否会绕障”缩小成了“占用栅格是否把无效 scan 当成真障碍”。因此最终修复不是继续放宽 Frenet 代价，而是先修正无效传感器输入的过滤逻辑。

### 曲率拒绝的进一步定位

在确认零值 `scan` 已过滤后，又发现一个新的误判来源：

- 离线参考中心线最大曲率只有约 `0.27 1/m`。
- 但 Frenet 对静止起步候选直接按等时间采样点做曲率数值微分时，前几帧几乎不动，导致局部空间步长极小，曲率会被错误放大到 `3~10+ 1/m`。

修正后：

- 对候选轨迹先按最小空间间距压缩采样点；
- 再按弧长而不是采样序号估计曲率；
- 新增回归测试，保证近重复起步点不会再把曲率错误抬高。

离线复核结果：

- 在 `frenet_test_loop_open` 专用赛道上，修正后最低候选曲率约 `0.19 1/m`。
- `pytest` 更新后结果为 `12 passed`。

### Launch 层发现的第二个耦合问题

在线复核时还发现，旧 launch 会把 Frenet 的 `v_max` 直接绑定到 LQR 的 `target_speed`。当控制器巡航速度设为 `1.0m/s` 时，Frenet 纵向采样会被压得过短，不利于在起步阶段生成空间上足够平滑的候选。

修正后：

- Frenet 纵向采样参数独立成 `frenet_target_speed`、`frenet_v_min`、`frenet_v_max`；
- 默认值改为更适合局部规划的 `1.5 / 0.6 / 2.5`；
- 不再因为 LQR 巡航速度偏低而限制 Frenet 候选的几何可行性。

### Launch 覆盖链路里的实际 bug

后续继续查“为什么专用赛道验证现象和离线分析差这么多”时，发现还有一个更底层的问题：

- 命令行虽然已经传了
  `map_path:=.../frenet_test_open_two_blocks sx:=0.0 sy:=5.0 stheta:=3.1416`
- 但 `gym_bridge` 实际启动日志仍然显示：
  `map=/sim_ws/src/f1tenth_gym_ros/maps/my_map`
  `ego_start=(-1.998, -0.925, 1.682)`

也就是说，之前一部分“专用赛道在线验证”其实根本没有跑在专用赛道和指定起点上，而是悄悄退回到了 `sim.yaml` 默认配置。

修正方式：

- `pnc_sim_launch.py` 不再依赖 `Node(parameters=[sim_config, {...}])` 这种混合覆盖链路；
- 改成先生成一份带覆盖值的临时 bridge YAML，再只用这份 YAML 启动 `gym_bridge`；
- `gym_bridge.py` 同时补了启动日志，明确打印实际 `map` 和 `ego_start`，以后再出现参数未生效时能第一时间看出来。

本地验证存在两个环境限制：

1. `python3 -m compileall code test` 能跑到已修改文件，但工作区内现有 `__pycache__` 目录权限不足，导致 `.pyc` 回写失败。
2. 当前宿主环境缺少 `numpy`，无法直接在宿主机运行 `test/test_frenet_planner.py`。

因此，建议在容器内执行以下命令完成最终验证：

```bash
cd /sim_ws
colcon build --symlink-install
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
python3 -m pytest /sim_ws/src/f1tenth_gym_ros/test/test_frenet_planner.py -q
ros2 launch f1tenth_gym_ros pnc_sim_launch.py enable_frenet_planner:=true
```

### 当前在线验证状态

在修正 bridge 覆盖链路之后，基于 `frenet_test_open_two_blocks` 专用赛道重新做 headless `ros2 launch` 复核，现状比最初明显更好：

- `gym_bridge` 已明确打印：
  `map=/sim_ws/src/f1tenth_gym_ros/maps/generated_static_obstacles/frenet_test_open_two_blocks`
  `ego_start=(0.000, 5.000, 3.142)`
- Frenet 能长时间连续发布 `21` 点局部轨迹。
- 在默认专用赛道参数下，仍会偶发无解帧，但主要已经收缩为：
  - 短时 `reusing last safe path`
  - 少量 `heading_reject` 主导的无解
- 在更合理的验证参数下：
  - `frenet_d_min:=-1.8`
  - `frenet_d_max:=1.8`
  - `frenet_max_heading_jump:=0.85`
  - `frenet_grid_inflation_radius_m:=0.22`
  headless 日志未再出现明显的持续性“全量 stop path 抖动”。

这说明当前剩余问题已经不再是“launch 起点错了”或“长期没有 Frenet 解”，而是专用赛道上仍存在少量瞬时无解帧。通过复用上一条安全轨迹，这些瞬时无解目前已经不会直接演化成之前那种持续撞墙。

换句话说：

- 离线几何、零值 scan、RViz 可视化、launch 参数耦合、闭环投影串支路这些主问题已经修到位；
- 对专用 Frenet 验证赛道，当前建议直接使用上面的更宽横向采样范围和更宽航向跳变阈值作为默认验证参数。

如果你确认当前 review 图对应的赛道布局可以接受，建议把最终在线验证切到这组更干净的专用地图：

```bash
ros2 launch f1tenth_gym_ros pnc_sim_launch.py \
  enable_frenet_planner:=true \
  map_path:=/sim_ws/src/f1tenth_gym_ros/maps/generated_static_obstacles/frenet_test_open_two_blocks \
  track_csv:=/sim_ws/src/f1tenth_gym_ros/code/outputs/generated_tracks/frenet_test_loop_open_processed_track.csv \
  trajectory_mode:=centerline \
  sx:=0.0 sy:=5.0 stheta:=3.1416 \
  frenet_d_min:=-1.4 frenet_d_max:=1.4 \
  frenet_grid_inflation_radius_m:=0.22
```

## 后续观察点

如果修复后仍出现贴墙，优先观察以下现象：

- `/frenet/debug/candidates` 是否持续只剩一侧贴边解。
- `No safe Frenet candidate` 日志中的 `heading_reject` 或 `curvature_reject` 是否激增。
- `frenet_d_min` / `frenet_d_max` 是否仍不足以覆盖赛道可行宽度。
- 静态地图膨胀半径是否过于保守，导致可行空间被人为压缩。

## 2026-05-25 追加修复：动态可跟踪性与安全走廊

### 回滚点

在继续修改前，已把上一版 review 状态提交并推送到 GitHub：

```text
ac345dc Checkpoint Frenet wall fix review state
origin/stage/frenet-static-obstacle-avoidance
```

这版保留了 launch override、RViz 候选轨迹显示、专用测试地图和前一轮 Frenet 修复，后续如果需要可以直接回滚到该提交。

### 新发现

subagent review 和 headless 复核指出，剩余“撞墙/突然失效”的主因已经不是地图未切换，而是以下工程约束没有对齐：

- Frenet 原本允许 `max_curvature=1.8 1/m`，但 LQR `max_steering_angle=0.36rad` 和 `wheelbase=0.3302m` 对应车辆最大可跟踪曲率约 `1.14 1/m`。
- 原碰撞检测只检查候选中心线，没有覆盖车身宽度和 LQR 跟踪误差。
- 复用上一条 safe candidate 时只检查 age，没有用当前 occupancy 重新验证。
- LQR 把 Frenet 滚动局部路径当成普通 open path，接近局部 horizon 末端时会周期性触发 endpoint stop。
- `speed=1.00` 但 `s_dot=0.00` 的日志说明 Frenet 仍可能使用语义不明确的 odom twist，导致初始纵向速度错误。

### 修改

- `FrenetPlannerConfig.max_curvature` 默认改为 `1.1 1/m`，并通过 launch 参数 `frenet_max_curvature` 暴露，默认低于车辆转角极限对应的 `1.14 1/m`。
- Frenet 预测时间从固定 `2.0s` 扩展为 `2.0s~3.0s`，launch 新增 `frenet_t_min/t_max/t_step`，让绕障候选更平滑。
- `OccupancyGrid.query_path()` 支持沿候选轨迹生成横向 swept corridor，默认 `frenet_corridor_radius_m=0.16`，不再只检查中心点。
- 新增 hard clearance reject：`frenet_min_clearance_m=0.05`，日志新增 `clearance_reject`。
- 复用上一条 safe candidate 前，使用当前 occupancy 和 corridor 重新检查 collision/clearance。
- Frenet reference 支持 `reference_closed_loop`，launch 暴露为 `frenet_reference_closed_loop`。专用 open-style 测试赛道使用 `false`。
- Frenet node 优先用连续 odom 位姿差分估计 map-frame 速度，stamp 不可用或重复时保留最近有效差分速度，减少 body-frame twist 污染 `s_dot/d_dot`。
- LQR 新增 `open_loop_endpoint_stop_max_path_length`。Frenet 模式下普通滚动局部路径不再触发 endpoint stop，只有很短的 emergency stop path 才会触发停车。
- LQR 到 open-loop endpoint 时刹车但保持最近转角，不再强制把 steering 清零。

### 验证

容器内单元测试：

```text
17 passed in 0.21s
```

新增测试覆盖：

- swept corridor 会横向展开路径；
- corridor 能检测中心点不会撞、但车身宽度会撞的场景；
- hard clearance reject 会拒绝 clearance 不足的候选；
- open reference 不会在终点 wrap 到起点。

headless launch 验证命令：

```bash
ros2 launch f1tenth_gym_ros pnc_sim_launch.py \
  enable_rviz:=false enable_frenet_planner:=true \
  map_path:=/sim_ws/src/f1tenth_gym_ros/maps/generated_static_obstacles/frenet_test_open_two_blocks \
  track_csv:=/sim_ws/src/f1tenth_gym_ros/code/outputs/generated_tracks/frenet_test_loop_open_processed_track.csv \
  trajectory_mode:=centerline sx:=0.0 sy:=5.0 stheta:=3.1416 \
  frenet_reference_closed_loop:=false \
  frenet_d_min:=-1.8 frenet_d_max:=1.8 \
  frenet_max_heading_jump:=0.85 \
  frenet_grid_inflation_radius_m:=0.22 \
  frenet_max_curvature:=1.1 \
  frenet_corridor_radius_m:=0.16 \
  frenet_min_clearance_m:=0.05
```

结果：

- `gym_bridge` 打印正确地图和起点：
  `map=/sim_ws/src/f1tenth_gym_ros/maps/generated_static_obstacles/frenet_test_open_two_blocks`
  `ego_start=(0.000, 5.000, 3.142)`
- 25 秒 headless 窗口内，Frenet 持续发布 `31` 点局部轨迹。
- 未再出现 `No safe Frenet candidate`。
- 普通滚动局部路径未再触发 `LQR open-loop endpoint reached; stopping`。

当前结论：

- 地图 override、Frenet 单元逻辑、动态可跟踪性、车身安全走廊、复用路径校验和 rolling local path 的 LQR endpoint 语义已经修到位。
- 这不等价于所有可能地图都已经实车级验证；后续如果换回更窄/更复杂赛道，需要重新看 `clearance_reject/collision_reject/curvature_reject` 的占比。
