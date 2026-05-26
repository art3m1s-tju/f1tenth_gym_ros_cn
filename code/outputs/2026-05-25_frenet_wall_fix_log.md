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
- `OccupancyGrid.query_path()` 支持沿候选轨迹生成 footprint swept corridor，不再只检查 `base_link` 中心点。
- `frenet_corridor_radius_m` 调整为 `0.14`，并新增 `frenet_footprint_front_m=0.38` / `frenet_footprint_rear_m=0.05`，覆盖车体从 `base_link` 向车头的前向长度。
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

- swept corridor 会同时展开横向车宽和前向 footprint；
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
  frenet_corridor_radius_m:=0.14 \
  frenet_footprint_front_m:=0.38 \
  frenet_footprint_rear_m:=0.05 \
  frenet_safe_clearance_m:=0.35 \
  frenet_min_clearance_m:=0.05
```

结果：

- `gym_bridge` 打印正确地图和起点：
  `map=/sim_ws/src/f1tenth_gym_ros/maps/generated_static_obstacles/frenet_test_open_two_blocks`
  `ego_start=(0.000, 5.000, 3.142)`
- 25 秒 headless 窗口内，Frenet 持续发布局部轨迹。
- 未再出现 `No safe Frenet candidate`。
- 普通滚动局部路径未再触发 `LQR open-loop endpoint reached; stopping`。
- 针对用户反馈“第一个障碍物擦撞”，追加 footprint corridor 后复核 tracking log：在第一障碍物 x 范围内，车辆中心 `y=6.29~6.42`，障碍物中心 `y=4.91`，不再是之前 `y≈5.14` 的擦边绕行。

当前结论：

- 地图 override、Frenet 单元逻辑、动态可跟踪性、车身安全走廊、复用路径校验和 rolling local path 的 LQR endpoint 语义已经修到位。
- 这不等价于所有可能地图都已经实车级验证；后续如果换回更窄/更复杂赛道，需要重新看 `clearance_reject/collision_reject/curvature_reject` 的占比。

## 2026-05-26 复核：障碍膨胀语义

用户指出：真实赛场不应依赖预先知道的障碍物中心点，而应对雷达在线检测到的障碍物边缘/占据栅格做膨胀。

复核结论：

- 这个判断是正确的；实车/真实赛场语义应该是“占据栅格膨胀”，不是“障碍物中心点膨胀”。
- 当前 Frenet planner 的实现实际也是对 raw occupied cells 膨胀，而不是读取 `.json` 中的障碍物中心点参与规划。
- `.json` 里的 `center_x_m/center_y_m` 只用于复现实验和离线分析障碍物位置；在线规划代码没有用这些中心点构造障碍物。
- `build_occupancy_grid()` 会先把 `LaserScan` 有效测距端点投到局部栅格，形成 raw occupied cells；如果静态地图可用，也会把局部 static occupied cells 合并进去。
- 随后通过 `ndimage.distance_transform_edt(~raw)` 计算到 raw occupied cells 的距离，并用 `distance_to_raw <= inflation_radius_m` 生成 inflated occupied grid。

因此，当前碰撞风险的主要问题不是“错误地对中心点膨胀”，而是：

- 对 LaserScan 端点/静态地图占据格子的膨胀半径偏小；
- hard clearance margin 偏小；
- 轨迹 collision check 只检查离散轨迹点展开后的 footprint/corridor，没有先沿轨迹段做连续空间加密；
- 当前日志没有 signed longitudinal velocity，无法直接用 CSV 判断用户在 RViz 看到的反弹/速度变号。

下一步修复仍应围绕连续 swept collision check、采样密度、硬安全裕度和 signed velocity 日志展开，而不是改成基于障碍物中心的逻辑。

## 2026-05-26 修复：连续碰撞检测、诊断日志和保守测试 preset

针对用户在 RViz 中看到“第一障碍物反弹/速度变号”的反馈，本轮按 review 结果继续修改。

### 代码修改

- `OccupancyGrid.query_path()` 在 footprint/corridor 展开前新增轨迹段空间加密，默认 `path_collision_sample_step_m=0.05m`。
- 新增 `densify_path_points()`，避免只检查离散轨迹点导致“两个点之间穿过障碍物但未命中栅格”的漏检。
- Frenet node 新增参数 `path_collision_sample_step_m`，并在候选路径检查和 last safe path 复查时同时使用。
- Launch 新增可配置参数：
  - `frenet_v_step`
  - `frenet_d_step`
  - `frenet_trajectory_dt`
  - `frenet_corridor_sample_step_m`
  - `frenet_path_collision_sample_step_m`
  - `frenet_grid_forward_m`
  - `frenet_grid_rear_m`
  - `frenet_grid_half_width_m`
- LQR tracking CSV 新增：
  - `v_longitudinal_signed`
  - `v_path_signed`
- signed velocity 不再直接相信 gym odom twist 的坐标语义，而是优先用连续 odom 位姿差分投影到车体前向和路径切向。
- `gym_bridge` 增加碰撞诊断：如果 gym obs 暴露 `collisions` 字段，首次 ego collision 会打印 `Ego collision detected at (...)`。
- `sim_harness.py` 修复 `segment_lengths` 未定义问题，`TrajectoryCache` 现在保存闭环路径段长度，离线 LQR harness 的 lookahead 投影可正常引用。
- `run_frenet_test.sh` 调整为更保守的一键验证 preset：
  - LQR `target_speed=0.55m/s`
  - LQR `max_lateral_accel=1.0`
  - Frenet 几何 horizon 与跟踪速度解耦，使用 `frenet_t_min=4.0`、`frenet_t_max=6.0`、`frenet_target_speed=1.2`
  - `frenet_trajectory_dt=0.05`
  - `frenet_path_collision_sample_step_m=0.03`
  - `frenet_grid_inflation_radius_m=0.45`
  - `frenet_corridor_radius_m=0.25`
  - `frenet_safe_clearance_m=0.60`
  - `frenet_min_clearance_m=0.15`
  - `frenet_reuse_last_candidate_timeout_s=0.0`

### 新增测试

- `test_densify_path_limits_segment_spacing`
- `test_occupancy_grid_detects_obstacle_between_sparse_path_points`

第二个测试明确覆盖之前的核心漏检：障碍物位于两个稀疏轨迹点之间时，关闭 path densify 会漏检，开启 path densify 必须检测到 collision。

### 验证过程

容器内单元测试：

```text
21 passed in 0.21s
```

同时通过：

```text
python3 -m py_compile code/lqr_sweep/sim_harness.py
```

第一次保守低速 preset 验证失败，输出：

```text
Ego collision detected at (-5.376, 5.316).
```

原因分析：

- LQR 速度降到了 `0.55m/s`，但 Frenet 几何采样也随之偏短；
- 低速 `frenet_target_speed=0.7`、`t_max=5.0` 只覆盖约 `3.5m`；
- 第一障碍物约在起点前方 `6m`，因此规划器太晚才把第一个障碍物纳入候选路径碰撞评估。

修正后将 Frenet 几何 horizon 与 LQR 跟踪速度解耦：

- LQR 继续慢速跟踪；
- Frenet 使用更长、更快的纵向采样生成远距绕障几何。

最终 headless 验证：

```text
21 passed in 0.23s
```

launch 输出未再出现 `Ego collision detected`。

本次 `code/outputs/logs/lqr_tracking_log.csv` 复核结果：

```text
rows: 1960
min_dist_to_first_obstacle_center: 0.472m
negative_longitudinal_count: 0
negative_path_count: 0
max_abs_curvature_ref: 1.247
max_abs_curvature_preview: 1.524
```

当前结论：

- 第一障碍物反弹问题在当前 headless preset 下已消失；
- CSV 已能直接检查 signed velocity 是否变负；
- `gym_bridge` 已能在真实 gym collision 时打印 warning；
- 仍建议后续继续降低局部轨迹曲率尖峰，当前 LQR 侧看到的最大曲率仍略高于 Frenet 的 `1.1 1/m` 目标，虽然本次未触发碰撞。

## 2026-05-26: 全局巡航速度与 Frenet 避障局部限速解耦

### 问题

`target_speed` 应表示 LQR 的全局巡航速度。此前为了让避障慢下来，只能把 `target_speed` 整体降到 `0.75m/s` 或更低，导致无障碍中心线段也跑不快。更合理的语义是：

- centerline / global tracking：按 `target_speed` 巡航；
- avoidance / hold：Frenet 发布局部限速；
- stop path：Frenet 发布 `0.0m/s` 限速。

### 修正

- Frenet 新增 `/local_trajectory_speed_limit` 发布，使用 `std_msgs/Float32`：
  - `centerline_speed_limit_mps < 0` 表示清除局部限速；
  - `avoidance_speed_limit_mps` 表示绕障段限速；
  - `stop_speed_limit_mps=0.0` 表示 emergency stop。
- LQR 新增局部限速订阅，速度命令变为：

```text
v_cmd = min(target_speed, curvature_speed_limit, local_speed_limit)
```

- 局部限速独立于路径发布 debounce：即使 `/local_trajectory` 不重发，Frenet 也会持续刷新当前模式限速，避免 LQR 在避障路径中途恢复巡航速度。
- Frenet 规划种子过滤异常 `s_ddot`：当 odom/projection 差分给出不可信大加速度时，将纵向初始加速度置零，避免 quartic 候选在起点处过快冲入障碍物。

### 当前一键测试 preset

- LQR `target_speed=0.85m/s`
- Frenet avoidance local speed limit `0.65m/s`
- Frenet reference `closed_loop=true`
- speed-test 默认 `90s`，用于覆盖超过一圈的运行。

### 验证结果

容器内单元测试：

```text
26 passed in 0.25s
```

90 秒 headless speed-test：

```text
stop/no-safe/open-loop endpoint count: 0
centerline_return_count: 40
frenet_plan_count: 12
duration: 83.64s
distance: 62.33m
v_actual mean/p50/p90/max: 0.728 / 0.650 / 0.850 / 0.850 m/s
v_cmd mean/p50/p90/max: 0.733 / 0.650 / 0.850 / 0.850 m/s
negative_v_path_count: 0
near_zero_v_cmd_count: 13
```

结论：当前分支下，第一圈和进入第二圈后均未出现 stop path 或速度反向；避障段按局部限速慢行，非避障段恢复全局巡航速度。

## 2026-05-26: 提高 Frenet 避障段局部限速

### 调整

- `run_frenet_test.sh` 新增 `--avoidance-speed` / `--avoidancespeed` 参数；
- 新增环境变量入口 `FRENET_AVOIDANCE_SPEED`；
- 默认避障局部限速从 `0.65m/s` 提高到 `0.75m/s`；
- launch 默认 `frenet_avoidance_speed_limit_mps` 同步为 `0.75`。

### 验证结果

使用新默认值运行：

```text
./run_frenet_test.sh --speed-test
```

结果：

```text
stop/no-safe/open-loop endpoint count: 0
ego_collision_count: 0
centerline_return_count: 41
duration: 83.62s
distance: 66.61m
v_actual mean/p50/p90/max: 0.784 / 0.750 / 0.850 / 0.850 m/s
v_cmd mean/p50/p90/max: 0.790 / 0.750 / 0.850 / 0.850 m/s
local_speed_limit: 0.75 m/s
negative_v_path_count: 0
```

结论：`0.75m/s` 作为当前测试地图的默认避障限速比 `0.65m/s` 更合适，速度提升明显，未复现停车、碰撞或反向速度。

## 2026-05-26: 按巡航速度动态决定 Frenet 接管距离

### 问题

原逻辑中 `frenet_centerline_threat_lookahead_m=8.0` 同时承担两个语义：

- 前方多远开始认为 centerline 有 obstacle threat；
- 前方多远开始切到 Frenet avoidance 并发布局部限速。

这导致低速下也会在障碍物很远时提前降速，表现为“还没靠近障碍就进入 Frenet / 跟线变差”。

### 修正

保留远距离 centerline threat 感知，但新增速度相关的真正接管距离：

```text
activation = base + v * reaction_time + v^2 / (2 * decel)
activation = clamp(activation, min_lookahead, max_lookahead)
```

当前默认参数：

```text
base = 2.2m
reaction_time = 1.0s
decel = 2.0m/s^2
min_lookahead = 3.0m
max_lookahead = 8.0m
```

对应接管距离大致为：

```text
0.5m/s -> 3.00m
0.85m/s -> 3.23m
1.5m/s -> 4.26m
2.0m/s -> 5.20m
3.0m/s -> 7.45m
```

也就是说：

- 低速只做近距离接管，不再 8m 外就降速；
- 高速会自然提前切 Frenet，因为需要更多反应和制动距离；
- `centerline_threat_lookahead_m` 仍用于远距离感知，但 threat 距离大于 activation 时保持 centerline 并清除局部限速。

### 验证结果

使用默认 `target_speed=0.85m/s`、`avoidance_speed=0.75m/s`：

```text
./run_frenet_test.sh --speed-test
```

结果：

```text
stop/no-safe/open-loop endpoint count: 0
ego_collision_count: 0
far_threat_keep_centerline_count: 16
duration: 83.50s
distance: 68.10m
v_actual mean/p50/p90/max: 0.802 / 0.850 / 0.850 / 0.850 m/s
v_cmd mean/p50/p90/max: 0.809 / 0.850 / 0.850 / 0.850 m/s
negative_v_path_count: 0
```

日志中可见：

```text
threat_distance=5.20m, activation_lookahead=3.23m, cruise_speed=0.85m/s -> keep centerline
threat_distance=3.28m, activation_lookahead=3.23m, cruise_speed=0.85m/s -> still keep centerline
```

结论：低速场景下不再远距离提前降速，同时当前测试地图仍能稳定避障并覆盖超过一圈。

## 2026-05-26: 切换到带赛道边界的静态障碍验证地图

### 问题

此前一键脚本默认使用：

```text
maps/generated_static_obstacles/frenet_test_open_two_blocks.pgm
```

该地图除两个方块障碍外全是 free space：

```text
frenet_test_open_two_blocks.pgm: occupied=128, free=153088
```

这只能验证自由空间静态绕障，不能证明候选轨迹会避开赛道内外边界。

### 修正

- `run_frenet_test.sh` 默认改用带内外边界的地图：

```text
maps/generated_static_obstacles/frenet_test_two_blocks.pgm
code/outputs/generated_tracks/frenet_test_loop_processed_track.csv
```

- 保留旧自由空间测试入口：

```text
./run_frenet_test.sh --open-map
```

- 默认 bounded 入口：

```text
./run_frenet_test.sh --bounded-map
```

### 边界约束来源

Frenet planner 已经订阅 `/map`，并在每次规划时把局部静态地图 occupied cells 合并进局部 occupancy grid。切换到 bounded map 后，内外赛道边界自然成为碰撞约束，不需要把边界单独硬编码成中心点或多边形。

本次 bounded map 加载日志：

```text
Loaded static occupancy map (504x304, resolution=0.050m, occupied_cells=87524)
```

对比 open map 只有两个方块障碍：

```text
frenet_test_open_two_blocks.pgm: occupied=128
frenet_test_two_blocks.pgm: occupied=87524
```

### 新增单测

新增：

```text
test_static_map_boundaries_constrain_frenet_candidates
```

该测试构造一个带上下边界的局部静态地图，证明：

- 赛道中心路径不 collision；
- 越过边界的路径会 collision；
- `local_static_map_occupancy + build_occupancy_grid + query_path` 链路能把静态地图边界作为 Frenet 碰撞约束。

### bounded headless 验证

运行：

```text
./run_frenet_test.sh --speed-test
```

当前默认参数：

```text
map: bounded (frenet_test_two_blocks)
target_speed: 0.85m/s
avoidance_speed: 0.75m/s
```

结果：

```text
stop/no-safe/open-loop endpoint count: 0
ego_collision_count: 0
far_threat_keep_centerline_count: 16
duration: 83.71s
distance: 68.27m
v_actual mean/p50/p90/max: 0.804 / 0.850 / 0.850 / 0.850 m/s
v_cmd mean/p50/p90/max: 0.810 / 0.850 / 0.850 / 0.850 m/s
negative_v_path_count: 0
```

Frenet 规划日志中出现大量边界/障碍 collision reject，但仍能找到安全候选：

```text
Frenet planning cycle elapsed=0.318s, total=78, safe=10, collision_reject=57, clearance_reject=11
Frenet planning cycle elapsed=0.389s, total=78, safe=17, collision_reject=49, clearance_reject=12
```

结论：当前 bounded 测试已经覆盖“静态障碍 + 赛道内外边界”约束，默认 preset 下可完成超过一圈且无碰撞、无 stop path、无速度反向。

## 2026-05-26: 修复高速场景候选束消失和 obstacle 前 emergency stop

### 问题

RViz 中出现“靠近障碍物时看不到 Frenet 候选轨迹束”，同时车辆在障碍物前停顿后再继续走。复核 `lqr_tracking_log.csv` 后确认：

```text
local_speed_limit=0.0
v_cmd≈0 持续约 1.5s
```

这说明不是普通降速，而是 Frenet 发布了 stop path。

### 根因

1. `Holding safe Frenet path` 和 no-candidate fallback 分支会调用：

```text
publish_debug_markers(stamp, [], None)
```

这会向 `/frenet/debug/candidates` 发送 `DELETEALL`，把上一轮候选束清空。因此候选轨迹在 RViz 里只短暂闪现，随后被 hold path 逻辑删除。

2. 高速测试中脚本的 LQR `target_speed=3.0m/s`，但 Frenet 几何采样仍固定为：

```text
frenet_target_speed=1.2
frenet_v_max=1.8
frenet_grid_forward_m=10.0
```

LQR 和 Frenet 几何尺度不一致。

3. Frenet node 使用 odom 位姿差分估计速度。高负载/慢规划时该差分会被异常放大，日志中出现：

```text
s_dot≈16m/s, speed≈16.7m/s
```

但 LQR CSV 中实际 `v_actual max=3.0m/s`。错误初始速度污染候选生成，导致后续周期全候选 collision reject 并触发 stop path。

### 修正

- Frenet debug marker 增加上一轮候选缓存：
  - fresh planning 成功时缓存候选束和 best candidate；
  - hold path / no-candidate fallback 时保留上一轮候选束，不再清空 RViz marker；
  - centerline clear 时才清空候选束。
- `run_frenet_test.sh` 根据 `--target-speed` 自动设置 Frenet 几何采样：

```text
target=3.0, avoidance=1.2
=> frenet_geom: target=3.000 v=[0.840, 3.750] grid_forward=20.000m
```

- Frenet odom 速度估计增加异常过滤：
  - 当位姿差分速度超过 `max(3.0, 1.5 * cruise_speed + 0.75)` 时，优先回退到 odom twist；
  - 如果 twist 也异常，则按上限裁剪；
  - 过滤时打印 warning 便于诊断。

### 高速 bounded 验证

运行：

```text
./run_frenet_test.sh --speed-test --target-speed 3.0 --avoidance-speed 1.2
```

结果：

```text
stop/no-safe/open-loop endpoint count: 0
ego_collision_count: 0
filtered_implausible_frenet_speed_count: 5
duration: 83.40s
distance: 163.47m
v_actual mean/p50/p90/max: 1.836 / 1.224 / 2.991 / 3.000 m/s
v_cmd mean/p50/p90/max: 1.865 / 1.275 / 3.000 / 3.000 m/s
local_speed_limit min/max: 1.2 / 1.2 m/s
zero_limit_rows: 0
negative_v_path_count: 0
```

结论：`target_speed=3.0m/s`、`avoidance_speed=1.2m/s` 的 bounded 场景下，不再触发 stop path；候选束不再被 hold/no-candidate 分支清空；Frenet 速度估计异常被过滤。
