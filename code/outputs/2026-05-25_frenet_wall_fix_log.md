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

## 2026-05-26 修复：Frenet 阶段卡住和第一障碍前停顿

用户继续反馈：Frenet 阶段走着走着会卡住，第一圈到第一个障碍物前经常停住。复核后确认这不是单纯 `publish_rate_hz` 设置太低，而是以下链路叠加：

- 高速下进入 Frenet 前仍按全局 `target_speed=3.0m/s` 行驶，只有 Frenet 正式发布 avoidance path 后才下发 `1.2m/s` 局部限速，预减速太晚。
- 单次规划耗时在高负载时可达 `0.5s+`，车在规划期间继续前进。
- `_publish_held_path_if_safe()` 原先没有年龄限制，只要旧轨迹仍判定安全就会一直 hold，不会重新规划。
- 第一版 hold 修复过于激进，低 clearance 时频繁丢弃仍可用的 held path，反而制造 no-candidate 窗口。
- 一旦 no-candidate fallback 发布 stop path，LQR 会收到 3 点短路径和 `local_speed_limit=0.0`，出现 RViz 里看到的“障碍物前停一下/卡住”。

### 代码修改

- Frenet node 新增 approach 预减速模式：
  - 中心线前方有障碍但尚未进入 Frenet activation 时，如果威胁距离进入 slowdown lookahead，就继续发布中心线路径但使用 `avoidance_speed_limit_mps` 限速；
  - 日志打印 `pre-slowing on centerline`，明确区分“保持中心线”和“预减速中心线”。
- Frenet node 新增 held path 重规划约束：
  - `max_held_path_age_s`
  - `held_path_replan_clearance_m`
  - `held_path_min_remaining_m`
- Launch 暴露新增参数：
  - `frenet_approach_slowdown_extra_m`
  - `frenet_max_held_path_age_s`
  - `frenet_held_path_replan_clearance_m`
  - `frenet_held_path_min_remaining_m`
- `run_frenet_test.sh` 默认起点从 `(0.0, 5.0)` 后移到 `(3.0, 5.0)`，让高速测试到第一个障碍前有更合理的加速/减速距离。
- `run_frenet_test.sh` 新增 `--sx/--sy/--stheta` 起点入口。
- `run_frenet_test.sh` 高速 preset 调整：
  - `frenet_trajectory_dt=0.10`
  - `frenet_max_held_path_age_s=0.80`
  - `frenet_held_path_replan_clearance_m=0.10`
  - `frenet_reuse_last_candidate_timeout_s=2.0`
  - `frenet_activation_max_lookahead_m` 随 `target_speed` 增大
  - `frenet_activation_reaction_time_s=1.2`
  - `frenet_approach_slowdown_extra_m=max(1.0, target_speed^2 / 4.0)`

### 验证

运行：

```text
./run_frenet_test.sh --speed-test --target-speed 3.0 --avoidance-speed 1.2
```

结果日志：`/tmp/frenet_stall_fix2.log`

```text
pytest: 28 passed
No safe Frenet candidate count: 0
stop path / local_speed_limit=0 count: 0
ego collision count: 0
negative v_path count: 0
low-speed stuck runs: 0
planning elapsed: about 0.27-0.50s in avoidance cycles
```

CSV 摘要：

```text
rows: 8700
v_actual: 0.0 .. 3.0 m/s
v_cmd: 0.0 .. 3.0 m/s
local_speed_limit zero runs: 0
low speed stuck runs: 0
negative_v_path: 0
```

结论：这次“卡住”的直接原因是 stop path/0 限速链路，不是 RViz 假象。修复后高速 bounded 测试中没有再触发 no-candidate stop，也没有第一障碍前长时间停住。

## 2026-05-26 修复：候选轨迹左右跳变导致蛇形

用户在 RViz 中观察到：小车在 Frenet 候选轨迹之间反复左右切换，导致实际跟踪路径出现蛇形。

### 根因

- `plan_frenet_path()` 每一帧只按当前帧 cost 排序候选轨迹。
- 左右/相邻横向 offset 的候选在障碍附近代价经常很接近。
- LaserScan、栅格膨胀和车辆位姿轻微变化会让当前帧 best candidate 在相邻 offset 甚至左右两侧之间跳变。
- LQR 每次追新的局部轨迹，轨迹端点横向跳变就表现为蛇形走位。

### 修正

- Frenet node 增加候选选择滞回：
  - 记录上一条已发布 Frenet candidate 的 `d[-1]`；
  - fresh planning 后不直接使用 raw best candidate；
  - 在所有 safe candidates 中重新打分，加入横向 endpoint 连续性惩罚；
  - 如果上一帧已经明显在某一侧，切到另一侧会额外增加 side-switch penalty；
  - 只有另一侧候选的原始代价优势足够大，或者同侧候选不可用时，才允许切边。
- 新增可调参数：
  - `candidate_lateral_consistency_weight`
  - `candidate_side_switch_penalty`
  - `candidate_side_deadband_m`
- Launch 暴露对应参数：
  - `frenet_candidate_lateral_consistency_weight`
  - `frenet_candidate_side_switch_penalty`
  - `frenet_candidate_side_deadband_m`
- `run_frenet_test.sh` 高速 preset 默认使用更强的抗蛇形滞回：
  - `FRENET_CANDIDATE_CONSISTENCY_WEIGHT=10.0`
  - `FRENET_CANDIDATE_SIDE_SWITCH_PENALTY=35.0`
  - `FRENET_CANDIDATE_SIDE_DEADBAND=0.20`

### 验证

运行：

```text
./run_frenet_test.sh --speed-test --target-speed 3.0 --avoidance-speed 1.2
```

结果日志：`/tmp/frenet_antizigzag.log`

```text
pytest: 28 passed
No safe Frenet candidate count: 0
stop path / local_speed_limit=0 count: 0
ego collision count: 0
negative v_path count: 0
low-speed stuck runs: 0
```

日志中可见候选滞回生效，例如：

```text
Selecting temporally consistent Frenet candidate
(raw_d=-0.75, selected_d=-0.05, previous_d=-0.05, ...)

Selecting temporally consistent Frenet candidate
(raw_d=-0.40, selected_d=-1.10, previous_d=-1.10, ...)
```

结论：这版不会改变碰撞判定，只改变“多个 safe candidate 之间如何选”。它通过时间连续性和同侧保持抑制候选轨迹在相邻横向 offset/左右两侧之间抖动，从源头减少 LQR 蛇形跟踪。

## 2026-05-26 新增：随机障碍鲁棒性批量测试

为评估 Frenet 避障算法对随机障碍位置的鲁棒性，新增批量测试入口：

```text
./run_frenet_random_obstacle_robustness.sh
```

默认测试定义：

```text
trials: 10
laps_per_trial: 3
target_speed: 3.0m/s
avoidance_speed: 1.2m/s
obstacle_count: 2
obstacle_size: 0.4m
seed_start: 0
timeout_per_trial: 240s
```

### 实现

- 新增 `code/lqr_sweep/frenet_random_robustness.py`：
  - 对每个 seed 调用 `generate_static_obstacle_test_map.write_obstacle_map()` 生成随机障碍地图；
  - 使用同一套高速 Frenet preset 运行 headless ROS 仿真；
  - 通过车辆 `(x,y)` 投影到全局中心线累计圈数，避免 Frenet 模式下 LQR `closest_idx` 指向局部轨迹导致圈数统计失效；
  - 解析 launch log 中的 `Ego collision detected`；
  - 解析 tracking CSV 中的负向速度、长时间 0 限速、低速卡死；
  - 每轮输出 `tracking.csv`、`launch.log`、地图 `.yaml/.pgm/.json`；
  - 汇总 `summary.csv` 和 `summary.json`，给出成功率。
- 新增 `run_frenet_random_obstacle_robustness.sh`：
  - 在 Docker 内构建 workspace；
  - 先运行相关 pytest；
  - 再执行批量鲁棒性测试；
  - 支持 `--trials`、`--laps`、`--timeout`、`--target-speed`、`--avoidance-speed`、`--obstacle-count`、`--seed-start` 等入口。
- 新增 `test/test_frenet_random_robustness.py`：
  - 覆盖高速 preset 计算；
  - 覆盖基于全局位置投影的圈数统计；
  - 覆盖 launch/tracking 日志摘要。

### 验证

单元测试：

```text
python3 -m pytest test/test_frenet_planner.py test/test_frenet_random_robustness.py -q
33 passed
```

Docker smoke：

```text
./run_frenet_random_obstacle_robustness.sh --trials 1 --laps 1 --timeout 100 --batch-name smoke_tmp2
```

结果：

```text
Trial seed_000: PASS
completed_laps: 1/1
success_rate: 1.000
```

正式 10 seed / 3 laps 测试命令：

```text
./run_frenet_random_obstacle_robustness.sh --trials 10 --laps 3 --timeout 240 --target-speed 3.0 --avoidance-speed 1.2
```

## 2026-05-26 更新：随机障碍 RViz 单 seed 复现模式

随机障碍鲁棒性脚本新增可视化入口，用于把批量测试中的某个 seed 直接放到 RViz 中观察候选轨迹、最优轨迹和车辆行为：

```text
./run_frenet_random_obstacle_robustness.sh --rviz --seed 0 --target-speed 3.0 --avoidance-speed 1.2
```

实现要点：

- `run_frenet_random_obstacle_robustness.sh` 新增 `--rviz` 和 `--seed` 参数；
- RViz 模式自动挂载 X11：`DISPLAY`、`QT_X11_NO_MITSHM`、`/tmp/.X11-unix`；
- RViz 模式只生成并启动一个 seed，不跑批量评分，也跳过 pytest 以减少等待；
- Python runner 新增 `--mode rviz`，复用批量测试同一套 Frenet preset、地图生成和 launch 参数；
- `build_launch_cmd()` 支持 `enable_rviz:=true/false`，避免 shell 里复制第二套 launch 参数。

验证：

```text
bash -n run_frenet_random_obstacle_robustness.sh
python3 -m py_compile code/lqr_sweep/frenet_random_robustness.py
python3 -m pytest test/test_frenet_planner.py test/test_frenet_random_robustness.py -q
34 passed
```

## 2026-05-26 更新：提前预激活 Frenet 避障

问题现象：RViz 中候选轨迹束偶尔到障碍物较近时才出现，小车在第一障碍物前仍可能短暂停住。原因是旧逻辑把 `activation_lookahead` 同时作为威胁检测和真正切入 Frenet 的边界；`approach` 区间只发布中心线并限速，不会提前生成绕障轨迹。

改动：

- `threat_lookahead` 改为覆盖 `slowdown_lookahead = activation_lookahead + approach_extra`；
- 当障碍物进入 `slowdown_lookahead` 后立即尝试 Frenet 规划并发布候选轨迹；
- 若仍在预激活区间且当前帧无 safe candidate，不直接发布 stop path，而是继续发布 approach 中心线路径并保持 avoidance speed，等待下一帧重试；
- 真正进入 `activation_lookahead` 后若仍无解，才进入 stop/reuse 保护逻辑；
- 3.0m/s preset 的 `approach_extra` 从 2.25m 调整为 3.0m，候选轨迹束预计从约 8.05m 提前到约 11.05m 左右出现；低速 0.5-1.0m/s 只额外提前约 0.75m。

验证：

```text
python3 -m py_compile code/pnc_rc/frenet/node.py code/pnc_rc/frenet/planner.py code/lqr_sweep/frenet_random_robustness.py
bash -n run_frenet_test.sh && bash -n run_frenet_random_obstacle_robustness.sh
docker pytest: test/test_frenet_planner.py test/test_frenet_random_robustness.py
34 passed
```

## 2026-05-26 更新：候选轨迹选择安全裕度保护

观察：RViz 中有时 best candidate 看起来安全裕度不大。复查后发现问题不在 hard collision check，而在 safe candidates 之间的二次选择：

- `min_clearance_m` 只是硬拒绝阈值，过线后仍可能作为 safe candidate；
- 原始 cost 已包含 clearance deficit，但权重不能无限加大，否则会为了远离障碍选择过大横向偏移，反而靠近赛道边界或造成跟踪不稳；
- 防蛇形的 temporal consistency 逻辑只看横向连续性，之前可能从 raw best 切到 cost 明显更差的同侧候选，例如历史日志中出现过 `raw_cost=2.97`、`selected_cost=8.44`；
- 之前选择日志没有输出 raw/selected clearance，不方便判断是不是为了稳定性牺牲了安全裕度。

改动：

- 保留原始 clearance cost 公式，不做全局激进加权，避免把车推向赛道边界；
- temporal consistency 只允许在候选成本接近时覆盖 raw best：如果 `selected_cost - raw_cost > 3.0`，保留 raw best；
- 如果 temporal candidate 比 raw best 少了超过 0.08m clearance，且 selected clearance 低于 `safe_clearance_m`，保留 raw best；
- 选择日志新增 `raw_clearance` 和 `selected_clearance`，以后可以直接从 launch log 判断是否因为防蛇形而牺牲了安全裕度。

验证：

```text
python3 -m py_compile code/pnc_rc/frenet/node.py code/pnc_rc/frenet/planner.py
docker pytest: test/test_frenet_planner.py test/test_frenet_random_robustness.py
35 passed
```

补充：尝试过把 clearance cost 做归一化强惩罚，但 seed 0 smoke 会失败，说明单纯“越远越好”不是正确方向。候选安全性应该通过 hard check、clearance-aware override guard 和赛道边界约束共同保证，而不是只拉大障碍 clearance 权重。

## 2026-05-26 更新：同侧候选硬锁定

问题：仅靠 `candidate_side_switch_penalty` 仍然是 soft cost，另一侧候选如果 raw cost 更低，可能压过上一侧候选，表现为小车在左右两侧反复切换。

改动：

- 新增 `select_side_consistent_candidate()` 纯函数；
- 如果上一条已选候选在左侧/右侧，且同侧仍存在 safe candidate，优先只在同侧候选集合内选最优；
- 只有上一侧没有 safe candidate，或上一侧候选相对 raw best 损失超过 0.08m clearance 且低于 `safe_clearance_m`，才允许切到另一侧；
- 保留原有 temporal cost 作为同侧集合内部排序依据，而不是作为跨侧切换依据；
- RViz/日志中会输出 `reason=same_side`，便于确认当前是同侧锁定而不是普通 cost 选择。

验证：

```text
python3 -m py_compile code/pnc_rc/frenet/planner.py code/pnc_rc/frenet/node.py
bash -n run_frenet_test.sh && bash -n run_frenet_random_obstacle_robustness.sh
docker pytest: test/test_frenet_planner.py test/test_frenet_random_robustness.py
37 passed
```

## 2026-05-26 更新：同侧锁定改为整条轨迹主导侧

继续观察 RViz 后发现，仅用 `d[-1]` 判断左右侧仍不充分。原因是 RViz 展示的是整条候选轨迹形状，而不是终点；一条轨迹可能中段先跨到另一侧，最后终点又回到上一侧，旧逻辑仍会把它误判为同侧，表现上仍然蛇形。

改动：

- 新增 `candidate_path_side()`，用整条 `d(t)` 的主导侧判断候选轨迹侧别；
- 如果轨迹在两侧都有明显横向偏移，则归为中性，不参与“同侧候选集合”；
- Frenet node 新增 `last_selected_candidate_side`，记录上一条被选中轨迹的主导侧，而不是只记录 `d_final`；
- 同侧锁定现在筛选的是“整条轨迹主导侧一致”的候选，避免终点同侧但中段跨侧的候选被选中；
- 新增测试覆盖：纯左/纯右轨迹分类、跨两侧轨迹分类、终点同侧但中段跨侧时不作为同侧候选。

验证：

```text
python3 -m py_compile code/pnc_rc/frenet/planner.py code/pnc_rc/frenet/node.py
docker pytest: test/test_frenet_planner.py test/test_frenet_random_robustness.py
39 passed
```

## 2026-05-26 更新：review 结论补齐

目标：把 subagent review 中剩余的参数冗余、测试 launch 参数重复和注释不够详细继续收敛。

改动：

- `FRENET_NODE_DEFAULTS`、`FRENET_LAUNCH_ARGUMENT_DEFAULTS`、`FRENET_NODE_PARAM_MAP` 统一搬到 `code/pnc_rc/frenet/preset.py`；
- `node.py` 不再手写几十行 `declare_parameter()` 默认值，改为遍历 `FRENET_NODE_DEFAULTS`；
- `launch/pnc_sim_launch.py` 不再本地维护 Frenet launch 参数表，改为从 `preset.py` 导入共享表；
- 新增 `frenet_static_test_launch_args()`，固定测试脚本和随机障碍 runner 共用同一份 47 个 Frenet launch 覆盖参数；
- `run_frenet_test.sh` 不再手写完整 Frenet 参数长列表，而是调用 `python3 -m pnc_rc.frenet.preset --shell-launch-args` 生成；
- `planner.py` 新增 `local_grid_shape()`，消除 `build_occupancy_grid()` 和 `local_static_map_occupancy()` 中重复的局部栅格尺寸公式；
- `node.py`、`planner.py`、`preset.py`、随机测试 runner 的 class/function docstring 全部补齐，当前 AST 检查没有缺失 docstring。

验证：

```text
PYTHONPYCACHEPREFIX=/tmp/f1tenth_pycache python3 -m py_compile \
  code/pnc_rc/frenet/planner.py \
  code/pnc_rc/frenet/node.py \
  code/pnc_rc/frenet/preset.py \
  code/lqr_sweep/frenet_random_robustness.py \
  code/run_frenet_planner.py \
  launch/pnc_sim_launch.py
bash -n run_frenet_test.sh && bash -n run_frenet_random_obstacle_robustness.sh
PYTHONPATH=code python3 -m pnc_rc.frenet.preset --target-speed 3.0 --avoidance-speed 1.2 --shell-launch-args
docker pytest: test/test_frenet_planner.py test/test_frenet_random_robustness.py
39 passed
docker: colcon build + ros2 launch --show-args 可看到 enable_frenet_planner、frenet_target_speed、frenet_grid_forward_m
```

## 2026-05-26 更新：同侧内部蛇形抑制

问题：旧的同侧锁定只判断候选在中心线左侧还是右侧，因此只能防止 `d>0` 和
`d<0` 之间切换。若车辆始终在中心线同一侧，但候选在 `d=0.4m` 和 `d=1.4m`
之间来回跳，旧逻辑仍会认为它们是“同侧”，RViz 中依然会出现蛇形。

改动：

- `select_side_consistent_candidate()` 新增横向通道 profile 比较；
- node 现在记录上一条选中候选的 `s/d` 曲线，而不是只记录 `d_final` 和左右侧；
- 新候选会在前方 `frenet_candidate_profile_lookahead_m` 范围内与上一条 `d(s)` 对齐比较；
- 若 profile 最大横向跳变超过 `frenet_candidate_profile_max_jump_m`，默认不切到该候选；
- 只有 raw best 比当前通道至少多出 `frenet_candidate_profile_unlock_clearance_gain_m` 的 clearance，且达到 `safe_clearance_m`，才允许跳出原通道；
- 中心线短暂安全时只清掉可复用 path cache，不立刻清掉横向通道记忆；通道记忆由 `frenet_candidate_channel_memory_timeout_s` 控制；
- 新增测试覆盖：同侧内部大横向跳变会选择原通道；当新通道 clearance 明显更高时允许解锁。

新增参数：

```text
frenet_candidate_profile_consistency_weight:=20.0
frenet_candidate_profile_max_jump_m:=0.35
frenet_candidate_profile_lookahead_m:=4.0
frenet_candidate_profile_unlock_clearance_gain_m:=0.12
frenet_candidate_channel_memory_timeout_s:=1.5
```

验证：

```text
PYTHONPYCACHEPREFIX=/tmp/f1tenth_pycache python3 -m py_compile \
  code/pnc_rc/frenet/planner.py \
  code/pnc_rc/frenet/node.py \
  code/pnc_rc/frenet/preset.py \
  test/test_frenet_planner.py \
  test/test_frenet_random_robustness.py
bash -n run_frenet_test.sh && bash -n run_frenet_random_obstacle_robustness.sh
docker pytest: test/test_frenet_planner.py test/test_frenet_random_robustness.py
41 passed
docker: colcon build + ros2 launch --show-args 可看到全部 frenet_candidate_profile_* 和 frenet_candidate_channel_memory_timeout_s
```

## 2026-05-26 更新：Frenet 代码去重和中文注释整理

目标：方便人工继续排查 Frenet 行为问题，不再让参数公式、缓存轨迹复用逻辑和核心碰撞工具分散难读。

改动：

- 新增 `code/pnc_rc/frenet/preset.py`，把 target speed / avoidance speed 派生出的 Frenet preset 公式集中到一个 Python 模块；
- `run_frenet_test.sh` 通过 `python3 -m pnc_rc.frenet.preset --shell-defaults` 获取默认 Frenet 参数，避免 bash 和随机测试 runner 各写一份公式；
- `code/lqr_sweep/frenet_random_robustness.py` 复用同一个 preset 模块，并补充 batch/rviz 随机障碍测试的中文 Google 风格 docstring；
- `launch/pnc_sim_launch.py` 用 `FRENET_LAUNCH_ARGUMENT_DEFAULTS` 和 `FRENET_NODE_PARAM_MAP` 集中声明/转发 Frenet launch 参数，减少超长重复 `DeclareLaunchArgument` 和 `-p` 列表；
- `code/pnc_rc/frenet/node.py` 抽出 `_safe_cached_candidate()` 和 `_trim_cached_candidate()`，让 hold/reuse 两条路径共用缓存候选安全复检和重锚定逻辑；
- `planner.py` 给 `OccupancyGrid.query_path()`、地图坐标转换、多项式求解、路径空间加密、扫掠通道、路径裁剪、速度相关激活距离等核心函数补充中文 Google 风格注释；
- `node.py` 给 ROS callback、发布路径/限速、debug marker、fallback 和 helper 补充中文 Google 风格注释；
- `code/run_frenet_planner.py` 入口说明改为中文。

保留风险：

- Frenet 默认参数仍存在三层来源：node 内部默认值、launch 默认值、测试脚本覆盖值。当前没有强行统一，因为这会改变现有 launch 覆盖语义；
- 固定测试脚本和随机测试 runner 的完整 launch 覆盖参数列表仍是两份手写列表，只共享了速度相关 preset 公式；
- 低速/高速下 `approach_extra_m` 当前为 `max(0.75, target^2 / 3.0)`，3.0m/s 时会提前 3.0m 进入 approach 区间，若你觉得降速过早，应优先看这个参数。

验证：

```text
PYTHONPYCACHEPREFIX=/tmp/f1tenth_pycache python3 -m py_compile \
  code/pnc_rc/frenet/planner.py \
  code/pnc_rc/frenet/node.py \
  code/pnc_rc/frenet/preset.py \
  code/lqr_sweep/frenet_random_robustness.py \
  code/run_frenet_planner.py \
  launch/pnc_sim_launch.py
bash -n run_frenet_test.sh
bash -n run_frenet_random_obstacle_robustness.sh
PYTHONPATH=code python3 -m pnc_rc.frenet.preset --target-speed 3.0 --avoidance-speed 1.2 --shell-defaults
docker pytest: test/test_frenet_planner.py test/test_frenet_random_robustness.py
39 passed
```

## 2026-05-26 最终补充：同侧内部蛇形剩余漏洞

RViz 截图继续出现左上角候选左右跳变后，复查发现剩余漏洞是：上一版 profile
锁定只处理“存在阈值内候选”的情况。如果所有候选都超过
`frenet_candidate_profile_max_jump_m`，代码会退回同侧选择，导致同一侧内部仍可大幅横跳。

本次修复：

- 只要上一条 `d(s)` profile 和当前候选可比较，就优先按 profile 误差决策；
- 阈值内有候选时仍选 `same_profile`；
- 阈值内没有候选时选 `max profile error` 最小的候选，reason 为
  `least_profile_jump`；
- 仅当 raw best 有明确 clearance 收益时才允许打破该 profile 锁；
- 新增单测 `test_profile_consistent_selection_uses_least_jump_when_all_exceed_limit()`。

验证：

```text
PYTHONPYCACHEPREFIX=/tmp/f1tenth_pycache python3 -m py_compile \
  code/pnc_rc/frenet/planner.py \
  code/pnc_rc/frenet/node.py \
  code/pnc_rc/frenet/preset.py \
  test/test_frenet_planner.py \
  test/test_frenet_random_robustness.py
bash -n run_frenet_test.sh && bash -n run_frenet_random_obstacle_robustness.sh
docker pytest: test/test_frenet_planner.py test/test_frenet_random_robustness.py
42 passed
```

## 2026-05-26 更新：修复绕障后硬切中心线导致的蛇形

录屏复查结论：RViz 中看起来像候选轨迹左右乱选，但 LQR 日志显示 `e_y`
会瞬间从接近 0 跳到约 0.7m。根因是 Frenet 状态机只要检测到中心线前方
无障碍，就直接发布 `d=0` 中心线路径；此时车辆仍在障碍外侧，局部路径会
横向硬跳到中心线，控制器随后反向追线，表现为蛇形。

改动：

- 新增 `sample_return_to_centerline_segment()`，从当前 `d` 平滑收敛到中心线；
- `_publish_centerline_path()` 新增 `start_d`，当 `abs(d)` 大于
  `frenet_centerline_return_direct_d_threshold_m` 时发布平滑回中路径，而不是直接
  发布中心线；
- 回中阶段使用 avoidance speed limit，避免刚绕完障碍就高速横向收敛；
- 新增 `path_pose_error()`，计算车辆相对旧路径的横向距离和航向误差；
- held path 复用新增 `frenet_held_path_max_lateral_error_m` 和
  `frenet_held_path_max_heading_error_rad`，车辆离旧路径太远或朝向不一致时强制
  重规划，不再复用滞后的旧轨迹；
- 新增单测覆盖平滑回中路径和 stale held path 几何误差检测。

新增参数：

```text
frenet_centerline_return_direct_d_threshold_m:=0.20
frenet_held_path_max_lateral_error_m:=0.45
frenet_held_path_max_heading_error_rad:=0.85
```

验证：

```text
PYTHONPYCACHEPREFIX=/tmp/f1tenth_pycache python3 -m py_compile \
  code/pnc_rc/frenet/planner.py \
  code/pnc_rc/frenet/node.py \
  code/pnc_rc/frenet/preset.py \
  code/lqr_sweep/frenet_random_robustness.py \
  code/run_frenet_planner.py \
  launch/pnc_sim_launch.py \
  test/test_frenet_planner.py \
  test/test_frenet_random_robustness.py
bash -n run_frenet_test.sh && bash -n run_frenet_random_obstacle_robustness.sh
PYTHONPATH=code python3 -m pnc_rc.frenet.preset --target-speed 3.0 --avoidance-speed 1.2 --shell-launch-args | rg "held_path_max|centerline_return_direct"
docker pytest: test/test_frenet_planner.py test/test_frenet_random_robustness.py
44 passed
docker: colcon build --packages-select f1tenth_gym_ros + ros2 launch --show-args 可看到新增的 3 个 launch 参数
```

## 2026-05-26 更新：保留可信减速度并修复 stop path 退化

继续 review 后发现两个问题：

- `_forward_progress_planning_state()` 使用硬编码 `1.0m/s^2` 判断加速度是否可信，
  且最终 `max(0.0, s_ddot)` 会把所有负加速度抹掉；这会丢掉真实刹车状态；
- `stop_path_length_m=0.25` 会让 LQR 在超短 3 点 stop path 上反复触发
  open-loop endpoint，短仿真日志里曾出现 `max_abs_e_y=5.62m`、`|Δe_y|>0.25m`
  共 297 次的路径误差突跳。

改动：

- `FrenetPlannerConfig` 新增 `max_initial_s_accel_mps2` 和
  `max_reliable_initial_s_accel_mps2`；
- `_forward_progress_planning_state()` 现在保留可信范围内的正/负初始加速度，
  只把超过可靠上界的离群估计拒绝为 0；
- 默认值设为 `max_initial=2.0m/s^2`、`max_reliable=3.0m/s^2`，并通过 launch
  参数暴露；
- `stop_path_length_m` 默认从 `0.25m` 改为 `2.0m`；
- `publish_stop_path()` 不再只发 3 个点，而是按 `centerline_return_step_m`
  采样一条非退化停车保持路径；停车仍由 `stop` 模式 0 速限制完成。

新增/调整参数：

```text
frenet_max_initial_s_accel_mps2:=2.0
frenet_max_reliable_initial_s_accel_mps2:=3.0
frenet_stop_path_length_m:=2.0
```

验证：

```text
PYTHONPYCACHEPREFIX=/tmp/f1tenth_pycache python3 -m py_compile \
  code/pnc_rc/frenet/planner.py \
  code/pnc_rc/frenet/node.py \
  code/pnc_rc/frenet/preset.py \
  test/test_frenet_planner.py \
  test/test_frenet_random_robustness.py \
  launch/pnc_sim_launch.py
bash -n run_frenet_test.sh && bash -n run_frenet_random_obstacle_robustness.sh
docker pytest: test/test_frenet_planner.py test/test_frenet_random_robustness.py
44 passed
docker: colcon build --packages-select f1tenth_gym_ros + ros2 launch --show-args 可看到新增/调整参数
FRENET_HEADLESS_TIMEOUT_S=35 ./run_frenet_test.sh --headless --target-speed 3.0 --avoidance-speed 1.2
latest LQR log: max_abs_e_y=0.193m, jumps |Δe_y|>0.25m = 0
```

## 2026-05-26 更新：简化 Frenet 判定逻辑，修复慢退出和候选粘滞

subagent 只读审计结论：

- `node.py` 的主要粘滞来自三点叠加：回中路径仍用 `avoidance` 模式发布、
  approach 阶段提前生成候选束、held/reuse 继续保留旧 candidate marker；
- `planner.py` 的候选选择逻辑过度复杂，把 profile 锁定、中心线左右侧锁定、
  endpoint 连续性、clearance 解锁和 cost gap 多套规则混在一个函数里；
- 真正必须保留的是 hard-safe 过滤：progress、collision、min clearance、
  curvature 和路径扫掠检查。

本次简化：

- `approach` 只做提前限速和中心线跟踪，不再触发 Frenet 候选规划；
- 只有障碍进入 `activation_lookahead` 后才进入 Frenet 候选生成；
- 回中路径按调用方模式发布，不再因为 `abs(d)` 大就强制使用 `avoidance`
  限速；
- 删除运行时“中心线左右侧”候选锁定，不再维护
  `last_selected_candidate_side`；
- `select_side_consistent_candidate()` 重命名并简化为
  `select_temporally_consistent_candidate()`；
- 候选二次选择现在只使用一个 adjusted cost：
  `raw_cost + endpoint_d_jump_penalty + profile_error_penalty`；
- 删除 `candidate_side_switch_penalty`、`candidate_side_deadband_m` 及对应
  launch/script/test 参数。

当前语义：

- `centerline`：无威胁或威胁仍在 slowdown 外，发布中心线/回中路径；
- `approach`：威胁进入 slowdown 但仍在 activation 外，只提前限速，不显示候选束；
- `frenet`：威胁进入 activation 后才规划候选；
- `stop`：activation 内无安全候选且不能复用旧安全路径。

验证：

```text
PYTHONPYCACHEPREFIX=/tmp/f1tenth_pycache python3 -m py_compile \
  code/pnc_rc/frenet/planner.py \
  code/pnc_rc/frenet/node.py \
  code/pnc_rc/frenet/preset.py \
  test/test_frenet_planner.py \
  test/test_frenet_random_robustness.py \
  launch/pnc_sim_launch.py
bash -n run_frenet_test.sh && bash -n run_frenet_random_obstacle_robustness.sh
PYTHONPATH=code python3 -m pytest test/test_frenet_planner.py test/test_frenet_random_robustness.py -q
```

结果：

```text
py_compile: passed
bash -n: passed
docker pytest: 40 passed
```

## 2026-05-27 更新：缩短绕障后的平滑回中距离

问题：小车越过障碍物后已经切回 centerline 状态，但仍沿绕障后的偏移轨迹慢慢
回中。原因是 `frenet_centerline_return_lookahead_m` 固定为 5.0m，平滑回中
曲线会在 5m 内逐渐把当前 `d` 收敛到 0，对小赛道来说太慢。

改动：

- `FrenetPreset` 新增 `centerline_return_lookahead_m`；
- 固定测试和随机障碍测试的 launch 参数不再写死 5.0m，改为随目标速度生成；
- 速度相关默认值：
  - `target_speed <= 0.75m/s` 时使用 `2.0m`；
  - `target_speed = 3.0m/s` 时使用 `3.5m`；
  - 中间速度线性过渡并限制在 `[2.0m, 3.5m]`；
- `run_frenet_test.sh` 新增环境变量覆盖入口：
  `FRENET_CENTERLINE_RETURN_LOOKAHEAD_M`；
- 脚本启动摘要新增 `centerline_return: lookahead=...m`，方便确认当前值。

预期效果：绕障结束后仍保留平滑回中，避免硬切 centerline 引起横跳；但回中长度
从 5.0m 缩短到 2.0-3.5m，小车会更快贴回 centerline。

验证：

```text
PYTHONPYCACHEPREFIX=/tmp/f1tenth_pycache python3 -m py_compile \
  code/pnc_rc/frenet/preset.py \
  code/pnc_rc/frenet/node.py \
  test/test_frenet_random_robustness.py
bash -n run_frenet_test.sh && bash -n run_frenet_random_obstacle_robustness.sh
PYTHONPATH=code python3 -m pnc_rc.frenet.preset --target-speed 3.0 --avoidance-speed 1.2 --shell-launch-args
PYTHONPATH=code python3 -m pnc_rc.frenet.preset --target-speed 0.5 --avoidance-speed 0.5 --shell-launch-args
docker pytest: test/test_frenet_planner.py test/test_frenet_random_robustness.py
41 passed
FRENET_HEADLESS_TIMEOUT_S=35 ./run_frenet_test.sh --headless --target-speed 3.0 --avoidance-speed 1.2
startup summary: centerline_return: lookahead=3.500m
latest LQR log: max_abs_e_y=0.211m, max |Δe_y|=0.211m, jumps |Δe_y|>0.25m = 0
```

## 2026-05-27 更新：Frenet 激活距离与候选平滑性选择

问题：

- `target_speed=3.0m/s` 时，速度公式给出的 Frenet activation 约 `8.05m`，
  但实际发布给 LQR 的局部路径曾只有约 `3.9m`；
- 过早进入 Frenet 时，障碍还没有进入有效局部路径窗口，候选容易仍沿中心线；
- 等车辆走完约 80% 发布路径后再次规划时，障碍已经很近，可能出现急停或无解；
- 候选选择不应简单偏好“低绝对曲率/越直越好”，而应优先选择局部曲率变化率小、
  更平滑、不弯弯绕绕的安全轨迹。

改动：

- 新增 `activation_path_margin_m` / `frenet_activation_path_margin_m`，当前默认
  `0.50m`；
- 有发布路径长度上限时，实际 Frenet activation 改为：
  `min(raw_activation, max_published_path_length_m + activation_path_margin_m)`；
- 在 `target_speed=3.0m/s`、`max_published_path_length_m=5.4m` 下：
  - 原始速度公式 activation 约 `8.05m`；
  - 实际 Frenet activation 变为约 `5.90m`；
  - slowdown/approach 距离变为约 `6.65m`；
- 调整状态机顺序：先检测中心线威胁；若中心线已安全或威胁仍较远，发布
  centerline/approach，不继续 hold 旧 Frenet 路径；
- 只有障碍进入有效 activation 距离后，才允许 hold 旧 Frenet 路径或重新生成
  Frenet 候选；
- `FrenetPlannerConfig.max_curvature` 默认改为 `1.14 1/m`，对应
  `tan(0.36rad) / 0.3302m`，即仅按车辆前轮转角限幅约束；
- `weight_curvature` 降为 `0.4`，避免额外偏好“越直越好”；
- `weight_curvature_rate` 提高到 `8.0`，并新增
  `curvature_smoothness_cost()`，按弧长归一化惩罚 `dk/ds` 的平均平方值；
- 新增回归测试，验证同样安全时，局部曲率变化率更大的 wavy 轨迹 cost 更高。

当前关键参数：

```text
frenet_max_curvature:=1.14
frenet_weight_curvature:=0.4
frenet_weight_curvature_rate:=8.0
frenet_max_published_path_length_m:=5.4
frenet_activation_path_margin_m:=0.50
```

验证：

```text
PYTHONPYCACHEPREFIX=/tmp/f1tenth_pycache python3 -m py_compile \
  code/pnc_rc/frenet/planner.py \
  code/pnc_rc/frenet/node.py \
  code/pnc_rc/frenet/preset.py \
  test/test_frenet_planner.py
bash -n run_frenet_test.sh
docker run --rm --entrypoint /bin/bash \
  -v /home/art3m1s/f1tenth_frenet_static_avoidance:/sim_ws/src/f1tenth_gym_ros \
  f1tenth_gym_ros:latest \
  -lc 'cd /sim_ws/src/f1tenth_gym_ros && PYTHONPATH=code python3 -m pytest test/test_frenet_planner.py test/test_frenet_random_robustness.py -q'
FRENET_HEADLESS_TIMEOUT_S=20 ./run_frenet_test.sh --headless \
  --target-speed 3.0 --avoidance-speed 1.2
```

结果：

```text
py_compile: passed
bash -n: passed
docker pytest: 44 passed
headless startup summary: activation path_margin=0.500m, max_path_length=5.4m
runtime expectation: activation_lookahead≈5.90m, slowdown_lookahead≈6.65m
```

## 2026-05-27 更新：随机障碍 RViz 可视化 45s 自动停止

问题：

- 随机障碍鲁棒性 batch 模式按圈数/碰撞/timeout 停止；
- RViz 可视化模式原来直接前台运行 `ros2 launch`，需要人工关闭，不适合快速
  单 seed 观察。

改动：

- `code/lqr_sweep/frenet_random_robustness.py` 的 `run_rviz_seed()` 改为
  `subprocess.Popen(..., preexec_fn=os.setsid)`；
- RViz 模式不按圈数判定，只用于人工观察；
- `--timeout > 0` 时到时自动向整个 `ros2 launch` 进程组发送 SIGINT；
- `--timeout 0` 时保持原手动关闭行为；
- `run_frenet_random_obstacle_robustness.sh --rviz` 默认 timeout 改为 `45s`，
  batch 模式默认仍是 `240s`；
- 新增一键脚本 `run_frenet_random_obstacle_rviz.sh`，默认：
  - `--rviz`
  - `--trials 1`
  - `--timeout 45`
  - 其他参数透传给原 runner。

强制停车/停止相关入口：

- Frenet 无 LaserScan：`FrenetStaticObstaclePlanner.plan_once()` 发布 stop path；
- activation 内无安全 Frenet 候选且不能复用旧路径：
  `_publish_approach_or_stop(..., preactivation_only=False)` 发布 stop path；
- `publish_stop_path()` 发布 `mode="stop"` 的局部路径；
- `_speed_limit_for_mode("stop")` 发布 `stop_speed_limit_mps=0.0`；
- LQR `speed_limit_callback()` 接收 0 限速，并在 `_compute_speed_command()` 中把
  `speed_cmd` 压到 0；
- LQR `_open_loop_finished()` 触发时也会把 `v_cmd=0.0`。

使用：

```text
./run_frenet_random_obstacle_rviz.sh --seed 0 --target-speed 3.0 --avoidance-speed 1.2
./run_frenet_random_obstacle_rviz.sh --seed 0 --timeout 60
./run_frenet_random_obstacle_rviz.sh --seed 0 --timeout 0  # 手动关闭
```

验证：

```text
bash -n run_frenet_random_obstacle_robustness.sh
bash -n run_frenet_random_obstacle_rviz.sh
PYTHONPYCACHEPREFIX=/tmp/f1tenth_pycache python3 -m py_compile \
  code/lqr_sweep/frenet_random_robustness.py
docker run --rm --entrypoint /bin/bash \
  -v /home/art3m1s/f1tenth_frenet_static_avoidance:/sim_ws/src/f1tenth_gym_ros \
  f1tenth_gym_ros:latest \
  -lc 'cd /sim_ws/src/f1tenth_gym_ros && PYTHONPATH=code python3 -m pytest test/test_frenet_random_robustness.py -q'
```

结果：

```text
bash -n: passed
py_compile: passed
docker pytest: 7 passed
```
