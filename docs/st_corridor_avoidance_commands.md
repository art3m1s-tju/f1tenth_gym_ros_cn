# ST Corridor Avoidance Test Commands

本分支使用局部轨迹层做避障：全局 planner 继续发布 `/global_trajectory`，ST corridor planner 发布 `/local_trajectory` 和 `/local_speed_limit`，LQR 跟踪 `/local_trajectory` 并把速度限制取最小值。

## 1. 启动无障碍回归测试

在容器内：

```bash
cd /sim_ws
source install/local_setup.bash
ros2 launch f1tenth_gym_ros pnc_sim_launch.py \
  enable_rviz:=true \
  enable_st_corridor_avoidance:=true \
  target_speed:=1.0 \
  st_avoidance_max_speed:=1.0 \
  st_min_speed:=0.35
```

期望现象：

- `/local_trajectory` 基本贴合 `/global_trajectory`。
- `/local_speed_limit` 接近 `target_speed`。
- 小车完成 3 圈，无明显贴墙、急转或停顿。

## 2. RViz 可视化配置

RViz 中建议打开或新增这些 Display：

| Display | Topic | Type | 建议颜色 |
|---|---|---|---|
| Global trajectory | `/global_trajectory` | `nav_msgs/Path` | 绿色 |
| Local trajectory | `/local_trajectory` | `nav_msgs/Path` | 蓝色或黄色 |
| Tracked path | `/tracked_path_lqr` | `nav_msgs/Path` | 红色 |
| Laser scan | `/scan` | `sensor_msgs/LaserScan` | 默认 |
| Map | `/map` | `nav_msgs/OccupancyGrid` | 默认 |
| Robot model | robot model | `RobotModel` | 默认 |

判断标准：

- 无障碍时：蓝色 `/local_trajectory` 不应该离绿色 `/global_trajectory` 很远。
- 有障碍时：蓝色轨迹应提前偏移绕过障碍，而不是贴墙绕大圈。
- 如果 `/local_trajectory` 消失，先检查 ST planner 是否收到 `/global_trajectory` 和 `/ego_racecar/odom`。

## 3. 查看关键话题

```bash
ros2 topic list | grep -E 'global_trajectory|local_trajectory|local_speed_limit|scan|odom|drive'
ros2 topic echo /local_speed_limit
ros2 topic hz /local_trajectory
```

## 4. 随机 0.5m 方块障碍 5 圈自动验证

生成 2 或 3 个随机静态方块障碍地图：

```bash
cd /sim_ws/src/f1tenth_gym_ros
python3 code/make_static_obstacle_map.py \
  --seed 7 \
  --obstacle-count 3 \
  --output-dir maps/generated_static_obstacles \
  --name smoke_obs3_seed7
```

无 RViz 自动跑 5 圈并评估：

```bash
cd /sim_ws
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
python3 /sim_ws/src/f1tenth_gym_ros/code/lqr_sweep/validate_ros.py \
  --enable-st-corridor-avoidance \
  --disable-rviz \
  --speed 1.0 \
  --laps 5 \
  --timeout 170 \
  --map-path /sim_ws/src/f1tenth_gym_ros/maps/generated_static_obstacles/smoke_obs3_seed7 \
  --map-yaml /sim_ws/src/f1tenth_gym_ros/maps/generated_static_obstacles/smoke_obs3_seed7.yaml \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/evaluation_ros/st_corridor_smoke_seed7_lap5_v1p0 \
  --log-path /sim_ws/src/f1tenth_gym_ros/code/outputs/logs/lqr_tracking_st_seed7_lap5_v1p0.csv \
  --st-log-path /sim_ws/src/f1tenth_gym_ros/code/outputs/logs/st_corridor_seed7_lap5_v1p0.csv \
  --collision-log-path /sim_ws/src/f1tenth_gym_ros/code/outputs/logs/collision_seed7_lap5_v1p0.csv \
  --st-avoidance-max-speed 0.9 \
  --st-min-speed 0.35 \
  --st-lookahead-m 5.5 \
  --use-tf-pose
```

通过标准：

- 终端输出 `5 laps detected`。
- `collision_*.csv` 中 `ego_collision` 全为 `0`。
- `st_corridor_*.csv` 至少出现 `mode=avoid`，且 `min_obstacle_clearance > 0`。
- `lookahead_summary.json` 中横向误差保持厘米级，且速度没有长时间降到 0。

本分支已在容器中验证过：

| 场景 | 速度 | 圈数 | 碰撞 | ST 避障行数 | 最小净空 | 用时 |
|---|---:|---:|---:|---:|---:|---:|
| seed 7, 3 obstacles | 1.0 m/s | 5 | 0 | 8 | 0.543 m | 62.58 s |
| seed 13, 2 obstacles | 1.0 m/s | 5 | 0 | 2 | 0.668 m | 62.73 s |

## 5. 单障碍测试建议

使用小于 `0.5m x 0.5m` 的静态障碍地图或仿真配置后启动同一条 launch 命令。建议先保持低速：

```bash
ros2 launch f1tenth_gym_ros pnc_sim_launch.py \
  enable_rviz:=true \
  enable_st_corridor_avoidance:=true \
  target_speed:=0.8 \
  st_avoidance_max_speed:=0.8 \
  st_min_speed:=0.30
```

通过后再试：

```bash
ros2 launch f1tenth_gym_ros pnc_sim_launch.py \
  enable_rviz:=true \
  enable_st_corridor_avoidance:=true \
  target_speed:=1.2 \
  st_avoidance_max_speed:=1.0 \
  st_min_speed:=0.35
```

## 6. 日志位置

- LQR 跟踪日志：`code/outputs/logs/lqr_tracking_log.csv`
- ST corridor 日志：`code/outputs/logs/st_corridor_log.csv`
- 碰撞/终止日志：`code/outputs/logs/collision_log.csv`
- 实验记录：`code/outputs/2026-05-22_st_corridor_avoidance_log.md`

ST corridor CSV 字段：

- `mode`: `pass_through` / `avoid` / `blocked`
- `obstacle_count`: 当前前方走廊内障碍数量
- `chosen_offset`: 选中的横向偏移
- `speed_limit`: 发布给 LQR 的速度上限
- `min_obstacle_clearance`: 候选轨迹最小障碍净空
- `candidate_count`: 候选横向偏移数量
- `collision_free_count`: 无碰撞候选数量

## 7. 常见问题

| 现象 | 优先检查 |
|---|---|
| 小车不动 | `/local_speed_limit` 是否为 0，`st_corridor_log.csv` 是否一直 `blocked` |
| 局部轨迹贴墙 | 增大 `min_border_clearance_m` 或减小 `max_lateral_offset_m` |
| 无障碍也绕行 | 看 `obstacle_count` 是否非 0；若是，说明墙体过滤太宽松，增大 `obstacle_wall_margin_m` |
| 避障太保守 | 提高 `st_avoidance_max_speed`，或减小 `inflation_margin_m` |
| 轨迹抖动 | 减小 `lateral_offset_step_m` 或降低 `target_speed` 先验证 |
