# Frenet 静态障碍物避障实验说明

## 目标

本分支从 `main` 新建，目标是在现有 LQR 控制器前面增加一个 Frenet 局部规划器。

数据流是：

```text
LaserScan -> 局部占用栅格 -> 膨胀 -> Frenet 候选轨迹筛选 -> /local_trajectory -> LQR
```

LQR 仍然只负责跟踪路径，不直接处理障碍物。

## 论文算法与修正

论文使用 Frenet 坐标，把轨迹拆成横向 `d(t)` 和纵向 `s(t)`。

本实现采用：

- 横向 `d(t)`：五次多项式。
- 纵向 `s(t)`：四次多项式。

第 8 页手写笔记指出了论文描述里容易漏掉的一点：纵向初始状态必须包含 `s_ddot0`。否则四次纵向多项式的边界条件不完整。

因此本实现的状态是：

```text
s0, d0, s_dot0, d_dot0, s_ddot0, d_ddot0
```

其中：

- `s_dot0` 由 odom 速度投影到参考路径切向得到。
- `d_dot0` 由 odom 速度投影到参考路径法向得到。
- `s_ddot0` 由最近两次 `s_dot0` 差分估计，第一帧为 `0`。
- `d_ddot0` 第一版固定为 `0`。

纵向终点不约束 `s_f`，只约束终点速度和终点加速度：

```text
s_dot(tf) = s_dot_f
s_ddot(tf) = 0
```

这样可以避免强行规定纵向终点位置，让路径长度由速度和时间自然决定。

## 占用栅格

规划器订阅 `/scan`，把雷达命中点放到车辆局部坐标系中。

实现里会忽略 `0.0m` 或近似 `0.0m` 的无效 `LaserScan` 回波。这一点很重要，因为部分仿真或驱动会用 `0` 表示“没有返回”，如果把它当成真实障碍，车辆前方会被错误标成碰撞区，Frenet 会表现成“所有候选都撞墙”。

默认局部栅格参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `grid_forward_m` | `7.0` | 车辆前方范围 |
| `grid_rear_m` | `1.0` | 车辆后方范围 |
| `grid_half_width_m` | `3.0` | 左右半宽 |
| `grid_resolution_m` | `0.05` | 栅格分辨率 |
| `grid_inflation_radius_m` | `0.28` | 障碍膨胀半径 |
| `scan_offset_x_m` | `0.275` | 雷达到 base_link 的前向偏移 |

候选轨迹只要有任一点落入膨胀区域，就被判定为碰撞并剔除。

规划器还会订阅 `/map`。因此即使雷达当前没有扫到方块或赛道边界，黑色地图障碍和墙体也会进入局部占用栅格。

如果没有任何安全候选，规划器不会再发布全局中心线作为兜底。全局中心线很可能正好穿过障碍或靠近边界，继续跟踪它会导致直接撞上去。当前兜底策略是发布车辆当前位置前方 `0.10m` 的短路径，让开环 LQR 判定局部路径已经结束并停车。

## 目标函数

论文里的目标函数比较基础，主要由 jerk、时间、横向偏移和速度误差组成。

本实现保留这些 baseline 项，同时增加更适合 F1TENTH + LQR 跟踪的安全项：

- 靠近障碍物的惩罚。
- 曲率过大的惩罚。
- 曲率变化过快的惩罚。
- 横向偏移突变的惩罚。

第一版默认偏安全稳健。速度只是次级奖励，不为了快而选择 LQR 难跟踪的激进轨迹。

## ROS 接入

新增节点：

```text
python3 code/run_frenet_planner.py
```

主要话题：

| 话题 | 方向 | 说明 |
|---|---|---|
| `/global_trajectory` | 输入 | 全局参考轨迹 |
| `/ego_racecar/odom` | 输入 | 车辆位姿和速度 |
| `/scan` | 输入 | 2D 雷达 |
| `/local_trajectory` | 输出 | Frenet 局部避障轨迹 |
| `/frenet/debug/candidates` | 输出 | Frenet 安全候选轨迹 MarkerArray，可在 RViz 中对比最佳轨迹与备选轨迹 |

主要参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `frenet_d_min` | `-1.8` | 候选轨迹最小横向偏移 |
| `frenet_d_max` | `1.8` | 候选轨迹最大横向偏移 |
| `frenet_target_speed` | `1.5` | Frenet 代价函数使用的目标终点速度，独立于 LQR 巡航速度 |
| `frenet_v_min` | `0.6` | Frenet 纵向终点速度采样下界 |
| `frenet_v_max` | `2.5` | Frenet 纵向终点速度采样上界 |
| `frenet_max_heading_jump` | `0.85` | 单条候选轨迹相邻段允许的最大航向跳变（rad） |
| `frenet_min_progress_step_m` | `0.20` | 单条候选轨迹从起点到终点的最小总前向推进距离 |
| `frenet_reuse_last_candidate_timeout_s` | `1.0` | 当前周期无解时，允许继续复用最近一条安全局部轨迹的最长时长 |
| `frenet_projection_search_window_m` | `6.0` | Frenet 在闭环参考线中围绕上一帧 `s` 保持投影连续性的弧长搜索窗口 |
| `frenet_grid_inflation_radius_m` | `0.28` | 障碍膨胀半径 |
| `frenet_grid_resolution_m` | `0.05` | 局部栅格分辨率 |

launch 开关：

```text
enable_frenet_planner:=true
```

关闭时，LQR 仍跟踪闭环 `/global_trajectory`。

开启时，LQR 跟踪短段 `/local_trajectory`，并自动使用：

```text
path_closed_loop:=false
```

局部轨迹不是闭环，不能让 LQR 把最后一个点连接回第一个点。

## RViz 调试可视化

`launch/gym_bridge.rviz` 已加入两层 Frenet 可视化：

- `/local_trajectory`：当前发布给 LQR 的局部轨迹。
- `/frenet/debug/candidates`：安全候选轨迹集合，其中更粗更亮的线表示当前选中的最佳轨迹。

如果你怀疑局部规划结果突然折返或贴墙，优先同时观察这两个 topic：前者回答“LQR 正在跟哪条线”，后者回答“Frenet 当时有哪些可行备选”。二者一起看，能很快分清问题是在候选生成、候选打分，还是控制跟踪。

## 控制预瞄说明

模拟器默认不启用 LQR 控制预瞄：

```text
lqr_lookahead_distance_m:=0.0
```

控制预瞄主要是给实车补偿系统延迟用的。仿真环境里先关闭它，避免把“控制延迟补偿”和“局部避障规划”的效果混在一起。

如果后续上实车，可以手动覆盖：

```text
lqr_lookahead_distance_m:=1.0
```

## 运行命令

### 0. 重新构建

如果改过 launch 或新增 Python 入口，先重新构建并 source：

```bash
cd /sim_ws
colcon build --symlink-install
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
```

否则 `ros2 launch f1tenth_gym_ros pnc_sim_launch.py` 可能仍在使用 install 目录里的旧 launch 文件。

### 1. 生成两个随机方块障碍地图

默认从 `maps/my_map.yaml` 复制地图，并在 `global_trajectory.csv` 上随机选择两个位置，写入 `0.4m x 0.4m` 方块障碍。

```bash
cd /sim_ws/src/f1tenth_gym_ros
python3 code/lqr_sweep/generate_static_obstacle_test_map.py \
  --obstacle-distances-m 4.0,7.0 \
  --obstacle-count 2 \
  --obstacle-size-m 0.4 \
  --output-prefix maps/generated_static_obstacles/two_blocks_seed7
```

输出：

```text
maps/generated_static_obstacles/two_blocks_seed7.pgm
maps/generated_static_obstacles/two_blocks_seed7.yaml
maps/generated_static_obstacles/two_blocks_seed7.json
```

`.json` 里记录两个障碍物中心点，方便复现实验。

如果想完全随机放置，把 `--obstacle-distances-m` 设为空字符串：

```bash
python3 code/lqr_sweep/generate_static_obstacle_test_map.py \
  --obstacle-distances-m "" \
  --seed 7
```

### 2. 对照测试

无 Frenet 避障的基线：

```bash
ros2 launch f1tenth_gym_ros pnc_sim_launch.py \
  map_path:=/sim_ws/src/f1tenth_gym_ros/maps/generated_static_obstacles/two_blocks_seed7 \
  enable_frenet_planner:=false \
  lqr_lookahead_distance_m:=0.0
```

开启 Frenet 静态避障：

```bash
ros2 launch f1tenth_gym_ros pnc_sim_launch.py \
  map_path:=/sim_ws/src/f1tenth_gym_ros/maps/generated_static_obstacles/two_blocks_seed7 \
  enable_frenet_planner:=true \
  lqr_lookahead_distance_m:=0.0 \
  frenet_d_min:=-1.8 \
  frenet_d_max:=1.8 \
  frenet_grid_inflation_radius_m:=0.28
```

如果切换到自定义 Frenet 测试赛道，建议同时通过 launch 参数覆盖起始位姿，而不是反复手改 `config/sim.yaml`：

```bash
sx:=0.0 sy:=5.0 stheta:=3.1416
```

`pnc_sim_launch.py` 现已支持直接覆盖 `sx` / `sy` / `stheta`。当前 launch 会先生成一份带覆盖值的临时 bridge 配置，再启动 `gym_bridge`，避免出现“命令行看起来改了，但 simulator 实际仍然沿用 `sim.yaml` 默认地图和默认起点”的问题。

预期现象：

- baseline：全局轨迹仍穿过障碍物，车辆应更容易撞到方块。
- Frenet：`/local_trajectory` 会绕开膨胀栅格中的方块，LQR 跟踪局部轨迹。

Frenet 默认预测时间固定为 `2.0s`，局部轨迹通常是 21 个点，避免 LQR 频繁跟踪过短轨迹。

`frenet_target_speed` / `frenet_v_min` / `frenet_v_max` 不再和 LQR 的 `target_speed` 绑死。这样即使控制器当前巡航速度设置得偏低，Frenet 也仍然可以用更合理的前向采样范围生成空间上足够平滑的候选轨迹。

`frenet_projection_search_window_m` 用于避免回环赛道上的投影串支路。对存在平行直道的闭环赛道，这个参数非常重要；如果只按欧式最近点投影，Frenet 可能会突然跳到另一条支路，导致横向偏移 `d` 瞬间变成不合理的大值。

如果终端出现：

```text
No safe Frenet candidate; publishing stop path
```

说明当前局部栅格里所有候选轨迹都被碰撞检测剔除了。括号里的 `total`、`safe`、`collision_reject`、`best_clearance` 和 `occupied_cells` 用来判断是障碍膨胀太保守、横向采样范围不够，还是地图边界已经把可行空间堵住。第一步可以尝试：

```bash
frenet_grid_inflation_radius_m:=0.22 frenet_d_min:=-1.8 frenet_d_max:=1.8
```

如果 `occupied_cells` 明显偏高，但 RViz 里又看不到对应障碍，下一步优先检查 `/scan` 是否包含大量 `0.0` range。现在规划器会过滤这些零值回波；如果你本地分支里还没带这个修复，日志上通常会表现为 `collision_reject` 快速打满、`best_clearance` 接近 `0.0m`。

如果日志偶尔出现当前周期 `No safe Frenet candidate`，但前后几个周期又能恢复 21 点局部轨迹，规划器现在会优先短时复用上一条安全 Frenet 轨迹，而不是立刻退回 3 点停车路径。这个机制主要用于吸收瞬时 scan 抖动或局部几何判定抖动，不替代真正的候选生成修复。

## 调试建议

如果 RViz 里能看到黑色方块，但 LaserScan 没有打到方块边界，先确认方块是否在车辆雷达视场内。推荐用：

```bash
--obstacle-distances-m 4.0,7.0
```

把障碍放在起点后较近的全局轨迹前方。

即使雷达没有扫到方块，Frenet 规划器现在也会读取 `/map`，把静态地图边界和黑色方块加入局部占用栅格，避免直接规划到墙里。

## 推荐验证赛道

为了单独验证 Frenet 避障，而不是把问题混进复杂赛道几何里，当前新增了一组更干净的测试赛道：

- `maps/frenet_test_loop_open.yaml`
- `maps/generated_static_obstacles/frenet_test_open_two_blocks.yaml`
- `code/outputs/generated_tracks/frenet_test_open_two_blocks_review.png`

这组地图是开阔的回环赛道，两个静态方块都放在可绕行区域内，适合直接观察：

- Frenet 候选是否会稳定分到障碍两侧。
- 最佳候选是否会突然贴外墙。
- `No safe Frenet candidate` 是否真的是环境无解，而不是感知栅格构建错误。

## 验证记录

| 场景 | 预期结果 | 当前结果 |
|---|---|---|
| 无障碍 | `/local_trajectory` 接近 `/global_trajectory` | 待跑 |
| 中心静态障碍 | 局部轨迹绕开障碍 | 待跑 |
| 关闭 Frenet | 保持 `main` 的 LQR 行为 | 待跑 |
| 开启 Frenet | LQR 跟踪 `/local_trajectory` | 待跑 |

## 本次实现文件

- `code/pnc_rc/frenet/planner.py`：Frenet 多项式、占用栅格、候选轨迹评分。
- `code/pnc_rc/frenet/node.py`：ROS 节点，负责订阅消息和发布局部路径。
- `code/run_frenet_planner.py`：节点启动入口。
- `launch/pnc_sim_launch.py`：增加 Frenet 开关，并在开启时让 LQR 跟踪 `/local_trajectory`。
- `test/test_frenet_planner.py`：核心单元测试。
