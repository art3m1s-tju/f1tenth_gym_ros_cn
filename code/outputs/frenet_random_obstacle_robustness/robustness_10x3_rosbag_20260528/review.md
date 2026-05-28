# Frenet 随机障碍物鲁棒性测试复盘

批次：`robustness_10x3_rosbag_20260528`

本机输出目录：

```text
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528
```

## 实验设置

执行命令：

```bash
./run_frenet_random_obstacle_robustness.sh \
  --trials 10 \
  --laps 3 \
  --timeout 240 \
  --stuck-duration 3.0 \
  --target-speed 3.0 \
  --avoidance-speed 1.2 \
  --obstacle-count 2 \
  --obstacle-size 0.4 \
  --seed-start 0 \
  --batch-name robustness_10x3_rosbag_20260528
```

通过标准：每个随机种子至少完成 `3` 圈，中途不能碰撞，也不能出现超过 `3.0 s` 的停车或低速卡死。

每个 seed 都记录了：

- 批次根目录下的 `summary.csv` 和 `summary.json`。
- 每个 seed 的 `tracking.csv`、`global_trajectory.csv`、`launch.log`、生成地图文件。
- `ros2 bag record -a` 全 topic rosbag。10 个 seed 都有 `metadata.yaml` 和 sqlite `.db3` 数据。整个批次约 `13G`。

本次固定参数如下：

| 参数 | 数值 |
|---|---:|
| target_speed | `3.0 m/s` |
| avoidance_speed | `1.2 m/s` |
| obstacle_count | `2` |
| obstacle_size | `0.4 m` |
| timeout | `240 s` |
| stuck_duration | `3.0 s` |
| v_min / v_max / v_step | `0.84 / 3.75 / 0.75 m/s` |
| d_step | `0.35 m` |
| trajectory_dt | `0.1 s` |
| activation_max_m | `10.6 m` |
| activation_path_margin_m | `1.25 m` |
| activation_reaction_s | `1.2 s` |
| approach_extra_m | `0.75 m` |
| max_published_path_length_m | `6.6 m` |
| held_path_replan_clearance_m | `0.10 m` |
| held_path_min_remaining_m | `0.90 m` |
| centerline_return_lookahead_m | `3.5 m` |
| grid_forward_m | `20.0 m` |

`launch.log` 中确认的规划器关键参数还包括：

- `safe_clearance_m:=0.30`
- `min_clearance_m:=0.06`
- `corridor_radius_m:=0.20`
- `corridor_sample_step_m:=0.05`
- `path_collision_sample_step_m:=0.05`
- `footprint_front_m:=0.45`
- `footprint_rear_m:=0.05`
- `max_heading_jump:=0.85`
- `min_progress_step_m:=0.20`
- `reuse_last_candidate_timeout_s:=2.000`
- `candidate_profile_consistency_weight:=20.0`
- `candidate_profile_max_jump_m:=0.35`
- `candidate_channel_memory_timeout_s:=1.5`
- `stop_path_length_m:=2.0`

## 总体结果

总体结果：`8 / 10` 通过，通过率 `80%`。

| Seed | 结果 | 失败原因 | 完成圈数 | no-safe 次数 | 碰撞次数 | 最长 0 限速 | 最长低速段 | Rosbag 大小 |
|---:|---|---|---:|---:|---:|---:|---:|---:|
| 000 | 失败 | collision | 0 | 1 | 1 | `0.048 s` | `0.000 s` | `415 MB` |
| 001 | 通过 | - | 3 | 0 | 0 | `0.000 s` | `0.000 s` | `1186 MB` |
| 002 | 通过 | - | 3 | 0 | 0 | `0.000 s` | `0.000 s` | `1197 MB` |
| 003 | 通过 | - | 3 | 0 | 0 | `0.000 s` | `0.000 s` | `1155 MB` |
| 004 | 通过 | 短暂停车 | 3 | 5 | 0 | `1.360 s` | `1.284 s` | `1240 MB` |
| 005 | 通过 | 短暂停车 | 3 | 2 | 0 | `0.417 s` | `0.000 s` | `1106 MB` |
| 006 | 通过 | - | 3 | 0 | 0 | `0.000 s` | `0.000 s` | `1075 MB` |
| 007 | 通过 | - | 3 | 0 | 0 | `0.000 s` | `0.000 s` | `1068 MB` |
| 008 | 失败 | stuck_zero_speed_limit | 1 | 279 | 0 | `197.035 s` | `196.472 s` | `3246 MB` |
| 009 | 通过 | 短暂停车 | 3 | 1 | 0 | `0.480 s` | `0.000 s` | `1178 MB` |

## 失败样本：Seed 000 碰撞

障碍物中心：

- `(-7.7276745, -4.1904405)`
- `(8.9586213, 3.0543932)`

现象：

- `launch.log` 明确记录：`Ego collision detected at (9.363, 2.530)`。
- 最终 tracking 位姿约为 `(10.108, 3.255)`，距离障碍物中心 `(8.959, 3.054)` 约 `1.167 m`。
- 碰撞前规划器一直在很小的 clearance 附近工作：
  - held path clearance 多次只有 `0.09-0.18 m`。
  - 有一次 no-safe 时复用了 `clearance=0.09 m` 的 last safe path。
  - channel-consistent 选择有时会为了时间连续性或通道一致性，选择 clearance 更低的候选。
- 碰撞附近候选统计变成：
  - `total=110`
  - `safe=0`
  - `collision_reject=110`
  - `best_clearance=0.00m`
  - `occupied_cells=47266`

`tracking.csv` 在 `1779904970-1779904975` 时间窗口内的证据：

- `max_abs_e_y = 1.625 m`
- `max_abs_e_psi = 2.483 rad`
- `delta_cmd` 达到 `0.36 rad` 饱和
- `negative_v_path_count=51`
- 碰撞前局部限速主要仍是 `1.2 m/s`

判断：

这不是没有记录到数据或节点没启动导致的失败。地图、scan、规划器、控制器和 rosbag 都在工作。主要问题是近障碍物时鲁棒性不够：候选复用和 channel consistency 可能让车继续执行 clearance 很小的路径；一旦横向误差和航向误差放大，后续周期已经没有安全候选，车也已经进入不可恢复区域。

## 失败样本：Seed 008 长时间停车死锁

障碍物中心：

- `(9.0070681, -2.9905527)`
- `(9.3953596, 2.3834458)`

现象：

- harness 判定失败原因：`stuck_zero_speed_limit`。
- 只完成 `1` 圈。
- 最终 tracking 位姿约为 `(8.440, -3.031)`，距离最近障碍物中心 `(9.007, -2.991)` 约 `0.568 m`。
- 最长 0 限速持续 `197.035 s`，时间大约从 `1779905602.291` 到 `1779905799.327`。
- `launch.log` 长时间反复输出 `No safe Frenet candidate; publishing stop path`。
- 长时间卡死期间规划状态基本不变：
  - `s=34.84`
  - `d=0.30`
  - `s_dot=0.00`
  - `speed=0.00`
- 候选统计反复为：
  - `total=110`
  - `safe=0`
  - `collision_reject=110`
  - `best_clearance=0.00m`
  - `occupied_cells` 约 `43940-43966`

Rosbag 证据：

- `seed_008` bag 时长约 `241.7 s`
- message_count 为 `395667`
- 关键 topic 包括 `/scan`、`/ego_racecar/odom`、`/drive`、`/local_trajectory`、`/local_trajectory_speed_limit`、`/frenet/debug/candidates`、`/global_trajectory`、`/map`、`/tf`、`/tf_static`、`/rosout`

判断：

这是停车后的恢复能力问题。车停在距离障碍物很近的位置之后，所有向前 Frenet 候选都发生碰撞拒绝，于是规划器只能持续发布 0 速度 stop path。当前逻辑没有倒车、退让、扩大搜索、改变拓扑或专门 escape path，因此进入死锁。

## 通过但比较危险的样本

Seed `004`、`005`、`009` 虽然完成了 3 圈，但出现过短暂 no-safe fallback：

- Seed `004`：`5` 次 no-safe stop，最长 0 限速 `1.360 s`，最长低速段 `1.284 s`。
- Seed `005`：`2` 次 no-safe stop，最长 0 限速 `0.417 s`。
- Seed `009`：`1` 次 no-safe stop，最长 0 限速 `0.480 s`。

这些都小于 `3.0 s` 卡死阈值，所以这次判为通过。但它们说明当前规划器即使在成功样本中，也可能短暂进入全候选不可用状态。

## 当前鲁棒性问题清单

1. 近障碍物时实际安全裕度不足。

   Seed `000` 碰撞前 clearance 只有 `0.09-0.18 m`，同时 tracking 误差开始放大。只用静态 clearance 判断 held path 是否还能继续执行不够稳。

2. channel consistency 有可能保留 clearance 更差的选择。

   这个机制能减少左右摆动，但在障碍物很近时，它可能让车继续坚持一个不够安全的绕行侧。

3. stop fallback 没有逃逸能力。

   Seed `008` 证明一旦停在障碍物附近，所有前向候选都碰撞拒绝，规划器会一直停住。

4. 短暂 no-safe 事件需要单独作为鲁棒性指标。

   Seed `004`、`005`、`009` 都通过了，但都出现过 no-safe。它们不该被简单忽略，因为这类现象扩大后就可能变成碰撞或长时间死锁。

5. 当前日志足够定位失败类型，但还不足以直接决定最优修复策略。

   日志已经有 total/safe/collision/clearance/progress 统计和状态量；rosbag 也保留了完整 topic。下一步改算法前，最好先回放 seed `000` 和 seed `008` 的 rosbag，在 RViz 里确认候选、车辆姿态和障碍物关系。

## 下一轮算法修改建议

本次实验之后没有继续改规划/控制算法。基于固定算法测试结果，下一轮建议优先处理：

### 2026-05-28 追加诊断：dense candidate + map fix 后的新失败类型

在 `candidate_dense_mapfix_seed000` 复测中，seed `000` 的随机地图本身是合理的：

- 障碍物 1 位于 `(-7.728, -4.190)`，障碍物 2 位于 `(8.959, 3.054)`，大小均为 `0.4m`。
- 两个障碍物都在中心线附近，最近中心线距离分别约 `0.101m` 和 `0.090m`，属于有效的中心线阻塞测试。
- 碰撞点变为 `(-8.386, 5.231)`，不是随机障碍物位置，而是左上侧赛道外边界附近。

这次失败暴露出另一个问题：`d_step=0.10m`、`v_step=0.30m/s`、`T=1.5..6.0s` 组合后，单帧候选数达到 `3108`。日志显示一次 Frenet 规划耗时 `2.642s`，车辆在等待规划结果期间已经从 `x≈1.5` 前进到 `x≈-3.2`。因此发布出来的避障轨迹已经明显滞后，后续 held path 继续跟踪过期轨迹，最终撞到赛道外边界。

结论：seed `000` 这张地图不应被判为无效；当前失败主因是密集候选束在 Python/CPU 实现下实时性不足。下一步需要在保持 `d_step=0.10m` 横向细度的前提下，减少 `T/v` 组合数，优先保留短时域和低速终点，并用 seed `000` 验证规划耗时和避障结果。

### 2026-05-28 追加诊断：参数折中后的剩余问题

后续对 seed `000` 做了几组只改参数的快速复测：

- `candidate_fast_seed000`：`d_step=0.10m`、`v_step=0.80m/s`、`T=1.0..3.0s` 后，单周期候选数降到 `666`，规划耗时约 `1.0s`，不再碰撞，但车辆停在第一个真实障碍前约 `0.96m`，长期 `No safe Frenet candidate`，最终 `stuck_zero_speed_limit`。
- `candidate_fast_mapstd_seed000`：确认 ROS OccupancyGrid 仍按标准 bottom-up 行序转换，避免把障碍镜像到上半圈；该设置下可以通过第一处障碍，但第二处障碍附近仍可能因触发较晚和跟踪误差放大而碰撞。
- `candidate_fast_early_seed000`：增大 threat/activation lookahead 后不再碰撞，但仍会在第一处障碍前进入长期 no-safe 停车。
- `candidate_fast_t4_seed000`：把 `T` 扩到 `1.0..4.0s` 可增加可行候选，但单周期耗时回到 `1.3s+`，第一条轨迹发布时车辆已接近障碍，跟踪误差放大并碰撞。

当前结论是：单靠参数在“实时性”和“可行绕障空间”之间出现明显拉扯。`T<=3s` 实时性较好但容易近障无解，`T=4s` 可行性提升但 CPU/Python 耗时过长。下一步先不引入 CUDA，把候选轨迹生成和主循环中低风险的逐点 Python 逻辑改为 NumPy 向量化，先确认 CPU 路径还能释放多少实时性余量。

### 2026-05-28 追加诊断：回滚 CUDA 原型，改走 CPU NumPy 向量化

当前 CUDA/Numba 原型已移除，代码中不再保留 `FRENET_USE_CUDA`、`cuda`、`numba` 等开关或依赖。现阶段优化方向改为 CPU NumPy 向量化，避免在当前容器没有 CUDA 设备的情况下引入额外部署复杂度。

已完成的向量化改动：

- 候选 Frenet 多项式 profile 生成改为批量 NumPy 计算，候选顺序保持原来的 `d_final -> duration -> speed_final`。
- `plan_frenet_path()` 先批量计算 progress mask，再只遍历满足前进性硬约束的候选，减少无效候选后续几何检查。
- `ReferencePath.sample_many()` 批量执行 `(s, d) -> xy` 采样，替代候选内部逐点调用 `reference.sample()`。
- `estimate_heading_jumps()` 改为 `np.diff()` 加 `atan2(sin, cos)`，去掉航向差上的 Python list comprehension。

轨迹 profile 生成 benchmark：

```bash
docker run --rm --entrypoint /bin/bash \
  -v /home/art3m1s/f1tenth_frenet_static_avoidance:/sim_ws/src/f1tenth_gym_ros \
  -w /sim_ws/src/f1tenth_gym_ros \
  f1tenth_gym_ros:latest \
  -lc 'source /opt/ros/foxy/setup.bash && python3 code/bench_frenet_trajectory_generation.py --repeat 50'
```

结果：

- candidate grid: `d=37, T=3, v=6, total=666`
- scalar Python profile generation: `16.853 ms +/- 3.400`
- NumPy batch profile generation: `0.769 ms +/- 0.057`
- speedup: `21.93x`

验证：

- `test/test_frenet_planner.py`
- `test/test_frenet_random_robustness.py`
- `test/test_frenet_node.py`
- `test/test_frenet_trajectory_generation.py`
- 结果：`51 passed`

剩余瓶颈判断：profile 生成已经不是主要问题。下一步真正影响整帧规划耗时的地方大概率是每条候选的 `occupancy.query_path()`，其中包含路径加密、扫掠通道采样、坐标转换、栅格碰撞和 clearance 查询。后续如果还要继续提速，应优先设计批量 `query_paths`，一次处理多条候选并按候选分组汇总 collision/min_clearance。

### 2026-05-28 追加复测：seed 001-003 当前 NumPy 版本

复测命令：

```bash
./run_frenet_random_obstacle_robustness.sh \
  --trials 3 \
  --laps 3 \
  --timeout 180 \
  --stuck-duration 3.0 \
  --target-speed 3.0 \
  --avoidance-speed 1.2 \
  --obstacle-count 2 \
  --obstacle-size 0.4 \
  --seed-start 1 \
  --batch-name numpy_vectorized_seeds001_003_baseline \
  --no-record-rosbag
```

结果：

| Seed | 结果 | 失败原因 | 规划周期数 | 规划耗时 min/mean/p95/max | no-safe 日志数 |
|---:|---|---|---:|---|---:|
| 001 | 失败 | `stuck_zero_speed_limit` | 212 | `0.509 / 0.754 / 0.802 / 0.967 s` | 203 |
| 002 | 失败 | `collision` | 6 | `0.725 / 0.858 / 0.910 / 1.073 s` | 1 |
| 003 | 失败 | `collision` | 4 | `0.761 / 0.849 / 0.821 / 1.014 s` | 1 |

seed `001` 的随机地图与旧 `robustness_10x3_rosbag_20260528` 中通过的 seed `001` 完全相同，障碍物仍为 `(-8.7547922, 3.3017473)` 和 `(-4.44, -5.0)`。旧实验中 seed `001` 通过，规划候选数为 `110`，规划耗时 `0.133 / 0.449 / 0.575 s`；当前 NumPy 版本默认候选数为 `666`，规划耗时变为 `0.509 / 0.754 / 0.967 s`，并长期进入 no-safe stop。

额外试验：临时把高速 preset 的 `v_step` 从 `0.80` 加密到 `0.60`，seed `001` 候选数变为 `777`，规划耗时 `0.541 / 0.638 / 0.772 s`，但 `10.1 s` 内发生碰撞，未改善鲁棒性。因此该加密参数没有保留。

结论：当前 profile 生成的 NumPy 向量化还不足以支撑 666 条候选的实时规划。整帧耗时仍主要被每条候选的世界坐标轨迹检查和占据栅格碰撞查询限制；继续增加采样密度会扩大风险，不应作为下一步主方向。下一步应优先优化或批量化 `occupancy.query_path()`，或者在保持实时性的前提下回退到更小候选束并单独处理无解恢复。

1. 增加停车近障碍物时的 recovery mode。

   可选方向包括低速倒车/后退、临时扩大横向搜索、解除某些前向进度约束，或者专门生成 escape path。Seed `008` 应作为主要回归测试。

2. 对 held path reuse 增加更严格的组合门限。

   不只看 clearance，还要同时看 lateral error、heading error、剩余路径长度、障碍物距离，以及这些误差是否正在变大。Seed `000` 是主要回归测试。

3. 重新平衡 channel consistency 和 clearance。

   当 raw candidate 和 channel-consistent candidate 的 clearance 差异已经影响安全时，不应该继续优先保留通道一致性。

4. 把短暂 no-safe 事件纳入自动化指标。

   即使实验最终完成 3 圈，也应该记录 no-safe 次数、最长 0 限速、最长低速段，并作为鲁棒性评分的一部分。

5. 先回放失败 rosbag，再改参数或逻辑。

   重点回放：

   ```text
   code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_000/logs/rosbag/seed_000
   code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_008/logs/rosbag/seed_008
   ```

## 数据路径

批次汇总：

```text
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/summary.csv
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/summary.json
```

失败样本日志：

```text
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_000/logs/launch.log
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_000/logs/tracking.csv
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_008/logs/launch.log
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_008/logs/tracking.csv
```

Rosbag 根目录：

```text
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_000/logs/rosbag/seed_000
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_001/logs/rosbag/seed_001
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_002/logs/rosbag/seed_002
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_003/logs/rosbag/seed_003
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_004/logs/rosbag/seed_004
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_005/logs/rosbag/seed_005
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_006/logs/rosbag/seed_006
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_007/logs/rosbag/seed_007
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_008/logs/rosbag/seed_008
code/outputs/frenet_random_obstacle_robustness/robustness_10x3_rosbag_20260528/trials/seed_009/logs/rosbag/seed_009
```
