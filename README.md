# F1TENTH Gym ROS 2 仿真环境

基于 ROS 2 Foxy + Docker 的 F1TENTH 赛车仿真，支持 RViz 可视化、键盘遥控、多车对抗。

> 上游仓库：[f1tenth/f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros)
> 本仓库针对国内网络做了 apt/pip 换源优化。

---

## 环境准备

- Linux（推荐 Ubuntu 22.04）
- Docker ≥ 20.10
- [rocker](https://github.com/osrf/rocker)（用于带图形界面和 GPU 加速的容器运行工具）
- NVIDIA 显卡 + `nvidia-container-toolkit`（可选，无 N 卡后续启动不加 `--nvidia` 参数即可）

### 依赖安装指南 (Ubuntu)

#### 1. 安装 Docker
```bash
# 1. 卸载旧版本（如果有）
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done

# 2. 设置 apt 仓库
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# 3. 添加仓库到 apt 源
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# 4. 安装最新版本 Docker
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# 5. 把当前用户加进 docker 组（免 sudo 运行 docker）
sudo usermod -aG docker $USER && newgrp docker
```

#### 2. 安装 NVIDIA Container Toolkit (仅 N 卡需要)
用于在 Docker 中透传 GPU 资源：
```bash
# 1. 配置仓库
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# 2. 更新并安装
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# 3. 配置 Docker 使用 NVIDIA 运行时并重启服务
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

#### 3. 安装 Rocker
```bash
sudo apt update
sudo apt install python3-rocker -y
# 或者使用 pip 安装： pip install rocker
```

---

## 快速开始

### 1. 克隆并重命名

```bash
git clone https://github.com/art3m1s-tju/f1tenth_gym_ros_cn.git
mv f1tenth_gym_ros_cn f1tenth_gym_ros
cd f1tenth_gym_ros
```

### 2. 构建镜像

```bash
DOCKER_BUILDKIT=1 docker build -t f1tenth_gym_ros -f Dockerfile .
```

首次构建约 10–20 分钟，镜像约 2.7 GB。

### 3. 启动仿真（终端 1）

```bash
rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros
```

无 N 卡：`rocker --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros`

进入容器后：

```bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### 4. 键盘遥控（终端 2）

```bash
docker exec -it $(docker ps -q) /bin/bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

键位：`i`/`,` 前进/倒车，`j`/`l` 左转/右转，`k` 刹车，`q`/`z` 调速。保持该终端焦点才能捕获按键。

---

## 配置

编辑 `config/sim.yaml`，重启 launch 生效。

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `map_path` | 地图路径（容器内绝对路径，不含扩展名） | `/sim_ws/src/f1tenth_gym_ros/maps/levine` |
| `num_agent` | 对手数（0=单车，1=对抗） | `1` |
| `sx`, `sy`, `stheta` | 主车初始位姿 | `0, 0, 0` |
| `scan_beams` | LiDAR 波束数 | `1080` |
| `kb_teleop` | 启用键盘遥控 | `True` |

切换地图只需改 `map_path`，自带 `maps/levine` 和 `maps/Spielberg_map`。自定义地图放同名 `.png` + `.yaml` 到 `maps/` 即可。

---

## 运行自定义规划控制算法

仓库内置了基于 LQR 的路径跟踪控制器和 control_friendly 轨迹规划器（位于 `code/` 目录），可以在仿真中测试闭环控制。

### 1. 生成赛道边界数据（首次运行）

进入容器后执行：

```bash
python3 /sim_ws/src/f1tenth_gym_ros/code/generate_track.py
```

脚本会从 `maps/my_map.pgm` 提取赛道内外边界，输出 `code/outputs/csv/processed_track.csv`。运行结束后会打印建议的初始位姿，将 `sx`/`sy`/`stheta` 更新到 `config/sim.yaml`。

### 2. 启动仿真 + 规划 + LQR 控制

```bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
ros2 launch f1tenth_gym_ros pnc_sim_launch.py
```

该 launch 会同时启动仿真器、轨迹规划器和 LQR 控制器。小车会自动沿规划轨迹行驶。

### 控制器参数调整

在 `launch/pnc_sim_launch.py` 中直接修改 LQR 参数：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `target_speed` | 目标速度 (m/s) | `1.0` |
| `lqr_q_lateral` | 横向误差权重（越大跟踪越紧） | `3.0` |
| `lqr_q_heading` | 航向误差权重 | `1.2` |
| `lqr_r_steering` | 转向代价（越大转向越平滑） | `8.0` |
| `max_lateral_accel` | 最大横向加速度限制 (m/s^2) | `4.0` |

详细调参指南见 [parameters.md](parameters.md)。

### 一键运行 + 评估

使用 `run_pnc_sim.sh` 可以一键启动仿真并在 Ctrl+C 停止后自动评估：

```bash
./run_pnc_sim.sh
```

脚本顶部可直接修改 LQR 参数和规划参数。仿真结束后自动调用评估脚本，结果保存到带参数命名的文件夹。

### 误差评估

跑完一轮控制后，在容器内执行：

```bash
python3 /sim_ws/src/f1tenth_gym_ros/code/tracker_evaluate.py \
  --log /sim_ws/src/f1tenth_gym_ros/code/outputs/logs/lqr_tracking_log.csv \
  --reference-trajectory-csv /sim_ws/src/f1tenth_gym_ros/code/outputs/csv/global_trajectory.csv \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/evaluation
```

输出指标包括：
- 横向误差：mean / P95 / max（单位 m）
- 航向误差：mean / P95 / max（单位 deg）
- 可视化图表保存在 `code/outputs/evaluation/`

### 评估文件夹命名规则

评估输出目录按 **关键参数 + 时间戳** 自动命名，格式为：

```
v{速度}_Ql{横向权重}_Qh{航向权重}_R{转向代价}_ff{前馈增益}_{YYYYMMDD_HHMMSS}
```

示例：

```
code/outputs/evaluation/v1.0_Ql3.0_Qh1.2_R8.0_ff1.0_20260515_143022/
```

这样可以直观对比不同参数组合的评估结果，无需手动重命名。

### 参数扫描（Parameter Sweep）

仓库提供了离线参数扫描工具 `code/lqr_sweep/`，可自动搜索不同速度下的最优 LQR 参数组合。

#### 3.0m/s 可行标准赛道

仓库提供了一个低曲率 stadium 测试赛道，用于标定 LQR 的 `Q/R/ff` 速度查找表：

```bash
cd /sim_ws/src/f1tenth_gym_ros
python3 code/lqr_sweep/generate_feasible_track.py \
  --open-map \
  --map-prefix maps/stadium_3ms_open \
  --trajectory-csv code/outputs/generated_tracks/stadium_3ms_trajectory.csv \
  --track-csv code/outputs/generated_tracks/stadium_3ms_processed_track.csv
```

输出：

```text
maps/stadium_3ms_open.pgm
maps/stadium_3ms_open.yaml
code/outputs/generated_tracks/stadium_3ms_trajectory.csv
code/outputs/generated_tracks/stadium_3ms_processed_track.csv
code/outputs/generated_tracks/stadium_3ms_open_preview.png
```

该赛道最大曲率约 `0.25 1/m`，3.0m/s 时横向加速度约 `2.25 m/s^2`，适合做低/中/高速 LQR 参数表标定。推荐起始位姿：

```text
sx=0.0, sy=4.0, stheta=3.1416
```

使用该标准赛道做 lookup table 扫描：

```bash
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

标准赛道使用开放空白地图，只用于标定 LQR 表；虚拟左右边界只参与可视化和误差理解，不参与碰撞。扫表时必须关闭曲率限速和速度斜坡，保证全程定速。得到 LQR 表后，再回到原赛道开启曲率限速/预瞄限速做完整系统验证。

低速多圈会显著增加仿真时间。stadium 赛道一圈约 41m，0.5m/s 跑 3 圈理论上需要约 247s，因此标准赛道示例显式设置 `--max-sim-time 420`。不传该参数时脚本会根据轨迹长度、圈数和最低测试速度自动估算。

#### 推荐测试流程

先用高速点做一个较小规模的 smoke test，确认 5 圈稳定性和约束筛选正常：

```bash
cd /sim_ws/src/f1tenth_gym_ros/code
python3 -m lqr_sweep.run_sweep --mode coarse-only \
  --speeds 2.5 3.0 \
  --coarse-grid 4,3,4,3 \
  --laps 5 \
  --max-lateral-accel 4.0 \
  --max-accel 1.0 \
  --max-decel 2.0 \
  --max-workers 4 \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_smoke
```

如果 smoke test 结果合理，再运行完整扫描生成正式增益表。

#### 完整扫描

```bash
cd /sim_ws/src/f1tenth_gym_ros/code
python3 -m lqr_sweep.run_sweep --mode full \
  --speeds 0.5 1.0 1.5 2.0 2.5 3.0 \
  --coarse-grid 5,4,5,3 \
  --laps 5 \
  --max-lateral-accel 4.0 \
  --max-accel 1.0 \
  --max-decel 2.0 \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep
```

流程：对每个速度点先做粗网格搜索（5×4×5×3 = 300 组合），再对 top-3 做局部细化（每个 3^4 = 81 组合）。每组参数默认连续跑 5 圈，最终输出增益查找表。

离线 sweep 默认启用与 ROS 控制器一致的纵向速度斜坡：`--max-accel 1.0`、`--max-decel 2.0`。若要临时关闭，可加 `--disable-speed-ramp`。

#### 仅粗网格（快速预览）

```bash
python3 -m lqr_sweep.run_sweep --mode coarse-only \
  --speeds 1.0 2.0 \
  --coarse-grid 4,3,4,2 \
  --laps 5 \
  --max-lateral-accel 4.0 \
  --max-accel 1.0 \
  --max-decel 2.0 \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep
```

#### 单组参数验证

```bash
python3 -m lqr_sweep.run_sweep --mode single \
  --speed 1.5 \
  --laps 5 \
  --max-lateral-accel 4.0 \
  --max-accel 1.0 \
  --max-decel 2.0 \
  --q-lateral 5.0 --q-heading 2.0 --r-steering 10.0 --feedforward-gain 0.9
```

#### 验证已有增益表

```bash
python3 -m lqr_sweep.run_sweep --mode validate \
  --laps 5 \
  --max-lateral-accel 4.0 \
  --max-accel 1.0 \
  --max-decel 2.0 \
  --table /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep/lqr_gain_table.yaml
```

#### 完整 ROS 多圈验证

离线 `--mode validate` 只使用 Python gym harness。若要启动完整 ROS 链路（仿真器 + 规划器 + LQR 控制器）验证新表，执行：

```bash
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

原地图验证默认开启曲率限速和速度斜坡，并默认使用 `/ego_racecar/odom` 位姿
（`use_tf_pose=false`），这样可以先排除 TF 链路不完整导致 LQR 不发 `/drive`
的问题。若要显式使用 TF 位姿，可加 `--use-tf-pose`。

#### 原地图 RViz 批量验证

阶段一推荐使用批量 RViz 验证脚本，从 `0.5m/s` 到 `3.0m/s` 每隔 `0.5m/s`
测试一组。每组运行 300 秒，自动保存 launch 输出、tracking log、评估 summary 和可视化图片。

注意：`--mode batch` 是阶段一分支 `stage/original-map-validate` 里的新功能。
如果你还在主目录 `/home/art3m1s/f1tenth_gym_ros` 启动容器，容器里会挂载 main
工作区，旧版 `validate_ros.py` 不认识 `--mode`、`--speeds`、`--batch-name`。

测试阶段一时需要从阶段一 worktree 启动容器：

```bash
cd /home/art3m1s/f1tenth_stage1_original_map_validate
rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros
```

无 N 卡则使用：

```bash
cd /home/art3m1s/f1tenth_stage1_original_map_validate
rocker --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros
```

进入容器后再运行批量验证：

```bash
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
  --noise-profile clean \
  --disable-rviz \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/evaluation_ros \
  --batch-name original_map_table_curvlimit_ramp_0p5_to_3p0_300s
```

如果要做更接近实车的鲁棒性验证，保持同一套参数和赛道，额外跑一组轻量噪声/延迟：

```bash
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
  --noise-profile light \
  --noise-seed 42 \
  --disable-rviz \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/evaluation_ros \
  --batch-name original_map_table_curvlimit_ramp_0p5_to_3p0_300s
```

`--noise-profile clean` 不注入噪声；`--noise-profile light` 会给控制器看到的位姿加入 `2cm`
位置噪声、`1deg` 航向噪声和 `60ms` 位姿延迟。评估仍使用真实 odom 日志，所以比较的是
“带噪声控制之后实际轨迹变差多少”。也可以用 `--position-noise-std`、
`--heading-noise-std-deg`、`--pose-delay-ms` 单独覆盖默认值。

输出目录结构示例：

```text
code/outputs/evaluation_ros/original_map_table_curvlimit_ramp_0p5_to_3p0_300s/
├── clean/
│   ├── manifest.csv
│   └── v0p5_table_stadium_curv1_ramp1_alat4p0_clean_YYYYmmdd_HHMMSS/
│       ├── logs/
│       │   ├── ..._launch.log
│       │   ├── ..._evaluator.log
│       │   └── ..._tracking.csv
│       └── evaluation/
│           ├── lookahead_summary.csv
│           ├── lookahead_summary.json
│           └── *_lateral_error.png / *_heading_error.png / *_path_overlay.png
└── noisy_light_pos2cm_yaw1deg_delay60ms/
    ├── manifest.csv
    └── v0p5_table_stadium_curv1_ramp1_alat4p0_noisy_light_pos2cm_yaw1deg_delay60ms_YYYYmmdd_HHMMSS/
        ├── logs/
        │   ├── ..._launch.log
        │   ├── ..._evaluator.log
        │   └── ..._tracking.csv
        └── evaluation/
            ├── lookahead_summary.csv
            ├── lookahead_summary.json
            └── *_lateral_error.png / *_heading_error.png / *_path_overlay.png
```

这里 `--laps 99` 的作用是不要因为完成几圈就提前停止，而是尽量跑满 `--timeout 300`
秒。若希望完成指定圈数后自动停止，把 `--laps` 改成目标圈数即可。

如果临时把 `--timeout` 改成 `150`，建议把 `--batch-name` 也同步改成：

```text
original_map_table_curvlimit_ramp_0p5_to_3p0_150s
```

这样后续看结果目录时不会把 150 秒测试误认为 300 秒测试。

如果只需要自动统计和出图，可以加 `--disable-rviz` 关闭 RViz 渲染。若
`tracker_evaluate.py` 失败，脚本会在对应 `evaluation/` 目录写入
`evaluation_failed.txt`，并在 `logs/*_evaluator.log` 中保存失败原因，不再只留下空目录。

#### 第二阶段：预瞄限速 + 转角速率限制验证

第二阶段用于验证原地图高速段，重点降低 `3.0m/s` 下过大的转角变化率。需要从
stage2 worktree 启动容器，确保容器内挂载的是当前阶段代码：

```bash
cd /home/art3m1s/f1tenth_stage2_speed_planning
rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros
```

进入容器后先跑 clean 验证：

```bash
cd /sim_ws/src/f1tenth_gym_ros/code
python3 -m lqr_sweep.validate_ros \
  --mode batch \
  --table /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_stadium/lqr_gain_table.yaml \
  --speeds 1.5 2.0 2.5 3.0 \
  --timeout 100 \
  --laps 99 \
  --track-csv /sim_ws/src/f1tenth_gym_ros/code/outputs/csv/processed_track.csv \
  --trajectory-csv /sim_ws/src/f1tenth_gym_ros/code/outputs/csv/global_trajectory.csv \
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

如果想确认转角速率限制本身带来的变化，可以保持其它参数不变，关掉 rate limit 做对照：

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

如果关掉 rate limit 后 `steering_rate` 反而下降，说明前一轮锯齿主要来自硬限幅。Stage2
后续还修复了一个速度一致性问题：曲率限速降低 `v_cmd/v_actual` 后，LQR 模型速度和 lookup table
插值也会跟随实际/命令速度，而不是继续按 `target_speed` 计算。因此改完代码后先重跑
`max_lateral_accel=3.5` 作为新基线：

```bash
python3 -m lqr_sweep.validate_ros \
  --mode batch \
  --table /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_stadium/lqr_gain_table.yaml \
  --speeds 2.0 2.5 3.0 \
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
  --batch-name stage2_speed_consistent_preview1p0_no_rate_limit_alat3p5_clean_100s
```

然后保持 rate limit 关闭，测试更保守的曲率速度规划：

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
  --batch-name stage2_speed_consistent_preview1p0_no_rate_limit_alat3p0_clean_100s
```

如果 `3.0m/s` 仍然频繁打满转角，再试 `max_lateral_accel=2.5`：

```bash
python3 -m lqr_sweep.validate_ros \
  --mode batch \
  --table /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_stadium/lqr_gain_table.yaml \
  --speeds 2.5 3.0 \
  --timeout 100 \
  --laps 99 \
  --min-speed 0.4 \
  --max-lateral-accel 2.5 \
  --curvature-speed-lookahead-m 1.0 \
  --max-accel 1.0 \
  --max-decel 3.0 \
  --disable-steering-rate-limit \
  --noise-profile clean \
  --disable-rviz \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/evaluation_ros \
  --batch-name stage2_preview1p0_no_rate_limit_alat2p5_clean_100s
```

新日志字段包括 `curvature_preview`、`delta_raw`、`delta_rate_limited`。评估时重点比较：
`delta_cmd` 峰值、`steering_rate_rms`、`delta_rate p95/max`、`steering_saturation_ratio`、
`p95_abs_e_y` 和 `max_abs_e_y`。如果转角变平滑但横向误差明显变差，优先降低
`max_lateral_accel` 或把 `curvature-speed-lookahead-m` 从 `1.0` 增加到 `1.5`，最后再放宽
`max-steering-rate`。

如果 clean 通过但 `--noise-profile light` 明显恶化，不要直接调 QR，先拆分噪声来源。下面三组命令都使用
速度一致性修复后的基线参数，并关闭 steering rate limit。

delay-only：只测试 `60ms` 位姿延迟。

```bash
python3 -m lqr_sweep.validate_ros \
  --mode batch \
  --table /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_stadium/lqr_gain_table.yaml \
  --speeds 2.0 2.5 3.0 \
  --timeout 100 \
  --laps 99 \
  --min-speed 0.4 \
  --max-lateral-accel 3.5 \
  --curvature-speed-lookahead-m 1.0 \
  --max-accel 1.0 \
  --max-decel 3.0 \
  --disable-steering-rate-limit \
  --noise-profile clean \
  --pose-delay-ms 60 \
  --disable-rviz \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/evaluation_ros \
  --batch-name stage2_ablation_delay60ms_alat3p5_clean_100s
```

position-only：只测试 `2cm` 位置噪声。

```bash
python3 -m lqr_sweep.validate_ros \
  --mode batch \
  --table /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_stadium/lqr_gain_table.yaml \
  --speeds 2.0 2.5 3.0 \
  --timeout 100 \
  --laps 99 \
  --min-speed 0.4 \
  --max-lateral-accel 3.5 \
  --curvature-speed-lookahead-m 1.0 \
  --max-accel 1.0 \
  --max-decel 3.0 \
  --disable-steering-rate-limit \
  --noise-profile clean \
  --position-noise-std 0.02 \
  --disable-rviz \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/evaluation_ros \
  --batch-name stage2_ablation_pos2cm_alat3p5_clean_100s
```

heading-only：只测试 `1deg` 航向噪声。

```bash
python3 -m lqr_sweep.validate_ros \
  --mode batch \
  --table /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_stadium/lqr_gain_table.yaml \
  --speeds 2.0 2.5 3.0 \
  --timeout 100 \
  --laps 99 \
  --min-speed 0.4 \
  --max-lateral-accel 3.5 \
  --curvature-speed-lookahead-m 1.0 \
  --max-accel 1.0 \
  --max-decel 3.0 \
  --disable-steering-rate-limit \
  --noise-profile clean \
  --heading-noise-std-deg 1.0 \
  --disable-rviz \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/evaluation_ros \
  --batch-name stage2_ablation_yaw1deg_alat3p5_clean_100s
```

误差滤波验证：在 yaw/position/full-light 噪声下打开 `e_y/e_psi` 低通滤波。评估仍使用真实
`e_y/e_psi`，新增日志字段 `control_e_y/control_e_psi/filtered_e_y/filtered_e_psi`
用于查看滤波前后的控制误差。建议先用 `alpha_y=0.30`、`alpha_psi=0.25`。

```bash
python3 -m lqr_sweep.validate_ros \
  --mode batch \
  --table /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_stadium/lqr_gain_table.yaml \
  --speeds 2.0 2.5 3.0 \
  --timeout 100 \
  --laps 99 \
  --min-speed 0.4 \
  --max-lateral-accel 3.5 \
  --curvature-speed-lookahead-m 1.0 \
  --max-accel 1.0 \
  --max-decel 3.0 \
  --disable-steering-rate-limit \
  --noise-profile light \
  --noise-seed 42 \
  --enable-error-filter \
  --error-filter-alpha-y 0.30 \
  --error-filter-alpha-psi 0.25 \
  --disable-rviz \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/evaluation_ros \
  --batch-name stage2_filter_light_alphaY0p30_alphaPsi0p25_100s
```

若只想拆分滤波效果，把上面命令中的 `--noise-profile light` 换成：

```bash
--noise-profile clean --position-noise-std 0.02
```

或：

```bash
--noise-profile clean --heading-noise-std-deg 1.0
```

如果 delay-only 就明显变差，优先处理延迟补偿、降低高速上限或增大曲率预瞄；如果 position-only
明显变差，优先滤波 `x/y` 或 `e_y`；如果 heading-only 明显变差，优先滤波 yaw 或 `e_psi`。

Stage2 当前推荐基线：

```text
max_lateral_accel = 3.0
curvature_speed_lookahead_m = 2.0
enable_error_filter = true
error_filter_alpha_y = 0.30
error_filter_alpha_psi = 0.25
enable_steering_rate_limit = false
```

极限 RViz 可视化检查用 `3.0m/s + light noise` 跑一组单次测试。这里故意不加
`--disable-rviz`，用于肉眼观察高速入弯、减速时机、车身姿态、转角饱和和蛇形：

```bash
python3 -m lqr_sweep.validate_ros \
  --mode single \
  --table /sim_ws/src/f1tenth_gym_ros/code/outputs/sweep_stadium/lqr_gain_table.yaml \
  --speed 3.0 \
  --timeout 100 \
  --laps 99 \
  --min-speed 0.4 \
  --max-lateral-accel 3.0 \
  --curvature-speed-lookahead-m 2.0 \
  --max-accel 1.0 \
  --max-decel 3.0 \
  --disable-steering-rate-limit \
  --noise-profile light \
  --noise-seed 42 \
  --enable-error-filter \
  --error-filter-alpha-y 0.30 \
  --error-filter-alpha-psi 0.25 \
  --output-dir /sim_ws/src/f1tenth_gym_ros/code/outputs/evaluation_ros/stage2_rviz_limit_baseline_v3p0_light
```

看 RViz 时重点观察：

- 入弯前是否已经开始降速，而不是到了弯心才降；
- `delta_cmd` 是否长时间贴近 `±0.36rad`；
- 车尾/车头是否出现肉眼可见左右摆动；
- 如果 3.0m/s 看起来紧张，实车首轮应从 `2.0m/s` 或 `2.5m/s` 开始。

#### 扫描时间估算

| 模式 | 速度点数 | 网格规模 | 预计耗时 |
|------|---------|---------|---------|
| `full`（默认 5,4,5,3，5 圈） | 6 | 300 + 243/速度点 | 1–3 h |
| `coarse-only`（4,3,4,2，5 圈） | 2 | 96/速度点 | 10–30 min |
| `single` | 1 | 1 | 30–90 s |
| `validate` | 按表 | 按表条目数 | 5–10 min |

实际耗时取决于赛道长度和 CPU 核数（`--max-workers` 控制并行度）。

#### 扫描输出

```
code/outputs/sweep/
├── lqr_gain_table.yaml    # 速度-增益查找表（可直接用于控制器）
└── sweep_summary.json     # 扫描元数据（耗时、网格配置、各速度点最优参数）
```

#### 低速航向优化 sweep

如果原地图验证出现低速横向误差能通过、但航向误差偏大的情况，可以使用低速航向优化
profile。该 profile 会提高 `mean/p95(e_psi)` 在目标函数中的权重，并默认使用
`q_heading=2.0~12.0`、`R=3.0~15.0`，避免低速段总选到过低航向权重和过保守转向。

```bash
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

如果需要更明确地限制搜索范围，也可以手动覆盖：

```bash
--q-heading-range 2.0,12.0 --r-steering-range 3.0,15.0
```

### RViz 可视化

启动后在 RViz 中 Add Display：
- `/global_trajectory`（Path）— 参考轨迹（绿色）
- `/tracked_path_lqr`（Path）— 实际行驶轨迹（红色）
- `/ego_racecar/odom`（Odometry）— 实时位姿

### 文件结构

```
code/
├── generate_track.py     # 离线赛道边界提取
├── run_planner.py        # 规划器入口
├── run_lqr.py            # LQR 控制器入口
├── planner.py            # control_friendly 轨迹规划
├── pnc_rc/lqr/
│   ├── controller.py     # LQR 路径跟踪控制器
│   └── math.py           # LQR 数学计算
└── outputs/              # 运行时输出（轨迹CSV、日志）
```

### 话题映射

| 话题 | 类型 | 说明 |
|------|------|------|
| `/global_trajectory` | `nav_msgs/Path` | 规划器发布的全局轨迹 |
| `/ego_racecar/odom` | `Odometry` | 仿真器发布，LQR 订阅 |
| `/drive` | `AckermannDriveStamped` | LQR 发布，仿真器订阅 |

---

## 话题（仿真器原生）

| 话题 | 类型 | 方向 |
|------|------|------|
| `/drive` | `AckermannDriveStamped` | 订阅（控制） |
| `/ego_racecar/scan` | `LaserScan` | 发布（LiDAR） |
| `/ego_racecar/odom` | `Odometry` | 发布（里程计） |
| `/map` | `OccupancyGrid` | 发布 |

多车模式下对手用 `opp_racecar` 前缀。

---

## 常见问题

| 问题 | 解决 |
|------|------|
| apt Hash Sum mismatch | 已换阿里云源，若仍失败可换清华源：`sed -i 's@mirrors.aliyun.com@mirrors.tuna.tsinghua.edu.cn@g' Dockerfile` |
| pip 报 invalid metadata | 已锁定 pip < 24.1，正常构建即可 |
| No map received | 检查 `sim.yaml` 的 `map_path` 是容器内绝对路径，不带 `.png` |
| RViz 白屏/OpenGL 错误 | 确认 `rocker` 带了 `--nvidia --x11`，宿主机 `nvidia-smi` 正常 |
| 键盘没反应 | 保持 teleop 终端窗口焦点 |
| generate_track.py 报错 | 确认地图中有清晰的环形赛道结构（内外墙闭合） |
| LQR 控制器无输出 | 确认 `generate_track.py` 已运行且 `code/outputs/csv/processed_track.csv` 存在 |
| 改了源码要重编译 | 容器内：`cd /sim_ws && colcon build && source install/local_setup.bash` |

---

## 许可证

MIT License。引用：

```bibtex
@inproceedings{okelly2020f1tenth,
  title={F1TENTH: An Open-source Evaluation Environment for Continuous Control and Reinforcement Learning},
  author={O'Kelly, Matthew and Zheng, Hongrui and Karthik, Dhruv and Mangharam, Rahul},
  booktitle={NeurIPS 2019 Competition and Demonstration Track},
  pages={77--89},
  year={2020},
  organization={PMLR}
}
```
