# F1TENTH Gym ROS 2 仿真环境（中文版）

基于 ROS 2 Foxy 和 Docker 的 F1TENTH 开源赛车仿真环境，支持 RViz 可视化、键盘遥控、多车对抗，适合控制算法、路径规划和强化学习的开发与验证。

> 上游仓库：[f1tenth/f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros)
> 本仓库针对国内网络环境做了 apt / pip 换源，优化了 Docker 构建成功率。

---

## 系统要求

- Linux（推荐 Ubuntu 22.04）
- Docker（≥ 20.10）
- [rocker](https://github.com/osrf/rocker)，用于把 X11 和 NVIDIA GPU 透传进容器
- NVIDIA 显卡 + `nvidia-container-toolkit`（可选，但强烈推荐；无 N 卡可去掉 `--nvidia` 参数走软件渲染）

### 安装 rocker 和 NVIDIA 支持

```bash
# rocker
sudo apt install python3-rocker
# 或：pip install rocker

# NVIDIA Container Toolkit（有 N 卡才需要）
# 参考 https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html
```

安装后确认 docker 能识别 nvidia runtime：
```bash
docker info | grep -i runtime
# 应该看到：Runtimes: nvidia runc ...
```

把当前用户加进 docker 组可省掉 sudo：
```bash
sudo usermod -aG docker $USER
newgrp docker
```

---

## 快速开始

### 1. 克隆仓库

```bash
cd ~
git clone https://github.com/art3m1s-tju/f1tenth_gym_ros_cn.git
cd f1tenth_gym_ros_cn
```

### 2. 构建 Docker 镜像

```bash
docker build --no-cache -t f1tenth_gym_ros -f Dockerfile .
```

构建过程会：
- 基于 `ros:foxy` 基础镜像
- 自动把 apt 源换成阿里云、pip 源换成清华（已写进 Dockerfile）
- 安装 RViz、ackermann_msgs、xacro、nav2 等 ROS 2 依赖
- 从上游拉 `f1tenth_gym` Python 仿真库并安装
- `colcon build` 编译 ROS 2 包

首次构建约 10–20 分钟，取决于网速。完成后镜像约 2.7 GB。

如果遇到 apt hash mismatch，多半是网络缓存层捣乱，重试 `docker build --no-cache ...` 即可；仍不行可参考「常见问题」。

### 3. 启动主仿真（终端 1）

使用 rocker 把 GPU 和 X11 透传进容器，同时把当前目录挂载到容器内 `/sim_ws/src/f1tenth_gym_ros`，方便实时改代码和配置：

```bash
cd ~/f1tenth_gym_ros_cn
rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros
```

无 NVIDIA 显卡去掉 `--nvidia`：
```bash
rocker --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros
```

进入容器后执行：
```bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

正常会弹出 RViz 窗口，显示小车、激光扫描和地图。

> **关于重新编译：** 镜像构建时已经 `colcon build` 过一次。如果你只是跑仿真，不需要再编译。只有在改了挂载进来的源码（比如改了 `f1tenth_gym_ros/` 下的 Python 节点）后，才需要：
> ```bash
> cd /sim_ws
> colcon build
> source install/local_setup.bash
> ```

### 4. 启动键盘遥控（终端 2）

打开一个新的**本地**终端，进入已经跑起来的容器：

```bash
docker ps                       # 找到正在运行的容器 ID
docker exec -it <容器ID> /bin/bash
```

在容器里启动遥控节点：
```bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**键位：**
- `i` / `,` 前进 / 倒车
- `j` / `l` 左转 / 右转
- `k` 刹车
- `u` / `o` / `m` / `.` 斜向
- `q` / `z` 提高 / 降低线速度和角速度
- `w` / `x` 只调线速度
- `e` / `c` 只调角速度

**必须让这个终端窗口保持焦点**，否则按键不会被捕获。

---

## 配置说明

核心配置在 `config/sim.yaml`，挂载后在宿主机直接编辑即可生效（重启 launch 后）。

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `map_path` | 地图文件路径（容器内绝对路径，不含扩展名） | `/sim_ws/src/f1tenth_gym_ros/maps/levine` |
| `map_img_ext` | 地图图片扩展名 | `.png` |
| `num_agent` | 对手车辆数（0 = 单车，1 = 1v1 对抗） | `1` |
| `sx`, `sy`, `stheta` | 主车初始位姿 | `0, 0, 0` |
| `sx1`, `sy1`, `stheta1` | 对手车初始位姿 | `2.0, 0.5, 0` |
| `scan_beams` | LiDAR 波束数 | `1080` |
| `scan_fov` | LiDAR 视场角（弧度） | `4.7` |
| `kb_teleop` | 启用键盘遥控话题 | `True` |

### 切换地图
仓库自带 `maps/levine` 和 `maps/Spielberg_map`。换地图只需改 `map_path`：
```yaml
map_path: '/sim_ws/src/f1tenth_gym_ros/maps/Spielberg_map'
```

### 自定义地图
放一对同名的 `.png` 和 `.yaml` 到 `maps/` 目录即可，YAML 格式跟 ROS `map_server` 一致。

---

## 话题列表

主车（`ego_racecar` 命名空间）：
- 订阅 `/drive` (`ackermann_msgs/AckermannDriveStamped`) 控制指令
- 订阅 `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`) RViz 2D Pose Estimate 重置位姿
- 发布 `/ego_racecar/scan` (`sensor_msgs/LaserScan`) LiDAR
- 发布 `/ego_racecar/odom` (`nav_msgs/Odometry`) 里程计
- 发布 `/map` (`nav_msgs/OccupancyGrid`) 占据栅格地图
- 发布 `/tf` 坐标变换

多车模式（`num_agent: 1`）下对手车用 `opp_racecar` 前缀对应话题。

---

## 常见问题

**Q: `docker build` 报 apt Hash Sum mismatch？**
A: 国内网络到 Ubuntu 官方源的 CDN 层经常返回不同步的缓存文件。本仓库 Dockerfile 已经换到阿里云 + 禁用 HTTP 缓存。如果仍然失败，可换成清华源：
```
sed -i 's@mirrors.aliyun.com@mirrors.tuna.tsinghua.edu.cn@g' Dockerfile
```
并加 `--no-cache` 重构建。

**Q: `pip install gym==0.19.0` 报 "invalid metadata" 或 "Expected end or semicolon"？**
A: `gym==0.19.0` 的 `setup.py` 有非法版本写法（`opencv-python>=3.`），pip ≥ 24.1 拒绝解析。本仓库 Dockerfile 已经把 pip 锁到 `<24.1`。

**Q: 启动时提示 `No map received`？**
A: 检查 `config/sim.yaml` 里 `map_path`，必须是**容器内**的绝对路径（通常是 `/sim_ws/src/f1tenth_gym_ros/maps/xxx`，不要带 `.png`）。

**Q: RViz 白屏 / 报 OpenGL 错误？**
A:
- 先确认 `rocker` 带了 `--nvidia --x11`
- 确认宿主机 `nvidia-smi` 正常
- 对于 RTX 40/50 系等新卡，若 Mesa 软件渲染报错，必须走 `--nvidia`

**Q: 键盘按键没反应？**
A: 让跑 `teleop_twist_keyboard` 的那个终端窗口保持鼠标焦点，ROS 是从 stdin 读键盘的。

**Q: 要怎么重新 `colcon build`？**
A: 只有改了挂载进来的 ROS 源码才需要。在容器内：
```bash
cd /sim_ws && colcon build && source install/local_setup.bash
```

---

## 许可证

MIT License，沿用上游仓库。引用请见：

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
