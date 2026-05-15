#!/bin/bash
# LQR 控制器仿真启动脚本
# 修改下方参数后直接运行：./run_pnc_sim.sh
# Ctrl+C 停止后自动运行误差评估并保存到带参数名+时间戳的文件夹

# ============================================================
# LQR 控制器参数（调参改这里）
# ============================================================
TARGET_SPEED=1.0              # 目标速度 (m/s)
MIN_SPEED=0.4                 # 最低速度 (m/s)
LQR_Q_LATERAL=3.0             # 横向误差权重（越大跟踪越紧）
LQR_Q_HEADING=1.2             # 航向误差权重（越大入弯越快）
LQR_R_STEERING=8.0            # 转向代价（越大越平滑）
LQR_FEEDFORWARD_GAIN=1.0      # 曲率前馈增益
MAX_LATERAL_ACCEL=4.0         # 弯道限速横向加速度 (m/s^2)
MAX_STEERING_ANGLE=0.36       # 最大转向角 (rad)，与实车物理一致
ENABLE_SPEED_RAMP=true        # 加减速限幅
MAX_ACCEL=1.0                 # 最大加速度 (m/s^2)
MAX_DECEL=2.0                 # 最大减速度 (m/s^2)

# ============================================================
# 轨迹规划参数
# ============================================================
TRAJECTORY_MODE=control_friendly
CONTROL_FRIENDLY_ALPHA=0.56
CONTROL_FRIENDLY_AUTO_ALPHA=true
CONTROL_FRIENDLY_SMOOTHING=2.0
CONTROL_FRIENDLY_MAX_CURVATURE=1.0
CONTROL_FRIENDLY_MIN_CLEARANCE=0.35

# ============================================================
# 路径（一般不用改）
# ============================================================
CODE_DIR=/sim_ws/src/f1tenth_gym_ros/code
TRACK_CSV=${CODE_DIR}/outputs/csv/processed_track.csv
TRAJECTORY_CSV=${CODE_DIR}/outputs/csv/global_trajectory.csv
LOG_PATH=${CODE_DIR}/outputs/logs/lqr_tracking_log.csv

# ============================================================
# 生成评估输出目录名：关键参数 + 时间戳
# ============================================================
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
EVAL_DIR_NAME="v${TARGET_SPEED}_Ql${LQR_Q_LATERAL}_Qh${LQR_Q_HEADING}_R${LQR_R_STEERING}_ff${LQR_FEEDFORWARD_GAIN}_${TIMESTAMP}"
EVAL_OUTPUT_DIR=${CODE_DIR}/outputs/evaluation/${EVAL_DIR_NAME}

# ============================================================
# 启动
# ============================================================
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash

echo "=========================================="
echo " LQR 仿真参数"
echo "=========================================="
echo " target_speed:      ${TARGET_SPEED} m/s"
echo " min_speed:         ${MIN_SPEED} m/s"
echo " Q_lateral:         ${LQR_Q_LATERAL}"
echo " Q_heading:         ${LQR_Q_HEADING}"
echo " R_steering:        ${LQR_R_STEERING}"
echo " feedforward_gain:  ${LQR_FEEDFORWARD_GAIN}"
echo " max_lateral_accel: ${MAX_LATERAL_ACCEL} m/s^2"
echo " max_steering:      ${MAX_STEERING_ANGLE} rad"
echo " trajectory_mode:   ${TRAJECTORY_MODE}"
echo ""
echo " 评估输出目录:      ${EVAL_DIR_NAME}"
echo "=========================================="

# 检查赛道数据
if [ ! -f "${TRACK_CSV}" ]; then
    echo "[ERROR] 赛道数据不存在，先运行: python3 ${CODE_DIR}/generate_track.py"
    exit 1
fi

# 启动仿真（Ctrl+C 停止后继续执行评估）
ros2 launch f1tenth_gym_ros pnc_sim_launch.py \
    target_speed:=${TARGET_SPEED} \
    min_speed:=${MIN_SPEED} \
    lqr_q_lateral:=${LQR_Q_LATERAL} \
    lqr_q_heading:=${LQR_Q_HEADING} \
    lqr_r_steering:=${LQR_R_STEERING} \
    lqr_feedforward_gain:=${LQR_FEEDFORWARD_GAIN} \
    max_lateral_accel:=${MAX_LATERAL_ACCEL} \
    max_steering_angle:=${MAX_STEERING_ANGLE} \
    enable_speed_ramp:=${ENABLE_SPEED_RAMP} \
    max_accel:=${MAX_ACCEL} \
    max_decel:=${MAX_DECEL} \
    trajectory_mode:=${TRAJECTORY_MODE} \
    control_friendly_alpha:=${CONTROL_FRIENDLY_ALPHA} \
    control_friendly_auto_alpha:=${CONTROL_FRIENDLY_AUTO_ALPHA} \
    control_friendly_smoothing:=${CONTROL_FRIENDLY_SMOOTHING} \
    control_friendly_max_curvature:=${CONTROL_FRIENDLY_MAX_CURVATURE} \
    control_friendly_min_clearance:=${CONTROL_FRIENDLY_MIN_CLEARANCE} \
    track_csv:=${TRACK_CSV} \
    trajectory_csv:=${TRAJECTORY_CSV} \
    log_path:=${LOG_PATH}

# ============================================================
# 仿真结束后自动评估
# ============================================================
echo ""
echo "=========================================="
echo " 正在评估跟踪误差..."
echo "=========================================="

if [ ! -f "${LOG_PATH}" ]; then
    echo "[WARN] 日志文件不存在: ${LOG_PATH}，跳过评估"
    exit 0
fi

python3 ${CODE_DIR}/tracker_evaluate.py \
    --log ${LOG_PATH} \
    --reference-trajectory-csv ${TRAJECTORY_CSV} \
    --output-dir ${EVAL_OUTPUT_DIR}

echo ""
echo "=========================================="
echo " 评估完成，结果保存在:"
echo " ${EVAL_OUTPUT_DIR}"
echo "=========================================="
