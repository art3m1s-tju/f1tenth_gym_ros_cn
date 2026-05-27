"""启动 F1TENTH 仿真、全局规划、Frenet 局部规划、LQR 控制和 RViz。"""
import os
import sys
import tempfile
from pathlib import Path

import yaml

_LOCAL_CODE_DIR = Path(__file__).resolve().parents[1] / 'code'
_CONTAINER_CODE_DIR = Path('/sim_ws/src/f1tenth_gym_ros/code')
for _code_dir in (_LOCAL_CODE_DIR, _CONTAINER_CODE_DIR):
    if _code_dir.exists() and str(_code_dir) not in sys.path:
        sys.path.insert(0, str(_code_dir))

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from pnc_rc.frenet.preset import FRENET_LAUNCH_ARGUMENT_DEFAULTS
from pnc_rc.frenet.preset import FRENET_NODE_PARAM_MAP


def generate_launch_description():
    """创建仿真、全局规划、Frenet 局部规划、LQR 和 RViz 的 launch 描述。

    Returns:
        `LaunchDescription`。Frenet 相关参数通过 `FRENET_LAUNCH_ARGUMENT_DEFAULTS`
        和 `FRENET_NODE_PARAM_MAP` 统一声明并转发。
    """
    pkg_share = get_package_share_directory('f1tenth_gym_ros')
    sim_config = os.path.join(pkg_share, 'config', 'sim.yaml')
    code_dir = '/sim_ws/src/f1tenth_gym_ros/code'

    config_dict = yaml.safe_load(open(sim_config, 'r'))

    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument(
            'map_path',
            default_value=config_dict['bridge']['ros__parameters']['map_path'],
        ),
        DeclareLaunchArgument(
            'map_img_ext',
            default_value=config_dict['bridge']['ros__parameters']['map_img_ext'],
        ),
        DeclareLaunchArgument(
            'sx',
            default_value=str(config_dict['bridge']['ros__parameters']['sx']),
        ),
        DeclareLaunchArgument(
            'sy',
            default_value=str(config_dict['bridge']['ros__parameters']['sy']),
        ),
        DeclareLaunchArgument(
            'stheta',
            default_value=str(config_dict['bridge']['ros__parameters']['stheta']),
        ),
        DeclareLaunchArgument('target_speed', default_value='1.0'),
        DeclareLaunchArgument('min_speed', default_value='0.4'),
        DeclareLaunchArgument('lqr_q_lateral', default_value='3.0'),
        DeclareLaunchArgument('lqr_q_heading', default_value='1.2'),
        DeclareLaunchArgument('lqr_r_steering', default_value='8.0'),
        DeclareLaunchArgument('lqr_feedforward_gain', default_value='1.0'),
        DeclareLaunchArgument('lqr_lookahead_distance_m', default_value='0.0'),
        DeclareLaunchArgument('lqr_gain_table_path', default_value=''),
        DeclareLaunchArgument('max_lateral_accel', default_value='4.0'),
        DeclareLaunchArgument('max_steering_angle', default_value='0.36'),
        DeclareLaunchArgument('use_tf_pose', default_value='true'),
        DeclareLaunchArgument('enable_curvature_speed_limit', default_value='true'),
        DeclareLaunchArgument('local_speed_limit_topic', default_value='/local_trajectory_speed_limit'),
        DeclareLaunchArgument('local_speed_limit_timeout_s', default_value='1.0'),
        DeclareLaunchArgument('curvature_speed_lookahead_m', default_value='1.0'),
        DeclareLaunchArgument('enable_speed_ramp', default_value='true'),
        DeclareLaunchArgument('max_accel', default_value='1.0'),
        DeclareLaunchArgument('max_decel', default_value='2.0'),
        DeclareLaunchArgument('enable_steering_rate_limit', default_value='true'),
        DeclareLaunchArgument('max_steering_rate', default_value='2.0'),
        DeclareLaunchArgument('validation_position_noise_std', default_value='0.0'),
        DeclareLaunchArgument('validation_heading_noise_std_deg', default_value='0.0'),
        DeclareLaunchArgument('validation_pose_delay_ms', default_value='0.0'),
        DeclareLaunchArgument('validation_noise_seed', default_value='42'),
        DeclareLaunchArgument('enable_error_filter', default_value='false'),
        DeclareLaunchArgument('error_filter_alpha_y', default_value='0.30'),
        DeclareLaunchArgument('error_filter_alpha_psi', default_value='0.25'),
        DeclareLaunchArgument('enable_frenet_planner', default_value='false'),
        *[
            DeclareLaunchArgument(name, default_value=default_value)
            for name, default_value in FRENET_LAUNCH_ARGUMENT_DEFAULTS
        ],
        DeclareLaunchArgument('trajectory_mode', default_value='control_friendly'),
        DeclareLaunchArgument('control_friendly_alpha', default_value='0.56'),
        DeclareLaunchArgument('control_friendly_auto_alpha', default_value='true'),
        DeclareLaunchArgument('control_friendly_smoothing', default_value='2.0'),
        DeclareLaunchArgument('control_friendly_max_curvature', default_value='1.0'),
        DeclareLaunchArgument('control_friendly_min_clearance', default_value='0.35'),
        DeclareLaunchArgument('track_csv',
            default_value=f'{code_dir}/outputs/csv/processed_track.csv'),
        DeclareLaunchArgument('trajectory_csv',
            default_value=f'{code_dir}/outputs/csv/global_trajectory.csv'),
        DeclareLaunchArgument('log_path',
            default_value=f'{code_dir}/outputs/logs/lqr_tracking_log.csv'),
    ]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_share, 'launch', 'gym_bridge.rviz')],
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[
            {'yaml_filename': [LaunchConfiguration('map_path'), '.yaml']},
            {'topic': 'map'},
            {'frame_id': 'map'},
            {'output': 'screen'},
            {'use_sim_time': True},
        ],
    )

    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server']},
        ],
    )

    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                'xacro ',
                os.path.join(pkg_share, 'launch', 'ego_racecar.xacro'),
            ])
        }],
        remappings=[('/robot_description', 'ego_robot_description')],
    )

    # Use OpaqueFunction to resolve LaunchConfiguration at launch time
    from launch.actions import OpaqueFunction

    def create_bridge_node(context):
        """按 launch 参数重写 gym bridge 临时配置。

        Args:
            context: ROS 2 launch 运行时上下文。

        Returns:
            包含一个 `gym_bridge` Node action 的列表。
        """
        map_path = LaunchConfiguration('map_path').perform(context)
        map_img_ext = LaunchConfiguration('map_img_ext').perform(context)
        sx = float(LaunchConfiguration('sx').perform(context))
        sy = float(LaunchConfiguration('sy').perform(context))
        stheta = float(LaunchConfiguration('stheta').perform(context))
        bridge_config = yaml.safe_load(open(sim_config, 'r'))
        bridge_params = bridge_config['bridge']['ros__parameters']
        bridge_params['map_path'] = map_path
        bridge_params['map_img_ext'] = map_img_ext
        bridge_params['sx'] = sx
        bridge_params['sy'] = sy
        bridge_params['stheta'] = stheta
        temp_config = os.path.join(
            tempfile.gettempdir(),
            'f1tenth_bridge_launch.yaml',
        )
        with open(temp_config, 'w', encoding='utf-8') as f:
            yaml.safe_dump(bridge_config, f, sort_keys=False)
        return [
            Node(
                package='f1tenth_gym_ros',
                executable='gym_bridge',
                name='bridge',
                parameters=[temp_config],
            )
        ]

    def launch_pnc_nodes(context):
        """在运行时解析参数并创建 planner/Frenet/LQR 进程。

        Args:
            context: ROS 2 launch 运行时上下文。

        Returns:
            `ExecuteProcess` 列表。`enable_frenet_planner=true` 时会把 LQR 输入
            切换到 `/local_trajectory` 并附加 Frenet planner 进程。
        """
        target_speed = LaunchConfiguration('target_speed').perform(context)
        min_speed = LaunchConfiguration('min_speed').perform(context)
        q_lat = LaunchConfiguration('lqr_q_lateral').perform(context)
        q_head = LaunchConfiguration('lqr_q_heading').perform(context)
        r_steer = LaunchConfiguration('lqr_r_steering').perform(context)
        ff_gain = LaunchConfiguration('lqr_feedforward_gain').perform(context)
        lqr_lookahead = LaunchConfiguration('lqr_lookahead_distance_m').perform(context)
        gain_table_path = LaunchConfiguration('lqr_gain_table_path').perform(context)
        max_lat_accel = LaunchConfiguration('max_lateral_accel').perform(context)
        max_steer = LaunchConfiguration('max_steering_angle').perform(context)
        use_tf_pose = LaunchConfiguration('use_tf_pose').perform(context)
        curvature_speed_limit = LaunchConfiguration(
            'enable_curvature_speed_limit'
        ).perform(context)
        local_speed_limit_topic = LaunchConfiguration(
            'local_speed_limit_topic'
        ).perform(context)
        local_speed_limit_timeout = LaunchConfiguration(
            'local_speed_limit_timeout_s'
        ).perform(context)
        curvature_lookahead = LaunchConfiguration(
            'curvature_speed_lookahead_m'
        ).perform(context)
        speed_ramp = LaunchConfiguration('enable_speed_ramp').perform(context)
        accel = LaunchConfiguration('max_accel').perform(context)
        decel = LaunchConfiguration('max_decel').perform(context)
        steering_rate_limit = LaunchConfiguration(
            'enable_steering_rate_limit'
        ).perform(context)
        max_steering_rate = LaunchConfiguration('max_steering_rate').perform(context)
        position_noise = LaunchConfiguration('validation_position_noise_std').perform(context)
        heading_noise = LaunchConfiguration('validation_heading_noise_std_deg').perform(context)
        pose_delay = LaunchConfiguration('validation_pose_delay_ms').perform(context)
        noise_seed = LaunchConfiguration('validation_noise_seed').perform(context)
        error_filter = LaunchConfiguration('enable_error_filter').perform(context)
        error_alpha_y = LaunchConfiguration('error_filter_alpha_y').perform(context)
        error_alpha_psi = LaunchConfiguration('error_filter_alpha_psi').perform(context)
        enable_frenet = LaunchConfiguration('enable_frenet_planner').perform(context)
        frenet_values = {
            name: LaunchConfiguration(name).perform(context)
            for name, _ in FRENET_LAUNCH_ARGUMENT_DEFAULTS
        }
        frenet_param_args = []
        for node_param, launch_arg in FRENET_NODE_PARAM_MAP:
            frenet_param_args.extend([
                '-p',
                f'{node_param}:={frenet_values[launch_arg]}',
            ])
        traj_mode = LaunchConfiguration('trajectory_mode').perform(context)
        cf_alpha = LaunchConfiguration('control_friendly_alpha').perform(context)
        cf_auto = LaunchConfiguration('control_friendly_auto_alpha').perform(context)
        cf_smooth = LaunchConfiguration('control_friendly_smoothing').perform(context)
        cf_curv = LaunchConfiguration('control_friendly_max_curvature').perform(context)
        cf_clear = LaunchConfiguration('control_friendly_min_clearance').perform(context)
        track_csv = LaunchConfiguration('track_csv').perform(context)
        traj_csv = LaunchConfiguration('trajectory_csv').perform(context)
        log_path = LaunchConfiguration('log_path').perform(context)

        planner_proc = ExecuteProcess(
            cmd=[
                'python3', os.path.join(code_dir, 'run_planner.py'),
                '--ros-args',
                '-r', '__node:=planner',
                '-p', f'track_csv:={track_csv}',
                '-p', 'use_csv_topic:=false',
                '-p', f'trajectory_mode:={traj_mode}',
                '-p', f'control_friendly_alpha:={cf_alpha}',
                '-p', f'control_friendly_auto_alpha:={cf_auto}',
                '-p', f'control_friendly_spline_smoothing:={cf_smooth}',
                '-p', f'control_friendly_target_max_curvature:={cf_curv}',
                '-p', f'control_friendly_min_clearance:={cf_clear}',
                '-p', 'frame_id:=map',
                '-p', 'path_topic:=/global_trajectory',
                '-p', 'path_republish_period:=0.5',
                '-p', f'output_trajectory_csv:={traj_csv}',
            ],
            output='screen',
        )

        frenet_enabled = str(enable_frenet).lower() in ('true', '1', 'yes', 'on')
        lqr_path_topic = '/local_trajectory' if frenet_enabled else '/global_trajectory'
        lqr_path_closed_loop = 'false' if frenet_enabled else 'true'
        lqr_endpoint_stop_max_path_length = '0.3' if frenet_enabled else '1000000000.0'
        frenet_proc = ExecuteProcess(
            cmd=[
                'python3', os.path.join(code_dir, 'run_frenet_planner.py'),
                '--ros-args',
                '-r', '__node:=frenet_static_obstacle_planner',
                '-p', 'global_path_topic:=/global_trajectory',
                '-p', 'local_path_topic:=/local_trajectory',
                '-p', f'speed_limit_topic:={local_speed_limit_topic}',
                '-p', 'odom_topic:=/ego_racecar/odom',
                '-p', 'scan_topic:=/scan',
                '-p', 'map_topic:=/map',
                '-p', 'frame_id:=map',
                *frenet_param_args,
                '-p', f'cruise_speed_mps:={target_speed}',
            ],
            output='screen',
        )

        lqr_cmd = [
            'python3', os.path.join(code_dir, 'run_lqr.py'),
            '--ros-args',
            '-r', '__node:=lqr_controller',
            '-p', f'path_topic:={lqr_path_topic}',
            '-p', 'odom_topic:=/ego_racecar/odom',
            '-p', 'drive_topic:=/drive',
            '-p', f'use_tf_pose:={use_tf_pose}',
            '-p', 'vehicle_frame:=ego_racecar/base_link',
            '-p', 'tf_lookup_timeout_sec:=0.05',
            '-p', f'path_closed_loop:={lqr_path_closed_loop}',
            '-p', f'open_loop_endpoint_stop_max_path_length:={lqr_endpoint_stop_max_path_length}',
            '-p', 'wheelbase:=0.3302',
            '-p', f'target_speed:={target_speed}',
            '-p', f'min_speed:={min_speed}',
            '-p', f'max_steering_angle:={max_steer}',
            '-p', f'max_lateral_accel:={max_lat_accel}',
            '-p', f'enable_curvature_speed_limit:={curvature_speed_limit}',
            '-p', f'speed_limit_topic:={local_speed_limit_topic if frenet_enabled else ""}',
            '-p', f'speed_limit_timeout_s:={local_speed_limit_timeout}',
            '-p', f'curvature_speed_lookahead_m:={curvature_lookahead}',
            '-p', f'enable_speed_ramp:={speed_ramp}',
            '-p', f'max_accel:={accel}',
            '-p', f'max_decel:={decel}',
            '-p', f'enable_steering_rate_limit:={steering_rate_limit}',
            '-p', f'max_steering_rate:={max_steering_rate}',
            '-p', f'validation_position_noise_std:={position_noise}',
            '-p', f'validation_heading_noise_std_deg:={heading_noise}',
            '-p', f'validation_pose_delay_ms:={pose_delay}',
            '-p', f'validation_noise_seed:={noise_seed}',
            '-p', f'enable_error_filter:={error_filter}',
            '-p', f'error_filter_alpha_y:={error_alpha_y}',
            '-p', f'error_filter_alpha_psi:={error_alpha_psi}',
            '-p', f'lqr_q_lateral:={q_lat}',
            '-p', f'lqr_q_heading:={q_head}',
            '-p', f'lqr_r_steering:={r_steer}',
            '-p', f'lqr_feedforward_gain:={ff_gain}',
            '-p', f'lqr_lookahead_distance_m:={lqr_lookahead}',
            '-p', 'enable_tracking_csv_log:=true',
            '-p', f'tracking_log_path:={log_path}',
        ]
        if gain_table_path:
            lqr_cmd.extend(['-p', f'lqr_gain_table_path:={gain_table_path}'])

        lqr_proc = ExecuteProcess(
            cmd=lqr_cmd,
            output='screen',
        )

        processes = [planner_proc]
        if frenet_enabled:
            processes.append(frenet_proc)
        processes.append(lqr_proc)
        return processes

    ld = LaunchDescription(launch_args)
    ld.add_action(OpaqueFunction(function=create_bridge_node))
    ld.add_action(rviz_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)
    ld.add_action(OpaqueFunction(function=launch_pnc_nodes))
    return ld
