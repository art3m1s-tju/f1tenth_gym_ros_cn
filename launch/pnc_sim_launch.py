"""Launch file: F1TENTH simulator + control_friendly planner + LQR controller."""
import os
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
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
        DeclareLaunchArgument('frenet_publish_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('frenet_target_speed', default_value='1.5'),
        DeclareLaunchArgument('frenet_v_min', default_value='0.6'),
        DeclareLaunchArgument('frenet_v_max', default_value='2.5'),
        DeclareLaunchArgument('frenet_v_step', default_value='0.3'),
        DeclareLaunchArgument('frenet_t_min', default_value='2.0'),
        DeclareLaunchArgument('frenet_t_max', default_value='3.0'),
        DeclareLaunchArgument('frenet_t_step', default_value='0.5'),
        DeclareLaunchArgument('frenet_d_min', default_value='-1.8'),
        DeclareLaunchArgument('frenet_d_max', default_value='1.8'),
        DeclareLaunchArgument('frenet_d_step', default_value='0.1'),
        DeclareLaunchArgument('frenet_trajectory_dt', default_value='0.1'),
        DeclareLaunchArgument('frenet_max_curvature', default_value='1.1'),
        DeclareLaunchArgument('frenet_safe_clearance_m', default_value='0.35'),
        DeclareLaunchArgument('frenet_min_clearance_m', default_value='0.05'),
        DeclareLaunchArgument('frenet_corridor_radius_m', default_value='0.14'),
        DeclareLaunchArgument('frenet_corridor_sample_step_m', default_value='0.10'),
        DeclareLaunchArgument('frenet_path_collision_sample_step_m', default_value='0.05'),
        DeclareLaunchArgument('frenet_footprint_front_m', default_value='0.38'),
        DeclareLaunchArgument('frenet_footprint_rear_m', default_value='0.05'),
        DeclareLaunchArgument('frenet_max_heading_jump', default_value='0.85'),
        DeclareLaunchArgument('frenet_min_progress_step_m', default_value='0.20'),
        DeclareLaunchArgument('frenet_reuse_last_candidate_timeout_s', default_value='1.0'),
        DeclareLaunchArgument('frenet_projection_search_window_m', default_value='6.0'),
        DeclareLaunchArgument('frenet_stop_path_length_m', default_value='0.25'),
        DeclareLaunchArgument('frenet_min_published_path_length_m', default_value='0.75'),
        DeclareLaunchArgument('frenet_published_path_lookahead_m', default_value='0.25'),
        DeclareLaunchArgument('frenet_min_path_publish_interval_s', default_value='0.25'),
        DeclareLaunchArgument('frenet_path_republish_distance_m', default_value='0.50'),
        DeclareLaunchArgument('frenet_path_republish_min_remaining_m', default_value='2.0'),
        DeclareLaunchArgument('frenet_centerline_return_lookahead_m', default_value='5.0'),
        DeclareLaunchArgument('frenet_centerline_return_step_m', default_value='0.08'),
        DeclareLaunchArgument('frenet_centerline_threat_corridor_radius_m', default_value='0.32'),
        DeclareLaunchArgument('frenet_centerline_threat_lookahead_m', default_value='6.0'),
        DeclareLaunchArgument('frenet_reference_closed_loop', default_value='true'),
        DeclareLaunchArgument('frenet_centerline_speed_limit_mps', default_value='-1.0'),
        DeclareLaunchArgument('frenet_avoidance_speed_limit_mps', default_value='0.75'),
        DeclareLaunchArgument('frenet_stop_speed_limit_mps', default_value='0.0'),
        DeclareLaunchArgument('frenet_grid_inflation_radius_m', default_value='0.28'),
        DeclareLaunchArgument('frenet_grid_resolution_m', default_value='0.05'),
        DeclareLaunchArgument('frenet_grid_forward_m', default_value='7.0'),
        DeclareLaunchArgument('frenet_grid_rear_m', default_value='1.0'),
        DeclareLaunchArgument('frenet_grid_half_width_m', default_value='3.0'),
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
        frenet_rate = LaunchConfiguration('frenet_publish_rate_hz').perform(context)
        frenet_target_speed = LaunchConfiguration('frenet_target_speed').perform(context)
        frenet_v_min = LaunchConfiguration('frenet_v_min').perform(context)
        frenet_v_max = LaunchConfiguration('frenet_v_max').perform(context)
        frenet_v_step = LaunchConfiguration('frenet_v_step').perform(context)
        frenet_t_min = LaunchConfiguration('frenet_t_min').perform(context)
        frenet_t_max = LaunchConfiguration('frenet_t_max').perform(context)
        frenet_t_step = LaunchConfiguration('frenet_t_step').perform(context)
        frenet_d_min = LaunchConfiguration('frenet_d_min').perform(context)
        frenet_d_max = LaunchConfiguration('frenet_d_max').perform(context)
        frenet_d_step = LaunchConfiguration('frenet_d_step').perform(context)
        frenet_trajectory_dt = LaunchConfiguration('frenet_trajectory_dt').perform(context)
        frenet_max_curvature = LaunchConfiguration('frenet_max_curvature').perform(context)
        frenet_safe_clearance = LaunchConfiguration('frenet_safe_clearance_m').perform(context)
        frenet_min_clearance = LaunchConfiguration('frenet_min_clearance_m').perform(context)
        frenet_corridor_radius = LaunchConfiguration('frenet_corridor_radius_m').perform(context)
        frenet_corridor_sample_step = LaunchConfiguration(
            'frenet_corridor_sample_step_m'
        ).perform(context)
        frenet_path_collision_sample_step = LaunchConfiguration(
            'frenet_path_collision_sample_step_m'
        ).perform(context)
        frenet_footprint_front = LaunchConfiguration('frenet_footprint_front_m').perform(context)
        frenet_footprint_rear = LaunchConfiguration('frenet_footprint_rear_m').perform(context)
        frenet_max_heading_jump = LaunchConfiguration('frenet_max_heading_jump').perform(context)
        frenet_min_progress_step = LaunchConfiguration('frenet_min_progress_step_m').perform(context)
        frenet_reuse_timeout = LaunchConfiguration('frenet_reuse_last_candidate_timeout_s').perform(context)
        frenet_projection_search_window = LaunchConfiguration(
            'frenet_projection_search_window_m'
        ).perform(context)
        frenet_stop_path_length = LaunchConfiguration('frenet_stop_path_length_m').perform(context)
        frenet_min_published_path_length = LaunchConfiguration(
            'frenet_min_published_path_length_m'
        ).perform(context)
        frenet_published_path_lookahead = LaunchConfiguration(
            'frenet_published_path_lookahead_m'
        ).perform(context)
        frenet_min_path_publish_interval = LaunchConfiguration(
            'frenet_min_path_publish_interval_s'
        ).perform(context)
        frenet_path_republish_distance = LaunchConfiguration(
            'frenet_path_republish_distance_m'
        ).perform(context)
        frenet_path_republish_min_remaining = LaunchConfiguration(
            'frenet_path_republish_min_remaining_m'
        ).perform(context)
        frenet_centerline_return_lookahead = LaunchConfiguration(
            'frenet_centerline_return_lookahead_m'
        ).perform(context)
        frenet_centerline_return_step = LaunchConfiguration(
            'frenet_centerline_return_step_m'
        ).perform(context)
        frenet_centerline_threat_corridor_radius = LaunchConfiguration(
            'frenet_centerline_threat_corridor_radius_m'
        ).perform(context)
        frenet_centerline_threat_lookahead = LaunchConfiguration(
            'frenet_centerline_threat_lookahead_m'
        ).perform(context)
        frenet_reference_closed_loop = LaunchConfiguration(
            'frenet_reference_closed_loop'
        ).perform(context)
        frenet_centerline_speed_limit = LaunchConfiguration(
            'frenet_centerline_speed_limit_mps'
        ).perform(context)
        frenet_avoidance_speed_limit = LaunchConfiguration(
            'frenet_avoidance_speed_limit_mps'
        ).perform(context)
        frenet_stop_speed_limit = LaunchConfiguration(
            'frenet_stop_speed_limit_mps'
        ).perform(context)
        frenet_inflation = LaunchConfiguration('frenet_grid_inflation_radius_m').perform(context)
        frenet_resolution = LaunchConfiguration('frenet_grid_resolution_m').perform(context)
        frenet_grid_forward = LaunchConfiguration('frenet_grid_forward_m').perform(context)
        frenet_grid_rear = LaunchConfiguration('frenet_grid_rear_m').perform(context)
        frenet_grid_half_width = LaunchConfiguration('frenet_grid_half_width_m').perform(context)
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
                '-p', f'publish_rate_hz:={frenet_rate}',
                '-p', f'target_speed:={frenet_target_speed}',
                '-p', f'v_min:={frenet_v_min}',
                '-p', f'v_max:={frenet_v_max}',
                '-p', f'v_step:={frenet_v_step}',
                '-p', f't_min:={frenet_t_min}',
                '-p', f't_max:={frenet_t_max}',
                '-p', f't_step:={frenet_t_step}',
                '-p', f'd_min:={frenet_d_min}',
                '-p', f'd_max:={frenet_d_max}',
                '-p', f'd_step:={frenet_d_step}',
                '-p', f'trajectory_dt:={frenet_trajectory_dt}',
                '-p', f'max_curvature:={frenet_max_curvature}',
                '-p', f'safe_clearance_m:={frenet_safe_clearance}',
                '-p', f'min_clearance_m:={frenet_min_clearance}',
                '-p', f'corridor_radius_m:={frenet_corridor_radius}',
                '-p', f'corridor_sample_step_m:={frenet_corridor_sample_step}',
                '-p', f'path_collision_sample_step_m:={frenet_path_collision_sample_step}',
                '-p', f'footprint_front_m:={frenet_footprint_front}',
                '-p', f'footprint_rear_m:={frenet_footprint_rear}',
                '-p', f'max_heading_jump:={frenet_max_heading_jump}',
                '-p', f'min_progress_step_m:={frenet_min_progress_step}',
                '-p', f'reuse_last_candidate_timeout_s:={frenet_reuse_timeout}',
                '-p', f'projection_search_window_m:={frenet_projection_search_window}',
                '-p', f'stop_path_length_m:={frenet_stop_path_length}',
                '-p', f'min_published_path_length_m:={frenet_min_published_path_length}',
                '-p', f'published_path_lookahead_m:={frenet_published_path_lookahead}',
                '-p', f'min_path_publish_interval_s:={frenet_min_path_publish_interval}',
                '-p', f'path_republish_distance_m:={frenet_path_republish_distance}',
                '-p', f'path_republish_min_remaining_m:={frenet_path_republish_min_remaining}',
                '-p', f'centerline_return_lookahead_m:={frenet_centerline_return_lookahead}',
                '-p', f'centerline_return_step_m:={frenet_centerline_return_step}',
                '-p', f'centerline_threat_corridor_radius_m:={frenet_centerline_threat_corridor_radius}',
                '-p', f'centerline_threat_lookahead_m:={frenet_centerline_threat_lookahead}',
                '-p', f'reference_closed_loop:={frenet_reference_closed_loop}',
                '-p', f'centerline_speed_limit_mps:={frenet_centerline_speed_limit}',
                '-p', f'avoidance_speed_limit_mps:={frenet_avoidance_speed_limit}',
                '-p', f'stop_speed_limit_mps:={frenet_stop_speed_limit}',
                '-p', f'grid_inflation_radius_m:={frenet_inflation}',
                '-p', f'grid_resolution_m:={frenet_resolution}',
                '-p', f'grid_forward_m:={frenet_grid_forward}',
                '-p', f'grid_rear_m:={frenet_grid_rear}',
                '-p', f'grid_half_width_m:={frenet_grid_half_width}',
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
