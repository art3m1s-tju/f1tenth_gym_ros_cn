"""Launch file: F1TENTH simulator + control_friendly planner + LQR controller."""
import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_gym_ros')
    sim_config = os.path.join(pkg_share, 'config', 'sim.yaml')
    code_dir = '/sim_ws/src/f1tenth_gym_ros/code'

    config_dict = yaml.safe_load(open(sim_config, 'r'))
    bridge_params = config_dict['bridge']['ros__parameters']
    default_map_path = bridge_params['map_path']
    default_map_img_ext = bridge_params['map_img_ext']

    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('map_path', default_value=default_map_path),
        DeclareLaunchArgument('map_yaml', default_value=f'{default_map_path}.yaml'),
        DeclareLaunchArgument('map_img_ext', default_value=default_map_img_ext),
        DeclareLaunchArgument('sx', default_value=str(bridge_params['sx'])),
        DeclareLaunchArgument('sy', default_value=str(bridge_params['sy'])),
        DeclareLaunchArgument('stheta', default_value=str(bridge_params['stheta'])),
        DeclareLaunchArgument('target_speed', default_value='1.0'),
        DeclareLaunchArgument('min_speed', default_value='0.4'),
        DeclareLaunchArgument('lqr_q_lateral', default_value='3.0'),
        DeclareLaunchArgument('lqr_q_heading', default_value='1.2'),
        DeclareLaunchArgument('lqr_r_steering', default_value='8.0'),
        DeclareLaunchArgument('lqr_feedforward_gain', default_value='1.0'),
        DeclareLaunchArgument('lqr_lookahead_distance_m', default_value='0.0'),
        DeclareLaunchArgument('lqr_gain_table_path', default_value=''),
        DeclareLaunchArgument('max_lateral_accel', default_value='2.0'),
        DeclareLaunchArgument('max_steering_angle', default_value='0.36'),
        DeclareLaunchArgument('use_tf_pose', default_value='true'),
        DeclareLaunchArgument('enable_curvature_speed_limit', default_value='true'),
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
        DeclareLaunchArgument('trajectory_mode', default_value='control_friendly'),
        DeclareLaunchArgument('centerline_smoothing', default_value='5.0'),
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
        DeclareLaunchArgument('collision_log_path',
            default_value=f'{code_dir}/outputs/logs/collision_log.csv'),
        DeclareLaunchArgument('enable_st_corridor_avoidance', default_value='false'),
        DeclareLaunchArgument('local_path_topic', default_value='/local_trajectory'),
        DeclareLaunchArgument('local_speed_limit_topic', default_value='/local_speed_limit'),
        DeclareLaunchArgument('st_lookahead_m', default_value='5.0'),
        DeclareLaunchArgument('st_sample_ds_m', default_value='0.25'),
        DeclareLaunchArgument('st_front_fov_deg', default_value='150.0'),
        DeclareLaunchArgument('st_inflation_margin_m', default_value='0.10'),
        DeclareLaunchArgument('st_min_border_clearance_m', default_value='0.22'),
        DeclareLaunchArgument('st_max_lateral_offset_m', default_value='0.65'),
        DeclareLaunchArgument('st_lateral_offset_step_m', default_value='0.10'),
        DeclareLaunchArgument('st_takeover_distance_m', default_value='0.0'),
        DeclareLaunchArgument('st_desired_obstacle_clearance_m', default_value='0.12'),
        DeclareLaunchArgument('st_max_cluster_radius_m', default_value='0.45'),
        DeclareLaunchArgument('st_avoidance_max_speed', default_value='1.0'),
        DeclareLaunchArgument('st_min_speed', default_value='0.35'),
        DeclareLaunchArgument('st_static_obstacle_manifest_path', default_value=''),
        DeclareLaunchArgument('st_log_path',
            default_value=f'{code_dir}/outputs/logs/st_corridor_log.csv'),
    ]

    bridge_runtime_params = dict(bridge_params)
    bridge_runtime_params.update({
        'map_path': LaunchConfiguration('map_path'),
        'map_img_ext': LaunchConfiguration('map_img_ext'),
        'sx': ParameterValue(LaunchConfiguration('sx'), value_type=float),
        'sy': ParameterValue(LaunchConfiguration('sy'), value_type=float),
        'stheta': ParameterValue(LaunchConfiguration('stheta'), value_type=float),
        'collision_log_path': LaunchConfiguration('collision_log_path'),
    })
    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[bridge_runtime_params],
    )

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
            {'yaml_filename': LaunchConfiguration('map_yaml')},
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
        traj_mode = LaunchConfiguration('trajectory_mode').perform(context)
        centerline_smoothing = LaunchConfiguration('centerline_smoothing').perform(context)
        cf_alpha = LaunchConfiguration('control_friendly_alpha').perform(context)
        cf_auto = LaunchConfiguration('control_friendly_auto_alpha').perform(context)
        cf_smooth = LaunchConfiguration('control_friendly_smoothing').perform(context)
        cf_curv = LaunchConfiguration('control_friendly_max_curvature').perform(context)
        cf_clear = LaunchConfiguration('control_friendly_min_clearance').perform(context)
        track_csv = LaunchConfiguration('track_csv').perform(context)
        traj_csv = LaunchConfiguration('trajectory_csv').perform(context)
        log_path = LaunchConfiguration('log_path').perform(context)
        enable_st = LaunchConfiguration('enable_st_corridor_avoidance').perform(context).lower() in ('1', 'true', 'yes')
        local_path_topic = LaunchConfiguration('local_path_topic').perform(context)
        local_speed_limit_topic = LaunchConfiguration('local_speed_limit_topic').perform(context)
        st_lookahead = LaunchConfiguration('st_lookahead_m').perform(context)
        st_sample_ds = LaunchConfiguration('st_sample_ds_m').perform(context)
        st_front_fov = LaunchConfiguration('st_front_fov_deg').perform(context)
        st_inflation_margin = LaunchConfiguration('st_inflation_margin_m').perform(context)
        st_min_border_clearance = LaunchConfiguration('st_min_border_clearance_m').perform(context)
        st_max_lateral_offset = LaunchConfiguration('st_max_lateral_offset_m').perform(context)
        st_lateral_offset_step = LaunchConfiguration('st_lateral_offset_step_m').perform(context)
        st_takeover_distance = LaunchConfiguration('st_takeover_distance_m').perform(context)
        st_desired_obstacle_clearance = LaunchConfiguration('st_desired_obstacle_clearance_m').perform(context)
        st_max_cluster_radius = LaunchConfiguration('st_max_cluster_radius_m').perform(context)
        st_avoidance_max_speed = LaunchConfiguration('st_avoidance_max_speed').perform(context)
        st_min_speed = LaunchConfiguration('st_min_speed').perform(context)
        st_static_obstacle_manifest = LaunchConfiguration('st_static_obstacle_manifest_path').perform(context)
        st_log_path = LaunchConfiguration('st_log_path').perform(context)

        planner_proc = ExecuteProcess(
            cmd=[
                'python3', os.path.join(code_dir, 'run_planner.py'),
                '--ros-args',
                '-r', '__node:=planner',
                '-p', f'track_csv:={track_csv}',
                '-p', 'use_csv_topic:=false',
                '-p', f'trajectory_mode:={traj_mode}',
                '-p', f'centerline_spline_smoothing:={centerline_smoothing}',
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

        lqr_cmd = [
            'python3', os.path.join(code_dir, 'run_lqr.py'),
            '--ros-args',
            '-r', '__node:=lqr_controller',
            '-p', f'path_topic:={local_path_topic if enable_st else "/global_trajectory"}',
            '-p', 'odom_topic:=/ego_racecar/odom',
            '-p', 'drive_topic:=/drive',
            '-p', f'use_tf_pose:={use_tf_pose}',
            '-p', 'vehicle_frame:=ego_racecar/base_link',
            '-p', 'tf_lookup_timeout_sec:=0.05',
            '-p', f'path_closed_loop:={str(not enable_st).lower()}',
            '-p', 'wheelbase:=0.3302',
            '-p', f'target_speed:={target_speed}',
            '-p', f'min_speed:={st_min_speed if enable_st else min_speed}',
            '-p', f'max_steering_angle:={max_steer}',
            '-p', f'max_lateral_accel:={max_lat_accel}',
            '-p', f'enable_curvature_speed_limit:={curvature_speed_limit}',
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
            '-p', f'lqr_lookahead_distance_m:={0.0 if enable_st else lqr_lookahead}',
            '-p', 'enable_tracking_csv_log:=true',
            '-p', f'tracking_log_path:={log_path}',
        ]
        if enable_st:
            lqr_cmd.extend([
                '-p', f'speed_limit_topic:={local_speed_limit_topic}',
                '-p', 'speed_limit_timeout_sec:=0.5',
                '-p', 'preserve_state_on_path_update:=true',
            ])
        if gain_table_path:
            lqr_cmd.extend(['-p', f'lqr_gain_table_path:={gain_table_path}'])

        lqr_proc = ExecuteProcess(
            cmd=lqr_cmd,
            output='screen',
        )

        if not enable_st:
            return [planner_proc, lqr_proc]

        st_proc = ExecuteProcess(
            cmd=[
                'python3', os.path.join(code_dir, 'run_st_corridor_planner.py'),
                '--ros-args',
                '-r', '__node:=st_corridor_planner',
                '-p', 'global_path_topic:=/global_trajectory',
                '-p', f'local_path_topic:={local_path_topic}',
                '-p', f'speed_limit_topic:={local_speed_limit_topic}',
                '-p', 'scan_topic:=/scan',
                '-p', 'odom_topic:=/ego_racecar/odom',
                '-p', f'track_csv:={track_csv}',
                '-p', 'frame_id:=map',
                '-p', 'vehicle_width:=0.31',
                '-p', 'scan_x_offset_m:=0.275',
                '-p', f'inflation_margin_m:={st_inflation_margin}',
                '-p', f'lookahead_m:={st_lookahead}',
                '-p', f'sample_ds_m:={st_sample_ds}',
                '-p', f'front_fov_deg:={st_front_fov}',
                '-p', f'min_border_clearance_m:={st_min_border_clearance}',
                '-p', f'max_lateral_offset_m:={st_max_lateral_offset}',
                '-p', f'lateral_offset_step_m:={st_lateral_offset_step}',
                '-p', f'takeover_distance_m:={st_takeover_distance}',
                '-p', f'desired_obstacle_clearance_m:={st_desired_obstacle_clearance}',
                '-p', f'max_cluster_radius_m:={st_max_cluster_radius}',
                '-p', f'target_speed:={target_speed}',
                '-p', f'avoidance_max_speed:={st_avoidance_max_speed}',
                '-p', f'min_speed:={st_min_speed}',
                '-p', f'static_obstacle_manifest_path:={st_static_obstacle_manifest}',
                '-p', f'log_path:={st_log_path}',
            ],
            output='screen',
        )
        return [planner_proc, st_proc, lqr_proc]

    ld = LaunchDescription(launch_args)
    ld.add_action(bridge_node)
    ld.add_action(rviz_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(ego_robot_publisher)
    ld.add_action(OpaqueFunction(function=launch_pnc_nodes))
    return ld
