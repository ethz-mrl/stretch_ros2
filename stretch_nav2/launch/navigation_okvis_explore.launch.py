import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory, get_package_share_path

from stretch_nav2.launch_utils import write_okvis_nav_params, write_okvis_vio_config
from stretch_aruco_localizer import transform_utils as goal_transform_utils

# OKVIS ACCURACY EXPLORATION: single-session, no relocalization.
#
# Diagnostic variant of navigation_okvis.launch.py, for isolating OKVIS's own VIO
# accuracy from AMCL/ArUco relocalization error. There is no saved map and no
# map->odom correction here -- map==odom==world==this session's start pose (a
# static identity transform, i.e. the `reloc:=none` case in navigation_okvis.launch.py,
# hardcoded rather than selectable). Loop closures are forced OFF (pure VIO): loop
# closures re-optimize the pose graph and snap the pose discretely, which would
# mask/confound the drift this launch is meant to expose.
#
# Workflow: bring this up, drive the robot around with the joystick
# (teleop_type:=joystick, default), call /record_goal a few times at spots you want
# to check, then call /goto/<goal_name> (e.g. /goto/goal_0) to send Nav2 back to
# that recorded pose purely off OKVIS odometry. Any position/yaw error you observe
# on return is OKVIS drift, not relocalization error.
#
# TF chain: map --(static identity)--> odom --(anchor,static)--> world --(OKVIS)--> base_link


def generate_launch_description():
    stretch_core_path = get_package_share_directory('stretch_core')
    stretch_nav2_path = get_package_share_directory('stretch_nav2')
    stretch_okvis_share = get_package_share_directory('stretch_okvis')
    okvis_package = str(get_package_share_path("okvis"))
    realsense_package = get_package_share_directory('realsense2_camera')

    # ---------------- launch args ----------------
    rviz_param = DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false'])

    teleop_type = DeclareLaunchArgument(
        'teleop_type', default_value='joystick',
        description="how to teleop ('keyboard', 'joystick' or 'none')")

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation/Gazebo clock')

    autostart_param = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Whether to autostart the nav2 lifecycle nodes')

    map_name_param = DeclareLaunchArgument(
        'map_name', default_value='explore_test',
        description="Basename for the <map_name>_goals.yaml sidecar recorded/"
                    "replayed this session (kept separate from real map-associated "
                    "goal files). Stored under $HELLO_FLEET_PATH/maps.")

    settle_sec_param = DeclareLaunchArgument(
        'settle_sec', default_value='1.0',
        description="goal_navigator: seconds to wait after NavigateToPose returns "
                    "before measuring/reporting pos_err/yaw_err against the recorded "
                    "goal. Nav2's own goal-reached check happens instantly at "
                    "whatever pose it has right then; set this near 0 to see that "
                    "same instant instead of pose drift/settle after the fact.")

    # Same staging + xy-correction params as navigation_okvis.launch.py.
    staging_enable_param = DeclareLaunchArgument(
        'staging_enable', default_value='true', choices=['true', 'false'],
        description="goal_navigator: send Nav2 to a pose 'staging_offset_m' "
                    "short of the goal (along the goal's own facing direction) "
                    "instead of the goal itself, then let xy/yaw correction "
                    "below close the remaining known-direction gap. Enable "
                    "together with xy_correct_enable -- staging alone runs no "
                    "local correction.")
    staging_offset_m_param = DeclareLaunchArgument(
        'staging_offset_m', default_value='0.30',
        description='goal_navigator: distance short of the goal Nav2 is sent to '
                    'when staging_enable is true.')

    xy_correct_enable_param = DeclareLaunchArgument(
        'xy_correct_enable', default_value='true', choices=['true', 'false'],
        description="goal_navigator: after Nav2 reports SUCCEEDED (and before "
                    "yaw correction), face the residual position vector and "
                    "drive straight to trim it (bounded, closed-loop off the "
                    "same map->base_link TF). MUST stay enabled while "
                    "staging_enable is true, or every goto stops "
                    "staging_offset_m short of the goal.")
    xy_correct_tolerance_m_param = DeclareLaunchArgument(
        'xy_correct_tolerance_m', default_value='0.015',
        description='goal_navigator: xy trim stops once within this many meters.')
    final_approach_mode_param = DeclareLaunchArgument(
        'final_approach_mode', default_value='joint',
        choices=['sequential', 'joint'],
        description="goal_navigator: last-mile correction style. 'sequential' = "
                    "drive to xy, then rotate to yaw (terminal in-place rotation "
                    "can smear position via wheel slip). 'joint' = converge "
                    "position and heading together, blending bearing into goal "
                    "heading as distance shrinks -- ideal with staging_enable, "
                    "since the leg starts on the goal's approach line.")
    xy_correct_linear_vel_param = DeclareLaunchArgument(
        'xy_correct_linear_vel', default_value='0.03',
        description='goal_navigator: |linear.x| (m/s) for the final approach drive. '
                    'Kept slow on purpose -- this is the precision leg.')
    xy_correct_angular_vel_param = DeclareLaunchArgument(
        'xy_correct_angular_vel', default_value='0.1',
        description='goal_navigator: |angular.z| (rad/s) while aligning to the '
                    'residual vector during the final approach.')
    xy_correct_align_tolerance_deg_param = DeclareLaunchArgument(
        'xy_correct_align_tolerance_deg', default_value='15.0',
        description='goal_navigator: xy trim only drives once facing the residual '
                    'within this many degrees; otherwise it turns in place first.')
    xy_correct_timeout_sec_param = DeclareLaunchArgument(
        'xy_correct_timeout_sec', default_value='30.0',
        description='goal_navigator: safety cutoff for the xy trim maneuver. Keep it '
                    'covering staging_offset_m / xy_correct_linear_vel plus a few '
                    'seconds of alignment turning.')

    yaw_correct_enable_param = DeclareLaunchArgument(
        'yaw_correct_enable', default_value='true', choices=['true', 'false'],
        description="goal_navigator: after Nav2 reports SUCCEEDED, rotate in "
                    "place (bounded, closed-loop off the same map->base_link "
                    "TF) to trim any residual yaw error before measuring/"
                    "reporting it. Set false to see Nav2's raw arrival heading "
                    "uncorrected (e.g. for a pure OKVIS-drift measurement).")
    yaw_correct_tolerance_deg_param = DeclareLaunchArgument(
        'yaw_correct_tolerance_deg', default_value='3.0',
        description='goal_navigator: yaw trim stops once within this many degrees.')
    yaw_correct_vel_param = DeclareLaunchArgument(
        'yaw_correct_vel', default_value='0.15',
        description='goal_navigator: fixed |angular.z| (rad/s) used while trimming yaw.')
    yaw_correct_timeout_sec_param = DeclareLaunchArgument(
        'yaw_correct_timeout_sec', default_value='6.0',
        description='goal_navigator: safety cutoff for the yaw trim rotation.')

    wall_square_enable_param = DeclareLaunchArgument(
        'wall_square_enable', default_value='false', choices=['true', 'false'],
        description="goal_navigator: after the yaw trim, check the RPLidar "
                    "for a flat surface directly behind the robot and, if "
                    "found, do one more small rotation to square base_link's "
                    "x-axis to it (a real wall is a steadier heading "
                    "reference than VIO/the recorded goal orientation). A "
                    "no-op if nothing suitable is in range.")
    wall_square_max_range_m_param = DeclareLaunchArgument(
        'wall_square_max_range_m', default_value='1.2',
        description='goal_navigator: ignore lidar returns behind the robot farther than this.')
    wall_square_tolerance_deg_param = DeclareLaunchArgument(
        'wall_square_tolerance_deg', default_value='0.0',
        description='goal_navigator: wall-square trim stops once within this many degrees.')

    # Move to the manipulation posture on startup (one-shot). NOTE include_head:=true
    # turns the head camera to the arm, which disables OKVIS VIO while turned.
    startup_posture_param = DeclareLaunchArgument(
        'startup_posture', default_value='true', choices=['true', 'false'],
        description='Move to the manipulation posture at startup')
    include_head_param = DeclareLaunchArgument(
        'include_head', default_value='true', choices=['true', 'false'],
        description='Include head pan/tilt in the startup posture (points camera at '
                    'the arm; breaks OKVIS while turned)')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_name = LaunchConfiguration('map_name')

    # Same OKVIS-navigation param overrides as navigation_okvis.launch.py (goal
    # tolerances, Hybrid-A* planner, wait-only recovery, voxel origin_z).
    source_params = os.path.join(stretch_nav2_path, 'config', 'nav2_params.yaml')
    wait_only_bt = os.path.join(stretch_nav2_path, 'config',
                                'navigate_to_pose_wait_only_recovery.xml')
    wait_only_bt_through = os.path.join(stretch_nav2_path, 'config',
                                        'navigate_through_poses_wait_only_recovery.xml')
    # rolling_global_costmap:=True -- no map_server here (see module docstring), so
    # the stock fixed-origin-(0,0) global costmap would leave the robot stranded
    # outside its bounds the moment it drives negative in x/y. Make it follow the
    # robot instead.
    # use_rpp_controller:=True -- Regulated Pure Pursuit instead of DWB; smoother
    # tracking of SmacPlannerHybrid's curved final approach arcs, no discrete
    # rotate-in-place switchover near the goal (see module docstring).
    # max_linear_vel/max_angular_vel lowered from the 0.26 m/s / 0.4 rad/s
    # stock/production caps -- this is a slow, careful accuracy check, not a speed run.
    okvis_nav_params = write_okvis_nav_params(
        source_params, wait_only_bt, wait_only_bt_through,
        rolling_global_costmap=True, use_rpp_controller=True,
        max_linear_vel=0.05, max_angular_vel=0.05)

    # Loop closures OFF: patch do_loop_closures:true -> false in a temp copy of the
    # stock OKVIS config (pure VIO; see module docstring).
    okvis_vio_config = write_okvis_vio_config(
        os.path.join(okvis_package, 'config', 'realsense_D435if_stretch_bag.yaml'))

    # ---------------- OKVIS state-estimation stack (as in navigation_okvis.launch.py) ----
    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'navigation', 'broadcast_odom_tf': 'False'}.items())

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/rplidar.launch.py']))

    base_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_nav2_path, '/launch/teleop_twist.launch.py']),
        launch_arguments={'teleop_type': LaunchConfiguration('teleop_type')}.items())

    # Launch RViz directly (not nav2_bringup's rviz_launch.py) so closing RViz
    # doesn't kill OKVIS/Nav2 -- same reasoning as navigation_okvis.launch.py.
    rviz_config = os.path.join(
        os.environ.get('HELLO_FLEET_PATH', '/home/hello-robot/stretch_user'),
        'nav2_default_view.rviz')
    rviz_launch = Node(
        package='rviz2', executable='rviz2', name='rviz', output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')))

    # OKVIS via the subscriber node, isolated onto private topics exactly as in
    # navigation_okvis.launch.py (see that file's comments for the rationale).
    def make_okvis_launch(condition=None):
        return GroupAction([
            SetRemap('/odom', '/okvis/wheel_odom_disabled'),
            SetRemap('/tf', '/okvis/tf_raw'),
            SetRemap('/tf_static', '/okvis/tf_static_raw'),
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource([
                    os.path.join(okvis_package, 'launch', 'okvis_node_subscriber.launch.xml')
                ]),
                launch_arguments={
                    'rviz': 'false',
                    'config_filename': okvis_vio_config,
                }.items()),
        ], condition=condition)

    okvis_nav_tf_bridge = Node(
        package='stretch_okvis', executable='nav_tf_bridge',
        name='okvis_nav_tf_bridge', output='screen')

    # Live RealSense D435i as OKVIS input (stereo IR + IMU only -- no ArUco here).
    realsense_params_file = os.path.join(stretch_okvis_share, 'config', 'okvis_realsense_params.yaml')
    realsense_launch = GroupAction([
        SetRemap('/camera/infra1/image_rect_raw', '/okvis/cam0/image_raw'),
        SetRemap('/camera/infra2/image_rect_raw', '/okvis/cam1/image_raw'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([realsense_package, '/launch/rs_launch.py']),
            launch_arguments={
                'camera_namespace': '',
                'camera_name': 'camera',
                'enable_color': 'false',
                'enable_depth': 'false',
                'align_depth.enable': 'false',
                'rgb_camera.color_profile': '640,480,15',
                'enable_infra1': 'true',
                'enable_infra2': 'true',
                'depth_module.infra_profile': '640,480,15',
                'depth_module.depth_profile': '640,480,15',
                'depth_module.infra1_format': 'Y8',
                'depth_module.infra2_format': 'Y8',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': '2',
                'initial_reset': 'false',
                'config_file': realsense_params_file,
            }.items()),
    ])

    # Anchor odom == base_link(t=0) on the floor (see map_anchor.py).
    map_anchor = Node(
        package='stretch_okvis', executable='map_anchor', name='map_anchor',
        output='screen')

    imu_qos_bridge = Node(
        package='stretch_okvis', executable='imu_qos_bridge', name='imu_qos_bridge',
        output='screen',
        parameters=[{'input_topic': '/camera/imu', 'output_topic': '/okvis/imu0'}])

    # ---------------- map -> odom: identity (no relocalization) ----------------
    # This is the whole point of this launch file: map == odom == session start
    # pose, so any error returning to a recorded goal is OKVIS drift alone.
    map_to_odom_identity = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'])

    # One-shot manipulation posture at startup (same as navigation_okvis.launch.py).
    startup_posture = Node(
        package='stretch_aruco_localizer', executable='go_to_posture',
        name='go_to_posture', output='screen',
        condition=IfCondition(LaunchConfiguration('startup_posture')),
        parameters=[{'include_head': LaunchConfiguration('include_head'),
                     'joint_lift': 0.58,
                     'joint_head_tilt': 0.0}])

    # Defer OKVIS until the startup posture has finished + settled, exactly as in
    # navigation_okvis.launch.py / offline_okvis_mapping.launch.py.
    okvis_settle_sec = 3.0
    okvis_after_posture = RegisterEventHandler(
        OnProcessExit(
            target_action=startup_posture,
            on_exit=[
                LogInfo(msg=('go_to_posture finished; settling '
                             f'{okvis_settle_sec:.0f}s before starting OKVIS')),
                TimerAction(period=okvis_settle_sec, actions=[make_okvis_launch()]),
            ]),
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('startup_posture'), "' == 'true'"])))
    okvis_no_posture = make_okvis_launch(
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('startup_posture'), "' == 'false'"])))

    # ---------------- nav2 planning/control ----------------
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_nav2_path, '/launch/navigation_launch.py']),
        launch_arguments={'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'params_file': okvis_nav_params}.items())

    # ---------------- goal recording + on-demand replay ----------------
    goal_recorder = Node(
        package='stretch_aruco_localizer', executable='goal_recorder',
        name='goal_recorder', output='screen',
        parameters=[{'map_name': map_name}])

    goal_navigator = Node(
        package='stretch_aruco_localizer', executable='goal_navigator',
        name='goal_navigator', output='screen',
        parameters=[{'map_name': map_name,
                     'settle_sec': LaunchConfiguration('settle_sec'),
                     'staging_enable': LaunchConfiguration('staging_enable'),
                     'staging_offset_m': LaunchConfiguration('staging_offset_m'),
                     'xy_correct_enable': LaunchConfiguration('xy_correct_enable'),
                     'xy_correct_tolerance_m': LaunchConfiguration('xy_correct_tolerance_m'),
                     'final_approach_mode': LaunchConfiguration('final_approach_mode'),
                     'xy_correct_linear_vel': LaunchConfiguration('xy_correct_linear_vel'),
                     'xy_correct_angular_vel': LaunchConfiguration('xy_correct_angular_vel'),
                     'xy_correct_align_tolerance_deg': LaunchConfiguration(
                         'xy_correct_align_tolerance_deg'),
                     'xy_correct_timeout_sec': LaunchConfiguration('xy_correct_timeout_sec'),
                     'yaw_correct_enable': LaunchConfiguration('yaw_correct_enable'),
                     'yaw_correct_tolerance_deg': LaunchConfiguration('yaw_correct_tolerance_deg'),
                     'yaw_correct_vel': LaunchConfiguration('yaw_correct_vel'),
                     'yaw_correct_timeout_sec': LaunchConfiguration('yaw_correct_timeout_sec'),
                     'wall_square_enable': LaunchConfiguration('wall_square_enable'),
                     'wall_square_max_range_m': LaunchConfiguration('wall_square_max_range_m'),
                     'wall_square_tolerance_deg': LaunchConfiguration('wall_square_tolerance_deg')}])

    # Goals recorded this session are keyed off map==odom==world==THIS session's
    # start pose (no relocalization ties them to anything more durable) -- they are
    # meaningless once the robot restarts at a different pose next session. Wipe the
    # sidecar on shutdown so a future run doesn't silently try to navigate to stale
    # coordinates from a different start pose.
    def _clear_session_goals(event, context):
        resolved_map_name = LaunchConfiguration('map_name').perform(context)
        maps_dir = os.path.join(
            os.environ.get('HELLO_FLEET_PATH', '/home/hello-robot/stretch_user'), 'maps')
        path = os.path.join(maps_dir, f'{resolved_map_name}_goals.yaml')
        if os.path.exists(path):
            goal_transform_utils.save_goals(path, [], map_name=resolved_map_name)

    clear_goals_on_shutdown = RegisterEventHandler(
        OnShutdown(on_shutdown=_clear_session_goals))

    return LaunchDescription([
        rviz_param,
        teleop_type,
        declare_use_sim_time_argument,
        autostart_param,
        map_name_param,
        settle_sec_param,
        staging_enable_param,
        staging_offset_m_param,
        xy_correct_enable_param,
        xy_correct_tolerance_m_param,
        final_approach_mode_param,
        xy_correct_linear_vel_param,
        xy_correct_angular_vel_param,
        xy_correct_align_tolerance_deg_param,
        xy_correct_timeout_sec_param,
        yaw_correct_enable_param,
        yaw_correct_tolerance_deg_param,
        yaw_correct_vel_param,
        yaw_correct_timeout_sec_param,
        wall_square_enable_param,
        wall_square_max_range_m_param,
        wall_square_tolerance_deg_param,
        startup_posture_param,
        include_head_param,
        # OKVIS odometry stack
        stretch_driver_launch,
        rplidar_launch,
        base_teleop_launch,
        realsense_launch,
        okvis_nav_tf_bridge,
        okvis_after_posture,
        okvis_no_posture,
        map_anchor,
        imu_qos_bridge,
        # relocalization (map -> odom): identity, always
        map_to_odom_identity,
        startup_posture,
        # navigation
        navigation_launch,
        rviz_launch,
        # goal recording + replay
        goal_recorder,
        goal_navigator,
        clear_goals_on_shutdown,
    ])
