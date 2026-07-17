import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory, get_package_share_path

from stretch_nav2.launch_utils import write_okvis_nav_params

# OKVIS-based NAVIGATION in a pre-built 2D map.
#
# This is the navigation counterpart of offline_okvis_mapping.launch.py. It reuses
# the exact OKVIS state-estimation stack (OKVIS is the sole odometry source; wheel
# odometry TF is off) and swaps the *mapping* half (octomap/scan_to_cloud) for the
# *navigation* half: nav2 map_server (serves the saved grid) + planner/controller.
#
# KEY IDEA -- relocalization lives entirely in the `map -> odom` edge:
#
#   map --(RELOC)--> odom --(anchor,static)--> world --(OKVIS)--> base_link --(URDF)--> camera_*
#
# `odom -> world` (map_anchor, static) + `world -> base_link` (OKVIS VIO) together
# ARE the odometry -- they place base_link relative to THIS session's floor-level
# start pose, exactly as wheel odometry would. `map -> odom` is the relocalization
# correction that ties this session into the saved map's frame.
#
# OKVIS here CANNOT load a prior map / relocalize on its own (no loadMap, no
# localization-only mode -- its place-recognition DB is rebuilt every run). So the
# `map -> odom` transform must come from an external relocalizer. That provider is
# HOT-SWAPPABLE via the `reloc` arg:
#   reloc:=amcl  -> nav2 AMCL matches lidar vs the saved grid and publishes
#                   map->odom continuously (default; robust, no TF conflict because
#                   AMCL owns map->odom while OKVIS owns everything below odom).
#   reloc:=none  -> publish an identity static map->odom (map == session start pose;
#                   use when the robot always starts at the mapped origin, or when
#                   an external transform e.g. Aria/hloc is published separately).
#
# NOTE: AMCL's default base_frame_id is base_footprint, which does NOT exist in the
# Stretch URDF; OKVIS drives base_link, so we override amcl base_frame_id:=base_link.


def generate_launch_description():
    stretch_core_path = get_package_share_directory('stretch_core')
    stretch_nav2_path = get_package_share_directory('stretch_nav2')
    nav2_bringup_package = str(get_package_share_path("nav2_bringup"))

    stretch_okvis_share = get_package_share_directory('stretch_okvis')
    okvis_package = str(get_package_share_path("okvis"))
    realsense_package = get_package_share_directory('realsense2_camera')

    # ---------------- launch args ----------------
    rviz_param = DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false'])

    teleop_type = DeclareLaunchArgument(
        'teleop_type', default_value="joystick", description="how to teleop ('keyboard', 'joystick' or 'none')")

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation/Gazebo clock')

    autostart_param = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Whether to autostart the nav2 lifecycle nodes')

    map_path_param = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            os.environ.get('HELLO_FLEET_PATH', '/home/hello-robot/stretch_user'),
            'maps', (os.environ.get('MAP_NAME', 'map_test') + '.yaml')),
        description='Full path to the map.yaml file to localize/navigate in')

    reloc_param = DeclareLaunchArgument(
        'reloc', default_value='aruco',
        choices=['amcl', 'amcl_oneshot', 'none', 'hloc', 'aruco', 'aruco_amcl',
                 'apriltag', 'apriltag_amcl'],
        description="How map->odom (relocalization) is provided: 'amcl' (lidar vs "
                    "saved grid, continuous), 'amcl_oneshot' (AMCL corrects the initial "
                    "2D Pose Estimate ONCE, then freezes map->odom so OKVIS carries a "
                    "smooth pose with no further jumps), 'none' (identity static; "
                    "map == start pose), 'aruco' (a fixed ChArUco board anchors "
                    "map->odom; requires a recorded <map>_anchor.yaml sidecar), "
                    "'aruco_amcl' (AMCL owns map->odom as in 'amcl', but a confident "
                    "board sighting periodically re-seeds AMCL's belief via "
                    "/initialpose -- joint lidar + board relocalization), 'apriltag' "
                    "(same as 'aruco' but anchored on the AprilTag grid board instead "
                    "of the ChArUco board -- must match the anchor_detector used when "
                    "the map's anchor sidecar was recorded), or 'apriltag_amcl' (the "
                    "AprilTag counterpart of 'aruco_amcl')")

    # For reloc:=aruco/apriltag. marker_name defaults empty -> the relocalizer uses
    # the name stored in the anchor sidecar. aruco_mode picks single_shot vs periodic
    # (applies to both board types -- name kept for backward compat with 'aruco').
    marker_name_param = DeclareLaunchArgument(
        'marker_name', default_value='map_anchor',
        description="reloc:=aruco/apriltag: marker/board NAME to anchor on (default "
                    "'map_anchor'). Empty => use the name recorded in the anchor "
                    "sidecar.")
    aruco_mode_param = DeclareLaunchArgument(
        'aruco_mode', default_value='single_shot',
        choices=['single_shot', 'periodic'],
        description="reloc:=aruco/apriltag: 'single_shot' (fix once, freeze) or "
                    "'periodic' (re-anchor on every fresh sighting to bound VIO drift)")

    # goal_recorder / goal_navigator: same record-once / replay-on-demand pair as
    # navigation_okvis_explore.launch.py, here keyed off the real, persistent map
    # (not a session-only one) so recorded goals remain valid across restarts.
    map_name_param = DeclareLaunchArgument(
        'map_name', default_value=os.environ.get('MAP_NAME', 'map_test'),
        description="Basename for the <map_name>_goals.yaml sidecar recorded via "
                    "/record_goal (goal_recorder) and replayed via goal_navigator's "
                    "/goto/<name> services. Defaults to the same MAP_NAME as the "
                    "loaded map, so goals live alongside it.")

    settle_sec_param = DeclareLaunchArgument(
        'settle_sec', default_value='1.0',
        description="goal_navigator: seconds to wait after NavigateToPose returns "
                    "before measuring/reporting pos_err/yaw_err against the recorded "
                    "goal. Nav2's own goal-reached check happens instantly at "
                    "whatever pose it has right then; set this near 0 to see that "
                    "same instant instead of pose drift/settle after the fact.")

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
                    "uncorrected.")
    yaw_correct_tolerance_deg_param = DeclareLaunchArgument(
        'yaw_correct_tolerance_deg', default_value='3.0',
        description='goal_navigator: yaw trim stops once within this many degrees.')
    yaw_correct_vel_param = DeclareLaunchArgument(
        'yaw_correct_vel', default_value='0.1',
        description='goal_navigator: fixed |angular.z| (rad/s) used while trimming yaw.')
    yaw_correct_timeout_sec_param = DeclareLaunchArgument(
        'yaw_correct_timeout_sec', default_value='5.0',
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
    # turns the head camera to the arm, which disables OKVIS VIO + ArUco while turned.
    startup_posture_param = DeclareLaunchArgument(
        'startup_posture', default_value='true', choices=['true', 'false'],
        description='Move to the manipulation posture at startup')
    include_head_param = DeclareLaunchArgument(
        'include_head', default_value='true', choices=['true', 'false'],
        description='Include head pan/tilt in the startup posture (points camera at '
                    'the arm; breaks OKVIS/ArUco while turned)')

    publish_debug_image_param = DeclareLaunchArgument(
        'publish_debug_image', default_value='true', choices=['true', 'false'],
        description='reloc:=aruco/apriltag(_amcl): have the active board detector '
                    'publish its annotated ~/debug_image (drawn markers/tags) for tuning')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml = LaunchConfiguration('map')
    reloc = LaunchConfiguration('reloc')
    marker_name = LaunchConfiguration('marker_name')
    aruco_mode = LaunchConfiguration('aruco_mode')
    map_name = LaunchConfiguration('map_name')
    publish_debug_image = LaunchConfiguration('publish_debug_image')

    # ALL OKVIS-navigation param overrides live here (see _write_okvis_nav_params):
    # goal tolerances, inflation, wait-only recovery + BT-xml, and voxel origin_z. The
    # shared nav2_params.yaml stays pristine for the wheel-odom navigation.launch.py.
    source_params = os.path.join(stretch_nav2_path, 'config', 'nav2_params.yaml')
    wait_only_bt = os.path.join(stretch_nav2_path, 'config',
                                'navigate_to_pose_wait_only_recovery.xml')
    wait_only_bt_through = os.path.join(stretch_nav2_path, 'config',
                                        'navigate_through_poses_wait_only_recovery.xml')
    # Same controller/velocity tuning as navigation_okvis_explore.launch.py --
    # use_rpp_controller:=True (Regulated Pure Pursuit instead of DWB) tracks
    # SmacPlannerHybrid's curved final approach arcs smoothly, with no discrete
    # rotate-in-place switchover near the goal; max_linear_vel/max_angular_vel
    # lowered from the 0.26 m/s / 0.4 rad/s stock caps to match the slower,
    # settled arrivals validated in the explore workflow. rolling_global_costmap
    # stays False here (unlike explore): this launch has a real map_server-served
    # map to size/position the static costmap layer against.
    okvis_nav_params = write_okvis_nav_params(
        source_params, wait_only_bt, wait_only_bt_through,
        use_rpp_controller=True, max_linear_vel=0.05, max_angular_vel=0.05)

    # map_server + AMCL run for amcl, amcl_oneshot, aruco_amcl AND apriltag_amcl (all
    # lidar-vs-grid localization via AMCL); oneshot additionally runs amcl_freeze,
    # which deactivates AMCL after it converges. The *_amcl variants additionally run
    # their board detector + *_amcl_bridge below, which nudges AMCL's belief via
    # /initialpose.
    use_amcl = IfCondition(PythonExpression(
        ["'", reloc, "' in ('amcl', 'amcl_oneshot', 'aruco_amcl', 'apriltag_amcl')"]))
    use_oneshot = IfCondition(PythonExpression(["'", reloc, "' == 'amcl_oneshot'"]))
    no_reloc = IfCondition(PythonExpression(["'", reloc, "' == 'none'"]))
    use_aruco = IfCondition(PythonExpression(["'", reloc, "' == 'aruco'"]))
    use_aruco_amcl = IfCondition(PythonExpression(["'", reloc, "' == 'aruco_amcl'"]))
    use_apriltag = IfCondition(PythonExpression(["'", reloc, "' == 'apriltag'"]))
    use_apriltag_amcl = IfCondition(PythonExpression(["'", reloc, "' == 'apriltag_amcl'"]))
    # ChArUco board detector is needed by BOTH 'aruco' (owns map->odom outright) and
    # 'aruco_amcl' (only nudges AMCL's belief). AprilTag grid detector mirrors this
    # for 'apriltag' / 'apriltag_amcl'.
    use_charuco = IfCondition(
        PythonExpression(["'", reloc, "' in ('aruco', 'aruco_amcl')"]))
    use_apriltag_detect = IfCondition(
        PythonExpression(["'", reloc, "' in ('apriltag', 'apriltag_amcl')"]))
    # map_server serves the saved grid for the costmap static layer -- every reloc
    # mode navigates in the saved map, including 'none' (map == this session's
    # start pose, but the static occupancy grid is the same saved lab01.yaml/.pgm
    # either way). Without it here, the global costmap never gets a static layer
    # at all and falls back to a tiny default rolling window at the origin,
    # making any real goal unplannable ("no map received" / "out of bounds of
    # the costmap", confirmed on hardware 2026-07-17).
    use_map_server = IfCondition(PythonExpression(
        ["'", reloc, "' in ('amcl', 'amcl_oneshot', 'none', 'aruco', 'aruco_amcl', "
                    "'apriltag', 'apriltag_amcl')"]))
    # RealSense color + aligned depth are needed for any board-anchored reloc mode
    # (the detector consumes /camera/color + /camera/aligned_depth_to_color). Off
    # otherwise so OKVIS keeps the IR/IMU bandwidth to itself.
    aruco_stream = PythonExpression(
        ["'true' if '", reloc, "' in ('aruco', 'aruco_amcl', 'apriltag', 'apriltag_amcl') "
                    "else 'false'"])
    # AprilTag's 36 small (88 mm) tags need far more pixels-per-tag than the single
    # ChArUco board's 49 mm squares; bump color resolution only for the AprilTag reloc
    # modes (mirrors offline_okvis_mapping.launch.py's color_profile logic). Color is
    # unused by OKVIS either way (it reads the IR streams).
    color_profile = PythonExpression(
        ["'1280,720,15' if '", reloc, "' in ('apriltag', 'apriltag_amcl') "
                    "else '640,480,15'"])

    # ---------------- OKVIS state-estimation stack (from offline_okvis_mapping) ----
    # Wheel-odom TF stays OFF: OKVIS is the only odometry. Driver still publishes the
    # /odom topic + URDF. mode:=navigation so the base accepts /stretch/cmd_vel.
    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'navigation', 'broadcast_odom_tf': 'False'}.items())

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/rplidar.launch.py']))

    # Joystick/keyboard teleop, same as navigation_okvis_explore.launch.py (default
    # 'none' here, unlike explore's 'joystick' default, since this is the
    # production/autonomous launch -- pass teleop_type:=joystick to drive manually
    # mid-mission).
    base_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_nav2_path, '/launch/teleop_twist.launch.py']),
        launch_arguments={'teleop_type': LaunchConfiguration('teleop_type')}.items())

    # Launch RViz directly. nav2_bringup/rviz_launch.py deliberately shuts down the
    # entire launch when RViz exits; here that would also kill map_server, AMCL,
    # OKVIS, and Nav2 just because the GUI was closed or its OpenGL process crashed.
    # Use the operator's tuned RViz config from stretch_user (mounted into the
    # container at the same path) rather than nav2_bringup's stock view.
    rviz_config = os.path.join(
        os.environ.get('HELLO_FLEET_PATH', '/home/hello-robot/stretch_user'),
        'nav2_default_view.rviz')
    rviz_launch = Node(
        package='rviz2', executable='rviz2', name='rviz', output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')))

    # OKVIS via the subscriber node (does NOT open the camera; subscribes to
    # /okvis/cam0|cam1/image_raw + /okvis/imu0).
    # The OKVIS subscriber has a hard-coded absolute /odom subscription and feeds
    # every message into addOdometryMeasurement(). Isolate that subscription here:
    # wheel odometry remains available to Nav2 for velocity feedback, but it cannot
    # alter the OKVIS pose estimate. This keeps stereo + IMU as OKVIS's only inputs.
    # Run OKVIS with loop closures DISABLED (VIO only, do_loop_closures:false in the
    # stock config). Loop closures re-optimize the pose graph and can make odom->base_link
    # jump discretely, which is undesirable during navigation.
    # Built via a factory so a fresh action can be handed to whichever startup branch
    # runs below (the same launch action object must not be reused in two places). The
    # SetRemaps live inside the returned GroupAction, so its scoping is preserved even
    # when the group is deferred by a TimerAction.
    def make_okvis_launch(condition=None):
        return GroupAction([
            SetRemap('/odom', '/okvis/wheel_odom_disabled'),
            # Keep OKVIS's raw 6-DoF TF off Nav2's TF tree. nav_tf_bridge relays the
            # robot/head transforms into these private topics and publishes a planar
            # world->base_link on the global /tf topic.
            SetRemap('/tf', '/okvis/tf_raw'),
            SetRemap('/tf_static', '/okvis/tf_static_raw'),
            IncludeLaunchDescription(
                XMLLaunchDescriptionSource([
                    os.path.join(okvis_package, 'launch', 'okvis_node_subscriber.launch.xml')
                ]),
                launch_arguments={
                    'rviz': 'false',
                    'config_filename': os.path.join(
                        okvis_package, 'config', 'realsense_D435if_stretch_bag.yaml'),
                }.items()),
        ], condition=condition)

    okvis_nav_tf_bridge = Node(
        package='stretch_okvis', executable='nav_tf_bridge',
        name='okvis_nav_tf_bridge', output='screen')

    # Live RealSense D435i as OKVIS input (stereo IR + united IMU). initial_reset is
    # left False on purpose -- resetting Stretch's head D435i kills depth until reboot.
    realsense_params_file = os.path.join(stretch_okvis_share, 'config', 'okvis_realsense_params.yaml')
    realsense_launch = GroupAction([
        SetRemap('/camera/infra1/image_rect_raw', '/okvis/cam0/image_raw'),
        SetRemap('/camera/infra2/image_rect_raw', '/okvis/cam1/image_raw'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([realsense_package, '/launch/rs_launch.py']),
            launch_arguments={
                'camera_namespace': '',
                'camera_name': 'camera',
                # color + depth + aligned-depth only for reloc:=aruco (ArUco detection).
                'enable_color': aruco_stream,
                'enable_depth': aruco_stream,
                'align_depth.enable': aruco_stream,
                # Color resolution is bumped (see color_profile above) only for the
                # AprilTag reloc modes; 15 fps (vs default 1280x720x30) still keeps
                # USB/CPU load down for OKVIS's IR streams.
                'rgb_camera.color_profile': color_profile,
                'enable_infra1': 'true',
                'enable_infra2': 'true',
                'depth_module.infra_profile': '640,480,15',
                # Depth shares the one D435i stereo module with infra1/2, so its
                # profile MUST match the infra profile (res + fps) or the driver
                # brings up IR and silently drops depth. Without depth, the ArUco
                # detector's color+depth TimeSynchronizer never fires.
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

    # Anchor odom == base_link(t=0) on the floor (see map_anchor.py). This makes
    # odom->world->base_link a proper odometry chain rooted at the session start pose.
    map_anchor = Node(
        package='stretch_okvis', executable='map_anchor', name='map_anchor',
        output='screen')

    # Bridge RealSense IMU (best_effort) -> /okvis/imu0 (reliable) to fix QoS mismatch.
    imu_qos_bridge = Node(
        package='stretch_okvis', executable='imu_qos_bridge', name='imu_qos_bridge',
        output='screen',
        parameters=[{'input_topic': '/camera/imu', 'output_topic': '/okvis/imu0'}])

    # OKVIS's sensor frame S == IMU frame == camera_gyro_optical_frame; OKVIS looks
    # that frame up directly from the URDF TF tree (no alias frame needed).

    # ---------------- relocalization: the map -> odom edge ----------------
    # reloc:=amcl -> map_server serves the saved grid; AMCL publishes map->odom by
    # matching /scan against it. AMCL owns map->odom; OKVIS owns odom->base_link, so
    # there is NO TF conflict. base_frame_id overridden to base_link (no base_footprint
    # in the Stretch URDF). Seed the initial pose with RViz "2D Pose Estimate".
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', condition=use_map_server,
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}])

    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl',
        output='screen', condition=use_amcl,
        parameters=[okvis_nav_params,
                    {'use_sim_time': use_sim_time, 'base_frame_id': 'base_link'}])

    localization_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen', condition=use_amcl,
        parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart,
                     'node_names': ['map_server', 'amcl']}])

    # reloc:=amcl_oneshot -> let AMCL correct the operator's 2D Pose Estimate ONCE,
    # then capture the converged map->odom, re-publish it as a steady dynamic transform,
    # and deactivate AMCL (see amcl_freeze.py). This kills AMCL's continuous map->odom
    # snapping (the "world jumping") so OKVIS's smooth odom->base_link carries the pose
    # afterward. Trade-off: no ongoing global correction, so OKVIS drift is uncorrected.
    amcl_freeze = Node(
        package='stretch_okvis', executable='amcl_freeze', name='amcl_freeze',
        output='screen', condition=use_oneshot,
        parameters=[{'use_sim_time': use_sim_time}])

    # reloc:=none -> map == session start pose. Publish an identity map->odom (an
    # external relocalizer, e.g. Aria/hloc, may overwrite this later).
    map_to_odom_identity = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='map_to_odom', condition=no_reloc,
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'])

    # reloc:=none/aruco/apriltag all need map_server (above, under use_map_server)
    # lifecycle-activated WITHOUT AMCL: 'none' just needs the static map served for
    # costmaps/planning (map->odom itself comes from map_to_odom_identity above);
    # 'aruco'/'apriltag' additionally have aruco_relocalizer/apriltag_relocalizer
    # own map->odom from the recorded anchor (<map>_anchor.yaml, same basename as
    # the loaded map; needs the color + aligned-depth streams enabled above via
    # aruco_stream).
    use_map_server_no_amcl = IfCondition(
        PythonExpression(["'", reloc, "' in ('none', 'aruco', 'apriltag')"]))
    non_amcl_localization_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        condition=use_map_server_no_amcl,
        parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart,
                     'node_names': ['map_server']}])

    # ChArUco board detector (replaces the single-marker stretch_core detector).
    # Publishes the board pose as TF camera_color_optical_frame -> <marker_name>,
    # which aruco_relocalizer (or aruco_amcl_bridge) consumes exactly as before. Board
    # is 6 columns x 4 rows, 66 mm squares, 49 mm markers, DICT_4X4.
    aruco_detect = Node(
        package='stretch_aruco_localizer', executable='charuco_detector',
        name='charuco_detector', output='screen', condition=use_charuco,
        parameters=[{
            'squares_x': 6, 'squares_y': 4,
            'square_length_m': 0.066, 'marker_length_m': 0.049,
            'aruco_dict': 'DICT_4X4_50', 'legacy_pattern': True,
            'min_charuco_corners': 4,
            'marker_name': marker_name,
            'publish_debug_image': publish_debug_image,
        }])

    anchor_yaml_path = PythonExpression(
        ["'", map_yaml, "'.replace('.yaml', '_anchor.yaml')"])
    aruco_relocalizer = Node(
        package='stretch_aruco_localizer', executable='aruco_relocalizer',
        name='aruco_relocalizer', output='screen', condition=use_aruco,
        parameters=[{'use_sim_time': use_sim_time,
                     'anchor_yaml_path': anchor_yaml_path,
                     'marker_name': marker_name,
                     'mode': aruco_mode}])

    # reloc:=aruco_amcl -> AMCL (above, under use_amcl) owns map->odom continuously
    # from lidar-vs-grid matching; this node only nudges AMCL's belief via
    # /initialpose on confident board sightings (see aruco_amcl_bridge.py).
    aruco_amcl_bridge = Node(
        package='stretch_aruco_localizer', executable='aruco_amcl_bridge',
        name='aruco_amcl_bridge', output='screen', condition=use_aruco_amcl,
        parameters=[{'use_sim_time': use_sim_time,
                     'anchor_yaml_path': anchor_yaml_path,
                     'marker_name': marker_name}])

    # reloc:=apriltag/apriltag_amcl -> AprilTag-grid counterpart of the aruco_detect /
    # aruco_relocalizer / aruco_amcl_bridge trio above. apriltag_grid_detector
    # publishes the same camera_color_optical_frame -> <marker_name> TF that
    # apriltag_relocalizer / apriltag_amcl_bridge consume. 6x6 grid, 88 mm tags,
    # DICT_APRILTAG_36H11 (rig defaults, not overridden here).
    apriltag_detect = Node(
        package='stretch_apriltag_localizer', executable='apriltag_grid_detector',
        name='apriltag_grid_detector', output='screen', condition=use_apriltag_detect,
        parameters=[{'marker_name': marker_name, 'publish_debug_image': publish_debug_image}])

    apriltag_relocalizer = Node(
        package='stretch_apriltag_localizer', executable='apriltag_relocalizer',
        name='apriltag_relocalizer', output='screen', condition=use_apriltag,
        parameters=[{'use_sim_time': use_sim_time,
                     'anchor_yaml_path': anchor_yaml_path,
                     'marker_name': marker_name,
                     'mode': aruco_mode}])

    apriltag_amcl_bridge = Node(
        package='stretch_apriltag_localizer', executable='apriltag_amcl_bridge',
        name='apriltag_amcl_bridge', output='screen', condition=use_apriltag_amcl,
        parameters=[{'use_sim_time': use_sim_time,
                     'anchor_yaml_path': anchor_yaml_path,
                     'marker_name': marker_name}])

    # One-shot manipulation posture at startup. Driver is already in navigation mode,
    # which accepts arm/lift/wrist/head trajectory goals, so no mode switch is needed.
    startup_posture = Node(
        package='stretch_aruco_localizer', executable='go_to_posture',
        name='go_to_posture', output='screen',
        condition=IfCondition(LaunchConfiguration('startup_posture')),
        # Same start position as offline_okvis_mapping: lift lowered to 0.58 and the
        # head kept LEVEL (joint_head_tilt 0.0, not the default -pi/4 downward tilt,
        # which points the camera at the floor and hurts OKVIS).
        parameters=[{'include_head': LaunchConfiguration('include_head'),
                     'joint_lift': 0.58,
                     'joint_head_tilt': 0.0}])

    # Waiting strategy (mirrors offline_okvis_mapping): defer OKVIS until the startup
    # posture has finished moving AND the arm/lift have settled a few seconds. OKVIS
    # bootstraps its VI-SLAM from the first frames, so arm motion / vibration in view
    # during init degrades the estimate. go_to_posture is one-shot (exits once the
    # posture is reached), so start a settle timer on its exit and only THEN bring
    # OKVIS up. Only OKVIS is deferred; Nav2 lifecycle already waits for the transforms
    # OKVIS provides, so it simply activates once they appear.
    #   startup_posture:=true  -> posture move -> wait settle -> OKVIS  (event handler)
    #   startup_posture:=false -> nothing to wait for          -> OKVIS starts directly
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
    # Reuse Stretch's navigation launcher rather than nav2_bringup's upstream launcher.
    # The Stretch launcher is the path used by navigation.launch.py and is known to:
    #   * load this package's DWB critics/costmap parameters correctly, and
    #   * route controller/recovery Twist commands to /stretch/cmd_vel.
    # The upstream launcher used here previously started extra smoother lifecycle nodes
    # and, in this image, left controller_server on its built-in defaults. DWB then
    # failed configuration with "No critics defined for FollowPath".
    # Include this directly, just as bringup_launch.py does for the working wheel-odom
    # navigation path. Wrapping the include in TimerAction caused the nested
    # params_file launch configuration to fall back to Nav2's node defaults, leaving
    # FollowPath.critics unset. Nav2 lifecycle activation already waits for required
    # transforms, so an artificial startup delay is neither needed nor desirable.
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_nav2_path, '/launch/navigation_launch.py']),
        launch_arguments={'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'params_file': okvis_nav_params}.items())

    # ---------------- goal recording + on-demand replay ----------------
    # Same pair as navigation_okvis_explore.launch.py: /record_goal to save named
    # poses, /goto/<name> to replay one (blocks until arrival, applies the yaw/
    # wall-square trim below, reports pos_err/yaw_err). Unlike explore, goals here
    # are kept across sessions (real map, not wiped on shutdown) since map->odom
    # is tied to a persistent, reloadable map rather than this session's start pose.
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

    return LaunchDescription([
        rviz_param,
        teleop_type,
        declare_use_sim_time_argument,
        autostart_param,
        map_path_param,
        reloc_param,
        marker_name_param,
        aruco_mode_param,
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
        publish_debug_image_param,
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
        # relocalization (map -> odom)
        map_server,
        amcl,
        localization_lifecycle,
        amcl_freeze,
        map_to_odom_identity,
        non_amcl_localization_lifecycle,
        aruco_detect,
        aruco_relocalizer,
        aruco_amcl_bridge,
        apriltag_detect,
        apriltag_relocalizer,
        apriltag_amcl_bridge,
        startup_posture,
        # navigation
        navigation_launch,
        rviz_launch,
        # goal recording + replay
        goal_recorder,
        goal_navigator,
    ])
