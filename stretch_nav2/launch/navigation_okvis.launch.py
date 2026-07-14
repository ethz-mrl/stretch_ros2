import os
import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory, get_package_share_path


def _write_okvis_nav_params(source_params, bt_nav_to_pose, bt_nav_through_poses):
    """Produce the OKVIS-tuned nav2 params file WITHOUT touching the shared
    nav2_params.yaml. Read the pristine params (used as-is by the wheel-odometry
    navigation.launch.py) and apply every OKVIS-navigation-specific override here,
    so all tuning lives in this launch file. Returns the path to a generated temp
    file. RewrittenYaml can only rewrite pre-existing keys and struggles with list
    values and adding the (Humble) BT-xml keys, so we edit the parsed YAML directly.
    """
    with open(source_params) as f:
        cfg = yaml.safe_load(f)

    cs = cfg['controller_server']['ros__parameters']
    # Tight goal tolerances for accurate arrival (was 0.25 / 0.25).
    cs['general_goal_checker']['xy_goal_tolerance'] = 0.05
    cs['general_goal_checker']['yaw_goal_tolerance'] = 0.5
    cs['FollowPath']['xy_goal_tolerance'] = 0.5
    # Must nearly stop before the final rotate-to-goal, so it settles precisely.
    cs['FollowPath']['trans_stopped_velocity'] = 0.05

    for scope in ('local_costmap', 'global_costmap'):
        p = cfg[scope][scope]['ros__parameters']
        # Inflation kept > the 0.22 m inscribed radius but well below the 0.55 default
        # so DWB can use lab passages that are physically safe for Stretch.
        p['inflation_layer']['inflation_radius'] = 0.3
        # OKVIS's 6-DoF pose can put the floor lidar a few cm below odom z=0; start the
        # voxel column below zero so the scan still raytraces and clears stale cells.
        p['voxel_layer']['origin_z'] = -0.10

    bs = cfg['behavior_server']['ros__parameters']
    # Wait-only recovery: spin/backup are disruptive on Stretch. Drop them (and their
    # now-unused plugin blocks) so only Wait remains.
    for plug in ('spin', 'backup', 'drive_on_heading', 'assisted_teleop'):
        bs.pop(plug, None)
    bs['behavior_plugins'] = ['wait']
    bs['wait'] = {'plugin': 'nav2_behaviors/Wait'}

    bn = cfg['bt_navigator']['ros__parameters']
    # Humble param names (`default_bt_xml_filename` is silently ignored). Point at the
    # wait-only-recovery trees. BOTH must be set: bt_navigator loads the nav-to-pose AND
    # nav-through-poses defaults at configure time, and the upstream trees invoke Spin/
    # BackUp whose (disabled) servers would abort bt_navigator startup.
    bn.pop('default_bt_xml_filename', None)
    bn['default_nav_to_pose_bt_xml'] = bt_nav_to_pose
    bn['default_nav_through_poses_bt_xml'] = bt_nav_through_poses

    fd, path = tempfile.mkstemp(prefix='okvis_nav2_params_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(cfg, f, default_flow_style=False)
    return path


def _write_okvis_vio_config(source_cfg):
    """OKVIS's config uses do_loop_closures:true (full VI-SLAM). Loop closures re-
    optimize the pose graph and make world->base_link JUMP discretely -- fine for
    building a map, but nav2/AMCL assume odom->base_link is SMOOTH, so a jumping
    odometry makes the robot pose lurch, AMCL over-corrects, and the goal appears to
    jump. For navigation we want pure VIO (smooth odometry; AMCL supplies the global
    map correction). Flip that one flag via a text substitution -- the file is OpenCV
    `%YAML:1.0`, not PyYAML-parseable -- and write a temp config. No okvis fork edit.
    """
    with open(source_cfg) as f:
        text = f.read()
    patched = text.replace('do_loop_closures: true', 'do_loop_closures: false')
    if patched == text:
        raise RuntimeError('did not find `do_loop_closures: true` in ' + source_cfg)
    fd, path = tempfile.mkstemp(prefix='okvis_vio_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        f.write(patched)
    return path

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
        'teleop_type', default_value="none", description="how to teleop ('keyboard', 'joystick' or 'none')")

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
        'reloc', default_value='amcl',
        choices=['amcl', 'amcl_oneshot', 'none', 'hloc'],
        description="How map->odom (relocalization) is provided: 'amcl' (lidar vs "
                    "saved grid, continuous), 'amcl_oneshot' (AMCL corrects the initial "
                    "2D Pose Estimate ONCE, then freezes map->odom so OKVIS carries a "
                    "smooth pose with no further jumps) or 'none' (identity static; "
                    "map == start pose)")

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml = LaunchConfiguration('map')
    reloc = LaunchConfiguration('reloc')

    # ALL OKVIS-navigation param overrides live here (see _write_okvis_nav_params):
    # goal tolerances, inflation, wait-only recovery + BT-xml, and voxel origin_z. The
    # shared nav2_params.yaml stays pristine for the wheel-odom navigation.launch.py.
    source_params = os.path.join(stretch_nav2_path, 'config', 'nav2_params.yaml')
    wait_only_bt = os.path.join(stretch_nav2_path, 'config',
                                'navigate_to_pose_wait_only_recovery.xml')
    wait_only_bt_through = os.path.join(stretch_nav2_path, 'config',
                                        'navigate_through_poses_wait_only_recovery.xml')
    okvis_nav_params = _write_okvis_nav_params(
        source_params, wait_only_bt, wait_only_bt_through)

    # map_server + AMCL run for BOTH amcl and amcl_oneshot (same lidar-vs-grid setup);
    # oneshot additionally runs amcl_freeze, which deactivates AMCL after it converges.
    use_amcl = IfCondition(
        PythonExpression(["'", reloc, "' in ('amcl', 'amcl_oneshot')"]))
    use_oneshot = IfCondition(PythonExpression(["'", reloc, "' == 'amcl_oneshot'"]))
    no_reloc = IfCondition(PythonExpression(["'", reloc, "' == 'none'"]))

    # ---------------- OKVIS state-estimation stack (from offline_okvis_mapping) ----
    # Wheel-odom TF stays OFF: OKVIS is the only odometry. Driver still publishes the
    # /odom topic + URDF. mode:=navigation so the base accepts /stretch/cmd_vel.
    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'navigation', 'broadcast_odom_tf': 'False'}.items())

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/rplidar.launch.py']))

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
    # Run OKVIS with loop closures ENABLED (full VI-SLAM, do_loop_closures:true from the
    # stock config). Loop closures re-optimize the pose graph and can make odom->base_link
    # jump discretely; AMCL/costmap must tolerate that.
    okvis_launch = GroupAction([
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
    ])

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
                'enable_color': 'false',
                'enable_depth': 'false',
                'enable_infra1': 'true',
                'enable_infra2': 'true',
                'depth_module.infra_profile': '640,480,15',
                'depth_module.infra1_format': 'Y8',
                'depth_module.infra2_format': 'Y8',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': '2',
                'initial_reset': 'false',
                'config_file': realsense_params_file,
            }.items()),
    ])

    def static_tf(name, parent, child):
        return Node(
            package='tf2_ros', executable='static_transform_publisher', name=name,
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', parent, '--child-frame-id', child])

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

    # OKVIS sensor frame S == IMU frame == camera_gyro_optical_frame.
    okvis_sensor_frame = static_tf(
        'okvis_sensor_frame', 'camera_gyro_optical_frame', 'camera_okvis_sensor_frame')

    # ---------------- relocalization: the map -> odom edge ----------------
    # reloc:=amcl -> map_server serves the saved grid; AMCL publishes map->odom by
    # matching /scan against it. AMCL owns map->odom; OKVIS owns odom->base_link, so
    # there is NO TF conflict. base_frame_id overridden to base_link (no base_footprint
    # in the Stretch URDF). Seed the initial pose with RViz "2D Pose Estimate".
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', condition=use_amcl,
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

    return LaunchDescription([
        rviz_param,
        teleop_type,
        declare_use_sim_time_argument,
        autostart_param,
        map_path_param,
        reloc_param,
        # OKVIS odometry stack
        stretch_driver_launch,
        rplidar_launch,
        realsense_launch,
        okvis_nav_tf_bridge,
        okvis_launch,
        map_anchor,
        imu_qos_bridge,
        okvis_sensor_frame,
        # relocalization (map -> odom)
        map_server,
        amcl,
        localization_lifecycle,
        amcl_freeze,
        map_to_odom_identity,
        # navigation
        navigation_launch,
        rviz_launch,
    ])
