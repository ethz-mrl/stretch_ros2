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

# OKVIS-only state estimation.
#
# OKVIS is the single source of state estimation: it localizes the camera and,
# via the robot model (base_link -> camera), we place base_link. Wheel odometry
# is ignored (broadcast_odom_tf: False) and gmapping plays NO role in TF.
#
# Resulting single-parent tree. map == odom is anchored to the INITIAL base_link
# pose (on the floor); `world` is OKVIS's internal origin (at sensor/head height).
# Only base_link moves.
#
#   map --(I)--> odom --(anchor)--> world --(OKVIS)--> base_link --(URDF)--> camera_*

def generate_launch_description():
    stretch_core_path = get_package_share_directory('stretch_core')

    stretch_okvis_share = get_package_share_directory('stretch_okvis')
    okvis_package = str(get_package_share_path("okvis"))

    rviz_param = DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false'])

    teleop_type = DeclareLaunchArgument(
        'teleop_type', default_value="joystick", description="how to teleop ('keyboard', 'joystick' or 'none')")

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    # Optional Phase-1 anchor recording. When record_anchor:=true, additionally
    # run a board/marker detector (needs color + aligned depth, enabled below) and
    # its matching anchor_recorder, chosen by anchor_detector. Park facing the
    # fixed board and call the /record_anchor service before saving the map; it
    # writes <map_name>_anchor.yaml that reloc:=<anchor_detector> later consumes.
    # Default false keeps plain OKVIS mapping intact.
    record_anchor_param = DeclareLaunchArgument(
        'record_anchor', default_value='true', choices=['true', 'false'],
        description='Also run a board/marker anchor recorder during mapping')
    anchor_detector_param = DeclareLaunchArgument(
        'anchor_detector', default_value='aruco', choices=['aruco', 'apriltag'],
        description="Which detector to run for anchor recording when "
                    "record_anchor:=true: 'aruco' (6x4 ChArUco board, "
                    "stretch_aruco_localizer) or 'apriltag' (6x6 AprilTag grid "
                    "board, stretch_apriltag_localizer). Should match the "
                    "reloc:=<...> value used later at mission time.")
    marker_name_param = DeclareLaunchArgument(
        'marker_name', default_value='map_anchor',
        description="TF frame name the chosen detector publishes the board pose as, "
                    "and that anchor_recorder anchors on (default 'map_anchor').")
    map_name_param = DeclareLaunchArgument(
        'map_name', default_value=os.environ.get('MAP_NAME', 'map'),
        description='Basename for the anchor sidecar (<map_name>_anchor.yaml)')
    publish_debug_image_param = DeclareLaunchArgument(
        'publish_debug_image', default_value='false', choices=['true', 'false'],
        description='Have the active anchor detector publish its annotated '
                    '~/debug_image (drawn markers/tags) for tuning')

    # Move to the manipulation posture on startup (one-shot). The driver runs in
    # gamepad mode here, which does NOT accept joint trajectory goals, so the posture
    # node briefly switches to position mode and back to gamepad (the base is not
    # touched). NOTE include_head:=true turns the head camera to the arm, which
    # disables OKVIS VIO while turned (so mapping cannot track meanwhile).
    startup_posture_param = DeclareLaunchArgument(
        'startup_posture', default_value='true', choices=['true', 'false'],
        description='Move to the manipulation posture at startup')
    include_head_param = DeclareLaunchArgument(
        'include_head', default_value='true', choices=['true', 'false'],
        description='Include head pan/tilt in the startup posture (points camera at '
                    'the arm; breaks OKVIS while turned)')

    record_anchor = LaunchConfiguration('record_anchor')
    anchor_detector = LaunchConfiguration('anchor_detector')
    marker_name = LaunchConfiguration('marker_name')
    map_name = LaunchConfiguration('map_name')
    publish_debug_image = LaunchConfiguration('publish_debug_image')
    use_aruco_anchor = IfCondition(PythonExpression(
        ["'", record_anchor, "' == 'true' and '", anchor_detector, "' == 'aruco'"]))
    use_apriltag_anchor = IfCondition(PythonExpression(
        ["'", record_anchor, "' == 'true' and '", anchor_detector, "' == 'apriltag'"]))
    # Color + aligned depth are needed only when recording an anchor (either
    # detector consumes them); off otherwise so OKVIS keeps the IR/IMU bandwidth
    # to itself.
    anchor_stream = PythonExpression(
        ["'true' if '", record_anchor, "' == 'true' else 'false'"])
    # 1280x720 gives more pixels-per-marker than 640x480 for both anchor
    # detectors (confirmed on hardware for the AprilTag grid board: 640x480
    # sometimes decoded 0/36 tags in a session where a 1280x720 grab of the
    # same board decoded roughly half). Color is unused by OKVIS either way
    # (it reads the IR streams), so bumping it doesn't cost OKVIS anything.
    # Only takes effect when record_anchor:=true (anchor_stream gates
    # enable_color); irrelevant otherwise.
    color_profile = PythonExpression([
        "'1280x720x15' if '", record_anchor, "' == 'true' else '640x480x15'"])

    # Wheel-odometry TF disabled: OKVIS is the only state estimator. The driver
    # still publishes the /odom topic and the URDF (robot_state_publisher).
    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'gamepad', 'broadcast_odom_tf': 'False'}.items())

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/rplidar.launch.py']))

    # Launch RViz directly. nav2_bringup/rviz_launch.py deliberately shuts down the
    # entire launch when RViz exits; here that would also kill OKVIS, octomap_server
    # and the rest of the mapping stack just because the GUI was closed or its OpenGL
    # process crashed. Use the operator's tuned RViz config from stretch_user rather
    # than nav2_bringup's stock view.
    rviz_config = os.path.join(
        os.environ.get('HELLO_FLEET_PATH', '/home/hello-robot/stretch_user'),
        'nav2_default_view.rviz')
    rviz_launch = Node(
        package='rviz2', executable='rviz2', name='rviz', output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz')))

    # OKVIS via the subscriber node: unlike okvis_node_realsense, it does NOT open the
    # camera; it only subscribes to /okvis/cam0|cam1/image_raw. Built via a factory so
    # a fresh action can be handed to whichever startup branch runs below (the same
    # launch action object must not be reused in two places).
    def make_okvis_launch():
        return IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                os.path.join(okvis_package, 'launch', 'okvis_node_subscriber.launch.xml')
            ]),
            launch_arguments={
                'rviz': 'false',
                # 2-camera stereo-IR config (no RGB "map" camera) is enough for state estimation
                'config_filename': os.path.join(okvis_package, 'config', 'realsense_D435if_stretch_bag.yaml'),
            }.items()
        )

    # Live RealSense D435i as the OKVIS input source. Enable stereo IR (infra1/infra2,
    # 640x480 Y8) + a united gyro/accel IMU, then remap the driver's native topics onto
    # the names okvis_node_subscriber expects. NOTE: initial_reset is left False on
    # purpose -- resetting Stretch's head D435i kills its depth module until a reboot.
    realsense_package = get_package_share_directory('realsense2_camera')

    realsense_params_file = os.path.join(stretch_okvis_share, 'config', 'okvis_realsense_params.yaml')
    realsense_launch = GroupAction([
        SetRemap('/camera/infra1/image_rect_raw', '/okvis/cam0/image_raw'),
        SetRemap('/camera/infra2/image_rect_raw', '/okvis/cam1/image_raw'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([realsense_package, '/launch/rs_launch.py']),
            launch_arguments={
                'camera_namespace': '',
                'camera_name': 'camera',
                # color + depth + aligned-depth only when recording an ArUco anchor.
                'enable_color': anchor_stream,
                'enable_depth': anchor_stream,
                'align_depth.enable': anchor_stream,
                # Color resolution is bumped (see color_profile above) only when
                # recording an anchor; 15 fps (vs default 1280x720x30) still keeps
                # USB/CPU load down for OKVIS's IR streams.
                'rgb_camera.color_profile': color_profile,
                'enable_infra1': 'true',
                'enable_infra2': 'true',
                'depth_module.infra_profile': '640x480x15',
                # Depth shares the one D435i stereo module with infra1/2, so its
                # profile MUST match the infra profile (res + fps) or the driver
                # brings up IR and silently drops depth. Without depth, the ArUco
                # detector's color+depth TimeSynchronizer never fires.
                'depth_module.depth_profile': '640x480x15',
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

    # Anchor map == odom to the INITIAL base_link pose (on the floor), not to the
    # OKVIS origin (which sits at the sensor/head height). map->odom stays identity;
    # map_anchor captures the first base_link->world transform and republishes it as
    # a static odom->world, so map == odom == base_link(t=0). See scripts/map_anchor.py.
    map_to_odom = static_tf('map_to_odom', 'map', 'odom')
    map_anchor = Node(
        package='stretch_okvis', executable='map_anchor', name='map_anchor',
        output='screen')

    # Bridge RealSense IMU (best_effort) -> /okvis/imu0 (reliable) to fix the QoS mismatch.
    imu_qos_bridge = Node(
        package='stretch_okvis', executable='imu_qos_bridge', name='imu_qos_bridge',
        output='screen',
        parameters=[{'input_topic': '/camera/imu', 'output_topic': '/okvis/imu0'}])

    # OKVIS's sensor frame S == IMU frame == camera_gyro_optical_frame (on the D435i);
    # OKVIS reads camera_gyro_optical_frame->base_link directly from the URDF TF tree
    # to build world->base_link as the true base pose (no alias frame needed).

    # --- 2D occupancy mapping with KNOWN POSES (OKVIS), no extra SLAM ---
    # octomap_server integrates the lidar using the sensor->map transform from TF
    # (driven solely by OKVIS); it does no pose estimation of its own. It wants a
    # PointCloud2, so convert the rplidar /scan first. Output: /projected_map (2D
    # OccupancyGrid), savable with `ros2 run nav2_map_server map_saver_cli -t /projected_map`.
    scan_to_cloud = Node(
        package='pointcloud_to_laserscan', executable='laserscan_to_pointcloud_node',
        name='scan_to_cloud', output='screen',
        # target_frame must be set: the node's tf2 MessageFilter never fires when it is
        # blank. Keep the cloud in the lidar frame so octomap raycasts from the true
        # sensor origin (transforming laser->map itself via the OKVIS TF chain).
        parameters=[{'target_frame': 'laser', 'transform_tolerance': 0.05}],
        remappings=[('scan_in', '/scan'), ('cloud', '/lidar_cloud')])

    octomap_server = Node(
        package='octomap_server', executable='octomap_server_node', name='octomap_server',
        output='screen',
        parameters=[{
            'resolution': 0.05,
            'frame_id': 'map',            # accumulate in the OKVIS-anchored, floor-level map frame
            'base_frame_id': 'base_link',
            'sensor_model.max_range': 10.0,
            'filter_ground': False,       # single horizontal lidar ring, nothing to filter
            'latch': True,                # publish /projected_map transient_local (latched)
                                          # so RViz Map display and map_saver_cli can receive it
        }],
        remappings=[('cloud_in', '/lidar_cloud')])

    # Phase-1 anchor recording (only when record_anchor:=true), detector chosen by
    # anchor_detector. Both publish the board pose as TF camera_color_optical_frame
    # -> <marker_name>, which the matching anchor_recorder consumes identically;
    # only one pair is ever active (mutually exclusive conditions).

    # ChArUco board detector (replaces the single-marker stretch_core detector).
    # Board is 6 columns x 4 rows, 66 mm squares, 49 mm markers, DICT_4X4.
    aruco_detect = Node(
        package='stretch_aruco_localizer', executable='charuco_detector',
        name='charuco_detector', output='screen', condition=use_aruco_anchor,
        parameters=[{
            'squares_x': 6, 'squares_y': 4,
            'square_length_m': 0.066, 'marker_length_m': 0.049,
            'aruco_dict': 'DICT_4X4_50', 'legacy_pattern': True,
            'min_charuco_corners': 4,
            'marker_name': marker_name,
            'publish_debug_image': publish_debug_image,
        }])
    aruco_anchor_recorder = Node(
        package='stretch_aruco_localizer', executable='aruco_anchor_recorder',
        name='aruco_anchor_recorder', output='screen', condition=use_aruco_anchor,
        parameters=[{'marker_name': marker_name, 'map_name': map_name}])

    # AprilTag grid-board detector: 6x6 grid, 88 mm tags, 26.4 mm separation,
    # DICT_APRILTAG_36H11 (defaults already match this rig; not overridden here).
    apriltag_detect = Node(
        package='stretch_apriltag_localizer', executable='apriltag_grid_detector',
        name='apriltag_grid_detector', output='screen', condition=use_apriltag_anchor,
        parameters=[{'marker_name': marker_name, 'publish_debug_image': publish_debug_image}])
    apriltag_anchor_recorder = Node(
        package='stretch_apriltag_localizer', executable='apriltag_anchor_recorder',
        name='apriltag_anchor_recorder', output='screen', condition=use_apriltag_anchor,
        parameters=[{'marker_name': marker_name, 'map_name': map_name}])

    # Demo-goal recording is always available during mapping (needs only the
    # map->base_link TF, no camera): drive to a pose, call /record_goal, and it
    # appends to <map_name>_goals.yaml. goal_sender replays these in a mission.
    goal_recorder = Node(
        package='stretch_aruco_localizer', executable='goal_recorder',
        name='goal_recorder', output='screen',
        parameters=[{'map_name': map_name}])

    # One-shot manipulation posture at startup. Driver is in gamepad mode, which
    # rejects joint trajectory goals, so switch to position to command, then restore
    # gamepad for teleop driving (base is untouched by the posture).
    # For mapping, lower the arm 20 cm along z vs the default posture (lift 0.78 ->
    # 0.58) so the raised arm is less likely to occlude the sensors / snag while
    # teleoperating the map.
    startup_posture = Node(
        package='stretch_aruco_localizer', executable='go_to_posture',
        name='go_to_posture', output='screen',
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('startup_posture'), "' == 'true'"])),
        parameters=[{'include_head': LaunchConfiguration('include_head'),
                     'move_mode': 'position',
                     'restore_mode': 'gamepad',
                     'joint_lift': 0.58,
                     # Keep the head level (do NOT tilt down): the default posture
                     # tilts the head -pi/4 to look at the arm, but a downward tilt
                     # points the camera at the floor and hurts OKVIS. Override to 0.
                     'joint_head_tilt': 0.0}])

    # Defer OKVIS until the startup posture has finished moving AND the arm/lift have
    # settled for a few seconds. OKVIS bootstraps its VI-SLAM from the first frames, so
    # arm motion / vibration in view during initialization degrades the estimate.
    # go_to_posture is one-shot (it exits once the posture is reached), so start a short
    # settle timer on its exit and only THEN bring OKVIS up.
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
    okvis_no_posture = GroupAction(
        [make_okvis_launch()],
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('startup_posture'), "' == 'false'"])))

    ld = LaunchDescription([
        rviz_param,
        teleop_type,
        declare_use_sim_time_argument,
        record_anchor_param,
        anchor_detector_param,
        marker_name_param,
        map_name_param,
        publish_debug_image_param,
        startup_posture_param,
        include_head_param,
        stretch_driver_launch,
        rplidar_launch,
        rviz_launch,
        realsense_launch,
        okvis_after_posture,
        okvis_no_posture,
        map_to_odom,
        map_anchor,
        imu_qos_bridge,
        scan_to_cloud,
        octomap_server,
        aruco_detect,
        aruco_anchor_recorder,
        apriltag_detect,
        apriltag_anchor_recorder,
        goal_recorder,
        startup_posture,
    ])

    return ld
