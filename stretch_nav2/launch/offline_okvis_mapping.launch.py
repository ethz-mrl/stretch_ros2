import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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
    nav2_bringup_package = str(get_package_share_path("nav2_bringup"))
    
    stretch_okvis_share = get_package_share_directory('stretch_okvis')
    okvis_package = str(get_package_share_path("okvis"))

    rviz_param = DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false'])

    teleop_type = DeclareLaunchArgument(
        'teleop_type', default_value="joystick", description="how to teleop ('keyboard', 'joystick' or 'none')")

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    # Wheel-odometry TF disabled: OKVIS is the only state estimator. The driver
    # still publishes the /odom topic and the URDF (robot_state_publisher).
    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'gamepad', 'broadcast_odom_tf': 'False'}.items())

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/rplidar.launch.py']))

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_package, '/launch/rviz_launch.py']),
        condition=IfCondition(LaunchConfiguration('use_rviz')))

    # OKVIS via the subscriber node: unlike okvis_node_realsense, it does NOT open the
    # camera; it only subscribes to /okvis/cam0|cam1/image_raw
    okvis_launch = IncludeLaunchDescription(
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

    # Tell OKVIS where its sensor frame sits on the robot: S == the IMU frame
    # (per OKVIS Parameters.hpp), which on the D435i is camera_gyro_optical_frame.
    # OKVIS reads camera_okvis_sensor_frame->base_link from TF to build world->base_link as the true base pose.
    okvis_sensor_frame = static_tf(
        'okvis_sensor_frame', 'camera_gyro_optical_frame', 'camera_okvis_sensor_frame')

    ld = LaunchDescription([
        rviz_param,
        teleop_type,
        declare_use_sim_time_argument,
        stretch_driver_launch,
        rplidar_launch,
        rviz_launch,
        realsense_launch,
        okvis_launch,
        map_to_odom,
        map_anchor,
        imu_qos_bridge,
        okvis_sensor_frame,
    ])

    return ld
