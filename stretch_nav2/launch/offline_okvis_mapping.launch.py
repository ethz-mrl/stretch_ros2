import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_share_path

# as a replacement for offline_mapping, we use okvis to compute the pose of the base,
# and use gmapping to construct 2D map for nav2

def generate_launch_description():
    stretch_core_path = get_package_share_directory('stretch_core')
    nav2_bringup_package = str(get_package_share_path("nav2_bringup"))
    gmapping_package = str(get_package_share_path("slam_gmapping"))
    okvis_package = str(get_package_share_path("okvis"))

    rviz_param = DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false'])
    
    teleop_type = DeclareLaunchArgument(
        'teleop_type', default_value="joystick", description="how to teleop ('keyboard', 'joystick' or 'none')")
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'gamepad', 'broadcast_odom_tf': 'True'}.items())

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/rplidar.launch.py']))

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_package, '/launch/rviz_launch.py']),
        condition=IfCondition(LaunchConfiguration('use_rviz'))) 
    
    gmapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gmapping_package, '/launch/slam_gmapping.launch.py']))
    
    
    okvis_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([
            os.path.join(okvis_package, 'launch', 'okvis_node_realsense.launch.xml')
        ]),
        launch_arguments={'rviz': 'false'}.items()
    )

    ld = LaunchDescription([
        rviz_param,
        teleop_type,
        declare_use_sim_time_argument,
        stretch_driver_launch,
        rplidar_launch,
        rviz_launch,
        gmapping_launch,
        okvis_launch,
    ])

    return ld