import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    dict_file_path = os.path.join(get_package_share_directory('stretch_core'), 'config', 'stretch_marker_dict.yaml')

    # Which cv2.aruco predefined dictionary to detect. Default DICT_6X6_250 (Stretch
    # body markers 130-134 etc.); override e.g. DICT_5X5_1000 for a 5x5 anchor marker.
    aruco_dict_arg = DeclareLaunchArgument(
        'aruco_dict', default_value='DICT_6X6_250',
        description="cv2.aruco predefined dictionary name (e.g. DICT_6X6_250, DICT_5X5_1000)")

    detect_aruco_markers = Node(
        package='stretch_core',
        executable='detect_aruco_markers',
        output='screen',
        parameters=[dict_file_path,
                    {'aruco_dict': LaunchConfiguration('aruco_dict')}],
        )

    return LaunchDescription([
        aruco_dict_arg,
        detect_aruco_markers,
        ])
