"""OKVIS mapping + ArUco anchor recording, in one command.

Same as offline_okvis_mapping.launch.py but with the ArUco anchor recorder always
on (record_anchor:=true): it additionally runs detect_aruco_markers (5x5 dict) and
the aruco_anchor_recorder, and enables the RealSense color + aligned-depth streams
the detector needs.

Workflow:
  1. ros2 launch stretch_nav2 offline_okvis_mapping_with_anchor.launch.py \
         map_name:=<name>            # basename for both the map and <name>_anchor.yaml
  2. Drive/teleop to build the map as usual (octomap /projected_map).
  3. Park stationary with the head camera pointed at the fixed 'map_anchor' marker,
     then record the anchor:
         ros2 service call /record_anchor std_srvs/srv/Trigger
     -> writes ${HELLO_FLEET_PATH}/maps/<map_name>_anchor.yaml
  4. Save the grid:
         ros2 run nav2_map_server map_saver_cli -t /projected_map \
             -f ${HELLO_FLEET_PATH}/maps/<map_name>
  5. Later, navigate with:
         ros2 launch stretch_nav2 navigation_okvis.launch.py reloc:=aruco \
             map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml

All arguments of offline_okvis_mapping.launch.py (use_rviz, teleop_type,
marker_name, map_name, ...) are forwarded unchanged; record_anchor is forced true.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    stretch_nav2_path = get_package_share_directory('stretch_nav2')

    marker_name = DeclareLaunchArgument(
        'marker_name', default_value='map_anchor',
        description="ArUco marker NAME (from stretch_marker_dict.yaml) to anchor on "
                    "(default 'map_anchor', the 5x5 id-777 entry)")
    map_name = DeclareLaunchArgument(
        'map_name', default_value=os.environ.get('MAP_NAME', 'map'),
        description='Basename for the saved map and its <map_name>_anchor.yaml sidecar')

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [stretch_nav2_path, '/launch/offline_okvis_mapping.launch.py']),
        launch_arguments={
            'record_anchor': 'true',
            'marker_name': LaunchConfiguration('marker_name'),
            'map_name': LaunchConfiguration('map_name'),
        }.items())

    return LaunchDescription([
        marker_name,
        map_name,
        mapping,
    ])
