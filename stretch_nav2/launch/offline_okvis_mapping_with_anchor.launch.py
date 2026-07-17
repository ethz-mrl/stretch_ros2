"""OKVIS mapping + board anchor recording, in one command.

Same as offline_okvis_mapping.launch.py but with anchor recording always on
(record_anchor:=true): it additionally runs the anchor_detector's board detector +
matching anchor_recorder, and enables the RealSense color + aligned-depth streams
the detector needs.

Workflow:
  1. ros2 launch stretch_nav2 offline_okvis_mapping_with_anchor.launch.py \
         map_name:=<name> anchor_detector:=aruco   # or apriltag
  2. Drive/teleop to build the map as usual (octomap /projected_map).
  3. Park stationary with the head camera pointed at the fixed board, then
     record the anchor:
         ros2 service call /record_anchor std_srvs/srv/Trigger
     -> writes ${HELLO_FLEET_PATH}/maps/<map_name>_anchor.yaml
  4. Save the grid:
         ros2 run nav2_map_server map_saver_cli -t /projected_map \
             -f ${HELLO_FLEET_PATH}/maps/<map_name>
  5. Later, navigate with:
         ros2 launch stretch_nav2 navigation_okvis.launch.py reloc:=<anchor_detector> \
             map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml

All arguments of offline_okvis_mapping.launch.py (use_rviz, teleop_type,
anchor_detector, marker_name, map_name, publish_debug_image, ...) are forwarded
unchanged; record_anchor is forced true.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    stretch_nav2_path = get_package_share_directory('stretch_nav2')

    anchor_detector = DeclareLaunchArgument(
        'anchor_detector', default_value='aruco', choices=['aruco', 'apriltag'],
        description="Which detector to run for anchor recording: 'aruco' (6x4 "
                    "ChArUco board, stretch_aruco_localizer) or 'apriltag' (6x6 "
                    "AprilTag grid board, stretch_apriltag_localizer). Should match "
                    "the reloc:=<...> value used later at mission time.")
    marker_name = DeclareLaunchArgument(
        'marker_name', default_value='map_anchor',
        description="Marker/board NAME to anchor on (default 'map_anchor').")
    map_name = DeclareLaunchArgument(
        'map_name', default_value=os.environ.get('MAP_NAME', 'map'),
        description='Basename for the saved map and its <map_name>_anchor.yaml sidecar')
    publish_debug_image = DeclareLaunchArgument(
        'publish_debug_image', default_value='true', choices=['true', 'false'],
        description='Have the active anchor detector publish its annotated '
                    '~/debug_image (drawn markers/tags) for tuning')

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [stretch_nav2_path, '/launch/offline_okvis_mapping.launch.py']),
        launch_arguments={
            'record_anchor': 'true',
            'anchor_detector': LaunchConfiguration('anchor_detector'),
            'marker_name': LaunchConfiguration('marker_name'),
            'map_name': LaunchConfiguration('map_name'),
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
        }.items())

    return LaunchDescription([
        anchor_detector,
        marker_name,
        map_name,
        publish_debug_image,
        mapping,
    ])
