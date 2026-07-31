import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

json_path = os.path.join(get_package_share_directory('stretch_core'), 'config', 'HighAccuracyPreset.json')

# Calls realsense2_camera's own rs_launch.launch_setup() directly (OpaqueFunction) instead of
# IncludeLaunchDescription(rs_launch.py) -- confirmed on hardware that going through
# IncludeLaunchDescription forwards depth_module.color_profile (the D405's color stream
# override) into rs_launch's own DeclareLaunchArgument-based parameter set (all ~70 of
# rs_launch.py's own configurable_parameters get re-declared+forwarded), and at that
# parameter volume the color profile is silently dropped at node startup -- the D405 opens
# color at its own auto-negotiated default (848x480x10) instead of the requested profile,
# needing a live `ros2 param set` + stream restart afterward to correct it. Calling
# launch_setup directly with just the parameters this file actually needs (~25, vs rs_launch's
# ~70) avoids the drop entirely -- color opens at the requested profile on the very first
# start. depth_module.depth_profile/infra_profile were never affected by this (they always
# applied correctly even via IncludeLaunchDescription); only color_profile was.
sys.path.insert(0, os.path.join(get_package_share_directory('realsense2_camera'), 'launch'))
import rs_launch  # noqa: E402

configurable_parameters = [
                           {'name': 'camera_namespace',             'default': '', 'description': 'namespace for camera'},
                           {'name': 'camera_name',                  'default': 'gripper_camera', 'description': 'camera unique name'},
                           {'name': 'device_type',                  'default': 'd405', 'description': 'camera unique name'},
                           {'name': 'config_file',                  'default': "''", 'description': 'yaml config file'},
                           {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'log_level',                    'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'json_file_path',               'default': json_path, 'description': 'allows advanced configuration'},
                           {'name': 'depth_module.depth_profile',   'default': '640x480x15', 'description': 'depth module profile'},
                           {'name': 'depth_module.infra_profile',   'default': '640x480x15', 'description': 'depth module profile'},
                           {'name': 'depth_module.enable_auto_exposure', 'default': 'true', 'description': 'enable/disable auto exposure for depth image'},
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'depth_module.color_profile',   'default': '640x480x15', 'description': 'color image profile (D405: color lives under the depth module)'},
                           {'name': 'rgb_camera.enable_auto_exposure', 'default': 'true', 'description': 'enable/disable auto exposure for color image'},
                           {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'enable_confidence',            'default': 'false', 'description': 'enable depth stream'},
                           {'name': 'gyro_fps',                     'default': '200', 'description': "''"},
                           {'name': 'accel_fps',                    'default': '100', 'description': "''"},
                           {'name': 'enable_gyro',                  'default': 'true', 'description': "''"},
                           {'name': 'enable_accel',                 'default': 'true', 'description': "''"},
                           {'name': 'pointcloud.enable',            'default': 'true', 'description': ''}, 
                           {'name': 'pointcloud.stream_filter',     'default': '2', 'description': 'texture stream for pointcloud'},
                           {'name': 'pointcloud.stream_index_filter','default': '0', 'description': 'texture stream index for pointcloud'},
                           {'name': 'enable_sync',                  'default': 'true', 'description': "''"},
                           {'name': 'align_depth.enable',           'default': 'true', 'description': "''"},
                           {'name': 'initial_reset',                'default': 'false', 'description': "''"},
                           {'name': 'pointcloud.allow_no_texture_points', 'default': 'true', 'description': "''"},
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def generate_launch_description():
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        OpaqueFunction(function=rs_launch.launch_setup,
                       kwargs={'params': rs_launch.set_configurable_parameters(configurable_parameters)}),
    ])
