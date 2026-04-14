from platform import system

from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
import launch_ros.parameter_descriptions
from launch_ros.actions import Node
import os

if system() == "Linux":
    # this fixes rviz launch issue
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = (
        "/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms/libqxcb.so"
    )
    os.environ["GTK_PATH"] = ""


def generate_launch_description():

    stretch_simulation_path = get_package_share_path("stretch_simulation")
    stretch_description_path = get_package_share_path("stretch_description")

    ld = LaunchDescription()

    # Declare launch arguments
    default_params_file = str(stretch_simulation_path / "config" / "stretch_mujoco_driver.yaml")

    ld.add_action(
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Path to ROS parameter file for stretch_mujoco_driver node",
        )
    )

    # Robot description setup
    calibrated_urdf = stretch_description_path / "urdf" / "stretch.urdf"
    uncalibrated_urdf = (
        stretch_description_path / "urdf" / "stretch_description_SE3_eoa_wrist_dw3_tool_sg3.xacro"
    )

    if calibrated_urdf.is_file():
        robot_description_content = launch_ros.parameter_descriptions.ParameterValue(
            Command(["xacro ", str(calibrated_urdf)]), value_type=str
        )
    else:
        robot_description_content = launch_ros.parameter_descriptions.ParameterValue(
            Command(["xacro ", str(uncalibrated_urdf)]), value_type=str
        )

    # Joint state publisher
    ld.add_action(
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="log",
            parameters=[
                {"source_list": ["/stretch/joint_states"]},
                {"rate": 30.0},
                {"robot_description": robot_description_content},
            ],
            arguments=["--ros-args", "--log-level", "error"],
        )
    )

    # Robot state publisher
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[
                {"robot_description": robot_description_content},
                {"publish_frequency": 30.0},
            ],
            arguments=["--ros-args", "--log-level", "error"],
        )
    )

    # Stretch Mujoco Driver - load parameters from file
    ld.add_action(
        Node(
            package="stretch_simulation",
            executable="stretch_mujoco_driver",
            emulate_tty=True,
            output="screen",
            remappings=[
                ("cmd_vel", "/stretch/cmd_vel"),
                ("joint_states", "/stretch/joint_states"),
            ],
            parameters=[LaunchConfiguration("params_file")],
        )
    )

    return ld
