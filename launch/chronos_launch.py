"""Camera plus Chronos recorder or pilot.

Run the base first (glitchmk2_launch.py brings up the Pico, joy and teleop), then:
    ros2 launch glitchmk2_bringup chronos_launch.py mode:=record
    ros2 launch glitchmk2_bringup chronos_launch.py mode:=pilot model_path:=~/pilot.pth
"""
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    chronos_config_path = get_package_share_path("glitchmk2_bringup") / "config/chronos.yaml"

    mode_arg = DeclareLaunchArgument(
        name="mode",
        default_value="record",
        choices=["record", "pilot"],
        description="record: save training data, pilot: drive with the model",
    )
    model_path_arg = DeclareLaunchArgument(
        name="model_path",
        default_value="~/glitchmk2_models/pilot.pth",
        description="state_dict of a ChronosResNetLite model (pilot mode)",
    )

    camera_node = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera",
        parameters=[str(chronos_config_path)],
    )

    recorder_node = Node(
        package="glitchmk2_bringup",
        executable="chronos_recorder",
        output="screen",
        parameters=[str(chronos_config_path)],
        condition=LaunchConfigurationEquals("mode", "record"),
    )

    pilot_node = Node(
        package="glitchmk2_bringup",
        executable="chronos_pilot",
        output="screen",
        parameters=[
            str(chronos_config_path),
            {"model_path": LaunchConfiguration("model_path")},
        ],
        condition=LaunchConfigurationEquals("mode", "pilot"),
    )

    return LaunchDescription(
        [
            mode_arg,
            model_path_arg,
            camera_node,
            recorder_node,
            pilot_node,
        ]
    )
