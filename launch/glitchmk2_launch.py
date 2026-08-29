from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from math import pi


def generate_launch_description():
    glitchmk2_package_path = get_package_share_path("glitchmk2_bringup")
    joy_config_path = glitchmk2_package_path / "config/gamepad.yaml"
    ekf_config_path = glitchmk2_package_path / "config/ekf.yaml"

    sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Flag to enable use simulation time",
    )

    footprint_static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "-0.0325",
            "--yaw",
            "0",
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "base_footprint",
        ],
    )

    imu_static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0.07",
            "--yaw",
            "0",
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "imu_link",
        ],
    )

    lidar_static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "-0.021",
            "--y",
            "0",
            "--z",
            "0.12",
            "--yaw",
            str(pi),
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "lidar_link",
        ],
    )

    pico_interface_node = Node(package="glitchmk2_bringup", executable="pico_interface")

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            str(ekf_config_path),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(glitchmk2_package_path / "launch/rplidar_launch.py")
        ),
    )

    launch_teleop_twist_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path("teleop_twist_joy") / "launch/teleop-launch.py")
        ),
        launch_arguments={
            "config_filepath": str(joy_config_path),
        }.items(),
    )

    return LaunchDescription(
        [
            sim_time_arg,
            pico_interface_node,
            robot_localization_node,
            rplidar_launch,
            launch_teleop_twist_joy,
            footprint_static_tf_node,
            lidar_static_tf_node,
            imu_static_tf_node,
        ]
    )
