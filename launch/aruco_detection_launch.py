from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rviz_config_path = get_package_share_path("glitchmk2_bringup") / "rviz/camera.rviz"

    rviz_arg = DeclareLaunchArgument(
        name="rvizconfig",
        default_value=str(rviz_config_path),
        description="Absolute path to rviz config file",
    )

    camera_node = Node(package='camera_ros', executable='camera_node')

    aruco_detection_node = Node(package="glitchmk2_bringup", executable="aruco_detector")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rvizconfig")],
    )

    return LaunchDescription(
        [
            rviz_arg,
            camera_node,
            aruco_detection_node,
            rviz_node,
        ]
    )
