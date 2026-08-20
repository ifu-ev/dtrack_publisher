from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("dtrack_publisher"), "config", "dtrack_multi_body.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="dtrack_publisher",
                executable="dtrack_multi_body_node",
                name="dtrack_multi_body_node",
                output="screen",
                emulate_tty=True,
                parameters=[config_file],
            ),
        ]
    )
