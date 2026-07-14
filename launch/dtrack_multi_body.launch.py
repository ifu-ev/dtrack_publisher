from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    frame_id = LaunchConfiguration("frame_id")
    parent_frame = LaunchConfiguration("parent_frame")
    config_file = PathJoinSubstitution(
        [FindPackageShare("dtrack_publisher"), "config", "dtrack_multi_body.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "frame_id",
                default_value="odom_mocap",
                description="Frame used for published poses and paths",
            ),
            DeclareLaunchArgument(
                "parent_frame",
                default_value="odom",
                description="Parent of the DTrack frame in TF",
            ),
            Node(
                package="dtrack_publisher",
                executable="dtrack_multi_body_node",
                name="dtrack_multi_body_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    config_file,
                    {
                        "frame_id": frame_id,
                        "parent_frame": parent_frame,
                    }
                ],
            ),
        ]
    )
