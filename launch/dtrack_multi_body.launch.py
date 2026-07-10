from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    udp_port = LaunchConfiguration("udp_port")
    frame_id = LaunchConfiguration("frame_id")
    parent_frame = LaunchConfiguration("parent_frame")
    max_path_length = LaunchConfiguration("max_path_length")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "udp_port",
                default_value="5000",
                description="UDP port receiving DTrack packets",
            ),
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
            DeclareLaunchArgument(
                "max_path_length",
                default_value="500",
                description="Maximum number of poses retained per body",
            ),
            Node(
                package="dtrack_publisher",
                executable="dtrack_multi_body_node",
                name="dtrack_multi_body_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "udp_port": ParameterValue(udp_port, value_type=int),
                        "frame_id": frame_id,
                        "parent_frame": parent_frame,
                        "max_path_length": ParameterValue(
                            max_path_length, value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
