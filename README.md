# dtrack_publisher

ROS 2 Jazzy package that receives multi-body tracking data from a DTrack
motion-capture system over UDP. Each detected body is published as a stamped
pose and a bounded path. The node also publishes the identity transform between
the configured odometry and motion-capture frames.

## Behavior

- Listens for DTrack ASCII packets on UDP port 5000 by default.
- Processes each complete body in a DTrack 6d line.
- Converts positions from millimetres to metres.
- Converts the DTrack rotation matrix to a normalized quaternion.
- Keeps positions in the calibrated DTrack coordinate-system origin.
- Maps raw DTrack body ID 0 to body_1, raw ID 1 to body_2, and so on.
- Reads DTrack quality values without filtering bodies by quality.

## Build

Source ROS 2 Jazzy and build the package from the workspace root:

    source /opt/ros/jazzy/setup.bash
    cd /home/ros2/Projects/kalimoro_ros2/kalimoro_ws
    colcon build --packages-select dtrack_publisher
    source install/setup.bash

## Run

Start with the default configuration:

    ros2 launch dtrack_publisher dtrack_multi_body.launch.py

Override launch arguments when needed:

    ros2 launch dtrack_publisher dtrack_multi_body.launch.py udp_port:=5001 max_path_length:=1000

The executable can also be started directly:

    ros2 run dtrack_publisher dtrack_multi_body_node --ros-args -p udp_port:=5000

## Parameters

| Parameter | Type | Default | Meaning |
| --- | --- | --- | --- |
| udp_port | integer | 5000 | UDP port receiving DTrack packets |
| frame_id | string | odom_mocap | Frame assigned to poses and paths |
| parent_frame | string | odom | Parent frame of the identity transform |
| max_path_length | integer | 500 | Maximum retained poses per body; values below 1 become 1 |

## Topics and TF

For every raw DTrack body ID N, the node creates:

- /dtrack/body_(N+1)/pose using geometry_msgs/msg/PoseStamped
- /dtrack/body_(N+1)/path using nav_msgs/msg/Path

Both publishers use a queue depth of 10. The identity transform from
parent_frame to frame_id is broadcast dynamically on /tf.

Useful inspection commands:

    ros2 topic list
    ros2 topic echo /dtrack/body_1/pose
    ros2 topic echo /dtrack/body_1/path
    ros2 topic hz /dtrack/body_1/pose
    ros2 run tf2_ros tf2_echo odom odom_mocap

## DTrack setup

1. Connect the DTrack controller to the computer using Ethernet.
2. Configure the computer interface with static IPv4 address 192.168.1.2.
3. Start the controller and DTrack software.
4. Configure DTrack to stream its ASCII tracking output to this computer and
   the configured UDP port.
5. Start the ROS 2 launch file and verify the body topics.

## Tests

Run the parser unit tests and UDP-to-ROS integration test:

    colcon test --packages-select dtrack_publisher
    colcon test-result --verbose

The integration test starts the installed node on an unused local UDP port,
sends synthetic DTrack packets, and verifies pose, bounded path, body-ID topic
mapping, and the dynamic identity transform.
