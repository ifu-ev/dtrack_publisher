# dtrack_publisher

This ROS 2 package publishes pose and path data received from a DTrack motion-capture system.

## Start DTrack

- before starting DTrack for the first time, copy the complete `art-tracking` folder from the USB stick to your home directory. The resulting path must be `~/art-tracking`
- connect the DTrack controller to your desktop computer via ethernet
- set your ethernet IPv4 to the static IP `192.168.1.2`
- start the DTrack controller
- synchronize the DTrack controller with the NTP server and enable the `ts` timestamp in the DTrack output settings
- start the DTrack software:

```bash
cd ~/art-tracking/ART-DTrack3/Software/LINUX/DTRACK3_v3.3.1_linux64/DTRACK3_v3.3.1/bin
./DTRACK3.bin
```

## Start the C++ multi-body node

Build and source the workspace:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/Projects/kalimoro_ros2/kalimoro_ws
colcon build
source install/setup.bash
```

Start the C++ node:

```bash
ros2 launch dtrack_publisher dtrack_multi_body.launch.py
```

Set the UDP port, frame names and maximum path length in
`config/dtrack_multi_body.yaml`.

## Listen to the topics

List the available DTrack topics:

```bash
ros2 topic list | grep dtrack
```

Each DTrack body gets its own pose and path topics.

Use the body ID shown by `ros2 topic list`:

```bash
ros2 topic echo /dtrack/body_<id>/pose
ros2 topic echo /dtrack/body_<id>/path
ros2 topic hz /dtrack/body_<id>/pose
```

Positions are converted from millimetres to metres and stay in the calibrated DTrack coordinate-system origin. Orientation is computed from the DTrack rotation matrix.

## Topic message format

The pose topic uses `geometry_msgs/msg/PoseStamped`:

- `header.stamp`: DTrack measurement timestamp from the `ts` line in the UDP packet, or local reception time if `ts` is unavailable
- `header.frame_id`: `odom_mocap`
- `pose.position`: x, y and z position in metres
- `pose.orientation`: orientation as a normalized quaternion in x, y, z, w order

The DTrack timestamp contains seconds since 00:00 UTC and resets at midnight. The node combines it with the UTC date of the NTP-synchronized computer and handles packets received around midnight by choosing the closest date. If a packet has no valid `ts` line, the node logs a throttled warning and falls back to the local reception time.

The path topic uses `nav_msgs/msg/Path`. Its `header.frame_id` is also `odom_mocap`, and its `header.stamp` and contained `geometry_msgs/msg/PoseStamped` messages use the selected DTrack or fallback reception timestamp. Its `poses` array contains the latest poses in chronological order. By default, the path contains up to 500 poses.
