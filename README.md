# dtrack_publisher

This ROS 2 package publishes pose and path data received from a DTrack motion-capture system.

## Start DTrack

- before starting DTrack for the first time, copy the complete `art-tracking` folder from the USB stick to your home directory. The resulting path must be `~/art-tracking`
- connect the DTrack controller to your desktop computer via ethernet
- set your ethernet IPv4 to the static IP `192.168.1.2`
- start the DTrack controller
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

Set the UDP port and maximum path length in `config/dtrack_multi_body.yaml`.

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

- `header.stamp`: ROS timestamp of the received pose
- `header.frame_id`: `odom_mocap`
- `pose.position`: x, y and z position in metres
- `pose.orientation`: orientation as a normalized quaternion in x, y, z, w order

The path topic uses `nav_msgs/msg/Path`. Its `header.frame_id` is also `odom_mocap`, and its `poses` array contains the latest `geometry_msgs/msg/PoseStamped` messages in chronological order. By default, the path contains up to 500 poses.
