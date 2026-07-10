# dtrack_publisher

This package publishes pose and path data received from a DTrack motion-capture system.

## Start DTrack

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
catkin build dtrack_publisher
```

Start the C++ node:

```bash
roslaunch dtrack_publisher dtrack_multi_body.launch
```

## Listen to the topics

List the available DTrack topics:

```bash
rostopic list | grep dtrack
```

Each DTrack body gets its own pose and path topics.

Use the body ID shown by `rostopic list`:

```bash
rostopic echo /dtrack/body_<id>/pose
rostopic echo /dtrack/body_<id>/path
rostopic hz /dtrack/body_<id>/pose
```

Positions are converted from millimetres to metres and stay in the calibrated DTrack coordinate-system origin. Orientation is computed from the DTrack rotation matrix.
