# dtrack_publisher

This package publishes pose and path data received from a DTrack motion-capture system.

## Multi-body C++ node

Start the multi-body publisher with:

```bash
roslaunch dtrack_publisher dtrack_multi_body.launch
```

Each DTrack body gets its own pose and path topics. Topic IDs are one-based, while
DTrack's raw body IDs are zero-based:

- raw DTrack body ID `0`: `/dtrack/body_1/pose` and `/dtrack/body_1/path`
- raw DTrack body ID `1`: `/dtrack/body_2/pose` and `/dtrack/body_2/path`

Positions are converted from millimetres to metres and remain relative to the
calibrated DTrack coordinate-system origin. The first received measurement does
not redefine the origin.

The legacy Python node publishes `/dtrack_pose` and `/dtrack_path`.

- To get the data first you need to start the DTRACK-Motion-Capute-System
  - first connect the DTRACK-Controller to your desktop computer via ethernet
  - set your ethernet-ipv4 to the static ip: 192.168.1.2
  - start the power switch of the DTRACK controller
  - then start the DTRACK software
    - copy the software from the USB-stick in your home folder
    - `cd ~/art-tracking/ART-DTrack3/Software/LINUX/DTRACK3_v3.3.1_linux64/DTRACK3_v3.3.1/bin`
    - `./DTRACK3.bin`
    - hopefully you have valid licence
- if you used the setup.sh skript in kalimoro_ws you should already have the dtrack_publisher package in your lokal workspace and already sourced it.
- start the launchfile: `roslaunch dtrack_publisher dtrack_with_odom.launch`
- check topics: `rostopic echo /odom`, `rostopic echo /imu`, `rostopic echo /dtrack_pose`
- Record data
  - start the record: `roslaunch dtrack_publisher record.launch`
  - move the robot
  - stop the record: Ctrl + C
  - the data is saved as combined_data.bag in the Data folder.
- you can you this for optimizing the localization ekf in the ekf_tuning_package 
