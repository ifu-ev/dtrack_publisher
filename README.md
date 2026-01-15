# dtrack_publisher

This package utilizes the pose and path of the detected objects and publishes it to the ros-topics `/dtrack_pose` and `/dtrack_path`.

- To get the data first you need to connect the DTRACK-Controller to your desktop computer.
- Then start the DTRACK software: 
  - start the power switch of the DTRACK controller
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
