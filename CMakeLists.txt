cmake_minimum_required(VERSION 3.0.2)
project(dtrack_publisher)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs           # Add nav_msgs since you're using Path messages
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/dtrack_node.py
  scripts/pose_to_path.py
  scripts/mocap_record.py
  scripts/odom_record.py
  scripts/data_logger_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
