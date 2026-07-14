cmake_minimum_required(VERSION 3.5)
project(dtrack_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs           # Add nav_msgs since you're using Path messages
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dtrack_multi_body_node
  cscripts/dtrack_multi_body_node.cpp
)

target_link_libraries(dtrack_multi_body_node
  ${catkin_LIBRARIES}
)

install(TARGETS dtrack_multi_body_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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
