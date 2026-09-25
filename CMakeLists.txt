# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.8)
project(dtrack_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

file(STRINGS
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/DTrackSDK/VERSION"
  DTRACKSDK_VERSION
  LIMIT_COUNT 1
)
if(NOT DTRACKSDK_VERSION)
  message(FATAL_ERROR "Bundled DTrackSDK version is missing")
endif()
message(STATUS "Using bundled DTrackSDK ${DTRACKSDK_VERSION}")

add_library(dtrack_sdk STATIC
  third_party/DTrackSDK/src/DTrackData.cpp
  third_party/DTrackSDK/src/DTrackNet.cpp
  third_party/DTrackSDK/src/DTrackParse.cpp
  third_party/DTrackSDK/src/DTrackParser.cpp
  third_party/DTrackSDK/src/DTrackSDK.cpp
)
target_include_directories(dtrack_sdk SYSTEM PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/DTrackSDK/include>"
)

add_library(dtrack_support STATIC
  src/dtrack_utils.cpp
)
target_include_directories(dtrack_support PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
ament_target_dependencies(dtrack_support
  geometry_msgs
  nav_msgs
  rclcpp
)

add_library(dtrack_node STATIC
  src/dtrack_multi_body_node.cpp
)
target_include_directories(dtrack_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
target_link_libraries(dtrack_node
  dtrack_sdk
  dtrack_support
)
ament_target_dependencies(dtrack_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_ros
)

add_executable(dtrack_multi_body_node
  src/dtrack_multi_body_main.cpp
)
target_link_libraries(dtrack_multi_body_node
  dtrack_node
)
ament_target_dependencies(dtrack_multi_body_node
  rclcpp
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(dtrack_support PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(dtrack_node PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(dtrack_multi_body_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS dtrack_multi_body_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(FILES LICENSE
  DESTINATION share/${PROJECT_NAME}
)
install(FILES third_party/DTrackSDK/license.txt
  DESTINATION share/${PROJECT_NAME}/licenses
  RENAME DTrackSDK-BSD-3-Clause.txt
)
install(FILES
  third_party/DTrackSDK/UPSTREAM.md
  third_party/DTrackSDK/VERSION
  DESTINATION share/${PROJECT_NAME}/third_party/DTrackSDK
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/ros_logs")

  ament_add_gtest(test_dtrack_utils
    test/test_dtrack_utils.cpp
  )
  target_link_libraries(test_dtrack_utils
    dtrack_sdk
    dtrack_support
  )
  ament_target_dependencies(test_dtrack_utils
    geometry_msgs
    nav_msgs
    rclcpp
  )

  ament_add_gtest(test_dtrack_node
    test/test_dtrack_node.cpp
    ENV "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
  )
  target_link_libraries(test_dtrack_node
    dtrack_node
  )
  ament_target_dependencies(test_dtrack_node
    geometry_msgs
    nav_msgs
    rclcpp
  )
endif()

ament_package()
