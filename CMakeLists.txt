cmake_minimum_required(VERSION 3.8)
project(dtrack_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(dtrack_parser STATIC
  src/dtrack_parser.cpp
)
target_include_directories(dtrack_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(dtrack_parser
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(dtrack_multi_body_node
  src/dtrack_multi_body_node.cpp
)
target_link_libraries(dtrack_multi_body_node
  dtrack_parser
)
ament_target_dependencies(dtrack_multi_body_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(dtrack_parser PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(dtrack_multi_body_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS dtrack_multi_body_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(FILES LICENSE
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(test_dtrack_parser
    test/test_dtrack_parser.cpp
  )
  target_link_libraries(test_dtrack_parser
    dtrack_parser
  )
  ament_target_dependencies(test_dtrack_parser
    geometry_msgs
    tf2
    tf2_geometry_msgs
  )

  add_launch_test(
    test/test_dtrack_integration.py
    TIMEOUT 45
  )
endif()

ament_package()
