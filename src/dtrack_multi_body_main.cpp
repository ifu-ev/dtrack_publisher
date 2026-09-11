// SPDX-License-Identifier: MIT
// Authors: David Schnermann, Dominik Birkel

#include <exception>
#include <memory>

#include "dtrack_publisher/dtrack_multi_body_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<dtrack_publisher::DTrackMultiBodyNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (rclcpp::ok()) {
      node->receive_and_publish();
      executor.spin_some();
    }

    executor.remove_node(node);
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("dtrack_multi_body_node"), "%s",
                 error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
