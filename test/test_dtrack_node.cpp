// SPDX-License-Identifier: MIT

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "dtrack_publisher/dtrack_multi_body_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "gtest/gtest.h"

namespace dtrack_publisher {
namespace {

using namespace std::chrono_literals;

class RclcppTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

std::uint16_t available_udp_port() {
  const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd < 0) {
    throw std::runtime_error("Could not create UDP socket for test");
  }

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = 0;
  if (bind(socket_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) <
      0) {
    close(socket_fd);
    throw std::runtime_error("Could not reserve UDP port for test");
  }

  socklen_t address_length = sizeof(address);
  if (getsockname(socket_fd, reinterpret_cast<sockaddr *>(&address),
                  &address_length) < 0) {
    close(socket_fd);
    throw std::runtime_error("Could not inspect UDP port for test");
  }
  const std::uint16_t port = ntohs(address.sin_port);
  close(socket_fd);
  return port;
}

bool send_udp_packet(std::uint16_t port, const std::string &packet) {
  const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd < 0) {
    return false;
  }

  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  destination.sin_port = htons(port);
  const ssize_t bytes_sent =
      sendto(socket_fd, packet.data(), packet.size(), 0,
             reinterpret_cast<sockaddr *>(&destination), sizeof(destination));
  close(socket_fd);
  return bytes_sent == static_cast<ssize_t>(packet.size());
}

bool spin_until(rclcpp::executors::SingleThreadedExecutor &executor,
                const std::function<bool()> &condition,
                std::chrono::milliseconds timeout = 1s) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return condition();
}

std::string tracked_packet(double x_millimetres) {
  return "fr 10\nts2 1695662575 867350 4568\n6d 1 [0 1.0][" +
         std::to_string(x_millimetres) +
         " 2000 3000 0 0 0][1 0 0 0 1 0 0 0 1]\n";
}

const std::string kUntrackedPacket =
    "fr 11\nts2 1695662575 900000 4568\n"
    "6d 1 [0 -1.0][0 0 0 0 0 0][1 0 0 0 1 0 0 0 1]\n";

TEST_F(RclcppTest, RejectsInvalidParameters) {
  auto options = [](const std::string &name,
                    const rclcpp::ParameterValue &value) {
    return rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter(name, value)});
  };

  EXPECT_THROW(DTrackMultiBodyNode(options(
                   "udp_port", rclcpp::ParameterValue(std::int64_t{0}))),
               std::invalid_argument);
  EXPECT_THROW(DTrackMultiBodyNode(options(
                   "udp_port", rclcpp::ParameterValue(std::int64_t{65536}))),
               std::invalid_argument);
  EXPECT_THROW(DTrackMultiBodyNode(options(
                   "max_path_length", rclcpp::ParameterValue(std::int64_t{0}))),
               std::invalid_argument);
  EXPECT_THROW(
      DTrackMultiBodyNode(options(
          "multicast_ip", rclcpp::ParameterValue(std::string("192.168.1.1")))),
      std::invalid_argument);
}

TEST_F(RclcppTest, PublishesNamespacedUdpDataAndRestartsPathAfterLoss) {
  const std::uint16_t port = available_udp_port();
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__ns:=/test_ns"});
  options.parameter_overrides(
      {rclcpp::Parameter("udp_port", static_cast<std::int64_t>(port)),
       rclcpp::Parameter("max_path_length", 2)});

  auto dtrack_node = std::make_shared<DTrackMultiBodyNode>(options);
  auto observer = std::make_shared<rclcpp::Node>("dtrack_test_observer");
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  std::vector<nav_msgs::msg::Path> paths;
  auto pose_subscription =
      observer->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/test_ns/dtrack/body_1/pose", 10,
          [&poses](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
            poses.push_back(*message);
          });
  auto path_subscription = observer->create_subscription<nav_msgs::msg::Path>(
      "/test_ns/dtrack/body_1/path", 10,
      [&paths](nav_msgs::msg::Path::ConstSharedPtr message) {
        paths.push_back(*message);
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(dtrack_node);
  executor.add_node(observer);

  ASSERT_TRUE(send_udp_packet(port, kUntrackedPacket));
  dtrack_node->receive_and_publish();
  spin_until(executor, [] { return false; }, 100ms);
  EXPECT_EQ(observer->count_publishers("/test_ns/dtrack/body_1/pose"), 0U);
  EXPECT_EQ(observer->count_publishers("/test_ns/dtrack/body_1/path"), 0U);
  EXPECT_TRUE(poses.empty());
  EXPECT_TRUE(paths.empty());

  ASSERT_TRUE(send_udp_packet(port, tracked_packet(1000.0)));
  dtrack_node->receive_and_publish();
  ASSERT_TRUE(spin_until(executor, [&observer] {
    return observer->count_publishers("/test_ns/dtrack/body_1/pose") == 1 &&
           observer->count_publishers("/test_ns/dtrack/body_1/path") == 1;
  }));

  // Send again after DDS discovery so the assertions do not depend on
  // transient-local durability.
  ASSERT_TRUE(send_udp_packet(port, tracked_packet(1500.0)));
  dtrack_node->receive_and_publish();
  ASSERT_TRUE(spin_until(
      executor, [&poses, &paths] { return !poses.empty() && !paths.empty(); }));
  EXPECT_DOUBLE_EQ(poses.back().pose.position.x, 1.5);
  EXPECT_EQ(poses.back().header.frame_id, "odom_mocap");
  EXPECT_EQ(poses.back().header.stamp.sec, 1695662575);
  EXPECT_EQ(poses.back().header.stamp.nanosec, 867350000U);

  const std::size_t pose_count_before_loss = poses.size();
  const std::size_t path_count_before_loss = paths.size();
  ASSERT_TRUE(send_udp_packet(port, kUntrackedPacket));
  dtrack_node->receive_and_publish();
  spin_until(executor, [] { return false; }, 100ms);
  EXPECT_EQ(poses.size(), pose_count_before_loss);
  EXPECT_EQ(paths.size(), path_count_before_loss);

  ASSERT_TRUE(send_udp_packet(port, tracked_packet(2500.0)));
  dtrack_node->receive_and_publish();
  ASSERT_TRUE(spin_until(executor, [&poses, &paths, pose_count_before_loss,
                                    path_count_before_loss] {
    return poses.size() > pose_count_before_loss &&
           paths.size() > path_count_before_loss;
  }));
  EXPECT_DOUBLE_EQ(poses.back().pose.position.x, 2.5);
  ASSERT_EQ(paths.back().poses.size(), 1U);
  EXPECT_DOUBLE_EQ(paths.back().poses.front().pose.position.x, 2.5);

  executor.remove_node(observer);
  executor.remove_node(dtrack_node);
  (void)pose_subscription;
  (void)path_subscription;
}

} // namespace
} // namespace dtrack_publisher
