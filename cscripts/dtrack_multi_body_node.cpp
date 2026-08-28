/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026,
 *  David Schnermann (david.schnermann@ifu.rwth-aachen.de).
 *  Dominik Birkel (dominik.birkel@ifu.rwth-aachen.de).
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: David Schnermann (david.schnermann@ifu.rwth-aachen.de).
 * Author: Dominik Birkel (dominik.birkel@ifu.rwth-aachen.de).
 *********************************************************************/

#include <arpa/inet.h>
#include <cerrno>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

#include "dtrack_multi_body_node.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace dtrack_publisher {

DTrackMultiBodyNode::DTrackMultiBodyNode(const rclcpp::NodeOptions &options)
    : Node("dtrack_multi_body_node", options) {
  udp_port_ = declare_parameter<int>("udp_port");
  frame_id_ = declare_parameter<std::string>("frame_id", "odom_mocap");
  parent_frame_ = declare_parameter<std::string>("parent_frame", "odom");
  max_path_length_ = declare_parameter<int>("max_path_length");
  if (max_path_length_ < 1) {
    RCLCPP_WARN(get_logger(), "Invalid max_path_length %d, using 1 instead",
                max_path_length_);
    max_path_length_ = 1;
  }

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  open_socket();
  RCLCPP_INFO(get_logger(), "DTrack multi-body UDP listener started on port %d",
              udp_port_);
}

DTrackMultiBodyNode::~DTrackMultiBodyNode() {
  if (socket_fd_ >= 0) {
    close(socket_fd_);
  }
}

void DTrackMultiBodyNode::receive_and_publish() {
  sockaddr_in sender_addr{};
  socklen_t sender_len = sizeof(sender_addr);
  const ssize_t received =
      recvfrom(socket_fd_, receive_buffer_.data(), receive_buffer_.size(), 0,
               reinterpret_cast<sockaddr *>(&sender_addr), &sender_len);

  if (received < 0) {
    if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Failed to receive DTrack UDP packet: %s",
                           std::strerror(errno));
    }
    return;
  }

  const std::int64_t reception_time_nanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();

  const std::string packet(receive_buffer_.data(),
                           static_cast<std::size_t>(received));
  const ParseResult result = parse_packet(packet);
  for (const std::string &warning : result.warnings) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s",
                         warning.c_str());
  }
  std::int64_t measurement_time_nanoseconds = reception_time_nanoseconds;
  if (result.timestamp_seconds_since_midnight.has_value()) {
    measurement_time_nanoseconds = dtrack_timestamp_nanoseconds_since_epoch(
        *result.timestamp_seconds_since_midnight, reception_time_nanoseconds);
  } else {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "DTrack packet has no valid ts timestamp; falling back to local "
        "reception time");
  }
  const rclcpp::Time measurement_time(measurement_time_nanoseconds,
                                      RCL_SYSTEM_TIME);

  for (const ParsedBody &body : result.bodies) {
    publish_pose(body, measurement_time);
  }

  publish_identity_transform(measurement_time);
}

DTrackMultiBodyNode::BodyState &
DTrackMultiBodyNode::get_body_state(int body_id) {
  const auto existing = bodies_.find(body_id);
  if (existing != bodies_.end()) {
    return existing->second;
  }

  BodyState body_state;
  const std::string body_namespace = body_topic_namespace(body_id);
  body_state.pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>(
      body_namespace + "/pose", 10);
  body_state.path_publisher =
      create_publisher<nav_msgs::msg::Path>(body_namespace + "/path", 10);
  body_state.path.header.frame_id = frame_id_;

  const int topic_body_id = body_id + 1;
  RCLCPP_INFO(get_logger(),
              "Created DTrack publishers for raw body %d under body_%d",
              body_id, topic_body_id);
  return bodies_.emplace(body_id, std::move(body_state)).first->second;
}

void DTrackMultiBodyNode::open_socket() {
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    throw std::runtime_error("Failed to create UDP socket: " +
                             std::string(std::strerror(errno)));
  }

  int reuse = 1;
  if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) <
      0) {
    RCLCPP_WARN(get_logger(), "Failed to set SO_REUSEADDR: %s",
                std::strerror(errno));
  }

  timeval timeout{};
  timeout.tv_sec = 1;
  if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                 sizeof(timeout)) < 0) {
    RCLCPP_WARN(get_logger(), "Failed to set UDP receive timeout: %s",
                std::strerror(errno));
  }

  sockaddr_in local_addr{};
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  local_addr.sin_port = htons(static_cast<std::uint16_t>(udp_port_));

  if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&local_addr),
           sizeof(local_addr)) < 0) {
    const int bind_error = errno;
    close(socket_fd_);
    socket_fd_ = -1;
    throw std::runtime_error("Failed to bind UDP socket on port " +
                             std::to_string(udp_port_) + ": " +
                             std::string(std::strerror(bind_error)));
  }
}

geometry_msgs::msg::PoseStamped
DTrackMultiBodyNode::make_pose(const ParsedBody &body,
                               const rclcpp::Time &stamp) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = frame_id_;

  const std::array<double, 3> position = position_metres(body);
  pose.pose.position.x = position[0];
  pose.pose.position.y = position[1];
  pose.pose.position.z = position[2];
  pose.pose.orientation = orientation_from_rotation(body);
  return pose;
}

void DTrackMultiBodyNode::publish_pose(const ParsedBody &body,
                                       const rclcpp::Time &stamp) {
  BodyState &body_state = get_body_state(body.body_id);
  const geometry_msgs::msg::PoseStamped pose = make_pose(body, stamp);
  body_state.pose_publisher->publish(pose);

  body_state.path.header.stamp = stamp;
  body_state.path.poses.push_back(pose);
  while (static_cast<int>(body_state.path.poses.size()) > max_path_length_) {
    body_state.path.poses.erase(body_state.path.poses.begin());
  }
  body_state.path_publisher->publish(body_state.path);
}

void DTrackMultiBodyNode::publish_identity_transform(
    const rclcpp::Time &stamp) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = parent_frame_;
  transform.child_frame_id = frame_id_;
  transform.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(transform);
}

} // namespace dtrack_publisher
