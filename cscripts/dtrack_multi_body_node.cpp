#include <arpa/inet.h>
#include <cerrno>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <array>
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
    publish_identity_transform();
    return;
  }

  const std::string packet(receive_buffer_.data(),
                           static_cast<std::size_t>(received));
  const ParseResult result = parse_packet(packet);
  for (const std::string &warning : result.warnings) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "%s",
                         warning.c_str());
  }
  for (const ParsedBody &body : result.bodies) {
    publish_pose(body);
  }

  publish_identity_transform();
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
DTrackMultiBodyNode::make_pose(const ParsedBody &body) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = now();
  pose.header.frame_id = frame_id_;

  const std::array<double, 3> position = position_metres(body);
  pose.pose.position.x = position[0];
  pose.pose.position.y = position[1];
  pose.pose.position.z = position[2];
  pose.pose.orientation = orientation_from_rotation(body);
  return pose;
}

void DTrackMultiBodyNode::publish_pose(const ParsedBody &body) {
  BodyState &body_state = get_body_state(body.body_id);
  const geometry_msgs::msg::PoseStamped pose = make_pose(body);
  body_state.pose_publisher->publish(pose);

  body_state.path.header.stamp = now();
  body_state.path.poses.push_back(pose);
  while (static_cast<int>(body_state.path.poses.size()) > max_path_length_) {
    body_state.path.poses.erase(body_state.path.poses.begin());
  }
  body_state.path_publisher->publish(body_state.path);
}

void DTrackMultiBodyNode::publish_identity_transform() {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now();
  transform.header.frame_id = parent_frame_;
  transform.child_frame_id = frame_id_;
  transform.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(transform);
}

} // namespace dtrack_publisher
