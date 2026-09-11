// SPDX-License-Identifier: MIT
// Authors: David Schnermann, Dominik Birkel

#include "dtrack_publisher/dtrack_multi_body_node.hpp"

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

#include "DTrackSDK.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace dtrack_publisher {
namespace {

constexpr int kReceiveTimeoutMicroseconds = 10000;

} // namespace

DTrackMultiBodyNode::DTrackMultiBodyNode(const rclcpp::NodeOptions &options)
    : Node("dtrack_multi_body_node", options) {
  udp_port_ = declare_parameter<int>("udp_port", 5000);
  frame_id_ = declare_parameter<std::string>("frame_id", "odom_mocap");
  parent_frame_ = declare_parameter<std::string>("parent_frame", "odom");
  max_path_length_ = declare_parameter<int>("max_path_length", 500);
  multicast_ip_ = declare_parameter<std::string>("multicast_ip", "");

  validate_parameters();
  initialize_sdk();
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  if (multicast_ip_.empty()) {
    RCLCPP_INFO(get_logger(),
                "DTrackSDK listener started on UDP port %d (unicast)",
                udp_port_);
  } else {
    RCLCPP_INFO(get_logger(), "DTrackSDK listener started on %s:%d (multicast)",
                multicast_ip_.c_str(), udp_port_);
  }
}

DTrackMultiBodyNode::~DTrackMultiBodyNode() = default;

void DTrackMultiBodyNode::validate_parameters() const {
  if (!is_valid_udp_port(udp_port_)) {
    throw std::invalid_argument("udp_port must be in the range [1, 65535]");
  }
  if (max_path_length_ < 1) {
    throw std::invalid_argument("max_path_length must be at least 1");
  }
  if (frame_id_.empty()) {
    throw std::invalid_argument("frame_id must not be empty");
  }
  if (parent_frame_.empty()) {
    throw std::invalid_argument("parent_frame must not be empty");
  }
  if (!multicast_ip_.empty() && !is_valid_multicast_ipv4(multicast_ip_)) {
    throw std::invalid_argument(
        "multicast_ip must be empty or a valid IPv4 multicast address");
  }
}

void DTrackMultiBodyNode::initialize_sdk() {
  const std::string connection =
      multicast_ip_.empty() ? std::to_string(udp_port_)
                            : multicast_ip_ + ":" + std::to_string(udp_port_);
  dtrack_ = std::make_unique<DTrackSDK>(connection);
  if (!dtrack_->isDataInterfaceValid()) {
    throw std::runtime_error("DTrackSDK failed to open UDP connection " +
                             connection);
  }
  if (!dtrack_->setDataTimeoutUS(kReceiveTimeoutMicroseconds)) {
    throw std::runtime_error("DTrackSDK rejected the UDP receive timeout");
  }
}

void DTrackMultiBodyNode::receive_and_publish() {
  if (!dtrack_->receive()) {
    handle_receive_error();
    return;
  }

  const std::int64_t reception_time_nanoseconds =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  const TimestampSelection timestamp = select_measurement_timestamp(
      dtrack_->getTimeStampSec(), dtrack_->getTimeStampUsec(),
      dtrack_->getTimeStamp(), reception_time_nanoseconds);
  if (timestamp.source == TimestampSource::kReception) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "DTrack packet has no valid ts or ts2 timestamp; using reception time");
  }
  const rclcpp::Time measurement_time(timestamp.nanoseconds_since_epoch,
                                      RCL_SYSTEM_TIME);

  for (int body_index = 0; body_index < dtrack_->getNumBody(); ++body_index) {
    const DTrackSDK_Datatypes::DTrackBody *body = dtrack_->getBody(body_index);
    if (body == nullptr) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "DTrackSDK returned no body for index %d",
                           body_index);
      continue;
    }
    if (!body->isTracked()) {
      handle_tracking_loss(body->id);
      continue;
    }
    try {
      publish_pose(*body, measurement_time);
    } catch (const std::invalid_argument &error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Skipping invalid DTrack body %d: %s", body->id,
                           error.what());
    }
  }

  publish_identity_transform(measurement_time);
}

void DTrackMultiBodyNode::handle_receive_error() {
  switch (dtrack_->getLastDataError()) {
  case DTrackSDK::ERR_TIMEOUT:
    return;
  case DTrackSDK::ERR_PARSE:
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "DTrackSDK could not parse the received UDP packet");
    return;
  case DTrackSDK::ERR_NET:
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "DTrackSDK reported a UDP network error");
    return;
  case DTrackSDK::ERR_NONE:
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "DTrackSDK receive failed without an error code");
    return;
  }
}

geometry_msgs::msg::PoseStamped
DTrackMultiBodyNode::make_pose(const DTrackSDK_Datatypes::DTrackBody &body,
                               const rclcpp::Time &stamp) const {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = frame_id_;
  pose.pose.position.x = body.loc[0] / 1000.0;
  pose.pose.position.y = body.loc[1] / 1000.0;
  pose.pose.position.z = body.loc[2] / 1000.0;

  const DTrackQuaternion quaternion = body.getQuaternion();
  pose.pose.orientation = normalized_quaternion(quaternion.x, quaternion.y,
                                                quaternion.z, quaternion.w);
  return pose;
}

DTrackMultiBodyNode::BodyState &
DTrackMultiBodyNode::get_body_state(int body_id) {
  const auto existing = bodies_.find(body_id);
  if (existing != bodies_.end()) {
    return existing->second;
  }

  BodyState body_state(static_cast<std::size_t>(max_path_length_));
  const std::string body_namespace = body_topic_namespace(body_id);
  body_state.pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>(
      body_namespace + "/pose", 10);
  body_state.path_publisher =
      create_publisher<nav_msgs::msg::Path>(body_namespace + "/path", 10);

  RCLCPP_INFO(get_logger(),
              "Created DTrack publishers for raw body %d under body_%d",
              body_id, body_id + 1);
  return bodies_.emplace(body_id, std::move(body_state)).first->second;
}

void DTrackMultiBodyNode::publish_pose(
    const DTrackSDK_Datatypes::DTrackBody &body, const rclcpp::Time &stamp) {
  BodyState &body_state = get_body_state(body.id);
  const geometry_msgs::msg::PoseStamped pose = make_pose(body, stamp);
  body_state.pose_publisher->publish(pose);

  body_state.path_history.append(pose);
  body_state.path_publisher->publish(
      body_state.path_history.make_message(frame_id_, stamp));
}

void DTrackMultiBodyNode::handle_tracking_loss(int body_id) {
  const auto existing = bodies_.find(body_id);
  if (existing != bodies_.end()) {
    existing->second.path_history.clear();
  }
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
