// SPDX-License-Identifier: MIT
// Authors: David Schnermann, Dominik Birkel

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "dtrack_publisher/dtrack_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

class DTrackSDK;
namespace DTrackSDK_Datatypes {
struct DTrackBody;
}

namespace dtrack_publisher {

class DTrackMultiBodyNode : public rclcpp::Node {
public:
  explicit DTrackMultiBodyNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~DTrackMultiBodyNode() override;

  void receive_and_publish();

private:
  struct BodyState {
    explicit BodyState(std::size_t max_path_length)
        : path_history(max_path_length) {}

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        pose_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
    BoundedPathHistory path_history;
  };

  void validate_parameters() const;
  void initialize_sdk();
  void handle_receive_error();
  geometry_msgs::msg::PoseStamped
  make_pose(const DTrackSDK_Datatypes::DTrackBody &body,
            const rclcpp::Time &stamp) const;
  void publish_pose(const DTrackSDK_Datatypes::DTrackBody &body,
                    const rclcpp::Time &stamp);
  BodyState &get_body_state(int body_id);
  void handle_tracking_loss(int body_id);
  void publish_identity_transform(const rclcpp::Time &stamp);

  int udp_port_ = 5000;
  int max_path_length_ = 500;
  std::string frame_id_ = "odom_mocap";
  std::string parent_frame_ = "odom";
  std::string multicast_ip_;

  std::unique_ptr<DTrackSDK> dtrack_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unordered_map<int, BodyState> bodies_;
};

} // namespace dtrack_publisher
