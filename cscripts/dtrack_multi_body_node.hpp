#pragma once

#include <array>
#include <memory>
#include <string>
#include <unordered_map>

#include "dtrack_parser.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace dtrack_publisher {

class DTrackMultiBodyNode : public rclcpp::Node {
public:
  explicit DTrackMultiBodyNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~DTrackMultiBodyNode() override;

  void receive_and_publish();

private:
  struct BodyState {
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        pose_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
    nav_msgs::msg::Path path;
  };

  void open_socket();
  geometry_msgs::msg::PoseStamped make_pose(const ParsedBody &body);
  void publish_pose(const ParsedBody &body);
  BodyState &get_body_state(int body_id);
  void publish_identity_transform();

  int udp_port_ = 0;
  int max_path_length_ = 0;
  int socket_fd_ = -1;
  std::string frame_id_ = "odom_mocap";
  std::string parent_frame_ = "odom";

  std::array<char, 65535> receive_buffer_{};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unordered_map<int, BodyState> bodies_;
};

} // namespace dtrack_publisher
