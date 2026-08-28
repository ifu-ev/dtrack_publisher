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
  geometry_msgs::msg::PoseStamped make_pose(const ParsedBody &body,
                                            const rclcpp::Time &stamp);
  void publish_pose(const ParsedBody &body, const rclcpp::Time &stamp);
  BodyState &get_body_state(int body_id);
  void publish_identity_transform(const rclcpp::Time &stamp);

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
