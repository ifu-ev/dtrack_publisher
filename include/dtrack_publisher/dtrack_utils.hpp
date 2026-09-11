// SPDX-License-Identifier: MIT
// Authors: David Schnermann, Dominik Birkel

#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/time.hpp"

namespace dtrack_publisher {

enum class TimestampSource {
  kExtended,
  kLegacy,
  kReception,
};

struct TimestampSelection {
  std::int64_t nanoseconds_since_epoch;
  TimestampSource source;
};

bool is_valid_udp_port(std::int64_t port);
bool is_valid_multicast_ipv4(const std::string &address);

std::int64_t dtrack_timestamp_nanoseconds_since_epoch(
    double seconds_since_midnight,
    std::int64_t reference_time_nanoseconds_since_epoch);

TimestampSelection select_measurement_timestamp(
    unsigned int extended_seconds_since_epoch,
    unsigned int extended_microseconds, double legacy_seconds_since_midnight,
    std::int64_t reception_time_nanoseconds_since_epoch);

std::string body_topic_namespace(int raw_body_id);

geometry_msgs::msg::Quaternion normalized_quaternion(double x, double y,
                                                     double z, double w);

class BoundedPathHistory {
public:
  explicit BoundedPathHistory(std::size_t max_length);

  void append(const geometry_msgs::msg::PoseStamped &pose);
  void clear();
  std::size_t size() const;
  nav_msgs::msg::Path make_message(const std::string &frame_id,
                                   const rclcpp::Time &stamp) const;

private:
  std::size_t max_length_;
  std::deque<geometry_msgs::msg::PoseStamped> poses_;
};

} // namespace dtrack_publisher
