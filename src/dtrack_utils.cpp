// SPDX-License-Identifier: MIT
// Authors: David Schnermann, Dominik Birkel

#include "dtrack_publisher/dtrack_utils.hpp"

#include <arpa/inet.h>

#include <cmath>
#include <stdexcept>

namespace dtrack_publisher {
namespace {

constexpr double kSecondsPerDay = 86400.0;
constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr std::int64_t kNanosecondsPerDay = 86400LL * kNanosecondsPerSecond;
constexpr std::int64_t kNanosecondsPerMicrosecond = 1000LL;

} // namespace

bool is_valid_udp_port(std::int64_t port) { return port >= 1 && port <= 65535; }

bool is_valid_multicast_ipv4(const std::string &address) {
  in_addr parsed_address{};
  if (inet_pton(AF_INET, address.c_str(), &parsed_address) != 1) {
    return false;
  }
  return IN_MULTICAST(ntohl(parsed_address.s_addr));
}

std::int64_t dtrack_timestamp_nanoseconds_since_epoch(
    double seconds_since_midnight,
    std::int64_t reference_time_nanoseconds_since_epoch) {
  if (!std::isfinite(seconds_since_midnight) || seconds_since_midnight < 0.0 ||
      seconds_since_midnight >= kSecondsPerDay) {
    throw std::invalid_argument(
        "DTrack timestamp must be in the range [0, 86400) seconds");
  }

  const std::int64_t timestamp_nanoseconds = static_cast<std::int64_t>(
      std::llround(seconds_since_midnight * kNanosecondsPerSecond));
  const std::int64_t current_utc_midnight =
      reference_time_nanoseconds_since_epoch -
      (reference_time_nanoseconds_since_epoch % kNanosecondsPerDay);
  std::int64_t candidate = current_utc_midnight + timestamp_nanoseconds;

  if (candidate - reference_time_nanoseconds_since_epoch >
      kNanosecondsPerDay / 2) {
    candidate -= kNanosecondsPerDay;
  } else if (reference_time_nanoseconds_since_epoch - candidate >
             kNanosecondsPerDay / 2) {
    candidate += kNanosecondsPerDay;
  }

  return candidate;
}

TimestampSelection select_measurement_timestamp(
    unsigned int extended_seconds_since_epoch,
    unsigned int extended_microseconds, double legacy_seconds_since_midnight,
    std::int64_t reception_time_nanoseconds_since_epoch) {
  if (extended_seconds_since_epoch > 0 && extended_microseconds < 1000000U) {
    return {static_cast<std::int64_t>(extended_seconds_since_epoch) *
                    kNanosecondsPerSecond +
                static_cast<std::int64_t>(extended_microseconds) *
                    kNanosecondsPerMicrosecond,
            TimestampSource::kExtended};
  }

  if (std::isfinite(legacy_seconds_since_midnight) &&
      legacy_seconds_since_midnight >= 0.0 &&
      legacy_seconds_since_midnight < kSecondsPerDay) {
    return {dtrack_timestamp_nanoseconds_since_epoch(
                legacy_seconds_since_midnight,
                reception_time_nanoseconds_since_epoch),
            TimestampSource::kLegacy};
  }

  return {reception_time_nanoseconds_since_epoch, TimestampSource::kReception};
}

std::string body_topic_namespace(int raw_body_id) {
  return "dtrack/body_" + std::to_string(raw_body_id + 1);
}

geometry_msgs::msg::Quaternion normalized_quaternion(double x, double y,
                                                     double z, double w) {
  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (!std::isfinite(norm) || norm <= 0.0) {
    throw std::invalid_argument(
        "DTrack rotation produced an invalid quaternion");
  }

  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = x / norm;
  quaternion.y = y / norm;
  quaternion.z = z / norm;
  quaternion.w = w / norm;
  return quaternion;
}

BoundedPathHistory::BoundedPathHistory(std::size_t max_length)
    : max_length_(max_length) {
  if (max_length_ == 0) {
    throw std::invalid_argument("Path history length must be at least 1");
  }
}

void BoundedPathHistory::append(const geometry_msgs::msg::PoseStamped &pose) {
  poses_.push_back(pose);
  while (poses_.size() > max_length_) {
    poses_.pop_front();
  }
}

void BoundedPathHistory::clear() { poses_.clear(); }

std::size_t BoundedPathHistory::size() const { return poses_.size(); }

nav_msgs::msg::Path
BoundedPathHistory::make_message(const std::string &frame_id,
                                 const rclcpp::Time &stamp) const {
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
  path.poses.assign(poses_.begin(), poses_.end());
  return path;
}

} // namespace dtrack_publisher
