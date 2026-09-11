// SPDX-License-Identifier: MIT

#include <cmath>
#include <cstdint>
#include <string>

#include "DTrackSDK.hpp"
#include "dtrack_publisher/dtrack_utils.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/time.hpp"
#include "gtest/gtest.h"

namespace dtrack_publisher {
namespace {

constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr std::int64_t kNanosecondsPerDay = 86400LL * kNanosecondsPerSecond;

TEST(DTrackSdkParser, ParsesTrackedAndUntrackedStandardBodies) {
  DTrackSDK sdk(static_cast<unsigned short>(0));
  const std::string packet =
      "fr 6932\n"
      "ts2 1695662575 867350 4568\n"
      "6d 3 "
      "[0 1.000][907.148 2161.005 843.812 0 0 0]"
      "[0.999995 0.003031 0.000966 -0.003024 0.999969 -0.007261 "
      "-0.000988 0.007258 0.999973] "
      "[1 -1.000][0 0 0 0 0 0][1 0 0 0 1 0 0 0 1] "
      "[2 0.500][-902.561 -2682.247 756.905 0 0 0]"
      "[-0.960313 0.277215 0.030849 -0.276184 -0.960509 0.033847 "
      "0.039013 0.023983 0.998951]\n";

  ASSERT_TRUE(sdk.processPacket(packet));
  EXPECT_EQ(sdk.getFrameCounter(), 6932U);
  EXPECT_EQ(sdk.getTimeStampSec(), 1695662575U);
  EXPECT_EQ(sdk.getTimeStampUsec(), 867350U);
  ASSERT_EQ(sdk.getNumBody(), 3);

  const DTrackBody *body_0 = sdk.getBody(0);
  const DTrackBody *body_1 = sdk.getBody(1);
  const DTrackBody *body_2 = sdk.getBody(2);
  ASSERT_NE(body_0, nullptr);
  ASSERT_NE(body_1, nullptr);
  ASSERT_NE(body_2, nullptr);
  EXPECT_TRUE(body_0->isTracked());
  EXPECT_FALSE(body_1->isTracked());
  EXPECT_TRUE(body_2->isTracked());
  EXPECT_DOUBLE_EQ(body_0->loc[0], 907.148);
  EXPECT_DOUBLE_EQ(body_2->loc[2], 756.905);

  const DTrackQuaternion quaternion = body_0->getQuaternion();
  const double norm =
      std::sqrt(quaternion.x * quaternion.x + quaternion.y * quaternion.y +
                quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  EXPECT_NEAR(norm, 1.0, 1e-6);
}

TEST(DTrackSdkParser, ParsesLegacyTimestampAndRejectsMalformedPacket) {
  DTrackSDK sdk(static_cast<unsigned short>(0));
  ASSERT_TRUE(sdk.processPacket("fr 1\nts 59250.094982\n6d 0\n"));
  EXPECT_EQ(sdk.getTimeStampSec(), 0U);
  EXPECT_DOUBLE_EQ(sdk.getTimeStamp(), 59250.094982);

  EXPECT_FALSE(sdk.processPacket("fr 2\n6d 1 [0 1.0][1 2]\n"));
  EXPECT_EQ(sdk.getLastDataError(), DTrackSDK::ERR_PARSE);
}

TEST(TimestampSelection, PrefersExtendedTimestamp) {
  const TimestampSelection selected =
      select_measurement_timestamp(1695662575U, 867350U, 42.0, 123LL);
  EXPECT_EQ(selected.source, TimestampSource::kExtended);
  EXPECT_EQ(selected.nanoseconds_since_epoch,
            1695662575LL * kNanosecondsPerSecond + 867350000LL);
}

TEST(TimestampSelection, SelectsPreviousDayJustAfterMidnight) {
  const std::int64_t current_day = 20000LL * kNanosecondsPerDay;
  const TimestampSelection selected =
      select_measurement_timestamp(0U, 0U, 86399.9, current_day + 100000000LL);
  EXPECT_EQ(selected.source, TimestampSource::kLegacy);
  EXPECT_EQ(selected.nanoseconds_since_epoch, current_day - 100000000LL);
}

TEST(TimestampSelection, SelectsNextDayJustBeforeMidnight) {
  const std::int64_t current_day = 20000LL * kNanosecondsPerDay;
  const TimestampSelection selected = select_measurement_timestamp(
      0U, 0U, 0.1, current_day + kNanosecondsPerDay - 100000000LL);
  EXPECT_EQ(selected.source, TimestampSource::kLegacy);
  EXPECT_EQ(selected.nanoseconds_since_epoch,
            current_day + kNanosecondsPerDay + 100000000LL);
}

TEST(TimestampSelection, FallsBackToReceptionTime) {
  const TimestampSelection selected =
      select_measurement_timestamp(0U, 0U, -1.0, 987654321LL);
  EXPECT_EQ(selected.source, TimestampSource::kReception);
  EXPECT_EQ(selected.nanoseconds_since_epoch, 987654321LL);
}

TEST(ParameterValidation, ValidatesPortsAndMulticastAddresses) {
  EXPECT_TRUE(is_valid_udp_port(1));
  EXPECT_TRUE(is_valid_udp_port(65535));
  EXPECT_FALSE(is_valid_udp_port(0));
  EXPECT_FALSE(is_valid_udp_port(65536));

  EXPECT_TRUE(is_valid_multicast_ipv4("224.0.1.0"));
  EXPECT_TRUE(is_valid_multicast_ipv4("239.255.255.255"));
  EXPECT_FALSE(is_valid_multicast_ipv4("223.255.255.255"));
  EXPECT_FALSE(is_valid_multicast_ipv4("240.0.0.1"));
  EXPECT_FALSE(is_valid_multicast_ipv4("not-an-address"));
}

TEST(TopicNames, UsesRelativeOneBasedBodyNamespaces) {
  EXPECT_EQ(body_topic_namespace(0), "dtrack/body_1");
  EXPECT_EQ(body_topic_namespace(2), "dtrack/body_3");
  EXPECT_NE(body_topic_namespace(0).front(), '/');
}

TEST(QuaternionConversion, NormalizesAndRejectsInvalidValues) {
  const geometry_msgs::msg::Quaternion quaternion =
      normalized_quaternion(0.0, 0.0, 0.0, 2.0);
  EXPECT_DOUBLE_EQ(quaternion.x, 0.0);
  EXPECT_DOUBLE_EQ(quaternion.y, 0.0);
  EXPECT_DOUBLE_EQ(quaternion.z, 0.0);
  EXPECT_DOUBLE_EQ(quaternion.w, 1.0);
  EXPECT_THROW(normalized_quaternion(0.0, 0.0, 0.0, 0.0),
               std::invalid_argument);
}

TEST(BoundedPathHistory, CapsAndClearsPathWithoutFrontVectorErase) {
  BoundedPathHistory history(2);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom_mocap";

  pose.pose.position.x = 1.0;
  history.append(pose);
  pose.pose.position.x = 2.0;
  history.append(pose);
  pose.pose.position.x = 3.0;
  history.append(pose);

  EXPECT_EQ(history.size(), 2U);
  const nav_msgs::msg::Path path =
      history.make_message("odom_mocap", rclcpp::Time(10, 20));
  ASSERT_EQ(path.poses.size(), 2U);
  EXPECT_DOUBLE_EQ(path.poses[0].pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(path.poses[1].pose.position.x, 3.0);
  EXPECT_EQ(path.header.frame_id, "odom_mocap");

  history.clear();
  EXPECT_EQ(history.size(), 0U);
  EXPECT_TRUE(
      history.make_message("odom_mocap", rclcpp::Time(0)).poses.empty());
}

TEST(BoundedPathHistory, RejectsZeroCapacity) {
  EXPECT_THROW(BoundedPathHistory(0), std::invalid_argument);
}

} // namespace
} // namespace dtrack_publisher
