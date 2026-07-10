#include <string>

#include "dtrack_publisher/dtrack_parser.hpp"
#include "gtest/gtest.h"

namespace dtrack_publisher {
namespace {

constexpr char kIdentityRotation[] = "1 0 0 0 1 0 0 0 1";

TEST(DTrackParser, IgnoresNonSixDLines) {
  const ParseResult result = parse_packet("fr 123\n"
                                          "6df 1 [0 1] [1 2 3]\n"
                                          "ts 42\n");

  EXPECT_TRUE(result.bodies.empty());
  EXPECT_TRUE(result.warnings.empty());
}

TEST(DTrackParser, ParsesOneBodyAndConvertsItsPose) {
  const ParseResult result =
      parse_packet(std::string("6d 1 [0 0.75] [1000 -2500 125 10 20 30] [") +
                   kIdentityRotation + "]\n");

  ASSERT_EQ(result.bodies.size(), 1U);
  EXPECT_TRUE(result.warnings.empty());

  const ParsedBody &body = result.bodies.front();
  EXPECT_EQ(body.body_id, 0);
  EXPECT_DOUBLE_EQ(body.quality, 0.75);
  EXPECT_DOUBLE_EQ(body.position_mm[0], 1000.0);
  EXPECT_DOUBLE_EQ(body.position_mm[1], -2500.0);
  EXPECT_DOUBLE_EQ(body.position_mm[2], 125.0);

  const auto position = position_metres(body);
  EXPECT_DOUBLE_EQ(position[0], 1.0);
  EXPECT_DOUBLE_EQ(position[1], -2.5);
  EXPECT_DOUBLE_EQ(position[2], 0.125);

  const auto orientation = orientation_from_rotation(body);
  EXPECT_NEAR(orientation.x, 0.0, 1e-12);
  EXPECT_NEAR(orientation.y, 0.0, 1e-12);
  EXPECT_NEAR(orientation.z, 0.0, 1e-12);
  EXPECT_NEAR(orientation.w, 1.0, 1e-12);
}

TEST(DTrackParser, ParsesMultipleBodies) {
  const ParseResult result = parse_packet(
      std::string("6d 2 ") + "[0 1.0] [1000 2000 3000 0 0 0] [" +
      kIdentityRotation + "] " + "[5 0.1] [-1000 -2000 -3000 0 0 0] [" +
      kIdentityRotation + "]\n");

  ASSERT_EQ(result.bodies.size(), 2U);
  EXPECT_EQ(result.bodies[0].body_id, 0);
  EXPECT_EQ(result.bodies[1].body_id, 5);
  EXPECT_DOUBLE_EQ(result.bodies[1].position_mm[2], -3000.0);
}

TEST(DTrackParser, WarnsAndUsesOnlyCompleteBodyGroups) {
  const ParseResult result = parse_packet(
      std::string("6d 2 [0 1] [1 2 3 0 0 0] [") + kIdentityRotation + "]\n");

  ASSERT_EQ(result.bodies.size(), 1U);
  ASSERT_EQ(result.warnings.size(), 1U);
  EXPECT_NE(result.warnings.front().find("expected 2 bodies"),
            std::string::npos);
}

TEST(DTrackParser, SkipsInvalidBodyPoseAndRotationGroups) {
  const ParseResult result =
      parse_packet(std::string("6d 3 ") + "[invalid 1] [1 2 3 0 0 0] [" +
                   kIdentityRotation + "] " + "[1 1] [1 2 3] [" +
                   kIdentityRotation + "] " + "[2 1] [1 2 3 0 0 0] [1 0 0]\n");

  EXPECT_TRUE(result.bodies.empty());
  EXPECT_EQ(result.warnings.size(), 3U);
}

TEST(DTrackParser, PreservesRawBodyIdMapping) {
  EXPECT_EQ(body_topic_namespace(0), "/dtrack/body_1");
  EXPECT_EQ(body_topic_namespace(5), "/dtrack/body_6");
}

} // namespace
} // namespace dtrack_publisher
