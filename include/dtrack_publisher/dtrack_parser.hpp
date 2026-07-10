#ifndef DTRACK_PUBLISHER__DTRACK_PARSER_HPP_
#define DTRACK_PUBLISHER__DTRACK_PARSER_HPP_

#include <array>
#include <string>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"

namespace dtrack_publisher {

using RotationValues = std::array<double, 9>;

struct ParsedBody {
  int body_id = 0;
  double quality = 0.0;
  std::array<double, 3> position_mm{};
  RotationValues rotation{};
};

struct ParseResult {
  std::vector<ParsedBody> bodies;
  std::vector<std::string> warnings;
};

ParseResult parse_packet(const std::string &packet);

std::array<double, 3> position_metres(const ParsedBody &body);

geometry_msgs::msg::Quaternion
orientation_from_rotation(const ParsedBody &body);

std::string body_topic_namespace(int raw_body_id);

} // namespace dtrack_publisher

#endif // DTRACK_PUBLISHER__DTRACK_PARSER_HPP_
