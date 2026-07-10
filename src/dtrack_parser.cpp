#include "dtrack_publisher/dtrack_parser.hpp"

#include <algorithm>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace dtrack_publisher {
namespace {

std::string trim(const std::string &input) {
  constexpr char whitespace[] = " \t\r\n";
  const std::size_t first = input.find_first_not_of(whitespace);
  if (first == std::string::npos) {
    return "";
  }

  const std::size_t last = input.find_last_not_of(whitespace);
  return input.substr(first, last - first + 1);
}

bool starts_with(const std::string &input, const std::string &prefix) {
  return input.compare(0, prefix.size(), prefix) == 0;
}

void parse_line(const std::string &line, ParseResult &result) {
  if (!starts_with(line, "6d ")) {
    return;
  }

  std::istringstream header_stream(line);
  std::string tag;
  int expected_body_count = 0;
  header_stream >> tag >> expected_body_count;
  if (expected_body_count <= 0) {
    return;
  }

  const std::regex group_regex("\\[([^\\]]*)\\]");
  std::vector<std::string> groups;
  for (std::sregex_iterator it(line.begin(), line.end(), group_regex), end;
       it != end; ++it) {
    groups.push_back((*it)[1].str());
  }

  const int parsed_body_count = static_cast<int>(groups.size() / 3);
  if (parsed_body_count < expected_body_count) {
    result.warnings.push_back(
        "DTrack 6d line expected " + std::to_string(expected_body_count) +
        " bodies but only " + std::to_string(parsed_body_count) +
        " complete groups were found");
  }

  const int body_count = std::min(expected_body_count, parsed_body_count);
  for (int body_index = 0; body_index < body_count; ++body_index) {
    const std::string &body_group = groups[body_index * 3];
    const std::string &pose_group = groups[body_index * 3 + 1];
    const std::string &rotation_group = groups[body_index * 3 + 2];

    ParsedBody body;
    std::istringstream body_stream(body_group);
    if (!(body_stream >> body.body_id >> body.quality)) {
      result.warnings.push_back(
          "Skipping DTrack body with invalid id/quality group: [" + body_group +
          "]");
      continue;
    }

    std::array<double, 6> pose_values{};
    std::istringstream pose_stream(pose_group);
    bool valid_pose = true;
    for (double &value : pose_values) {
      if (!(pose_stream >> value)) {
        valid_pose = false;
        break;
      }
    }
    if (!valid_pose) {
      result.warnings.push_back(
          "Skipping DTrack body " + std::to_string(body.body_id) +
          " with invalid pose group: [" + pose_group + "]");
      continue;
    }
    std::copy_n(pose_values.begin(), body.position_mm.size(),
                body.position_mm.begin());

    std::istringstream rotation_stream(rotation_group);
    bool valid_rotation = true;
    for (double &value : body.rotation) {
      if (!(rotation_stream >> value)) {
        valid_rotation = false;
        break;
      }
    }
    if (!valid_rotation) {
      result.warnings.push_back(
          "Skipping DTrack body " + std::to_string(body.body_id) +
          " with invalid rotation matrix group: [" + rotation_group + "]");
      continue;
    }

    result.bodies.push_back(body);
  }
}

} // namespace

ParseResult parse_packet(const std::string &packet) {
  ParseResult result;
  std::istringstream stream(packet);
  std::string line;
  while (std::getline(stream, line)) {
    parse_line(trim(line), result);
  }
  return result;
}

std::array<double, 3> position_metres(const ParsedBody &body) {
  return {body.position_mm[0] / 1000.0, body.position_mm[1] / 1000.0,
          body.position_mm[2] / 1000.0};
}

geometry_msgs::msg::Quaternion
orientation_from_rotation(const ParsedBody &body) {
  tf2::Matrix3x3 rotation;
  rotation.setValue(body.rotation[0], body.rotation[3], body.rotation[6],
                    body.rotation[1], body.rotation[4], body.rotation[7],
                    body.rotation[2], body.rotation[5], body.rotation[8]);

  tf2::Quaternion quaternion;
  rotation.getRotation(quaternion);
  quaternion.normalize();
  return tf2::toMsg(quaternion);
}

std::string body_topic_namespace(int raw_body_id) {
  return "/dtrack/body_" + std::to_string(raw_body_id + 1);
}

} // namespace dtrack_publisher
