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

#ifndef DTRACK_PARSER_HPP_
#define DTRACK_PARSER_HPP_

#include <array>
#include <cstdint>
#include <optional>
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
  std::optional<double> timestamp_seconds_since_midnight;
  std::vector<ParsedBody> bodies;
  std::vector<std::string> warnings;
};

ParseResult parse_packet(const std::string &packet);

std::int64_t dtrack_timestamp_nanoseconds_since_epoch(
    double seconds_since_midnight,
    std::int64_t reference_time_nanoseconds_since_epoch);

std::array<double, 3> position_metres(const ParsedBody &body);

geometry_msgs::msg::Quaternion
orientation_from_rotation(const ParsedBody &body);

std::string body_topic_namespace(int raw_body_id);

} // namespace dtrack_publisher

#endif // DTRACK_PARSER_HPP_
