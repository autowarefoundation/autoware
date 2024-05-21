// Copyright 2024 TIER IV
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PLANNING_TEST_UTILS__MOCK_DATA_PARSER_HPP_
#define PLANNING_TEST_UTILS__MOCK_DATA_PARSER_HPP_

#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

namespace test_utils
{
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using geometry_msgs::msg::Pose;

Pose parse_pose(const YAML::Node & node);

LaneletPrimitive parse_lanelet_primitive(const YAML::Node & node);

std::vector<LaneletPrimitive> parse_lanelet_primitives(const YAML::Node & node);

std::vector<LaneletSegment> parse_segments(const YAML::Node & node);

LaneletRoute parse_lanelet_route_file(const std::string & filename);
}  // namespace test_utils

#endif  // PLANNING_TEST_UTILS__MOCK_DATA_PARSER_HPP_
