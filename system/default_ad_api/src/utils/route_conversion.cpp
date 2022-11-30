// Copyright 2022 TIER IV, Inc.
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

#include "route_conversion.hpp"

#include <vector>

namespace
{

using ApiPrimitive = autoware_adapi_v1_msgs::msg::RoutePrimitive;
using LaneletPrimitive = autoware_planning_msgs::msg::LaneletPrimitive;
using HadPrimitive = autoware_planning_msgs::msg::LaneletPrimitive;
using ApiSegment = autoware_adapi_v1_msgs::msg::RouteSegment;
using MapSegment = autoware_planning_msgs::msg::LaneletSegment;
using HadSegment = autoware_planning_msgs::msg::LaneletSegment;

template <class RetT, class ArgT>
RetT convert(const ArgT & arg);

template <class RetT, class ArgT>
std::vector<RetT> convert_vector(const std::vector<ArgT> & args)
{
  std::vector<RetT> result;
  result.reserve(args.size());
  for (const auto & arg : args) {
    result.push_back(convert<RetT, ArgT>(arg));
  }
  return result;
}

template <>
ApiPrimitive convert(const HadPrimitive & had)
{
  ApiPrimitive api;
  api.id = had.id;
  api.type = had.primitive_type;
  return api;
}

template <>
LaneletPrimitive convert(const ApiPrimitive & api)
{
  LaneletPrimitive map;
  map.id = api.id;
  map.primitive_type = api.type;
  return map;
}

template <>
ApiSegment convert(const HadSegment & had)
{
  ApiSegment api;
  api.alternatives = convert_vector<ApiPrimitive>(had.primitives);
  for (auto iter = api.alternatives.begin(); iter != api.alternatives.end(); ++iter) {
    if (iter->id == had.preferred_primitive.id) {
      api.preferred = *iter;
      api.alternatives.erase(iter);
      break;
    }
  }
  return api;
}

template <>
MapSegment convert(const ApiSegment & api)
{
  MapSegment map;
  map.preferred_primitive = convert<LaneletPrimitive>(api.preferred);
  map.primitives = convert_vector<LaneletPrimitive>(api.alternatives);
  return map;
}

}  // namespace

namespace default_ad_api::conversion
{

ExternalRoute create_empty_route(const rclcpp::Time & stamp)
{
  ExternalRoute external;
  external.header.stamp = stamp;
  return external;
}

ExternalRoute convert_route(const InternalRoute & internal)
{
  autoware_adapi_v1_msgs::msg::RouteData data;
  data.start = internal.start_pose;
  data.goal = internal.goal_pose;
  data.segments = convert_vector<ApiSegment>(internal.segments);

  ExternalRoute external;
  external.header = internal.header;
  external.data.push_back(data);
  return external;
}

InternalSetRoute convert_set_route(const ExternalSetRoute & external)
{
  InternalSetRoute internal;
  internal.header = external.header;
  internal.goal = external.goal;
  internal.segments = convert_vector<MapSegment>(external.segments);
  return internal;
}

}  // namespace default_ad_api::conversion
