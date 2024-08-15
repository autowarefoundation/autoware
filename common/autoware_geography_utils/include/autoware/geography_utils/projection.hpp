// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__GEOGRAPHY_UTILS__PROJECTION_HPP_
#define AUTOWARE__GEOGRAPHY_UTILS__PROJECTION_HPP_

#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tier4_map_msgs/msg/map_projector_info.hpp>

namespace autoware::geography_utils
{
using MapProjectorInfo = tier4_map_msgs::msg::MapProjectorInfo;
using GeoPoint = geographic_msgs::msg::GeoPoint;
using LocalPoint = geometry_msgs::msg::Point;

LocalPoint project_forward(const GeoPoint & geo_point, const MapProjectorInfo & projector_info);
GeoPoint project_reverse(const LocalPoint & local_point, const MapProjectorInfo & projector_info);

}  // namespace autoware::geography_utils

#endif  // AUTOWARE__GEOGRAPHY_UTILS__PROJECTION_HPP_
