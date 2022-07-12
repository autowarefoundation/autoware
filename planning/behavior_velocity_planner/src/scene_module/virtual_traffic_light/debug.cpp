// Copyright 2021 Tier IV, Inc.
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

#include <motion_utils/motion_utils.hpp>
#include <scene_module/virtual_traffic_light/scene.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

using motion_utils::createStopVirtualWallMarker;
using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerOrientation;
using tier4_autoware_utils::createMarkerPosition;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::toMsg;
using namespace std::literals::string_literals;

namespace behavior_velocity_planner
{
namespace
{
[[maybe_unused]] tier4_autoware_utils::LinearRing3d createCircle(
  const tier4_autoware_utils::Point3d & p, const double radius, const size_t num_points = 50)
{
  tier4_autoware_utils::LinearRing3d ring;  // clockwise and closed

  for (size_t i = 0; i < num_points; ++i) {
    const double theta = i * (2 * tier4_autoware_utils::pi / num_points);
    const double x = p.x() + radius * std::sin(theta);
    const double y = p.y() + radius * std::cos(theta);
    ring.emplace_back(x, y, p.z());
  }

  // Make closed
  ring.emplace_back(p.x(), p.y() + radius, p.z());

  return ring;
}
}  // namespace

visualization_msgs::msg::MarkerArray VirtualTrafficLightModule::createVirtualWallMarkerArray()
{
  visualization_msgs::msg::MarkerArray wall_marker;

  const auto & d = module_data_;
  const auto now = clock_->now();

  // virtual_wall_stop_line
  if (d.stop_head_pose_at_stop_line) {
    const auto markers = createStopVirtualWallMarker(
      *d.stop_head_pose_at_stop_line, "virtual_traffic_light", now, module_id_);

    appendMarkerArray(markers, &wall_marker);
  }

  // virtual_wall_end_line
  if (d.stop_head_pose_at_end_line) {
    const auto markers = createStopVirtualWallMarker(
      *d.stop_head_pose_at_end_line, "virtual_traffic_light", now, module_id_);

    appendMarkerArray(markers, &wall_marker);
  }

  return wall_marker;
}

visualization_msgs::msg::MarkerArray VirtualTrafficLightModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;

  // Common
  const auto & m = map_data_;
  const auto now = clock_->now();

  // instrument_id
  {
    auto marker = createDefaultMarker(
      "map", now, "instrument_id", module_id_, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      createMarkerScale(0.0, 0.0, 1.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));

    marker.pose.position = toMsg(m.instrument_center);
    marker.text = m.instrument_id;

    debug_marker_array.markers.push_back(marker);
  }

  // instrument_center
  {
    auto marker = createDefaultMarker(
      "map", now, "instrument_center", module_id_, visualization_msgs::msg::Marker::SPHERE,
      createMarkerScale(0.3, 0.3, 0.3), createMarkerColor(1.0, 0.0, 0.0, 0.999));

    marker.pose.position = toMsg(m.instrument_center);

    debug_marker_array.markers.push_back(marker);
  }

  // stop_line
  if (m.stop_line) {
    auto marker = createDefaultMarker(
      "map", now, "stop_line", module_id_, visualization_msgs::msg::Marker::LINE_STRIP,
      createMarkerScale(0.3, 0.0, 0.0), createMarkerColor(1.0, 1.0, 1.0, 0.999));

    for (const auto & p : *m.stop_line) {
      marker.points.push_back(toMsg(p));
    }

    debug_marker_array.markers.push_back(marker);
  }

  // start_line
  {
    auto marker = createDefaultMarker(
      "map", now, "start_line", module_id_, visualization_msgs::msg::Marker::LINE_STRIP,
      createMarkerScale(0.3, 0.0, 0.0), createMarkerColor(0.0, 1.0, 0.0, 0.999));

    for (const auto & p : m.start_line) {
      marker.points.push_back(toMsg(p));
    }

    debug_marker_array.markers.push_back(marker);
  }

  // end_lines
  {
    auto marker = createDefaultMarker(
      "map", now, "end_lines", module_id_, visualization_msgs::msg::Marker::LINE_LIST,
      createMarkerScale(0.3, 0.0, 0.0), createMarkerColor(0.0, 1.0, 1.0, 0.999));

    for (const auto & line : m.end_lines) {
      for (size_t i = 1; i < line.size(); ++i) {
        marker.points.push_back(toMsg(line.at(i - 1)));
        marker.points.push_back(toMsg(line.at(i)));
      }
    }

    debug_marker_array.markers.push_back(marker);
  }

  return debug_marker_array;
}
}  // namespace behavior_velocity_planner
