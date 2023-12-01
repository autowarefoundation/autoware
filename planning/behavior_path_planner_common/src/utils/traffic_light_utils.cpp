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

#include <behavior_path_planner_common/utils/traffic_light_utils.hpp>
#include <motion_utils/trajectory/trajectory.hpp>

namespace behavior_path_planner::utils::traffic_light
{
using motion_utils::calcSignedArcLength;

bool hasTrafficLightCircleColor(const TrafficSignal & tl_state, const uint8_t & lamp_color)
{
  const auto it_lamp =
    std::find_if(tl_state.elements.begin(), tl_state.elements.end(), [&lamp_color](const auto & x) {
      return x.shape == TrafficSignalElement::CIRCLE && x.color == lamp_color;
    });

  return it_lamp != tl_state.elements.end();
}

bool hasTrafficLightShape(const TrafficSignal & tl_state, const uint8_t & lamp_shape)
{
  const auto it_lamp = std::find_if(
    tl_state.elements.begin(), tl_state.elements.end(),
    [&lamp_shape](const auto & x) { return x.shape == lamp_shape; });

  return it_lamp != tl_state.elements.end();
}

bool isTrafficSignalStop(const lanelet::ConstLanelet & lanelet, const TrafficSignal & tl_state)
{
  if (hasTrafficLightCircleColor(tl_state, TrafficSignalElement::GREEN)) {
    return false;
  }

  if (hasTrafficLightCircleColor(tl_state, TrafficSignalElement::UNKNOWN)) {
    return false;
  }

  const std::string turn_direction = lanelet.attributeOr("turn_direction", "else");

  if (turn_direction == "else") {
    return true;
  }
  if (
    turn_direction == "right" &&
    hasTrafficLightShape(tl_state, TrafficSignalElement::RIGHT_ARROW)) {
    return false;
  }
  if (
    turn_direction == "left" && hasTrafficLightShape(tl_state, TrafficSignalElement::LEFT_ARROW)) {
    return false;
  }
  if (
    turn_direction == "straight" &&
    hasTrafficLightShape(tl_state, TrafficSignalElement::UP_ARROW)) {
    return false;
  }

  return true;
}

double getDistanceToNextTrafficLight(
  const Pose & current_pose, const lanelet::ConstLanelets & lanelets)
{
  lanelet::ConstLanelet current_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(lanelets, current_pose, &current_lanelet)) {
    return std::numeric_limits<double>::infinity();
  }

  const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(current_pose.position);
  const auto to_object = lanelet::geometry::toArcCoordinates(
    lanelet::utils::to2D(current_lanelet.centerline()),
    lanelet::utils::to2D(lanelet_point).basicPoint());

  for (const auto & element : current_lanelet.regulatoryElementsAs<lanelet::TrafficLight>()) {
    lanelet::ConstLineString3d lanelet_stop_lines = element->stopLine().value();

    const auto to_stop_line = lanelet::geometry::toArcCoordinates(
      lanelet::utils::to2D(current_lanelet.centerline()),
      lanelet::utils::to2D(lanelet_stop_lines).front().basicPoint());

    const auto distance_object_to_stop_line = to_stop_line.length - to_object.length;

    if (distance_object_to_stop_line > 0.0) {
      return distance_object_to_stop_line;
    }
  }

  double distance = lanelet::utils::getLaneletLength3d(current_lanelet);

  bool found_current_lane = false;
  for (const auto & llt : lanelets) {
    if (llt.id() == current_lanelet.id()) {
      found_current_lane = true;
      continue;
    }

    if (!found_current_lane) {
      continue;
    }

    for (const auto & element : llt.regulatoryElementsAs<lanelet::TrafficLight>()) {
      lanelet::ConstLineString3d lanelet_stop_lines = element->stopLine().value();

      const auto to_stop_line = lanelet::geometry::toArcCoordinates(
        lanelet::utils::to2D(llt.centerline()),
        lanelet::utils::to2D(lanelet_stop_lines).front().basicPoint());

      return distance + to_stop_line.length - to_object.length;
    }

    distance += lanelet::utils::getLaneletLength3d(llt);
  }

  return std::numeric_limits<double>::infinity();
}

std::optional<double> calcDistanceToRedTrafficLight(
  const lanelet::ConstLanelets & lanelets, const PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> & planner_data)
{
  for (const auto & lanelet : lanelets) {
    for (const auto & element : lanelet.regulatoryElementsAs<TrafficLight>()) {
      const auto traffic_signal_stamped = planner_data->getTrafficSignal(element->id());
      if (!traffic_signal_stamped.has_value()) {
        continue;
      }

      if (!isTrafficSignalStop(lanelet, traffic_signal_stamped.value().signal)) {
        continue;
      }

      const auto & ego_pos = planner_data->self_odometry->pose.pose.position;
      lanelet::ConstLineString3d stop_line = *(element->stopLine());
      const auto x = 0.5 * (stop_line.front().x() + stop_line.back().x());
      const auto y = 0.5 * (stop_line.front().y() + stop_line.back().y());
      const auto z = 0.5 * (stop_line.front().z() + stop_line.back().z());

      return calcSignedArcLength(path.points, ego_pos, tier4_autoware_utils::createPoint(x, y, z));
    }
  }

  return std::nullopt;
}

bool isStoppedAtRedTrafficLightWithinDistance(
  const lanelet::ConstLanelets & lanelets, const PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> & planner_data, const double distance_threshold)
{
  const auto ego_velocity = std::hypot(
    planner_data->self_odometry->twist.twist.linear.x,
    planner_data->self_odometry->twist.twist.linear.y);
  constexpr double minimum_speed = 0.1;
  if (ego_velocity > minimum_speed) {
    return false;
  }

  const auto distance_to_red_traffic_light =
    calcDistanceToRedTrafficLight(lanelets, path, planner_data);

  if (!distance_to_red_traffic_light) {
    return false;
  }

  return (distance_to_red_traffic_light < distance_threshold);
}

}  // namespace behavior_path_planner::utils::traffic_light
