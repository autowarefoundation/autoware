// Copyright 2019 Autoware Foundation
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

#include "mission_planner/lanelet2_impl/mission_planner_lanelet2.hpp"

#include "mission_planner/lanelet2_impl/utility_functions.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <limits>
#include <memory>
#include <unordered_set>

namespace
{
using RouteSections = std::vector<autoware_auto_mapping_msgs::msg::HADMapSegment>;
RouteSections combineConsecutiveRouteSections(
  const RouteSections & route_sections1, const RouteSections & route_sections2)
{
  RouteSections route_sections;
  route_sections.reserve(route_sections1.size() + route_sections2.size());
  if (!route_sections1.empty()) {
    // remove end route section because it is overlapping with first one in next route_section
    route_sections.insert(route_sections.end(), route_sections1.begin(), route_sections1.end() - 1);
  }
  if (!route_sections2.empty()) {
    route_sections.insert(route_sections.end(), route_sections2.begin(), route_sections2.end());
  }
  return route_sections;
}

bool isRouteLooped(const RouteSections & route_sections)
{
  for (std::size_t i = 0; i < route_sections.size(); i++) {
    const auto & route_section = route_sections.at(i);
    for (const auto & lane_id : route_section.primitives) {
      for (std::size_t j = i + 1; j < route_sections.size(); j++) {
        const auto & future_route_section = route_sections.at(j);
        if (exists(future_route_section.primitives, lane_id)) {
          return true;
        }
      }
    }
  }
  return false;
}

double normalizeRadian(const double rad, const double min_rad = -M_PI, const double max_rad = M_PI)
{
  const auto value = std::fmod(rad, 2 * M_PI);
  if (min_rad < value && value <= max_rad) {
    return value;
  } else {
    return value - std::copysign(2 * M_PI, value);
  }
}

bool isInLane(const lanelet::ConstLanelet & lanelet, const lanelet::ConstPoint3d & point)
{
  // check if goal is on a lane at appropriate angle
  const auto distance = boost::geometry::distance(
    lanelet.polygon2d().basicPolygon(), lanelet::utils::to2D(point).basicPoint());
  constexpr double th_distance = std::numeric_limits<double>::epsilon();
  return distance < th_distance;
}

bool isInParkingSpace(
  const lanelet::ConstLineStrings3d & parking_spaces, const lanelet::ConstPoint3d & point)
{
  for (const auto & parking_space : parking_spaces) {
    lanelet::ConstPolygon3d parking_space_polygon;
    if (!lanelet::utils::lineStringWithWidthToPolygon(parking_space, &parking_space_polygon)) {
      continue;
    }

    const double distance = boost::geometry::distance(
      lanelet::utils::to2D(parking_space_polygon).basicPolygon(),
      lanelet::utils::to2D(point).basicPoint());
    constexpr double th_distance = std::numeric_limits<double>::epsilon();
    if (distance < th_distance) {
      return true;
    }
  }
  return false;
}

bool isInParkingLot(
  const lanelet::ConstPolygons3d & parking_lots, const lanelet::ConstPoint3d & point)
{
  for (const auto & parking_lot : parking_lots) {
    const double distance = boost::geometry::distance(
      lanelet::utils::to2D(parking_lot).basicPolygon(), lanelet::utils::to2D(point).basicPoint());
    constexpr double th_distance = std::numeric_limits<double>::epsilon();
    if (distance < th_distance) {
      return true;
    }
  }
  return false;
}

}  // anonymous namespace

namespace mission_planner
{
MissionPlannerLanelet2::MissionPlannerLanelet2(const rclcpp::NodeOptions & node_options)
: MissionPlanner("mission_planner_node", node_options), is_graph_ready_(false)
{
  using std::placeholders::_1;
  map_subscriber_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "input/vector_map", rclcpp::QoS{10}.transient_local(),
    std::bind(&MissionPlannerLanelet2::mapCallback, this, _1));
}

void MissionPlannerLanelet2::mapCallback(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg)
{
  route_handler_.setMap(*msg);
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lanelets);
  shoulder_lanelets_ = lanelet::utils::query::shoulderLanelets(all_lanelets);
  is_graph_ready_ = true;
}

bool MissionPlannerLanelet2::isRoutingGraphReady() const { return is_graph_ready_; }

void MissionPlannerLanelet2::visualizeRoute(
  const autoware_auto_planning_msgs::msg::HADMapRoute & route) const
{
  lanelet::ConstLanelets route_lanelets;
  lanelet::ConstLanelets end_lanelets;
  lanelet::ConstLanelets normal_lanelets;
  lanelet::ConstLanelets goal_lanelets;

  for (const auto & route_section : route.segments) {
    for (const auto & lane_id : route_section.primitives) {
      auto lanelet = lanelet_map_ptr_->laneletLayer.get(lane_id.id);
      route_lanelets.push_back(lanelet);
      if (route_section.preferred_primitive_id == lane_id.id) {
        goal_lanelets.push_back(lanelet);
      } else {
        end_lanelets.push_back(lanelet);
      }
    }
  }

  std_msgs::msg::ColorRGBA cl_route, cl_ll_borders, cl_end, cl_normal, cl_goal;
  setColor(&cl_route, 0.2, 0.4, 0.2, 0.05);
  setColor(&cl_goal, 0.2, 0.4, 0.4, 0.05);
  setColor(&cl_end, 0.2, 0.2, 0.4, 0.05);
  setColor(&cl_normal, 0.2, 0.4, 0.2, 0.05);
  setColor(&cl_ll_borders, 1.0, 1.0, 1.0, 0.999);

  visualization_msgs::msg::MarkerArray route_marker_array;
  insertMarkerArray(
    &route_marker_array,
    lanelet::visualization::laneletsBoundaryAsMarkerArray(route_lanelets, cl_ll_borders, false));
  insertMarkerArray(
    &route_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                           "route_lanelets", route_lanelets, cl_route));
  insertMarkerArray(
    &route_marker_array,
    lanelet::visualization::laneletsAsTriangleMarkerArray("end_lanelets", end_lanelets, cl_end));
  insertMarkerArray(
    &route_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                           "normal_lanelets", normal_lanelets, cl_normal));
  insertMarkerArray(
    &route_marker_array,
    lanelet::visualization::laneletsAsTriangleMarkerArray("goal_lanelets", goal_lanelets, cl_goal));
  marker_publisher_->publish(route_marker_array);
}

bool MissionPlannerLanelet2::isGoalValid() const
{
  lanelet::Lanelet closest_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(
        road_lanelets_, goal_pose_.pose, &closest_lanelet)) {
    return false;
  }
  const auto goal_lanelet_pt = lanelet::utils::conversion::toLaneletPoint(goal_pose_.pose.position);

  if (isInLane(closest_lanelet, goal_lanelet_pt)) {
    const auto lane_yaw =
      lanelet::utils::getLaneletAngle(closest_lanelet, goal_pose_.pose.position);
    const auto goal_yaw = tf2::getYaw(goal_pose_.pose.orientation);
    const auto angle_diff = normalizeRadian(lane_yaw - goal_yaw);

    constexpr double th_angle = M_PI / 4;

    if (std::abs(angle_diff) < th_angle) {
      return true;
    }
  }

  // check if goal is in parking space
  const auto parking_spaces = lanelet::utils::query::getAllParkingSpaces(lanelet_map_ptr_);
  if (isInParkingSpace(parking_spaces, goal_lanelet_pt)) {
    return true;
  }

  // check if goal is in parking lot
  const auto parking_lots = lanelet::utils::query::getAllParkingLots(lanelet_map_ptr_);
  if (isInParkingLot(parking_lots, goal_lanelet_pt)) {
    return true;
  }

  // check if goal is in shoulder lanelet
  lanelet::Lanelet closest_shoulder_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(
        shoulder_lanelets_, goal_pose_.pose, &closest_shoulder_lanelet)) {
    return false;
  }
  // check if goal pose is in shoulder lane
  if (isInLane(closest_shoulder_lanelet, goal_lanelet_pt)) {
    const auto lane_yaw =
      lanelet::utils::getLaneletAngle(closest_shoulder_lanelet, goal_pose_.pose.position);
    const auto goal_yaw = tf2::getYaw(goal_pose_.pose.orientation);
    const auto angle_diff = normalizeRadian(lane_yaw - goal_yaw);

    constexpr double th_angle = M_PI / 4;
    if (std::abs(angle_diff) < th_angle) {
      return true;
    }
  }

  return false;
}

autoware_auto_planning_msgs::msg::HADMapRoute MissionPlannerLanelet2::planRoute()
{
  std::stringstream ss;
  for (const auto & checkpoint : checkpoints_) {
    ss << "x: " << checkpoint.pose.position.x << " "
       << "y: " << checkpoint.pose.position.y << std::endl;
  }
  RCLCPP_INFO_STREAM(
    get_logger(), "start planning route with checkpoints: " << std::endl
                                                            << ss.str());

  autoware_auto_planning_msgs::msg::HADMapRoute route_msg;
  RouteSections route_sections;

  if (!isGoalValid()) {
    RCLCPP_WARN(get_logger(), "Goal is not valid! Please check position and angle of goal_pose");
    return route_msg;
  }

  for (std::size_t i = 1; i < checkpoints_.size(); i++) {
    const auto start_checkpoint = checkpoints_.at(i - 1);
    const auto goal_checkpoint = checkpoints_.at(i);
    lanelet::ConstLanelets path_lanelets;
    if (!route_handler_.planPathLaneletsBetweenCheckpoints(
          start_checkpoint.pose, goal_checkpoint.pose, &path_lanelets)) {
      return route_msg;
    }
    // create local route sections
    route_handler_.setRouteLanelets(path_lanelets);
    const auto local_route_sections = route_handler_.createMapSegments(path_lanelets);
    route_sections = combineConsecutiveRouteSections(route_sections, local_route_sections);
  }

  if (isRouteLooped(route_sections)) {
    RCLCPP_WARN(
      get_logger(), "Loop detected within route! Be aware that looped route is not debugged!");
  }

  route_msg.header.stamp = this->now();
  route_msg.header.frame_id = map_frame_;
  route_msg.segments = route_sections;
  route_msg.goal_pose = goal_pose_.pose;

  return route_msg;
}

}  // namespace mission_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mission_planner::MissionPlannerLanelet2)
