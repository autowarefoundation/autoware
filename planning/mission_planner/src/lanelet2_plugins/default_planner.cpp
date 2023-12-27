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

#include "default_planner.hpp"

#include "utility_functions.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <tf2/utils.h>

#include <limits>
#include <vector>

namespace
{
using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;
RouteSections combine_consecutive_route_sections(
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

bool is_in_lane(const lanelet::ConstLanelet & lanelet, const lanelet::ConstPoint3d & point)
{
  // check if goal is on a lane at appropriate angle
  const auto distance = boost::geometry::distance(
    lanelet.polygon2d().basicPolygon(), lanelet::utils::to2D(point).basicPoint());
  constexpr double th_distance = std::numeric_limits<double>::epsilon();
  return distance < th_distance;
}

bool is_in_parking_space(
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

bool is_in_parking_lot(
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

double project_goal_to_map(
  const lanelet::Lanelet & lanelet_component, const lanelet::ConstPoint3d & goal_point)
{
  const lanelet::ConstLineString3d center_line =
    lanelet::utils::generateFineCenterline(lanelet_component);
  lanelet::BasicPoint3d project = lanelet::geometry::project(center_line, goal_point.basicPoint());
  return project.z();
}

geometry_msgs::msg::Pose get_closest_centerline_pose(
  const lanelet::ConstLanelets & road_lanelets, const geometry_msgs::msg::Pose & point,
  vehicle_info_util::VehicleInfo vehicle_info)
{
  lanelet::Lanelet closest_lanelet;
  if (!lanelet::utils::query::getClosestLaneletWithConstrains(
        road_lanelets, point, &closest_lanelet, 0.0)) {
    // point is not on any lanelet.
    return point;
  }

  const auto refined_center_line = lanelet::utils::generateFineCenterline(closest_lanelet, 1.0);
  closest_lanelet.setCenterline(refined_center_line);

  const double lane_yaw = lanelet::utils::getLaneletAngle(closest_lanelet, point.position);

  const auto nearest_idx =
    motion_utils::findNearestIndex(convertCenterlineToPoints(closest_lanelet), point.position);
  const auto nearest_point = closest_lanelet.centerline()[nearest_idx];

  // shift nearest point on its local y axis so that vehicle's right and left edges
  // would have approx the same clearance from road border
  const auto shift_length = (vehicle_info.right_overhang_m - vehicle_info.left_overhang_m) / 2.0;
  const auto delta_x = -shift_length * std::sin(lane_yaw);
  const auto delta_y = shift_length * std::cos(lane_yaw);

  lanelet::BasicPoint3d refined_point(
    nearest_point.x() + delta_x, nearest_point.y() + delta_y, nearest_point.z());

  return convertBasicPoint3dToPose(refined_point, lane_yaw);
}

}  // anonymous namespace

namespace mission_planner::lanelet2
{

void DefaultPlanner::initialize_common(rclcpp::Node * node)
{
  is_graph_ready_ = false;
  node_ = node;

  const auto durable_qos = rclcpp::QoS(1).transient_local();
  pub_goal_footprint_marker_ =
    node_->create_publisher<MarkerArray>("debug/goal_footprint", durable_qos);

  vehicle_info_ = vehicle_info_util::VehicleInfoUtil(*node_).getVehicleInfo();
  param_.goal_angle_threshold_deg = node_->declare_parameter<double>("goal_angle_threshold_deg");
  param_.enable_correct_goal_pose = node_->declare_parameter<bool>("enable_correct_goal_pose");
  param_.consider_no_drivable_lanes = node_->declare_parameter<bool>("consider_no_drivable_lanes");
  param_.check_footprint_inside_lanes =
    node_->declare_parameter<bool>("check_footprint_inside_lanes");
}

void DefaultPlanner::initialize(rclcpp::Node * node)
{
  initialize_common(node);
  map_subscriber_ = node_->create_subscription<HADMapBin>(
    "input/vector_map", rclcpp::QoS{10}.transient_local(),
    std::bind(&DefaultPlanner::map_callback, this, std::placeholders::_1));
}

void DefaultPlanner::initialize(rclcpp::Node * node, const HADMapBin::ConstSharedPtr msg)
{
  initialize_common(node);
  map_callback(msg);
}

bool DefaultPlanner::ready() const
{
  return is_graph_ready_;
}

void DefaultPlanner::map_callback(const HADMapBin::ConstSharedPtr msg)
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

PlannerPlugin::MarkerArray DefaultPlanner::visualize(const LaneletRoute & route) const
{
  lanelet::ConstLanelets route_lanelets;
  lanelet::ConstLanelets end_lanelets;
  lanelet::ConstLanelets normal_lanelets;
  lanelet::ConstLanelets goal_lanelets;

  for (const auto & route_section : route.segments) {
    for (const auto & lane_id : route_section.primitives) {
      auto lanelet = lanelet_map_ptr_->laneletLayer.get(lane_id.id);
      route_lanelets.push_back(lanelet);
      if (route_section.preferred_primitive.id == lane_id.id) {
        goal_lanelets.push_back(lanelet);
      } else {
        end_lanelets.push_back(lanelet);
      }
    }
  }

  std_msgs::msg::ColorRGBA cl_route;
  std_msgs::msg::ColorRGBA cl_ll_borders;
  std_msgs::msg::ColorRGBA cl_end;
  std_msgs::msg::ColorRGBA cl_normal;
  std_msgs::msg::ColorRGBA cl_goal;
  set_color(&cl_route, 0.8, 0.99, 0.8, 0.15);
  set_color(&cl_goal, 0.2, 0.4, 0.4, 0.05);
  set_color(&cl_end, 0.2, 0.2, 0.4, 0.05);
  set_color(&cl_normal, 0.2, 0.4, 0.2, 0.05);
  set_color(&cl_ll_borders, 1.0, 1.0, 1.0, 0.999);

  visualization_msgs::msg::MarkerArray route_marker_array;
  insert_marker_array(
    &route_marker_array,
    lanelet::visualization::laneletsBoundaryAsMarkerArray(route_lanelets, cl_ll_borders, false));
  insert_marker_array(
    &route_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                           "route_lanelets", route_lanelets, cl_route));
  insert_marker_array(
    &route_marker_array,
    lanelet::visualization::laneletsAsTriangleMarkerArray("end_lanelets", end_lanelets, cl_end));
  insert_marker_array(
    &route_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                           "normal_lanelets", normal_lanelets, cl_normal));
  insert_marker_array(
    &route_marker_array,
    lanelet::visualization::laneletsAsTriangleMarkerArray("goal_lanelets", goal_lanelets, cl_goal));

  return route_marker_array;
}

visualization_msgs::msg::MarkerArray DefaultPlanner::visualize_debug_footprint(
  tier4_autoware_utils::LinearRing2d goal_footprint) const
{
  visualization_msgs::msg::MarkerArray msg;
  auto marker = tier4_autoware_utils::createDefaultMarker(
    "map", rclcpp::Clock().now(), "goal_footprint", 0, visualization_msgs::msg::Marker::LINE_STRIP,
    tier4_autoware_utils::createMarkerScale(0.05, 0.0, 0.0),
    tier4_autoware_utils::createMarkerColor(0.99, 0.99, 0.2, 1.0));
  marker.lifetime = rclcpp::Duration::from_seconds(2.5);

  marker.points.push_back(
    tier4_autoware_utils::createPoint(goal_footprint[0][0], goal_footprint[0][1], 0.0));
  marker.points.push_back(
    tier4_autoware_utils::createPoint(goal_footprint[1][0], goal_footprint[1][1], 0.0));
  marker.points.push_back(
    tier4_autoware_utils::createPoint(goal_footprint[2][0], goal_footprint[2][1], 0.0));
  marker.points.push_back(
    tier4_autoware_utils::createPoint(goal_footprint[3][0], goal_footprint[3][1], 0.0));
  marker.points.push_back(
    tier4_autoware_utils::createPoint(goal_footprint[4][0], goal_footprint[4][1], 0.0));
  marker.points.push_back(
    tier4_autoware_utils::createPoint(goal_footprint[5][0], goal_footprint[5][1], 0.0));
  marker.points.push_back(marker.points.front());

  msg.markers.push_back(marker);

  return msg;
}

bool DefaultPlanner::check_goal_footprint(
  const lanelet::ConstLanelet & current_lanelet,
  const lanelet::ConstLanelet & combined_prev_lanelet,
  const tier4_autoware_utils::Polygon2d & goal_footprint, double & next_lane_length,
  const double search_margin)
{
  std::vector<tier4_autoware_utils::Point2d> points_intersection;

  // check if goal footprint is in current lane
  boost::geometry::intersection(
    goal_footprint, combined_prev_lanelet.polygon2d().basicPolygon(), points_intersection);
  if (points_intersection.empty()) {
    return true;
  }
  points_intersection.clear();

  // check if goal footprint is in between many lanelets
  for (const auto & next_lane : routing_graph_ptr_->following(current_lanelet)) {
    next_lane_length += lanelet::utils::getLaneletLength2d(next_lane);
    lanelet::ConstLanelets lanelets;
    lanelets.push_back(combined_prev_lanelet);
    lanelets.push_back(next_lane);
    lanelet::ConstLanelet combined_lanelets = combine_lanelets(lanelets);

    // if next lanelet length longer than vehicle longitudinal offset
    if (vehicle_info_.max_longitudinal_offset_m + search_margin < next_lane_length) {
      next_lane_length -= lanelet::utils::getLaneletLength2d(next_lane);
      boost::geometry::intersection(
        goal_footprint, combined_lanelets.polygon2d().basicPolygon(), points_intersection);
      if (points_intersection.empty()) {
        return true;
      }
      points_intersection.clear();
    } else {  // if next lanelet length shorter than vehicle longitudinal offset -> recursive call
      if (!check_goal_footprint(next_lane, combined_lanelets, goal_footprint, next_lane_length)) {
        next_lane_length -= lanelet::utils::getLaneletLength2d(next_lane);
        continue;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool DefaultPlanner::is_goal_valid(
  const geometry_msgs::msg::Pose & goal, lanelet::ConstLanelets path_lanelets)
{
  const auto logger = node_->get_logger();

  const auto goal_lanelet_pt = lanelet::utils::conversion::toLaneletPoint(goal.position);

  // check if goal is in shoulder lanelet
  lanelet::Lanelet closest_shoulder_lanelet;
  if (lanelet::utils::query::getClosestLanelet(
        shoulder_lanelets_, goal, &closest_shoulder_lanelet)) {
    if (is_in_lane(closest_shoulder_lanelet, goal_lanelet_pt)) {
      const auto lane_yaw =
        lanelet::utils::getLaneletAngle(closest_shoulder_lanelet, goal.position);
      const auto goal_yaw = tf2::getYaw(goal.orientation);
      const auto angle_diff = tier4_autoware_utils::normalizeRadian(lane_yaw - goal_yaw);
      const double th_angle = tier4_autoware_utils::deg2rad(param_.goal_angle_threshold_deg);
      if (std::abs(angle_diff) < th_angle) {
        return true;
      }
    }
  }

  lanelet::Lanelet closest_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(road_lanelets_, goal, &closest_lanelet)) {
    return false;
  }

  const auto local_vehicle_footprint = vehicle_info_.createFootprint();
  tier4_autoware_utils::LinearRing2d goal_footprint =
    transformVector(local_vehicle_footprint, tier4_autoware_utils::pose2transform(goal));
  pub_goal_footprint_marker_->publish(visualize_debug_footprint(goal_footprint));
  const auto polygon_footprint = convert_linear_ring_to_polygon(goal_footprint);

  double next_lane_length = 0.0;
  // combine calculated route lanelets
  lanelet::ConstLanelet combined_prev_lanelet = combine_lanelets(path_lanelets);

  // check if goal footprint exceeds lane when the goal isn't in parking_lot
  if (
    param_.check_footprint_inside_lanes &&
    !check_goal_footprint(
      closest_lanelet, combined_prev_lanelet, polygon_footprint, next_lane_length) &&
    !is_in_parking_lot(
      lanelet::utils::query::getAllParkingLots(lanelet_map_ptr_),
      lanelet::utils::conversion::toLaneletPoint(goal.position))) {
    RCLCPP_WARN(logger, "Goal's footprint exceeds lane!");
    return false;
  }

  if (is_in_lane(closest_lanelet, goal_lanelet_pt)) {
    const auto lane_yaw = lanelet::utils::getLaneletAngle(closest_lanelet, goal.position);
    const auto goal_yaw = tf2::getYaw(goal.orientation);
    const auto angle_diff = tier4_autoware_utils::normalizeRadian(lane_yaw - goal_yaw);

    const double th_angle = tier4_autoware_utils::deg2rad(param_.goal_angle_threshold_deg);
    if (std::abs(angle_diff) < th_angle) {
      return true;
    }
  }

  // check if goal is in parking space
  const auto parking_spaces = lanelet::utils::query::getAllParkingSpaces(lanelet_map_ptr_);
  if (is_in_parking_space(parking_spaces, goal_lanelet_pt)) {
    return true;
  }

  // check if goal is in parking lot
  const auto parking_lots = lanelet::utils::query::getAllParkingLots(lanelet_map_ptr_);
  if (is_in_parking_lot(parking_lots, goal_lanelet_pt)) {
    return true;
  }

  return false;
}

PlannerPlugin::LaneletRoute DefaultPlanner::plan(const RoutePoints & points)
{
  const auto logger = node_->get_logger();

  std::stringstream log_ss;
  for (const auto & point : points) {
    log_ss << "x: " << point.position.x << " "
           << "y: " << point.position.y << std::endl;
  }
  RCLCPP_INFO_STREAM(
    logger, "start planning route with check points: " << std::endl
                                                       << log_ss.str());

  LaneletRoute route_msg;
  RouteSections route_sections;

  lanelet::ConstLanelets all_route_lanelets;
  for (std::size_t i = 1; i < points.size(); i++) {
    const auto start_check_point = points.at(i - 1);
    const auto goal_check_point = points.at(i);
    lanelet::ConstLanelets path_lanelets;
    if (!route_handler_.planPathLaneletsBetweenCheckpoints(
          start_check_point, goal_check_point, &path_lanelets, param_.consider_no_drivable_lanes)) {
      RCLCPP_WARN(logger, "Failed to plan route.");
      return route_msg;
    }
    for (const auto & lane : path_lanelets) {
      all_route_lanelets.push_back(lane);
    }
    // create local route sections
    route_handler_.setRouteLanelets(path_lanelets);
    const auto local_route_sections = route_handler_.createMapSegments(path_lanelets);
    route_sections = combine_consecutive_route_sections(route_sections, local_route_sections);
  }
  route_handler_.setRouteLanelets(all_route_lanelets);

  auto goal_pose = points.back();
  if (param_.enable_correct_goal_pose) {
    goal_pose = get_closest_centerline_pose(
      lanelet::utils::query::laneletLayer(lanelet_map_ptr_), goal_pose, vehicle_info_);
  }

  if (!is_goal_valid(goal_pose, all_route_lanelets)) {
    RCLCPP_WARN(logger, "Goal is not valid! Please check position and angle of goal_pose");
    return route_msg;
  }

  if (route_handler_.isRouteLooped(route_sections)) {
    RCLCPP_WARN(logger, "Loop detected within route!");
    return route_msg;
  }

  const auto refined_goal = refine_goal_height(goal_pose, route_sections);
  RCLCPP_DEBUG(logger, "Goal Pose Z : %lf", refined_goal.position.z);

  // The header is assigned by mission planner.
  route_msg.start_pose = points.front();
  route_msg.goal_pose = refined_goal;
  route_msg.segments = route_sections;
  return route_msg;
}

geometry_msgs::msg::Pose DefaultPlanner::refine_goal_height(
  const Pose & goal, const RouteSections & route_sections)
{
  const auto goal_lane_id = route_sections.back().preferred_primitive.id;
  lanelet::Lanelet goal_lanelet = lanelet_map_ptr_->laneletLayer.get(goal_lane_id);
  const auto goal_lanelet_pt = lanelet::utils::conversion::toLaneletPoint(goal.position);
  const auto goal_height = project_goal_to_map(goal_lanelet, goal_lanelet_pt);

  Pose refined_goal = goal;
  refined_goal.position.z = goal_height;
  return refined_goal;
}

void DefaultPlanner::updateRoute(const PlannerPlugin::LaneletRoute & route)
{
  route_handler_.setRoute(route);
}

void DefaultPlanner::clearRoute()
{
  route_handler_.clearRoute();
}

}  // namespace mission_planner::lanelet2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mission_planner::lanelet2::DefaultPlanner, mission_planner::PlannerPlugin)
