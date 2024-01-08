// Copyright 2022 Tier IV, Inc.
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

#include "static_centerline_optimizer/static_centerline_optimizer_node.hpp"

#include "lanelet2_extension/utility/message_conversion.hpp"
#include "lanelet2_extension/utility/query.hpp"
#include "lanelet2_extension/utility/utilities.hpp"
#include "map_loader/lanelet2_map_loader_node.hpp"
#include "motion_utils/resample/resample.hpp"
#include "motion_utils/trajectory/conversion.hpp"
#include "obstacle_avoidance_planner/node.hpp"
#include "path_smoother/elastic_band_smoother.hpp"
#include "static_centerline_optimizer/msg/points_with_lane_id.hpp"
#include "static_centerline_optimizer/type_alias.hpp"
#include "static_centerline_optimizer/utils.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"

#include <mission_planner/mission_planner_plugin.hpp>
#include <pluginlib/class_loader.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <tier4_map_msgs/msg/map_projector_info.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/distance.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace static_centerline_optimizer
{
namespace
{
Path convert_to_path(const PathWithLaneId & path_with_lane_id)
{
  Path path;
  path.header = path_with_lane_id.header;
  path.left_bound = path_with_lane_id.left_bound;
  path.right_bound = path_with_lane_id.right_bound;
  for (const auto & point : path_with_lane_id.points) {
    path.points.push_back(point.point);
  }

  return path;
}

Trajectory convert_to_trajectory(const Path & path)
{
  Trajectory traj;
  for (const auto & point : path.points) {
    TrajectoryPoint traj_point;
    traj_point.pose = point.pose;
    traj_point.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
    traj_point.lateral_velocity_mps = point.lateral_velocity_mps;
    traj_point.heading_rate_rps = point.heading_rate_rps;

    traj.points.push_back(traj_point);
  }
  return traj;
}

[[maybe_unused]] lanelet::ConstLanelets get_lanelets_from_route(
  const RouteHandler & route_handler, const LaneletRoute & route)
{
  lanelet::ConstLanelets lanelets;
  for (const auto & segment : route.segments) {
    const auto & target_lanelet_id = segment.preferred_primitive.id;
    const auto target_lanelet = route_handler.getLaneletsFromId(target_lanelet_id);
    lanelets.push_back(target_lanelet);
  }

  return lanelets;
}

std::vector<lanelet::Id> get_lane_ids_from_route(const LaneletRoute & route)
{
  std::vector<lanelet::Id> lane_ids;
  for (const auto & segment : route.segments) {
    const auto & target_lanelet_id = segment.preferred_primitive.id;
    lane_ids.push_back(target_lanelet_id);
  }

  return lane_ids;
}

lanelet::ConstLanelets get_lanelets_from_ids(
  const RouteHandler & route_handler, const std::vector<lanelet::Id> & lane_ids)
{
  lanelet::ConstLanelets lanelets;
  for (const lanelet::Id lane_id : lane_ids) {
    const auto lanelet = route_handler.getLaneletsFromId(lane_id);
    lanelets.push_back(lanelet);
  }
  return lanelets;
}

rclcpp::NodeOptions create_node_options()
{
  return rclcpp::NodeOptions{};
}

rclcpp::QoS create_transient_local_qos()
{
  return rclcpp::QoS{1}.transient_local();
}

lanelet::BasicPoint2d convertToLaneletPoint(const geometry_msgs::msg::Point & geom_point)
{
  lanelet::BasicPoint2d point(geom_point.x, geom_point.y);
  return point;
}

LinearRing2d create_vehicle_footprint(
  const geometry_msgs::msg::Pose & pose, const vehicle_info_util::VehicleInfo & vehicle_info,
  const double margin = 0.0)
{
  const auto & i = vehicle_info;

  const double x_front = i.front_overhang_m + i.wheel_base_m + margin;
  const double x_rear = -(i.rear_overhang_m + margin);
  const double y_left = i.wheel_tread_m / 2.0 + i.left_overhang_m + margin;
  const double y_right = -(i.wheel_tread_m / 2.0 + i.right_overhang_m + margin);

  std::vector<geometry_msgs::msg::Point> geom_points;
  geom_points.push_back(tier4_autoware_utils::calcOffsetPose(pose, x_front, y_left, 0.0).position);
  geom_points.push_back(tier4_autoware_utils::calcOffsetPose(pose, x_front, y_right, 0.0).position);
  geom_points.push_back(tier4_autoware_utils::calcOffsetPose(pose, x_rear, y_right, 0.0).position);
  geom_points.push_back(tier4_autoware_utils::calcOffsetPose(pose, x_rear, y_left, 0.0).position);

  LinearRing2d footprint;
  for (const auto & geom_point : geom_points) {
    footprint.push_back(Point2d{geom_point.x, geom_point.y});
  }
  footprint.push_back(footprint.back());

  boost::geometry::correct(footprint);

  return footprint;
}

geometry_msgs::msg::Pose get_text_pose(
  const geometry_msgs::msg::Pose & pose, const vehicle_info_util::VehicleInfo & vehicle_info)
{
  const auto & i = vehicle_info;

  const double x_front = i.front_overhang_m + i.wheel_base_m;
  const double y_left = i.wheel_tread_m / 2.0 + i.left_overhang_m + 1.0;

  return tier4_autoware_utils::calcOffsetPose(pose, x_front, y_left, 0.0);
}

std::array<double, 3> convertHexStringToDecimal(const std::string & hex_str_color)
{
  unsigned int hex_int_color;
  std::istringstream iss(hex_str_color);
  iss >> std::hex >> hex_int_color;

  unsigned int unit = 16 * 16;
  unsigned int b = hex_int_color % unit;
  unsigned int g = (hex_int_color - b) / unit % unit;
  unsigned int r = (hex_int_color - g * unit - b) / unit / unit;

  return std::array<double, 3>{r / 255.0, g / 255.0, b / 255.0};
}

std::vector<lanelet::Id> check_lanelet_connection(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & route_lanelets)
{
  std::vector<lanelet::Id> unconnected_lane_ids;

  for (size_t i = 0; i < route_lanelets.size() - 1; ++i) {
    const auto next_lanelets = route_handler.getNextLanelets(route_lanelets.at(i));

    const bool is_connected =
      std::find_if(next_lanelets.begin(), next_lanelets.end(), [&](const auto & next_lanelet) {
        return next_lanelet.id() == route_lanelets.at(i + 1).id();
      }) != next_lanelets.end();
    if (!is_connected) {
      unconnected_lane_ids.push_back(route_lanelets.at(i).id());
    }
  }

  return unconnected_lane_ids;
}
}  // namespace

StaticCenterlineOptimizerNode::StaticCenterlineOptimizerNode(
  const rclcpp::NodeOptions & node_options)
: Node("static_centerline_optimizer", node_options)
{
  // publishers
  pub_map_bin_ = create_publisher<HADMapBin>("lanelet2_map_topic", create_transient_local_qos());
  pub_raw_path_with_lane_id_ =
    create_publisher<PathWithLaneId>("input_centerline", create_transient_local_qos());
  pub_raw_path_ = create_publisher<Path>("debug/raw_centerline", create_transient_local_qos());
  pub_optimized_centerline_ =
    create_publisher<Trajectory>("output_centerline", create_transient_local_qos());

  // debug publishers
  pub_debug_unsafe_footprints_ =
    create_publisher<MarkerArray>("debug/unsafe_footprints", create_transient_local_qos());

  // services
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_load_map_ = create_service<LoadMap>(
    "/planning/static_centerline_optimizer/load_map",
    std::bind(
      &StaticCenterlineOptimizerNode::on_load_map, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_);
  srv_plan_route_ = create_service<PlanRoute>(
    "/planning/static_centerline_optimizer/plan_route",
    std::bind(
      &StaticCenterlineOptimizerNode::on_plan_route, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_);
  srv_plan_path_ = create_service<PlanPath>(
    "/planning/static_centerline_optimizer/plan_path",
    std::bind(
      &StaticCenterlineOptimizerNode::on_plan_path, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_);

  // vehicle info
  vehicle_info_ = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
}

void StaticCenterlineOptimizerNode::run()
{
  // declare planning setting parameters
  const auto lanelet2_input_file_path = declare_parameter<std::string>("lanelet2_input_file_path");
  const auto lanelet2_output_file_path =
    declare_parameter<std::string>("lanelet2_output_file_path");
  const lanelet::Id start_lanelet_id = declare_parameter<int64_t>("start_lanelet_id");
  const lanelet::Id end_lanelet_id = declare_parameter<int64_t>("end_lanelet_id");

  // process
  load_map(lanelet2_input_file_path);
  const auto route_lane_ids = plan_route(start_lanelet_id, end_lanelet_id);
  const auto optimized_traj_points = plan_path(route_lane_ids);
  save_map(lanelet2_output_file_path, route_lane_ids, optimized_traj_points);
}

void StaticCenterlineOptimizerNode::load_map(const std::string & lanelet2_input_file_path)
{
  // load map by the map_loader package
  map_bin_ptr_ = [&]() -> HADMapBin::ConstSharedPtr {
    // load map
    tier4_map_msgs::msg::MapProjectorInfo map_projector_info;
    map_projector_info.projector_type = tier4_map_msgs::msg::MapProjectorInfo::MGRS;
    const auto map_ptr =
      Lanelet2MapLoaderNode::load_map(lanelet2_input_file_path, map_projector_info);
    if (!map_ptr) {
      return nullptr;
    }

    // NOTE: The original map is stored here since the various ids in the lanelet map will change
    //       after lanelet::utils::overwriteLaneletCenterline, and saving map will fail.
    original_map_ptr_ =
      Lanelet2MapLoaderNode::load_map(lanelet2_input_file_path, map_projector_info);

    // overwrite more dense centerline
    lanelet::utils::overwriteLaneletsCenterline(map_ptr, 5.0, false);

    // create map bin msg
    const auto map_bin_msg =
      Lanelet2MapLoaderNode::create_map_bin_msg(map_ptr, lanelet2_input_file_path, now());

    return std::make_shared<HADMapBin>(map_bin_msg);
  }();

  // check if map_bin_ptr_ is not null pointer
  if (!map_bin_ptr_) {
    RCLCPP_ERROR(get_logger(), "Loading map failed");
    return;
  }
  RCLCPP_INFO(get_logger(), "Loaded map.");

  // publish map bin msg
  pub_map_bin_->publish(*map_bin_ptr_);
  RCLCPP_INFO(get_logger(), "Published map.");

  // create route_handler
  route_handler_ptr_ = std::make_shared<RouteHandler>();
  route_handler_ptr_->setMap(*map_bin_ptr_);
}

void StaticCenterlineOptimizerNode::on_load_map(
  const LoadMap::Request::SharedPtr request, const LoadMap::Response::SharedPtr response)
{
  const std::string tmp_lanelet2_input_file_path = "/tmp/input_lanelet2_map.osm";

  // save map file temporarily since load map's input must be a file
  std::ofstream map_writer;
  map_writer.open(tmp_lanelet2_input_file_path, std::ios::out);
  map_writer << request->map;
  map_writer.close();

  // load map from the saved map file
  load_map(tmp_lanelet2_input_file_path);

  if (map_bin_ptr_) {
    return;
  }

  response->message = "InvalidMapFormat";
}

std::vector<lanelet::Id> StaticCenterlineOptimizerNode::plan_route(
  const lanelet::Id start_lanelet_id, const lanelet::Id end_lanelet_id)
{
  if (!map_bin_ptr_ || !route_handler_ptr_) {
    RCLCPP_ERROR(get_logger(), "Map or route handler is not ready. Return empty lane ids.");
    return std::vector<lanelet::Id>{};
  }

  // calculate check points (= start and goal pose)
  const auto check_points = [&]() {
    const auto start_center_pose = utils::get_center_pose(*route_handler_ptr_, start_lanelet_id);
    const auto end_center_pose = utils::get_center_pose(*route_handler_ptr_, end_lanelet_id);
    return std::vector<geometry_msgs::msg::Pose>{start_center_pose, end_center_pose};
  }();
  RCLCPP_INFO(get_logger(), "Calculated check points.");

  // plan route by the mission_planner package
  const auto route = [&]() {
    // create mission_planner plugin
    auto plugin_loader = pluginlib::ClassLoader<mission_planner::PlannerPlugin>(
      "mission_planner", "mission_planner::PlannerPlugin");
    auto mission_planner =
      plugin_loader.createSharedInstance("mission_planner::lanelet2::DefaultPlanner");

    // initialize mission_planner
    auto node = rclcpp::Node("po");
    mission_planner->initialize(&node, map_bin_ptr_);

    // plan route
    const auto route = mission_planner->plan(check_points);

    return route;
  }();
  RCLCPP_INFO(get_logger(), "Planned route.");

  // get lanelets
  const auto route_lane_ids = get_lane_ids_from_route(route);
  return route_lane_ids;
}

void StaticCenterlineOptimizerNode::on_plan_route(
  const PlanRoute::Request::SharedPtr request, const PlanRoute::Response::SharedPtr response)
{
  if (!map_bin_ptr_ || !route_handler_ptr_) {
    response->message = "MapNotFound";
    RCLCPP_ERROR(get_logger(), "Map is not ready.");
    return;
  }

  const lanelet::Id start_lanelet_id = request->start_lane_id;
  const lanelet::Id end_lanelet_id = request->end_lane_id;

  // plan route
  const auto route_lane_ids = plan_route(start_lanelet_id, end_lanelet_id);
  const auto route_lanelets = get_lanelets_from_ids(*route_handler_ptr_, route_lane_ids);

  // extract lane ids
  std::vector<lanelet::Id> lane_ids;
  for (const auto & lanelet : route_lanelets) {
    lane_ids.push_back(lanelet.id());
  }

  // check calculation result
  if (lane_ids.empty()) {
    response->message = "RouteNotFound";
    RCLCPP_ERROR(get_logger(), "Route planning failed.");
    return;
  }

  // set response
  response->lane_ids = lane_ids;
}

std::vector<TrajectoryPoint> StaticCenterlineOptimizerNode::plan_path(
  const std::vector<lanelet::Id> & route_lane_ids)
{
  if (!route_handler_ptr_) {
    RCLCPP_ERROR(get_logger(), "Route handler is not ready. Return empty trajectory.");
    return std::vector<TrajectoryPoint>{};
  }

  const auto route_lanelets = get_lanelets_from_ids(*route_handler_ptr_, route_lane_ids);

  // optimize centerline inside the lane
  const auto start_center_pose =
    utils::get_center_pose(*route_handler_ptr_, route_lane_ids.front());

  // get ego nearest search parameters and resample interval in behavior_path_planner
  const double ego_nearest_dist_threshold =
    has_parameter("ego_nearest_dist_threshold")
      ? get_parameter("ego_nearest_dist_threshold").as_double()
      : declare_parameter<double>("ego_nearest_dist_threshold");
  const double ego_nearest_yaw_threshold =
    has_parameter("ego_nearest_yaw_threshold")
      ? get_parameter("ego_nearest_yaw_threshold").as_double()
      : declare_parameter<double>("ego_nearest_yaw_threshold");
  const double behavior_path_interval = has_parameter("output_path_interval")
                                          ? get_parameter("output_path_interval").as_double()
                                          : declare_parameter<double>("output_path_interval");

  // extract path with lane id from lanelets
  const auto raw_path_with_lane_id = [&]() {
    const auto non_resampled_path_with_lane_id = utils::get_path_with_lane_id(
      *route_handler_ptr_, route_lanelets, start_center_pose, ego_nearest_dist_threshold,
      ego_nearest_yaw_threshold);
    return motion_utils::resamplePath(non_resampled_path_with_lane_id, behavior_path_interval);
  }();
  pub_raw_path_with_lane_id_->publish(raw_path_with_lane_id);
  RCLCPP_INFO(get_logger(), "Calculated raw path with lane id and published.");

  // convert path with lane id to path
  const auto raw_path = [&]() {
    const auto non_resampled_path = convert_to_path(raw_path_with_lane_id);
    // NOTE: The behavior_velocity_planner resamples with the interval 1.0 somewhere.
    return motion_utils::resamplePath(non_resampled_path, 1.0);
  }();
  pub_raw_path_->publish(raw_path);
  RCLCPP_INFO(get_logger(), "Converted to path and published.");

  // smooth trajectory and road collision avoidance
  const auto optimized_traj_points = optimize_trajectory(raw_path);
  pub_optimized_centerline_->publish(
    motion_utils::convertToTrajectory(optimized_traj_points, raw_path.header));
  RCLCPP_INFO(
    get_logger(), "Smoothed trajectory and made it collision free with the road and published.");

  return optimized_traj_points;
}

std::vector<TrajectoryPoint> StaticCenterlineOptimizerNode::optimize_trajectory(
  const Path & raw_path) const
{
  // convert to trajectory points
  const auto raw_traj_points = [&]() {
    const auto raw_traj = convert_to_trajectory(raw_path);
    return motion_utils::convertToTrajectoryPointArray(raw_traj);
  }();

  // create an instance of elastic band and model predictive trajectory.
  const auto eb_path_smoother_ptr =
    path_smoother::ElasticBandSmoother(create_node_options()).getElasticBandSmoother();
  const auto mpt_optimizer_ptr =
    obstacle_avoidance_planner::ObstacleAvoidancePlanner(create_node_options()).getMPTOptimizer();

  // NOTE: The optimization is executed every valid_optimized_traj_points_num points.
  constexpr int valid_optimized_traj_points_num = 10;
  const int traj_segment_num = raw_traj_points.size() / valid_optimized_traj_points_num;

  // NOTE: num_initial_optimization exists to make the both optimizations stable since they may use
  // warm start.
  constexpr int num_initial_optimization = 2;

  std::vector<TrajectoryPoint> whole_optimized_traj_points;
  for (int virtual_ego_pose_idx = -num_initial_optimization;
       virtual_ego_pose_idx < traj_segment_num; ++virtual_ego_pose_idx) {
    // calculate virtual ego pose for the optimization
    constexpr int virtual_ego_pose_offset_idx = 1;
    const auto virtual_ego_pose =
      raw_traj_points
        .at(
          valid_optimized_traj_points_num * std::max(virtual_ego_pose_idx, 0) +
          virtual_ego_pose_offset_idx)
        .pose;

    // smooth trajectory by elastic band in the path_smoother package
    const auto smoothed_traj_points =
      eb_path_smoother_ptr->smoothTrajectory(raw_traj_points, virtual_ego_pose);

    // road collision avoidance by model predictive trajectory in the obstacle_avoidance_planner
    // package
    const obstacle_avoidance_planner::PlannerData planner_data{
      raw_path.header, smoothed_traj_points, raw_path.left_bound, raw_path.right_bound,
      virtual_ego_pose};
    const auto optimized_traj_points = mpt_optimizer_ptr->optimizeTrajectory(planner_data);

    // connect the previously and currently optimized trajectory points
    for (size_t j = 0; j < whole_optimized_traj_points.size(); ++j) {
      const double dist = tier4_autoware_utils::calcDistance2d(
        whole_optimized_traj_points.at(j), optimized_traj_points.front());
      if (dist < 0.5) {
        const std::vector<TrajectoryPoint> extracted_whole_optimized_traj_points{
          whole_optimized_traj_points.begin(),
          whole_optimized_traj_points.begin() + std::max(j, 1UL) - 1};
        whole_optimized_traj_points = extracted_whole_optimized_traj_points;
        break;
      }
    }
    for (size_t j = 0; j < optimized_traj_points.size(); ++j) {
      whole_optimized_traj_points.push_back(optimized_traj_points.at(j));
    }
  }

  return whole_optimized_traj_points;
}

void StaticCenterlineOptimizerNode::on_plan_path(
  const PlanPath::Request::SharedPtr request, const PlanPath::Response::SharedPtr response)
{
  if (!route_handler_ptr_) {
    response->message = "MapNotFound";
    RCLCPP_ERROR(get_logger(), "Route handler is not ready.");
    return;
  }

  // get lanelets from route lane ids
  const auto route_lane_ids = request->route;
  const auto route_lanelets = get_lanelets_from_ids(*route_handler_ptr_, route_lane_ids);

  // check if input route lanelets are connected to each other.
  const auto unconnected_lane_ids = check_lanelet_connection(*route_handler_ptr_, route_lanelets);
  if (!unconnected_lane_ids.empty()) {
    response->message = "LaneletsNotConnected";
    response->unconnected_lane_ids = unconnected_lane_ids;
    RCLCPP_ERROR(get_logger(), "Lanelets are not connected.");
    return;
  }

  // plan path
  const auto optimized_traj_points = plan_path(route_lane_ids);

  // check calculation result
  if (optimized_traj_points.empty()) {
    response->message = "PathNotFound";
    RCLCPP_ERROR(get_logger(), "Path planning failed.");
    return;
  }

  // publish unsafe_footprints
  evaluate(route_lane_ids, optimized_traj_points);

  // create output data
  auto target_traj_point = optimized_traj_points.cbegin();
  bool is_end_lanelet = false;
  for (const auto & lanelet : route_lanelets) {
    std::vector<geometry_msgs::msg::Point> current_lanelet_points;

    // check if target point is inside the lanelet
    while (
      lanelet::geometry::inside(lanelet, convertToLaneletPoint(target_traj_point->pose.position))) {
      // memorize points inside the lanelet
      current_lanelet_points.push_back(target_traj_point->pose.position);
      target_traj_point++;

      if (target_traj_point == optimized_traj_points.cend()) {
        is_end_lanelet = true;
        break;
      }
    }

    if (!current_lanelet_points.empty()) {
      // register points with lane_id
      static_centerline_optimizer::msg::PointsWithLaneId points_with_lane_id;
      points_with_lane_id.lane_id = lanelet.id();
      points_with_lane_id.points = current_lanelet_points;
      response->points_with_lane_ids.push_back(points_with_lane_id);
    }

    if (is_end_lanelet) {
      break;
    }
  }

  // empty string if error did not occur
  response->message = "";
}

void StaticCenterlineOptimizerNode::evaluate(
  const std::vector<lanelet::Id> & route_lane_ids,
  const std::vector<TrajectoryPoint> & optimized_traj_points)
{
  const auto route_lanelets = get_lanelets_from_ids(*route_handler_ptr_, route_lane_ids);
  const auto dist_thresh_vec =
    has_parameter("marker_color_dist_thresh")
      ? get_parameter("marker_color_dist_thresh").as_double_array()
      : declare_parameter<std::vector<double>>("marker_color_dist_thresh");
  const auto marker_color_vec = has_parameter("marker_color")
                                  ? get_parameter("marker_color").as_string_array()
                                  : declare_parameter<std::vector<std::string>>("marker_color");
  const auto get_marker_color = [&](const double dist) -> boost::optional<std::array<double, 3>> {
    for (size_t i = 0; i < dist_thresh_vec.size(); ++i) {
      const double dist_thresh = dist_thresh_vec.at(i);
      if (dist < dist_thresh) {
        return convertHexStringToDecimal(marker_color_vec.at(i));
      }
    }
    return boost::none;
  };

  // create right/left bound
  LineString2d right_bound;
  LineString2d left_bound;
  for (const auto & lanelet : route_lanelets) {
    for (const auto & point : lanelet.rightBound()) {
      boost::geometry::append(right_bound, Point2d(point.x(), point.y()));
    }
    for (const auto & point : lanelet.leftBound()) {
      boost::geometry::append(left_bound, Point2d(point.x(), point.y()));
    }
  }

  // calculate the distance between footprint and right/left bounds
  MarkerArray marker_array;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < optimized_traj_points.size(); ++i) {
    const auto & traj_point = optimized_traj_points.at(i);

    const auto footprint_poly = create_vehicle_footprint(traj_point.pose, vehicle_info_);

    const double dist_to_right = boost::geometry::distance(footprint_poly, right_bound);
    const double dist_to_left = boost::geometry::distance(footprint_poly, left_bound);
    const double min_dist_to_bound = std::min(dist_to_right, dist_to_left);

    if (min_dist_to_bound < min_dist) {
      min_dist = min_dist_to_bound;
    }

    // create marker
    const auto marker_color_opt = get_marker_color(min_dist_to_bound);
    if (marker_color_opt) {
      const auto & marker_color = marker_color_opt.get();

      // add footprint marker
      const auto footprint_marker =
        utils::create_footprint_marker(footprint_poly, marker_color, now(), i);
      tier4_autoware_utils::appendMarkerArray(footprint_marker, &marker_array);

      // add text of distance to bounds marker
      const auto text_pose = get_text_pose(traj_point.pose, vehicle_info_);
      const auto text_marker =
        utils::create_distance_text_marker(text_pose, min_dist_to_bound, marker_color, now(), i);
      tier4_autoware_utils::appendMarkerArray(text_marker, &marker_array);
    }
  }

  pub_debug_unsafe_footprints_->publish(marker_array);

  RCLCPP_INFO(get_logger(), "Minimum distance to road is %f [m]", min_dist);
}

void StaticCenterlineOptimizerNode::save_map(
  const std::string & lanelet2_output_file_path, const std::vector<lanelet::Id> & route_lane_ids,
  const std::vector<TrajectoryPoint> & optimized_traj_points)
{
  if (!route_handler_ptr_) {
    return;
  }

  const auto route_lanelets = get_lanelets_from_ids(*route_handler_ptr_, route_lane_ids);

  // update centerline in map
  utils::update_centerline(*route_handler_ptr_, route_lanelets, optimized_traj_points);
  RCLCPP_INFO(get_logger(), "Updated centerline in map.");

  // save map with modified center line
  lanelet::write(lanelet2_output_file_path, *original_map_ptr_);
  RCLCPP_INFO(get_logger(), "Saved map.");
}
}  // namespace static_centerline_optimizer
