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

#include "static_centerline_generator/static_centerline_generator_node.hpp"

#include "lanelet2_extension/utility/message_conversion.hpp"
#include "lanelet2_extension/utility/query.hpp"
#include "lanelet2_extension/utility/utilities.hpp"
#include "map_loader/lanelet2_map_loader_node.hpp"
#include "map_projection_loader/load_info_from_lanelet2_map.hpp"
#include "motion_utils/resample/resample.hpp"
#include "motion_utils/trajectory/conversion.hpp"
#include "static_centerline_generator/centerline_source/bag_ego_trajectory_based_centerline.hpp"
#include "static_centerline_generator/msg/points_with_lane_id.hpp"
#include "static_centerline_generator/type_alias.hpp"
#include "static_centerline_generator/utils.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/ros/parameter.hpp"

#include <geography_utils/lanelet2_projector.hpp>
#include <mission_planner/mission_planner_plugin.hpp>
#include <pluginlib/class_loader.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include <tier4_map_msgs/msg/map_projector_info.hpp>

#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/distance.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace static_centerline_generator
{
namespace
{
std::vector<lanelet::Id> get_lane_ids_from_route(const LaneletRoute & route)
{
  std::vector<lanelet::Id> lane_ids;
  for (const auto & segment : route.segments) {
    const auto & target_lanelet_id = segment.preferred_primitive.id;
    lane_ids.push_back(target_lanelet_id);
  }

  return lane_ids;
}

lanelet::BasicPoint2d convert_to_lanelet_point(const geometry_msgs::msg::Point & geom_point)
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

std::array<double, 3> convert_hex_string_to_decimal(const std::string & hex_str_color)
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

std_msgs::msg::Header create_header(const rclcpp::Time & now)
{
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = now;
  return header;
}

std::vector<TrajectoryPoint> resample_trajectory_points(
  const std::vector<TrajectoryPoint> & input_traj_points, const double resample_interval)
{
  // resample and calculate trajectory points' orientation
  const auto input_traj = motion_utils::convertToTrajectory(input_traj_points);
  auto resampled_input_traj = motion_utils::resampleTrajectory(input_traj, resample_interval);
  return motion_utils::convertToTrajectoryPointArray(resampled_input_traj);
}
}  // namespace

StaticCenterlineGeneratorNode::StaticCenterlineGeneratorNode(
  const rclcpp::NodeOptions & node_options)
: Node("static_centerline_generator", node_options)
{
  // publishers
  pub_map_bin_ =
    create_publisher<HADMapBin>("lanelet2_map_topic", utils::create_transient_local_qos());
  pub_whole_centerline_ =
    create_publisher<Trajectory>("output_whole_centerline", utils::create_transient_local_qos());
  pub_centerline_ =
    create_publisher<Trajectory>("output_centerline", utils::create_transient_local_qos());

  // debug publishers
  pub_debug_unsafe_footprints_ =
    create_publisher<MarkerArray>("debug/unsafe_footprints", utils::create_transient_local_qos());

  // subscribers
  sub_traj_start_index_ = create_subscription<std_msgs::msg::Int32>(
    "/centerline_updater_helper/traj_start_index", rclcpp::QoS{1},
    [this](const std_msgs::msg::Int32 & msg) {
      update_centerline_range(msg.data, traj_range_indices_.second);
    });
  sub_traj_end_index_ = create_subscription<std_msgs::msg::Int32>(
    "/centerline_updater_helper/traj_end_index", rclcpp::QoS{1},
    [this](const std_msgs::msg::Int32 & msg) {
      update_centerline_range(traj_range_indices_.first, msg.data);
    });
  sub_save_map_ = create_subscription<std_msgs::msg::Bool>(
    "/centerline_updater_helper/save_map", rclcpp::QoS{1}, [this](const std_msgs::msg::Bool & msg) {
      const auto lanelet2_output_file_path =
        tier4_autoware_utils::getOrDeclareParameter<std::string>(
          *this, "lanelet2_output_file_path");
      if (!centerline_with_route_ || msg.data) {
        const auto & c = *centerline_with_route_;
        const auto selected_centerline = std::vector<TrajectoryPoint>(
          c.centerline.begin() + traj_range_indices_.first,
          c.centerline.begin() + traj_range_indices_.second + 1);
        save_map(
          lanelet2_output_file_path, CenterlineWithRoute{selected_centerline, c.route_lane_ids});
      }
    });
  sub_traj_resample_interval_ = create_subscription<std_msgs::msg::Float32>(
    "/centerline_updater_helper/traj_resample_interval", rclcpp::QoS{1},
    [this]([[maybe_unused]] const std_msgs::msg::Float32 & msg) {
      // TODO(murooka)
    });

  // services
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_load_map_ = create_service<LoadMap>(
    "/planning/static_centerline_generator/load_map",
    std::bind(
      &StaticCenterlineGeneratorNode::on_load_map, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_);
  srv_plan_route_ = create_service<PlanRoute>(
    "/planning/static_centerline_generator/plan_route",
    std::bind(
      &StaticCenterlineGeneratorNode::on_plan_route, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_);
  srv_plan_path_ = create_service<PlanPath>(
    "/planning/static_centerline_generator/plan_path",
    std::bind(
      &StaticCenterlineGeneratorNode::on_plan_path, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, callback_group_);

  // vehicle info
  vehicle_info_ = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();

  centerline_source_ = [&]() {
    const auto centerline_source_param = declare_parameter<std::string>("centerline_source");
    if (centerline_source_param == "optimization_trajectory_base") {
      optimization_trajectory_based_centerline_ = OptimizationTrajectoryBasedCenterline(*this);
      return CenterlineSource::OptimizationTrajectoryBase;
    } else if (centerline_source_param == "bag_ego_trajectory_base") {
      return CenterlineSource::BagEgoTrajectoryBase;
    }
    throw std::logic_error(
      "The centerline source is not supported in static_centerline_generator.");
  }();
}

void StaticCenterlineGeneratorNode::update_centerline_range(
  const int traj_start_index, const int traj_end_index)
{
  if (!centerline_with_route_ || traj_range_indices_.second + 1 < traj_start_index) {
    return;
  }

  traj_range_indices_ = std::make_pair(traj_start_index, traj_end_index);

  const auto & centerline = centerline_with_route_->centerline;
  const auto selected_centerline = std::vector<TrajectoryPoint>(
    centerline.begin() + traj_range_indices_.first,
    centerline.begin() + traj_range_indices_.second + 1);

  pub_centerline_->publish(
    motion_utils::convertToTrajectory(selected_centerline, create_header(this->now())));
}

void StaticCenterlineGeneratorNode::run()
{
  // declare planning setting parameters
  const auto lanelet2_input_file_path = declare_parameter<std::string>("lanelet2_input_file_path");
  const auto lanelet2_output_file_path =
    declare_parameter<std::string>("lanelet2_output_file_path");

  // process
  load_map(lanelet2_input_file_path);
  const auto centerline_with_route = generate_centerline_with_route();
  traj_range_indices_ = std::make_pair(0, centerline_with_route.centerline.size() - 1);
  save_map(lanelet2_output_file_path, centerline_with_route);

  centerline_with_route_ = centerline_with_route;
}

CenterlineWithRoute StaticCenterlineGeneratorNode::generate_centerline_with_route()
{
  if (!route_handler_ptr_) {
    RCLCPP_ERROR(get_logger(), "Route handler is not ready. Return empty trajectory.");
    return CenterlineWithRoute{};
  }

  // generate centerline with route
  auto centerline_with_route = [&]() {
    if (centerline_source_ == CenterlineSource::OptimizationTrajectoryBase) {
      const lanelet::Id start_lanelet_id = declare_parameter<int64_t>("start_lanelet_id");
      const lanelet::Id end_lanelet_id = declare_parameter<int64_t>("end_lanelet_id");
      const auto route_lane_ids = plan_route(start_lanelet_id, end_lanelet_id);
      const auto optimized_centerline =
        optimization_trajectory_based_centerline_.generate_centerline_with_optimization(
          *this, *route_handler_ptr_, route_lane_ids);
      return CenterlineWithRoute{optimized_centerline, route_lane_ids};
    } else if (centerline_source_ == CenterlineSource::BagEgoTrajectoryBase) {
      const auto bag_centerline = generate_centerline_with_bag(*this);
      const auto start_lanelets =
        route_handler_ptr_->getClosestLanelets(bag_centerline.front().pose);
      const auto end_lanelets = route_handler_ptr_->getClosestLanelets(bag_centerline.back().pose);
      if (start_lanelets.empty() || end_lanelets.empty()) {
        RCLCPP_ERROR(get_logger(), "Nearest lanelets to the bag's centerline are not found.");
        return CenterlineWithRoute{};
      }

      const lanelet::Id start_lanelet_id = start_lanelets.front().id();
      const lanelet::Id end_lanelet_id = end_lanelets.front().id();
      const auto route_lane_ids = plan_route(start_lanelet_id, end_lanelet_id);

      return CenterlineWithRoute{bag_centerline, route_lane_ids};
    }
    throw std::logic_error(
      "The centerline source is not supported in static_centerline_generator.");
  }();

  // resample
  const double output_trajectory_interval = declare_parameter<double>("output_trajectory_interval");
  centerline_with_route.centerline =
    resample_trajectory_points(centerline_with_route.centerline, output_trajectory_interval);

  pub_whole_centerline_->publish(motion_utils::convertToTrajectory(
    centerline_with_route.centerline, create_header(this->now())));

  pub_centerline_->publish(motion_utils::convertToTrajectory(
    centerline_with_route.centerline, create_header(this->now())));

  return centerline_with_route;
}

void StaticCenterlineGeneratorNode::load_map(const std::string & lanelet2_input_file_path)
{
  // copy the input LL2 map to the temporary file for debugging
  const std::string debug_input_file_dir{"/tmp/static_centerline_generator/input/"};
  std::filesystem::create_directories(debug_input_file_dir);
  std::filesystem::copy(
    lanelet2_input_file_path, debug_input_file_dir + "lanelet2_map.osm",
    std::filesystem::copy_options::overwrite_existing);

  // load map by the map_loader package
  map_bin_ptr_ = [&]() -> HADMapBin::ConstSharedPtr {
    // load map
    const auto map_projector_info = load_info_from_lanelet2_map(lanelet2_input_file_path);
    const auto map_ptr =
      Lanelet2MapLoaderNode::load_map(lanelet2_input_file_path, map_projector_info);
    if (!map_ptr) {
      return nullptr;
    }

    // NOTE: generate map projector for lanelet::write().
    //       Without this, lat/lon of the generated LL2 map will be wrong.
    map_projector_ = geography_utils::get_lanelet2_projector(map_projector_info);

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

void StaticCenterlineGeneratorNode::on_load_map(
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

std::vector<lanelet::Id> StaticCenterlineGeneratorNode::plan_route(
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
    auto node = rclcpp::Node("mission_planner");
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

void StaticCenterlineGeneratorNode::on_plan_route(
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
  const auto route_lanelets = utils::get_lanelets_from_ids(*route_handler_ptr_, route_lane_ids);

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

void StaticCenterlineGeneratorNode::on_plan_path(
  const PlanPath::Request::SharedPtr request, const PlanPath::Response::SharedPtr response)
{
  if (!route_handler_ptr_) {
    response->message = "MapNotFound";
    RCLCPP_ERROR(get_logger(), "Route handler is not ready.");
    return;
  }

  // get lanelets from route lane ids
  const auto route_lane_ids = request->route;
  const auto route_lanelets = utils::get_lanelets_from_ids(*route_handler_ptr_, route_lane_ids);

  // check if input route lanelets are connected to each other.
  const auto unconnected_lane_ids = check_lanelet_connection(*route_handler_ptr_, route_lanelets);
  if (!unconnected_lane_ids.empty()) {
    response->message = "LaneletsNotConnected";
    response->unconnected_lane_ids = unconnected_lane_ids;
    RCLCPP_ERROR(get_logger(), "Lanelets are not connected.");
    return;
  }

  // plan path
  const auto optimized_traj_points =
    optimization_trajectory_based_centerline_.generate_centerline_with_optimization(
      *this, *route_handler_ptr_, route_lane_ids);

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
    while (lanelet::geometry::inside(
      lanelet, convert_to_lanelet_point(target_traj_point->pose.position))) {
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
      static_centerline_generator::msg::PointsWithLaneId points_with_lane_id;
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

void StaticCenterlineGeneratorNode::evaluate(
  const std::vector<lanelet::Id> & route_lane_ids,
  const std::vector<TrajectoryPoint> & optimized_traj_points)
{
  const auto route_lanelets = utils::get_lanelets_from_ids(*route_handler_ptr_, route_lane_ids);
  const auto dist_thresh_vec = tier4_autoware_utils::getOrDeclareParameter<std::vector<double>>(
    *this, "marker_color_dist_thresh");
  const auto marker_color_vec =
    tier4_autoware_utils::getOrDeclareParameter<std::vector<std::string>>(*this, "marker_color");
  const auto get_marker_color = [&](const double dist) -> boost::optional<std::array<double, 3>> {
    for (size_t i = 0; i < dist_thresh_vec.size(); ++i) {
      const double dist_thresh = dist_thresh_vec.at(i);
      if (dist < dist_thresh) {
        return convert_hex_string_to_decimal(marker_color_vec.at(i));
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

void StaticCenterlineGeneratorNode::save_map(
  const std::string & lanelet2_output_file_path, const CenterlineWithRoute & centerline_with_route)
{
  if (!route_handler_ptr_) {
    return;
  }

  const auto & c = centerline_with_route;
  const auto route_lanelets = utils::get_lanelets_from_ids(*route_handler_ptr_, c.route_lane_ids);

  // update centerline in map
  utils::update_centerline(*route_handler_ptr_, route_lanelets, c.centerline);
  RCLCPP_INFO(get_logger(), "Updated centerline in map.");

  // save map with modified center line
  lanelet::write(lanelet2_output_file_path, *original_map_ptr_, *map_projector_);
  RCLCPP_INFO(get_logger(), "Saved map.");

  // copy the output LL2 map to the temporary file for debugging
  const std::string debug_output_file_dir{"/tmp/static_centerline_generator/output/"};
  std::filesystem::create_directories(debug_output_file_dir);
  std::filesystem::copy(
    lanelet2_output_file_path, debug_output_file_dir + "lanelet2_map.osm",
    std::filesystem::copy_options::overwrite_existing);
}
}  // namespace static_centerline_generator
