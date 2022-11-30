// Copyright 2020 Tier IV, Inc.
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

#include "lane_departure_checker/lane_departure_checker_node.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/route_checker.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <autoware_planning_msgs/msg/lanelet_segment.hpp>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using tier4_autoware_utils::rad2deg;

namespace
{
using autoware_planning_msgs::msg::LaneletSegment;

std::array<geometry_msgs::msg::Point, 3> triangle2points(
  const geometry_msgs::msg::Polygon & triangle)
{
  std::array<geometry_msgs::msg::Point, 3> points;
  for (size_t i = 0; i < 3; ++i) {
    const auto & p = triangle.points.at(i);

    geometry_msgs::msg::Point point;
    point.x = static_cast<double>(p.x);
    point.y = static_cast<double>(p.y);
    point.z = static_cast<double>(p.z);
    points.at(i) = point;
  }
  return points;
}

lanelet::ConstLanelets getRouteLanelets(
  const lanelet::LaneletMapPtr & lanelet_map,
  const lanelet::routing::RoutingGraphPtr & routing_graph,
  const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr & route_ptr,
  const double vehicle_length)
{
  lanelet::ConstLanelets route_lanelets;

  bool is_route_valid = lanelet::utils::route::isRouteValid(*route_ptr, lanelet_map);
  if (!is_route_valid) {
    return route_lanelets;
  }

  // Add preceding lanes of front route_section to prevent detection errors
  {
    const auto extension_length = 2 * vehicle_length;

    for (const auto & primitive : route_ptr->segments.front().primitives) {
      const auto lane_id = primitive.id;
      for (const auto & lanelet_sequence : lanelet::utils::query::getPrecedingLaneletSequences(
             routing_graph, lanelet_map->laneletLayer.get(lane_id), extension_length)) {
        for (const auto & preceding_lanelet : lanelet_sequence) {
          route_lanelets.push_back(preceding_lanelet);
        }
      }
    }
  }

  for (const auto & route_section : route_ptr->segments) {
    for (const auto & primitive : route_section.primitives) {
      const auto lane_id = primitive.id;
      route_lanelets.push_back(lanelet_map->laneletLayer.get(lane_id));
    }
  }

  // Add succeeding lanes of last route_section to prevent detection errors
  {
    const auto extension_length = 2 * vehicle_length;

    for (const auto & primitive : route_ptr->segments.back().primitives) {
      const auto lane_id = primitive.id;
      for (const auto & lanelet_sequence : lanelet::utils::query::getSucceedingLaneletSequences(
             routing_graph, lanelet_map->laneletLayer.get(lane_id), extension_length)) {
        for (const auto & succeeding_lanelet : lanelet_sequence) {
          route_lanelets.push_back(succeeding_lanelet);
        }
      }
    }
  }

  return route_lanelets;
}

template <typename T>
void update_param(
  const std::vector<rclcpp::Parameter> & parameters, const std::string & name, T & value)
{
  auto it = std::find_if(
    parameters.cbegin(), parameters.cend(),
    [&name](const rclcpp::Parameter & parameter) { return parameter.get_name() == name; });
  if (it != parameters.cend()) {
    value = it->template get_value<T>();
  }
}

}  // namespace

namespace lane_departure_checker
{
LaneDepartureCheckerNode::LaneDepartureCheckerNode(const rclcpp::NodeOptions & options)
: Node("lane_departure_checker_node", options)
{
  using std::placeholders::_1;

  // Node Parameter
  node_param_.update_rate = declare_parameter("update_rate", 10.0);
  node_param_.visualize_lanelet = declare_parameter("visualize_lanelet", false);

  // Core Parameter

  // Vehicle Info
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  vehicle_length_m_ = vehicle_info.vehicle_length_m;

  param_.footprint_margin_scale = declare_parameter("footprint_margin_scale", 1.0);
  param_.resample_interval = declare_parameter("resample_interval", 0.3);
  param_.max_deceleration = declare_parameter("max_deceleration", 3.0);
  param_.delay_time = declare_parameter("delay_time", 0.3);
  param_.max_lateral_deviation = declare_parameter("max_lateral_deviation", 1.0);
  param_.max_longitudinal_deviation = declare_parameter("max_longitudinal_deviation", 1.0);
  param_.max_yaw_deviation_deg = declare_parameter("max_yaw_deviation_deg", 30.0);
  param_.ego_nearest_dist_threshold = declare_parameter<double>("ego_nearest_dist_threshold");
  param_.ego_nearest_yaw_threshold = declare_parameter<double>("ego_nearest_yaw_threshold");

  // Parameter Callback
  set_param_res_ =
    add_on_set_parameters_callback(std::bind(&LaneDepartureCheckerNode::onParameter, this, _1));

  // Core
  lane_departure_checker_ = std::make_unique<LaneDepartureChecker>();
  lane_departure_checker_->setParam(param_, vehicle_info);

  // Subscriber
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", 1, std::bind(&LaneDepartureCheckerNode::onOdometry, this, _1));
  sub_lanelet_map_bin_ = this->create_subscription<HADMapBin>(
    "~/input/lanelet_map_bin", rclcpp::QoS{1}.transient_local(),
    std::bind(&LaneDepartureCheckerNode::onLaneletMapBin, this, _1));
  sub_route_ = this->create_subscription<LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&LaneDepartureCheckerNode::onRoute, this, _1));
  sub_reference_trajectory_ = this->create_subscription<Trajectory>(
    "~/input/reference_trajectory", 1,
    std::bind(&LaneDepartureCheckerNode::onReferenceTrajectory, this, _1));
  sub_predicted_trajectory_ = this->create_subscription<Trajectory>(
    "~/input/predicted_trajectory", 1,
    std::bind(&LaneDepartureCheckerNode::onPredictedTrajectory, this, _1));

  // Publisher
  // Nothing

  // Diagnostic Updater
  updater_.setHardwareID("lane_departure_checker");

  updater_.add("lane_departure", this, &LaneDepartureCheckerNode::checkLaneDeparture);

  updater_.add("trajectory_deviation", this, &LaneDepartureCheckerNode::checkTrajectoryDeviation);

  // Wait for first self pose
  self_pose_listener_.waitForFirstPose();

  // Timer
  const auto period_ns = rclcpp::Rate(node_param_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&LaneDepartureCheckerNode::onTimer, this));
}

void LaneDepartureCheckerNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  current_odom_ = msg;
}

void LaneDepartureCheckerNode::onLaneletMapBin(const HADMapBin::ConstSharedPtr msg)
{
  lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_, &traffic_rules_, &routing_graph_);
}

void LaneDepartureCheckerNode::onRoute(const LaneletRoute::ConstSharedPtr msg) { route_ = msg; }

void LaneDepartureCheckerNode::onReferenceTrajectory(const Trajectory::ConstSharedPtr msg)
{
  reference_trajectory_ = msg;
}

void LaneDepartureCheckerNode::onPredictedTrajectory(const Trajectory::ConstSharedPtr msg)
{
  predicted_trajectory_ = msg;
}

bool LaneDepartureCheckerNode::isDataReady()
{
  if (!current_pose_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for current_pose...");
    return false;
  }

  if (!current_odom_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for current_twist msg...");
    return false;
  }

  if (!lanelet_map_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for lanelet_map msg...");
    return false;
  }

  if (!route_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for route msg...");
    return false;
  }

  if (!reference_trajectory_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for reference_trajectory msg...");
    return false;
  }

  if (!predicted_trajectory_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for predicted_trajectory msg...");
    return false;
  }

  return true;
}

bool LaneDepartureCheckerNode::isDataTimeout()
{
  const auto now = this->now();

  constexpr double th_pose_timeout = 1.0;
  const auto pose_time_diff = rclcpp::Time(current_pose_->header.stamp) - now;
  if (pose_time_diff.seconds() > th_pose_timeout) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "pose is timeout...");
    return true;
  }

  return false;
}

bool LaneDepartureCheckerNode::isDataValid()
{
  if (reference_trajectory_->points.empty()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "reference_trajectory is empty. Not expected!");
    return false;
  }

  if (predicted_trajectory_->points.empty()) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "predicted_trajectory is empty. Not expected!");
    return false;
  }

  return true;
}

void LaneDepartureCheckerNode::onTimer()
{
  std::map<std::string, double> processing_time_map;
  tier4_autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic("Total");

  current_pose_ = self_pose_listener_.getCurrentPose();

  if (!isDataReady()) {
    return;
  }

  if (isDataTimeout()) {
    return;
  }

  if (!isDataValid()) {
    return;
  }

  processing_time_map["Node: checkData"] = stop_watch.toc(true);

  // In order to wait for both of map and route will be ready, write this not in callback but here
  if (last_route_ != route_ && !route_->segments.empty()) {
    route_lanelets_ = getRouteLanelets(lanelet_map_, routing_graph_, route_, vehicle_length_m_);
    last_route_ = route_;
  }
  processing_time_map["Node: getRouteLanelets"] = stop_watch.toc(true);

  input_.current_odom = current_odom_;
  input_.current_pose = current_pose_;
  input_.lanelet_map = lanelet_map_;
  input_.route = route_;
  input_.route_lanelets = route_lanelets_;
  input_.reference_trajectory = reference_trajectory_;
  input_.predicted_trajectory = predicted_trajectory_;
  processing_time_map["Node: setInputData"] = stop_watch.toc(true);

  output_ = lane_departure_checker_->update(input_);
  processing_time_map["Node: update"] = stop_watch.toc(true);

  updater_.force_update();
  processing_time_map["Node: updateDiagnostics"] = stop_watch.toc(true);

  {
    const auto & deviation = output_.trajectory_deviation;
    debug_publisher_.publish<tier4_debug_msgs::msg::Float64Stamped>(
      "deviation/lateral", deviation.lateral);
    debug_publisher_.publish<tier4_debug_msgs::msg::Float64Stamped>("deviation/yaw", deviation.yaw);
    debug_publisher_.publish<tier4_debug_msgs::msg::Float64Stamped>(
      "deviation/yaw_deg", rad2deg(deviation.yaw));
  }
  processing_time_map["Node: publishTrajectoryDeviation"] = stop_watch.toc(true);

  debug_publisher_.publish<visualization_msgs::msg::MarkerArray>(
    std::string("marker_array"), createMarkerArray());
  processing_time_map["Node: publishDebugMarker"] = stop_watch.toc(true);

  // Merge processing_time_map
  for (const auto & m : output_.processing_time_map) {
    processing_time_map["Core: " + m.first] = m.second;
  }

  processing_time_map["Total"] = stop_watch.toc("Total");
  processing_time_publisher_.publish(processing_time_map);
}

rcl_interfaces::msg::SetParametersResult LaneDepartureCheckerNode::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    // Node
    update_param(parameters, "visualize_lanelet", node_param_.visualize_lanelet);

    // Core
    update_param(parameters, "footprint_margin_scale", param_.footprint_margin_scale);
    update_param(parameters, "resample_interval", param_.resample_interval);
    update_param(parameters, "max_deceleration", param_.max_deceleration);
    update_param(parameters, "delay_time", param_.delay_time);

    if (lane_departure_checker_) {
      lane_departure_checker_->setParam(param_);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

void LaneDepartureCheckerNode::checkLaneDeparture(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
  int8_t level = DiagStatus::OK;
  std::string msg = "OK";

  if (output_.will_leave_lane) {
    level = DiagStatus::WARN;
    msg = "vehicle will leave lane";
  }

  if (output_.is_out_of_lane) {
    level = DiagStatus::ERROR;
    msg = "vehicle is out of lane";
  }

  stat.summary(level, msg);
}

void LaneDepartureCheckerNode::checkTrajectoryDeviation(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
  int8_t level = DiagStatus::OK;

  if (std::abs(output_.trajectory_deviation.lateral) >= param_.max_lateral_deviation) {
    level = DiagStatus::ERROR;
  }

  if (std::abs(output_.trajectory_deviation.longitudinal) >= param_.max_longitudinal_deviation) {
    level = DiagStatus::ERROR;
  }

  if (std::abs(rad2deg(output_.trajectory_deviation.yaw)) >= param_.max_yaw_deviation_deg) {
    level = DiagStatus::ERROR;
  }

  std::string msg = "OK";
  if (level == DiagStatus::ERROR) {
    msg = "trajectory deviation is too large";
  }

  stat.addf("max lateral deviation", "%.3f", param_.max_lateral_deviation);
  stat.addf("lateral deviation", "%.3f", output_.trajectory_deviation.lateral);

  stat.addf("max longitudinal deviation", "%.3f", param_.max_longitudinal_deviation);
  stat.addf("longitudinal deviation", "%.3f", output_.trajectory_deviation.longitudinal);

  stat.addf("max yaw deviation", "%.3f", param_.max_yaw_deviation_deg);
  stat.addf("yaw deviation", "%.3f", rad2deg(output_.trajectory_deviation.yaw));

  stat.summary(level, msg);
}

visualization_msgs::msg::MarkerArray LaneDepartureCheckerNode::createMarkerArray() const
{
  using tier4_autoware_utils::createDefaultMarker;
  using tier4_autoware_utils::createMarkerColor;
  using tier4_autoware_utils::createMarkerScale;

  visualization_msgs::msg::MarkerArray marker_array;

  const auto base_link_z = current_pose_->pose.position.z;

  if (node_param_.visualize_lanelet) {
    // Route Lanelets
    {
      auto marker = createDefaultMarker(
        "map", this->now(), "route_lanelets", 0, visualization_msgs::msg::Marker::TRIANGLE_LIST,
        createMarkerScale(1.0, 1.0, 1.0), createMarkerColor(0.0, 0.5, 0.5, 0.5));

      for (const auto & lanelet : input_.route_lanelets) {
        std::vector<geometry_msgs::msg::Polygon> triangles;
        lanelet::visualization::lanelet2Triangle(lanelet, &triangles);

        for (const auto & triangle : triangles) {
          for (const auto & point : triangle2points(triangle)) {
            marker.points.push_back(point);
            marker.colors.push_back(marker.color);
          }
        }
      }

      marker_array.markers.push_back(marker);
    }

    // Candidate Lanelets
    {
      auto marker = createDefaultMarker(
        "map", this->now(), "candidate_lanelets", 0, visualization_msgs::msg::Marker::TRIANGLE_LIST,
        createMarkerScale(1.0, 1.0, 1.0), createMarkerColor(1.0, 1.0, 0.0, 0.5));

      for (const auto & lanelet : output_.candidate_lanelets) {
        std::vector<geometry_msgs::msg::Polygon> triangles;
        lanelet::visualization::lanelet2Triangle(lanelet, &triangles);

        for (const auto & triangle : triangles) {
          for (const auto & point : triangle2points(triangle)) {
            marker.points.push_back(point);
            marker.colors.push_back(marker.color);
          }
        }
      }

      marker_array.markers.push_back(marker);
    }
  }

  if (output_.resampled_trajectory.size() >= 2) {
    // Line of resampled_trajectory
    {
      auto marker = createDefaultMarker(
        "map", this->now(), "resampled_trajectory_line", 0,
        visualization_msgs::msg::Marker::LINE_STRIP, createMarkerScale(0.05, 0, 0),
        createMarkerColor(1.0, 1.0, 1.0, 0.999));

      for (const auto & p : output_.resampled_trajectory) {
        marker.points.push_back(p.pose.position);
        marker.colors.push_back(marker.color);
      }

      marker_array.markers.push_back(marker);
    }

    // Points of resampled_trajectory
    {
      auto marker = createDefaultMarker(
        "map", this->now(), "resampled_trajectory_points", 0,
        visualization_msgs::msg::Marker::SPHERE_LIST, createMarkerScale(0.1, 0.1, 0.1),
        createMarkerColor(0.0, 1.0, 0.0, 0.999));

      for (const auto & p : output_.resampled_trajectory) {
        marker.points.push_back(p.pose.position);
        marker.colors.push_back(marker.color);
      }

      marker_array.markers.push_back(marker);
    }
  }

  // Vehicle Footprints
  {
    const auto color_ok = createMarkerColor(0.0, 1.0, 0.0, 0.5);
    const auto color_will_leave_lane = createMarkerColor(0.5, 0.5, 0.0, 0.5);
    const auto color_is_out_of_lane = createMarkerColor(1.0, 0.0, 0.0, 0.5);

    auto color = color_ok;
    if (output_.will_leave_lane) {
      color = color_will_leave_lane;
    }
    if (output_.is_out_of_lane) {
      color = color_is_out_of_lane;
    }

    auto marker = createDefaultMarker(
      "map", this->now(), "vehicle_footprints", 0, visualization_msgs::msg::Marker::LINE_LIST,
      createMarkerScale(0.05, 0, 0), color);

    for (const auto & vehicle_footprint : output_.vehicle_footprints) {
      for (size_t i = 0; i < vehicle_footprint.size() - 1; ++i) {
        const auto p1 = vehicle_footprint.at(i);
        const auto p2 = vehicle_footprint.at(i + 1);

        marker.points.push_back(toMsg(p1.to_3d(base_link_z)));
        marker.points.push_back(toMsg(p2.to_3d(base_link_z)));
      }
    }

    marker_array.markers.push_back(marker);
  }

  return marker_array;
}
}  // namespace lane_departure_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lane_departure_checker::LaneDepartureCheckerNode)
