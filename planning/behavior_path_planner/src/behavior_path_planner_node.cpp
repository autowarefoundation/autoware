// Copyright 2021-2023 Tier IV, Inc.
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

#include "behavior_path_planner/behavior_path_planner_node.hpp"

#include "behavior_path_planner_common/marker_utils/utils.hpp"
#include "behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "behavior_path_planner_common/utils/path_utils.hpp"

#include <tier4_autoware_utils/ros/update_param.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <tier4_planning_msgs/msg/path_change_module_id.hpp>

#include <memory>
#include <string>
#include <vector>

namespace
{
rclcpp::SubscriptionOptions createSubscriptionOptions(rclcpp::Node * node_ptr)
{
  rclcpp::CallbackGroup::SharedPtr callback_group =
    node_ptr->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group;

  return sub_opt;
}
}  // namespace

namespace behavior_path_planner
{
using tier4_planning_msgs::msg::PathChangeModuleId;
using vehicle_info_util::VehicleInfoUtil;

BehaviorPathPlannerNode::BehaviorPathPlannerNode(const rclcpp::NodeOptions & node_options)
: Node("behavior_path_planner", node_options)
{
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  // data_manager
  {
    planner_data_ = std::make_shared<PlannerData>();
    planner_data_->parameters = getCommonParam();
    planner_data_->drivable_area_expansion_parameters.init(*this);
  }

  // publisher
  path_publisher_ = create_publisher<PathWithLaneId>("~/output/path", 1);
  turn_signal_publisher_ =
    create_publisher<TurnIndicatorsCommand>("~/output/turn_indicators_cmd", 1);
  hazard_signal_publisher_ = create_publisher<HazardLightsCommand>("~/output/hazard_lights_cmd", 1);
  const auto durable_qos = rclcpp::QoS(1).transient_local();
  modified_goal_publisher_ =
    create_publisher<PoseWithUuidStamped>("~/output/modified_goal", durable_qos);
  stop_reason_publisher_ = create_publisher<StopReasonArray>("~/output/stop_reasons", 1);
  reroute_availability_publisher_ =
    create_publisher<RerouteAvailability>("~/output/is_reroute_available", 1);
  debug_avoidance_msg_array_publisher_ =
    create_publisher<AvoidanceDebugMsgArray>("~/debug/avoidance_debug_message_array", 1);

  debug_turn_signal_info_publisher_ = create_publisher<MarkerArray>("~/debug/turn_signal_info", 1);

  bound_publisher_ = create_publisher<MarkerArray>("~/debug/bound", 1);

  const auto qos_transient_local = rclcpp::QoS{1}.transient_local();
  // subscriber
  velocity_subscriber_ = create_subscription<Odometry>(
    "~/input/odometry", 1, std::bind(&BehaviorPathPlannerNode::onOdometry, this, _1),
    createSubscriptionOptions(this));
  acceleration_subscriber_ = create_subscription<AccelWithCovarianceStamped>(
    "~/input/accel", 1, std::bind(&BehaviorPathPlannerNode::onAcceleration, this, _1),
    createSubscriptionOptions(this));
  perception_subscriber_ = create_subscription<PredictedObjects>(
    "~/input/perception", 1, std::bind(&BehaviorPathPlannerNode::onPerception, this, _1),
    createSubscriptionOptions(this));
  occupancy_grid_subscriber_ = create_subscription<OccupancyGrid>(
    "~/input/occupancy_grid_map", 1, std::bind(&BehaviorPathPlannerNode::onOccupancyGrid, this, _1),
    createSubscriptionOptions(this));
  costmap_subscriber_ = create_subscription<OccupancyGrid>(
    "~/input/costmap", 1, std::bind(&BehaviorPathPlannerNode::onCostMap, this, _1),
    createSubscriptionOptions(this));
  traffic_signals_subscriber_ =
    this->create_subscription<autoware_perception_msgs::msg::TrafficSignalArray>(
      "~/input/traffic_signals", 1, std::bind(&BehaviorPathPlannerNode::onTrafficSignals, this, _1),
      createSubscriptionOptions(this));
  lateral_offset_subscriber_ = this->create_subscription<LateralOffset>(
    "~/input/lateral_offset", 1, std::bind(&BehaviorPathPlannerNode::onLateralOffset, this, _1),
    createSubscriptionOptions(this));
  operation_mode_subscriber_ = create_subscription<OperationModeState>(
    "/system/operation_mode/state", qos_transient_local,
    std::bind(&BehaviorPathPlannerNode::onOperationMode, this, _1),
    createSubscriptionOptions(this));
  scenario_subscriber_ = create_subscription<Scenario>(
    "~/input/scenario", 1,
    [this](const Scenario::ConstSharedPtr msg) {
      current_scenario_ = std::make_shared<Scenario>(*msg);
    },
    createSubscriptionOptions(this));

  // route_handler
  vector_map_subscriber_ = create_subscription<HADMapBin>(
    "~/input/vector_map", qos_transient_local, std::bind(&BehaviorPathPlannerNode::onMap, this, _1),
    createSubscriptionOptions(this));
  route_subscriber_ = create_subscription<LaneletRoute>(
    "~/input/route", qos_transient_local, std::bind(&BehaviorPathPlannerNode::onRoute, this, _1),
    createSubscriptionOptions(this));

  {
    const std::string path_candidate_name_space = "/planning/path_candidate/";
    const std::string path_reference_name_space = "/planning/path_reference/";

    const std::lock_guard<std::mutex> lock(mutex_manager_);  // for planner_manager_

    const auto & p = planner_data_->parameters;
    planner_manager_ = std::make_shared<PlannerManager>(*this, p.max_iteration_num, p.verbose);

    for (const auto & name : declare_parameter<std::vector<std::string>>("launch_modules")) {
      // workaround: Since ROS 2 can't get empty list, launcher set [''] on the parameter.
      if (name == "") {
        break;
      }
      planner_manager_->launchScenePlugin(*this, name);
    }

    for (const auto & manager : planner_manager_->getSceneModuleManagers()) {
      path_candidate_publishers_.emplace(
        manager->name(), create_publisher<Path>(path_candidate_name_space + manager->name(), 1));
      path_reference_publishers_.emplace(
        manager->name(), create_publisher<Path>(path_reference_name_space + manager->name(), 1));
    }
  }

  m_set_param_res = this->add_on_set_parameters_callback(
    std::bind(&BehaviorPathPlannerNode::onSetParam, this, std::placeholders::_1));

  // turn signal decider
  {
    const double turn_signal_intersection_search_distance =
      planner_data_->parameters.turn_signal_intersection_search_distance;
    const double turn_signal_intersection_angle_threshold_deg =
      planner_data_->parameters.turn_signal_intersection_angle_threshold_deg;
    const double turn_signal_search_time = planner_data_->parameters.turn_signal_search_time;
    planner_data_->turn_signal_decider.setParameters(
      planner_data_->parameters.base_link2front, turn_signal_intersection_search_distance,
      turn_signal_search_time, turn_signal_intersection_angle_threshold_deg);
  }

  steering_factor_interface_ptr_ = std::make_unique<SteeringFactorInterface>(this, "intersection");

  // Start timer
  {
    const auto planning_hz = declare_parameter<double>("planning_hz");
    const auto period_ns = rclcpp::Rate(planning_hz).period();
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&BehaviorPathPlannerNode::run, this));
  }

  logger_configure_ = std::make_unique<tier4_autoware_utils::LoggerLevelConfigure>(this);
}

std::vector<std::string> BehaviorPathPlannerNode::getWaitingApprovalModules()
{
  auto all_scene_module_ptr = planner_manager_->getSceneModuleStatus();
  std::vector<std::string> waiting_approval_modules;
  for (const auto & module : all_scene_module_ptr) {
    if (module->is_waiting_approval == true) {
      waiting_approval_modules.push_back(module->module_name);
    }
  }
  return waiting_approval_modules;
}

std::vector<std::string> BehaviorPathPlannerNode::getRunningModules()
{
  auto all_scene_module_ptr = planner_manager_->getSceneModuleStatus();
  std::vector<std::string> running_modules;
  for (const auto & module : all_scene_module_ptr) {
    if (module->status == ModuleStatus::RUNNING) {
      running_modules.push_back(module->module_name);
    }
  }
  return running_modules;
}

BehaviorPathPlannerParameters BehaviorPathPlannerNode::getCommonParam()
{
  BehaviorPathPlannerParameters p{};

  p.verbose = declare_parameter<bool>("verbose");
  p.max_iteration_num = declare_parameter<int>("max_iteration_num");
  p.traffic_light_signal_timeout = declare_parameter<double>("traffic_light_signal_timeout");

  // vehicle info
  const auto vehicle_info = VehicleInfoUtil(*this).getVehicleInfo();
  p.vehicle_info = vehicle_info;
  p.vehicle_width = vehicle_info.vehicle_width_m;
  p.vehicle_length = vehicle_info.vehicle_length_m;
  p.wheel_tread = vehicle_info.wheel_tread_m;
  p.wheel_base = vehicle_info.wheel_base_m;
  p.front_overhang = vehicle_info.front_overhang_m;
  p.rear_overhang = vehicle_info.rear_overhang_m;
  p.left_over_hang = vehicle_info.left_overhang_m;
  p.right_over_hang = vehicle_info.right_overhang_m;
  p.base_link2front = vehicle_info.max_longitudinal_offset_m;
  p.base_link2rear = p.rear_overhang;

  // NOTE: backward_path_length is used not only calculating path length but also calculating the
  // size of a drivable area.
  //       The drivable area has to cover not the base link but the vehicle itself. Therefore
  //       rear_overhang must be added to backward_path_length. In addition, because of the
  //       calculation of the drivable area in the obstacle_avoidance_planner package, the drivable
  //       area has to be a little longer than the backward_path_length parameter by adding
  //       min_backward_offset.
  constexpr double min_backward_offset = 1.0;
  const double backward_offset = vehicle_info.rear_overhang_m + min_backward_offset;

  // ROS parameters
  p.backward_path_length = declare_parameter<double>("backward_path_length") + backward_offset;
  p.forward_path_length = declare_parameter<double>("forward_path_length");

  // acceleration parameters
  p.min_acc = declare_parameter<double>("normal.min_acc");
  p.max_acc = declare_parameter<double>("normal.max_acc");

  p.backward_length_buffer_for_end_of_pull_over =
    declare_parameter<double>("backward_length_buffer_for_end_of_pull_over");
  p.backward_length_buffer_for_end_of_pull_out =
    declare_parameter<double>("backward_length_buffer_for_end_of_pull_out");

  p.minimum_pull_over_length = declare_parameter<double>("minimum_pull_over_length");
  p.refine_goal_search_radius_range = declare_parameter<double>("refine_goal_search_radius_range");
  p.turn_signal_intersection_search_distance =
    declare_parameter<double>("turn_signal_intersection_search_distance");
  p.turn_signal_intersection_angle_threshold_deg =
    declare_parameter<double>("turn_signal_intersection_angle_threshold_deg");
  p.turn_signal_minimum_search_distance =
    declare_parameter<double>("turn_signal_minimum_search_distance");
  p.turn_signal_search_time = declare_parameter<double>("turn_signal_search_time");
  p.turn_signal_shift_length_threshold =
    declare_parameter<double>("turn_signal_shift_length_threshold");
  p.turn_signal_on_swerving = declare_parameter<bool>("turn_signal_on_swerving");

  p.enable_akima_spline_first = declare_parameter<bool>("enable_akima_spline_first");
  p.enable_cog_on_centerline = declare_parameter<bool>("enable_cog_on_centerline");
  p.input_path_interval = declare_parameter<double>("input_path_interval");
  p.output_path_interval = declare_parameter<double>("output_path_interval");
  p.ego_nearest_dist_threshold = declare_parameter<double>("ego_nearest_dist_threshold");
  p.ego_nearest_yaw_threshold = declare_parameter<double>("ego_nearest_yaw_threshold");

  return p;
}

// wait until mandatory data is ready
bool BehaviorPathPlannerNode::isDataReady()
{
  const auto missing = [this](const auto & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", name);
    return false;
  };

  if (!current_scenario_) {
    return missing("scenario_topic");
  }

  {
    std::lock_guard<std::mutex> lk_route(mutex_route_);
    if (!route_ptr_) {
      return missing("route");
    }
  }

  {
    std::lock_guard<std::mutex> lk_map(mutex_map_);
    if (!map_ptr_) {
      return missing("map");
    }
  }

  const std::lock_guard<std::mutex> lock(mutex_pd_);  // for planner_data_

  if (!planner_data_->dynamic_object) {
    return missing("dynamic_object");
  }

  if (!planner_data_->self_odometry) {
    return missing("self_odometry");
  }

  if (!planner_data_->self_acceleration) {
    return missing("self_acceleration");
  }

  if (!planner_data_->operation_mode) {
    return missing("operation_mode");
  }

  return true;
}

void BehaviorPathPlannerNode::run()
{
  if (!isDataReady()) {
    return;
  }

  RCLCPP_DEBUG(get_logger(), "----- BehaviorPathPlannerNode start -----");

  // behavior_path_planner runs only in LANE DRIVING scenario.
  if (current_scenario_->current_scenario != Scenario::LANEDRIVING) {
    return;
  }

  // check for map update
  HADMapBin::ConstSharedPtr map_ptr{nullptr};
  {
    std::lock_guard<std::mutex> lk_map(mutex_map_);  // for has_received_map_ and map_ptr_
    if (has_received_map_) {
      // Note: duplicating the shared_ptr prevents the data from being deleted by another thread!
      map_ptr = map_ptr_;
      has_received_map_ = false;
    }
  }

  // check for route update
  LaneletRoute::ConstSharedPtr route_ptr{nullptr};
  {
    std::lock_guard<std::mutex> lk_route(mutex_route_);  // for has_received_route_ and route_ptr_
    if (has_received_route_) {
      // Note: duplicating the shared_ptr prevents the data from being deleted by another thread!
      route_ptr = route_ptr_;
      has_received_route_ = false;
    }
  }

  std::unique_lock<std::mutex> lk_pd(mutex_pd_);  // for planner_data_

  // update map
  if (map_ptr) {
    planner_data_->route_handler->setMap(*map_ptr);
  }

  std::unique_lock<std::mutex> lk_manager(mutex_manager_);  // for planner_manager_

  // update route
  const bool is_first_time = !(planner_data_->route_handler->isHandlerReady());
  if (route_ptr) {
    planner_data_->route_handler->setRoute(*route_ptr);
    planner_manager_->resetRootLanelet(planner_data_);

    // uuid is not changed when rerouting with modified goal,
    // in this case do not need to rest modules.
    const bool has_same_route_id =
      planner_data_->prev_route_id && route_ptr->uuid == planner_data_->prev_route_id;
    // Reset behavior tree when new route is received,
    // so that the each modules do not have to care about the "route jump".
    if (!is_first_time && !has_same_route_id) {
      RCLCPP_INFO(get_logger(), "New uuid route is received. Resetting modules.");
      planner_manager_->reset();
      planner_data_->prev_modified_goal.reset();
    }
  }

  const auto controlled_by_autoware_autonomously =
    planner_data_->operation_mode->mode == OperationModeState::AUTONOMOUS &&
    planner_data_->operation_mode->is_autoware_control_enabled;
  if (!controlled_by_autoware_autonomously) {
    planner_manager_->resetRootLanelet(planner_data_);
  }

  // run behavior planner
  const auto output = planner_manager_->run(planner_data_);

  // path handling
  const auto path = getPath(output, planner_data_, planner_manager_);
  // update planner data
  planner_data_->prev_output_path = path;

  // compute turn signal
  computeTurnSignal(planner_data_, *path, output);

  // publish reroute availability
  publish_reroute_availability();

  // publish drivable bounds
  publish_bounds(*path);

  // NOTE: In order to keep backward_path_length at least, resampling interval is added to the
  // backward.
  const auto current_pose = planner_data_->self_odometry->pose.pose;
  if (!path->points.empty()) {
    const size_t current_seg_idx = planner_data_->findEgoSegmentIndex(path->points);
    path->points = motion_utils::cropPoints(
      path->points, current_pose.position, current_seg_idx,
      planner_data_->parameters.forward_path_length,
      planner_data_->parameters.backward_path_length +
        planner_data_->parameters.input_path_interval);

    if (!path->points.empty()) {
      path_publisher_->publish(*path);
    } else {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "behavior path output is empty! Stop publish.");
    }
  } else {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "behavior path output is empty! Stop publish.");
  }

  publishSceneModuleDebugMsg(planner_manager_->getDebugMsg());
  publishPathCandidate(planner_manager_->getSceneModuleManagers(), planner_data_);
  publishPathReference(planner_manager_->getSceneModuleManagers(), planner_data_);
  stop_reason_publisher_->publish(planner_manager_->getStopReasons());

  // publish modified goal only when it is updated
  if (
    output.modified_goal &&
    /* has changed modified goal */ (
      !planner_data_->prev_modified_goal || tier4_autoware_utils::calcDistance2d(
                                              planner_data_->prev_modified_goal->pose.position,
                                              output.modified_goal->pose.position) > 0.01)) {
    PoseWithUuidStamped modified_goal = *(output.modified_goal);
    modified_goal.header.stamp = path->header.stamp;
    planner_data_->prev_modified_goal = modified_goal;
    modified_goal_publisher_->publish(modified_goal);
  }

  planner_data_->prev_route_id = planner_data_->route_handler->getRouteUuid();

  lk_pd.unlock();  // release planner_data_

  planner_manager_->print();
  planner_manager_->publishProcessingTime();
  planner_manager_->publishMarker();
  planner_manager_->publishVirtualWall();
  lk_manager.unlock();  // release planner_manager_

  RCLCPP_DEBUG(get_logger(), "----- behavior path planner end -----\n\n");
}

void BehaviorPathPlannerNode::computeTurnSignal(
  const std::shared_ptr<PlannerData> planner_data, const PathWithLaneId & path,
  const BehaviorModuleOutput & output)
{
  TurnIndicatorsCommand turn_signal;
  TurnSignalDebugData debug_data;
  HazardLightsCommand hazard_signal;
  if (output.turn_signal_info.hazard_signal.command == HazardLightsCommand::ENABLE) {
    turn_signal.command = TurnIndicatorsCommand::DISABLE;
    hazard_signal.command = output.turn_signal_info.hazard_signal.command;
  } else {
    turn_signal = planner_data->getTurnSignal(path, output.turn_signal_info, debug_data);
    hazard_signal.command = HazardLightsCommand::DISABLE;
  }
  turn_signal.stamp = get_clock()->now();
  hazard_signal.stamp = get_clock()->now();
  turn_signal_publisher_->publish(turn_signal);
  hazard_signal_publisher_->publish(hazard_signal);

  publish_turn_signal_debug_data(debug_data);
  publish_steering_factor(planner_data, turn_signal);
}

void BehaviorPathPlannerNode::publish_steering_factor(
  const std::shared_ptr<PlannerData> & planner_data, const TurnIndicatorsCommand & turn_signal)
{
  const auto [intersection_flag, approaching_intersection_flag] =
    planner_data->turn_signal_decider.getIntersectionTurnSignalFlag();
  if (intersection_flag || approaching_intersection_flag) {
    const uint16_t steering_factor_direction = std::invoke([&turn_signal]() {
      if (turn_signal.command == TurnIndicatorsCommand::ENABLE_LEFT) {
        return SteeringFactor::LEFT;
      }
      return SteeringFactor::RIGHT;
    });

    const auto [intersection_pose, intersection_distance] =
      planner_data->turn_signal_decider.getIntersectionPoseAndDistance();

    steering_factor_interface_ptr_->updateSteeringFactor(
      {intersection_pose, intersection_pose}, {intersection_distance, intersection_distance},
      PlanningBehavior::INTERSECTION, steering_factor_direction, SteeringFactor::TURNING, "");
  } else {
    steering_factor_interface_ptr_->clearSteeringFactors();
  }
  steering_factor_interface_ptr_->publishSteeringFactor(get_clock()->now());
}

void BehaviorPathPlannerNode::publish_reroute_availability() const
{
  // In the current behavior path planner, we might encounter unexpected behavior when rerouting
  // while modules other than lane following are active. If non-lane-following module except
  // always-executable module is approved and running, rerouting will not be　possible.
  RerouteAvailability is_reroute_available;
  is_reroute_available.stamp = this->now();
  if (planner_manager_->hasNonAlwaysExecutableApprovedModules()) {
    is_reroute_available.availability = false;
  } else {
    is_reroute_available.availability = true;
  }

  reroute_availability_publisher_->publish(is_reroute_available);
}

void BehaviorPathPlannerNode::publish_turn_signal_debug_data(const TurnSignalDebugData & debug_data)
{
  MarkerArray marker_array;

  const auto current_time = rclcpp::Time();
  constexpr double scale_x = 1.0;
  constexpr double scale_y = 1.0;
  constexpr double scale_z = 1.0;
  const auto scale = tier4_autoware_utils::createMarkerScale(scale_x, scale_y, scale_z);
  const auto desired_section_color = tier4_autoware_utils::createMarkerColor(0.0, 1.0, 0.0, 0.999);
  const auto required_section_color = tier4_autoware_utils::createMarkerColor(1.0, 0.0, 1.0, 0.999);

  // intersection turn signal info
  {
    const auto & turn_signal_info = debug_data.intersection_turn_signal_info;

    auto desired_start_marker = tier4_autoware_utils::createDefaultMarker(
      "map", current_time, "intersection_turn_signal_desired_start", 0L, Marker::SPHERE, scale,
      desired_section_color);
    auto desired_end_marker = tier4_autoware_utils::createDefaultMarker(
      "map", current_time, "intersection_turn_signal_desired_end", 0L, Marker::SPHERE, scale,
      desired_section_color);
    desired_start_marker.pose = turn_signal_info.desired_start_point;
    desired_end_marker.pose = turn_signal_info.desired_end_point;

    auto required_start_marker = tier4_autoware_utils::createDefaultMarker(
      "map", current_time, "intersection_turn_signal_required_start", 0L, Marker::SPHERE, scale,
      required_section_color);
    auto required_end_marker = tier4_autoware_utils::createDefaultMarker(
      "map", current_time, "intersection_turn_signal_required_end", 0L, Marker::SPHERE, scale,
      required_section_color);
    required_start_marker.pose = turn_signal_info.required_start_point;
    required_end_marker.pose = turn_signal_info.required_end_point;

    marker_array.markers.push_back(desired_start_marker);
    marker_array.markers.push_back(desired_end_marker);
    marker_array.markers.push_back(required_start_marker);
    marker_array.markers.push_back(required_end_marker);
  }

  // behavior turn signal info
  {
    const auto & turn_signal_info = debug_data.behavior_turn_signal_info;

    auto desired_start_marker = tier4_autoware_utils::createDefaultMarker(
      "map", current_time, "behavior_turn_signal_desired_start", 0L, Marker::CUBE, scale,
      desired_section_color);
    auto desired_end_marker = tier4_autoware_utils::createDefaultMarker(
      "map", current_time, "behavior_turn_signal_desired_end", 0L, Marker::CUBE, scale,
      desired_section_color);
    desired_start_marker.pose = turn_signal_info.desired_start_point;
    desired_end_marker.pose = turn_signal_info.desired_end_point;

    auto required_start_marker = tier4_autoware_utils::createDefaultMarker(
      "map", current_time, "behavior_turn_signal_required_start", 0L, Marker::CUBE, scale,
      required_section_color);
    auto required_end_marker = tier4_autoware_utils::createDefaultMarker(
      "map", current_time, "behavior_turn_signal_required_end", 0L, Marker::CUBE, scale,
      required_section_color);
    required_start_marker.pose = turn_signal_info.required_start_point;
    required_end_marker.pose = turn_signal_info.required_end_point;

    marker_array.markers.push_back(desired_start_marker);
    marker_array.markers.push_back(desired_end_marker);
    marker_array.markers.push_back(required_start_marker);
    marker_array.markers.push_back(required_end_marker);
  }

  debug_turn_signal_info_publisher_->publish(marker_array);
}

void BehaviorPathPlannerNode::publish_bounds(const PathWithLaneId & path)
{
  constexpr double scale_x = 0.2;
  constexpr double scale_y = 0.2;
  constexpr double scale_z = 0.2;
  constexpr double color_r = 0.0 / 256.0;
  constexpr double color_g = 148.0 / 256.0;
  constexpr double color_b = 205.0 / 256.0;
  constexpr double color_a = 0.999;

  const auto current_time = path.header.stamp;
  auto left_marker = tier4_autoware_utils::createDefaultMarker(
    "map", current_time, "left_bound", 0L, Marker::LINE_STRIP,
    tier4_autoware_utils::createMarkerScale(scale_x, scale_y, scale_z),
    tier4_autoware_utils::createMarkerColor(color_r, color_g, color_b, color_a));
  for (const auto lb : path.left_bound) {
    left_marker.points.push_back(lb);
  }

  auto right_marker = tier4_autoware_utils::createDefaultMarker(
    "map", current_time, "right_bound", 0L, Marker::LINE_STRIP,
    tier4_autoware_utils::createMarkerScale(scale_x, scale_y, scale_z),
    tier4_autoware_utils::createMarkerColor(color_r, color_g, color_b, color_a));
  for (const auto rb : path.right_bound) {
    right_marker.points.push_back(rb);
  }

  MarkerArray msg;
  msg.markers.push_back(left_marker);
  msg.markers.push_back(right_marker);
  bound_publisher_->publish(msg);
}

void BehaviorPathPlannerNode::publishSceneModuleDebugMsg(
  const std::shared_ptr<SceneModuleVisitor> & debug_messages_data_ptr)
{
  const auto avoidance_debug_message = debug_messages_data_ptr->getAvoidanceModuleDebugMsg();
  if (avoidance_debug_message) {
    debug_avoidance_msg_array_publisher_->publish(*avoidance_debug_message);
  }
}

void BehaviorPathPlannerNode::publishPathCandidate(
  const std::vector<std::shared_ptr<SceneModuleManagerInterface>> & managers,
  const std::shared_ptr<PlannerData> & planner_data)
{
  for (auto & manager : managers) {
    if (path_candidate_publishers_.count(manager->name()) == 0) {
      continue;
    }

    if (manager->getSceneModuleObservers().empty()) {
      path_candidate_publishers_.at(manager->name())
        ->publish(convertToPath(nullptr, false, planner_data));
      continue;
    }

    for (auto & observer : manager->getSceneModuleObservers()) {
      if (observer.expired()) {
        continue;
      }
      const auto & status = observer.lock()->getCurrentStatus();
      const auto candidate_path = std::invoke([&]() {
        if (status == ModuleStatus::SUCCESS || status == ModuleStatus::FAILURE) {
          // clear candidate path if the module is finished
          return convertToPath(nullptr, false, planner_data);
        }
        return convertToPath(
          observer.lock()->getPathCandidate(), observer.lock()->isExecutionReady(), planner_data);
      });

      path_candidate_publishers_.at(observer.lock()->name())->publish(candidate_path);
    }
  }
}

void BehaviorPathPlannerNode::publishPathReference(
  const std::vector<std::shared_ptr<SceneModuleManagerInterface>> & managers,
  const std::shared_ptr<PlannerData> & planner_data)
{
  for (auto & manager : managers) {
    if (path_reference_publishers_.count(manager->name()) == 0) {
      continue;
    }

    if (manager->getSceneModuleObservers().empty()) {
      path_reference_publishers_.at(manager->name())
        ->publish(convertToPath(nullptr, false, planner_data));
      continue;
    }

    for (auto & observer : manager->getSceneModuleObservers()) {
      if (observer.expired()) {
        continue;
      }
      path_reference_publishers_.at(observer.lock()->name())
        ->publish(convertToPath(observer.lock()->getPathReference(), true, planner_data));
    }
  }
}

Path BehaviorPathPlannerNode::convertToPath(
  const std::shared_ptr<PathWithLaneId> & path_candidate_ptr, const bool is_ready,
  const std::shared_ptr<PlannerData> & planner_data)
{
  Path output;
  output.header = planner_data->route_handler->getRouteHeader();
  output.header.stamp = this->now();

  if (!path_candidate_ptr) {
    return output;
  }

  output = utils::toPath(*path_candidate_ptr);
  // header is replaced by the input one, so it is substituted again
  output.header = planner_data->route_handler->getRouteHeader();
  output.header.stamp = this->now();

  if (!is_ready) {
    for (auto & point : output.points) {
      point.longitudinal_velocity_mps = 0.0;
    }
  }

  return output;
}

PathWithLaneId::SharedPtr BehaviorPathPlannerNode::getPath(
  const BehaviorModuleOutput & output, const std::shared_ptr<PlannerData> & planner_data,
  const std::shared_ptr<PlannerManager> & planner_manager)
{
  // TODO(Horibe) do some error handling when path is not available.

  auto path = !output.path.points.empty() ? std::make_shared<PathWithLaneId>(output.path)
                                          : planner_data->prev_output_path;
  path->header = planner_data->route_handler->getRouteHeader();
  path->header.stamp = this->now();

  PathWithLaneId connected_path;
  const auto module_status_ptr_vec = planner_manager->getSceneModuleStatus();

  const auto resampled_path = utils::resamplePathWithSpline(
    *path, planner_data->parameters.output_path_interval, keepInputPoints(module_status_ptr_vec));
  return std::make_shared<PathWithLaneId>(resampled_path);
}

// This is a temporary process until motion planning can take the terminal pose into account
bool BehaviorPathPlannerNode::keepInputPoints(
  const std::vector<std::shared_ptr<SceneModuleStatus>> & statuses) const
{
  const std::vector<std::string> target_modules = {"goal_planner", "avoidance"};

  const auto target_status = ModuleStatus::RUNNING;

  for (auto & status : statuses) {
    if (status->is_waiting_approval || status->status == target_status) {
      if (
        std::find(target_modules.begin(), target_modules.end(), status->module_name) !=
        target_modules.end()) {
        return true;
      }
    }
  }
  return false;
}

void BehaviorPathPlannerNode::onOdometry(const Odometry::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_pd_);
  planner_data_->self_odometry = msg;
}
void BehaviorPathPlannerNode::onAcceleration(const AccelWithCovarianceStamped::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_pd_);
  planner_data_->self_acceleration = msg;
}
void BehaviorPathPlannerNode::onPerception(const PredictedObjects::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_pd_);
  planner_data_->dynamic_object = msg;
}
void BehaviorPathPlannerNode::onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_pd_);
  planner_data_->occupancy_grid = msg;
}
void BehaviorPathPlannerNode::onCostMap(const OccupancyGrid::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_pd_);
  planner_data_->costmap = msg;
}
void BehaviorPathPlannerNode::onTrafficSignals(const TrafficSignalArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_pd_);

  for (const auto & signal : msg->signals) {
    TrafficSignalStamped traffic_signal;
    traffic_signal.stamp = msg->stamp;
    traffic_signal.signal = signal;
    planner_data_->traffic_light_id_map[signal.traffic_signal_id] = traffic_signal;
  }
}
void BehaviorPathPlannerNode::onMap(const HADMapBin::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_map_);
  map_ptr_ = msg;
  has_received_map_ = true;
}
void BehaviorPathPlannerNode::onRoute(const LaneletRoute::ConstSharedPtr msg)
{
  if (msg->segments.empty()) {
    RCLCPP_ERROR(get_logger(), "input route is empty. ignored");
    return;
  }

  const std::lock_guard<std::mutex> lock(mutex_route_);
  route_ptr_ = msg;
  has_received_route_ = true;
}
void BehaviorPathPlannerNode::onOperationMode(const OperationModeState::ConstSharedPtr msg)
{
  const std::lock_guard<std::mutex> lock(mutex_pd_);
  planner_data_->operation_mode = msg;
}
void BehaviorPathPlannerNode::onLateralOffset(const LateralOffset::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_pd_);

  if (!planner_data_->lateral_offset) {
    planner_data_->lateral_offset = msg;
    return;
  }

  const auto & new_offset = msg->lateral_offset;
  const auto & old_offset = planner_data_->lateral_offset->lateral_offset;

  // offset is not changed.
  if (std::abs(old_offset - new_offset) < 1e-4) {
    return;
  }

  planner_data_->lateral_offset = msg;
}

SetParametersResult BehaviorPathPlannerNode::onSetParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  rcl_interfaces::msg::SetParametersResult result;

  {
    const std::lock_guard<std::mutex> lock(mutex_manager_);  // for planner_manager_
    planner_manager_->updateModuleParams(parameters);
  }

  result.successful = true;
  result.reason = "success";

  try {
    // Drivable area expansion parameters
    using drivable_area_expansion::DrivableAreaExpansionParameters;
    const std::lock_guard<std::mutex> lock(mutex_pd_);  // for planner_data_
    updateParam(
      parameters, DrivableAreaExpansionParameters::DRIVABLE_AREA_RIGHT_BOUND_OFFSET_PARAM,
      planner_data_->drivable_area_expansion_parameters.drivable_area_right_bound_offset);
    updateParam(
      parameters, DrivableAreaExpansionParameters::DRIVABLE_AREA_LEFT_BOUND_OFFSET_PARAM,
      planner_data_->drivable_area_expansion_parameters.drivable_area_left_bound_offset);
    updateParam(
      parameters, DrivableAreaExpansionParameters::DRIVABLE_AREA_TYPES_TO_SKIP_PARAM,
      planner_data_->drivable_area_expansion_parameters.drivable_area_types_to_skip);
    updateParam(
      parameters, DrivableAreaExpansionParameters::ENABLED_PARAM,
      planner_data_->drivable_area_expansion_parameters.enabled);
    updateParam(
      parameters, DrivableAreaExpansionParameters::AVOID_DYN_OBJECTS_PARAM,
      planner_data_->drivable_area_expansion_parameters.avoid_dynamic_objects);
    updateParam(
      parameters, DrivableAreaExpansionParameters::AVOID_LINESTRING_TYPES_PARAM,
      planner_data_->drivable_area_expansion_parameters.avoid_linestring_types);
    updateParam(
      parameters, DrivableAreaExpansionParameters::AVOID_LINESTRING_DIST_PARAM,
      planner_data_->drivable_area_expansion_parameters.avoid_linestring_dist);
    updateParam(
      parameters, DrivableAreaExpansionParameters::EGO_EXTRA_FRONT_OVERHANG,
      planner_data_->drivable_area_expansion_parameters.extra_front_overhang);
    updateParam(
      parameters, DrivableAreaExpansionParameters::EGO_EXTRA_WHEELBASE,
      planner_data_->drivable_area_expansion_parameters.extra_wheelbase);
    updateParam(
      parameters, DrivableAreaExpansionParameters::EGO_EXTRA_WIDTH,
      planner_data_->drivable_area_expansion_parameters.extra_width);
    updateParam(
      parameters, DrivableAreaExpansionParameters::DYN_OBJECTS_EXTRA_OFFSET_FRONT,
      planner_data_->drivable_area_expansion_parameters.dynamic_objects_extra_front_offset);
    updateParam(
      parameters, DrivableAreaExpansionParameters::DYN_OBJECTS_EXTRA_OFFSET_REAR,
      planner_data_->drivable_area_expansion_parameters.dynamic_objects_extra_rear_offset);
    updateParam(
      parameters, DrivableAreaExpansionParameters::DYN_OBJECTS_EXTRA_OFFSET_LEFT,
      planner_data_->drivable_area_expansion_parameters.dynamic_objects_extra_left_offset);
    updateParam(
      parameters, DrivableAreaExpansionParameters::DYN_OBJECTS_EXTRA_OFFSET_RIGHT,
      planner_data_->drivable_area_expansion_parameters.dynamic_objects_extra_right_offset);
    updateParam(
      parameters, DrivableAreaExpansionParameters::MAX_EXP_DIST_PARAM,
      planner_data_->drivable_area_expansion_parameters.max_expansion_distance);
    updateParam(
      parameters, DrivableAreaExpansionParameters::MAX_PATH_ARC_LENGTH_PARAM,
      planner_data_->drivable_area_expansion_parameters.max_path_arc_length);
    updateParam(
      parameters, DrivableAreaExpansionParameters::RESAMPLE_INTERVAL_PARAM,
      planner_data_->drivable_area_expansion_parameters.resample_interval);
    updateParam(
      parameters, DrivableAreaExpansionParameters::MAX_REUSE_DEVIATION_PARAM,
      planner_data_->drivable_area_expansion_parameters.max_reuse_deviation);
    updateParam(
      parameters, DrivableAreaExpansionParameters::SMOOTHING_CURVATURE_WINDOW_PARAM,
      planner_data_->drivable_area_expansion_parameters.curvature_average_window);
    updateParam(
      parameters, DrivableAreaExpansionParameters::SMOOTHING_MAX_BOUND_RATE_PARAM,
      planner_data_->drivable_area_expansion_parameters.max_bound_rate);
    updateParam(
      parameters, DrivableAreaExpansionParameters::SMOOTHING_ARC_LENGTH_RANGE_PARAM,
      planner_data_->drivable_area_expansion_parameters.arc_length_range);
    updateParam(
      parameters, DrivableAreaExpansionParameters::PRINT_RUNTIME_PARAM,
      planner_data_->drivable_area_expansion_parameters.print_runtime);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}
}  // namespace behavior_path_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(behavior_path_planner::BehaviorPathPlannerNode)
