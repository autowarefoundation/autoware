// Copyright 2024 The Autoware Contributors
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

#include "reaction_analyzer_node.hpp"

#include <functional>
#include <memory>

namespace reaction_analyzer
{

void ReactionAnalyzerNode::on_operation_mode_state(OperationModeState::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  operation_mode_ptr_ = std::move(msg_ptr);
}

void ReactionAnalyzerNode::on_route_state(RouteState::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_route_state_ptr_ = std::move(msg_ptr);
}

void ReactionAnalyzerNode::on_vehicle_pose(Odometry::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  odometry_ptr_ = msg_ptr;
}

void ReactionAnalyzerNode::on_localization_initialization_state(
  LocalizationInitializationState::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  initialization_state_ptr_ = std::move(msg_ptr);
}

void ReactionAnalyzerNode::on_ground_truth_pose(PoseStamped::ConstSharedPtr msg_ptr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ground_truth_pose_ptr_ = std::move(msg_ptr);
}

ReactionAnalyzerNode::ReactionAnalyzerNode(rclcpp::NodeOptions node_options)
: Node("reaction_analyzer_node", node_options.automatically_declare_parameters_from_overrides(true))
{
  using std::placeholders::_1;

  node_params_.running_mode = get_parameter("running_mode").as_string();

  // set running mode
  if (node_params_.running_mode == "planning_control") {
    node_running_mode_ = RunningMode::PlanningControl;
  } else if (node_params_.running_mode == "perception_planning") {
    node_running_mode_ = RunningMode::PerceptionPlanning;
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid running mode. Node couldn't be initialized. Failed.");
    return;
  }

  node_params_.output_file_path = get_parameter("output_file_path").as_string();
  // Check if the output file path is valid
  if (!does_folder_exist(node_params_.output_file_path)) {
    RCLCPP_ERROR(get_logger(), "Output file path is not valid. Node couldn't be initialized.");
    return;
  }

  node_params_.timer_period = get_parameter("timer_period").as_double();
  node_params_.test_iteration = get_parameter("test_iteration").as_int();
  node_params_.spawn_time_after_init = get_parameter("spawn_time_after_init").as_double();
  node_params_.spawn_distance_threshold = get_parameter("spawn_distance_threshold").as_double();

  // Position parameters
  node_params_.initial_pose.x = get_parameter("poses.initialization_pose.x").as_double();
  node_params_.initial_pose.y = get_parameter("poses.initialization_pose.y").as_double();
  node_params_.initial_pose.z = get_parameter("poses.initialization_pose.z").as_double();
  node_params_.initial_pose.roll = get_parameter("poses.initialization_pose.roll").as_double();
  node_params_.initial_pose.pitch = get_parameter("poses.initialization_pose.pitch").as_double();
  node_params_.initial_pose.yaw = get_parameter("poses.initialization_pose.yaw").as_double();

  node_params_.goal_pose.x = get_parameter("poses.goal_pose.x").as_double();
  node_params_.goal_pose.y = get_parameter("poses.goal_pose.y").as_double();
  node_params_.goal_pose.z = get_parameter("poses.goal_pose.z").as_double();
  node_params_.goal_pose.roll = get_parameter("poses.goal_pose.roll").as_double();
  node_params_.goal_pose.pitch = get_parameter("poses.goal_pose.pitch").as_double();
  node_params_.goal_pose.yaw = get_parameter("poses.goal_pose.yaw").as_double();

  node_params_.entity_params.x = get_parameter("poses.entity_params.x").as_double();
  node_params_.entity_params.y = get_parameter("poses.entity_params.y").as_double();
  node_params_.entity_params.z = get_parameter("poses.entity_params.z").as_double();
  node_params_.entity_params.roll = get_parameter("poses.entity_params.roll").as_double();
  node_params_.entity_params.pitch = get_parameter("poses.entity_params.pitch").as_double();
  node_params_.entity_params.yaw = get_parameter("poses.entity_params.yaw").as_double();
  node_params_.entity_params.x_l = get_parameter("poses.entity_params.x_dimension").as_double();
  node_params_.entity_params.y_l = get_parameter("poses.entity_params.y_dimension").as_double();
  node_params_.entity_params.z_l = get_parameter("poses.entity_params.z_dimension").as_double();

  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", 1, std::bind(&ReactionAnalyzerNode::on_vehicle_pose, this, _1),
    create_subscription_options(this));
  sub_localization_init_state_ = create_subscription<LocalizationInitializationState>(
    "input/localization_initialization_state", rclcpp::QoS(1).transient_local(),
    std::bind(&ReactionAnalyzerNode::on_localization_initialization_state, this, _1),
    create_subscription_options(this));
  sub_route_state_ = create_subscription<RouteState>(
    "input/routing_state", rclcpp::QoS{1}.transient_local(),
    std::bind(&ReactionAnalyzerNode::on_route_state, this, _1), create_subscription_options(this));
  sub_operation_mode_ = create_subscription<OperationModeState>(
    "input/operation_mode_state", rclcpp::QoS{1}.transient_local(),
    std::bind(&ReactionAnalyzerNode::on_operation_mode_state, this, _1),
    create_subscription_options(this));

  pub_goal_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("output/goal", rclcpp::QoS(1));
  pub_marker_ = create_publisher<visualization_msgs::msg::Marker>("~/debug", 10);

  init_analyzer_variables();

  if (node_running_mode_ == RunningMode::PlanningControl) {
    pub_initial_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "output/initialpose", rclcpp::QoS(1));

    client_change_to_autonomous_ =
      create_client<ChangeOperationMode>("service/change_to_autonomous");

  } else if (node_running_mode_ == RunningMode::PerceptionPlanning) {
    sub_ground_truth_pose_ = create_subscription<PoseStamped>(
      "input/ground_truth_pose", rclcpp::QoS{1},
      std::bind(&ReactionAnalyzerNode::on_ground_truth_pose, this, _1),
      create_subscription_options(this));
  }

  topic_publisher_ptr_ = std::make_unique<topic_publisher::TopicPublisher>(
    this, spawn_object_cmd_, ego_initialized_, spawn_cmd_time_, node_running_mode_,
    node_params_.entity_params);

  // initialize the odometry before init the subscriber
  odometry_ptr_ = std::make_shared<Odometry>();
  // Load the subscriber to listen the topics for reactions
  subscriber_ptr_ = std::make_unique<subscriber::SubscriberBase>(
    this, odometry_ptr_, spawn_object_cmd_, node_params_.entity_params);

  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(node_params_.timer_period));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&ReactionAnalyzerNode::on_timer, this));
}

void ReactionAnalyzerNode::on_timer()
{
  mutex_.lock();
  const auto current_odometry_ptr = odometry_ptr_;
  const auto initialization_state_ptr = initialization_state_ptr_;
  const auto route_state_ptr = current_route_state_ptr_;
  const auto operation_mode_ptr = operation_mode_ptr_;
  const auto ground_truth_pose_ptr = ground_truth_pose_ptr_;
  const auto spawn_cmd_time = spawn_cmd_time_;
  mutex_.unlock();

  // Init the test environment
  if (!test_environment_init_time_) {
    init_test_env(
      initialization_state_ptr, route_state_ptr, operation_mode_ptr, ground_truth_pose_ptr,
      current_odometry_ptr);
    return;
  }

  pub_marker_->publish(entity_debug_marker_);

  // Spawn the obstacle if the conditions are met
  spawn_obstacle(current_odometry_ptr->pose.pose.position);

  // If the spawn_cmd_time is not set by pointcloud publishers, don't do anything
  if (!spawn_cmd_time) return;

  // Get the reacted messages, if all modules reacted
  const auto message_buffers = subscriber_ptr_->get_message_buffers_map();

  if (message_buffers) {
    // if reacted, calculate the results
    calculate_results(message_buffers.value(), spawn_cmd_time.value());
    // reset the variables if the iteration is not finished, otherwise write the results
    reset();
  }
}

void ReactionAnalyzerNode::spawn_obstacle(const geometry_msgs::msg::Point & ego_pose)
{
  if (node_running_mode_ == RunningMode::PerceptionPlanning) {
    rclcpp::Duration time_diff = this->now() - test_environment_init_time_.value();
    if (time_diff > rclcpp::Duration::from_seconds(node_params_.spawn_time_after_init)) {
      if (!spawn_object_cmd_) {
        spawn_object_cmd_ = true;
        RCLCPP_INFO(this->get_logger(), "Spawn command is sent.");
      }
    }
  } else {
    if (
      autoware::universe_utils::calcDistance3d(ego_pose, entity_pose_.position) <
      node_params_.spawn_distance_threshold) {
      if (!spawn_object_cmd_) {
        spawn_object_cmd_ = true;
        RCLCPP_INFO(this->get_logger(), "Spawn command is sent.");
      }
    }
  }
}

void ReactionAnalyzerNode::calculate_results(
  const std::map<std::string, subscriber::MessageBufferVariant> & message_buffers,
  const rclcpp::Time & spawn_cmd_time)
{
  // Map the reaction times w.r.t its header time to categorize it
  PipelineMap pipeline_map;

  {
    // set the spawn_cmd_time as the first reaction pair
    ReactionPair reaction_pair;
    reaction_pair.first = "spawn_cmd_time";
    reaction_pair.second.header.stamp = spawn_cmd_time;
    reaction_pair.second.published_stamp = spawn_cmd_time;
    pipeline_map[reaction_pair.second.header.stamp].emplace_back(reaction_pair);
  }

  for (const auto & [key, variant] : message_buffers) {
    ReactionPair reaction_pair;
    if (auto * control_message = std::get_if<subscriber::ControlCommandBuffer>(&variant)) {
      if (control_message->second) {
        reaction_pair.first = key;
        reaction_pair.second = control_message->second.value();
      }
    } else if (auto * general_message = std::get_if<subscriber::MessageBuffer>(&variant)) {
      if (general_message->has_value()) {
        reaction_pair.first = key;
        reaction_pair.second = general_message->value();
      }
    }
    pipeline_map[reaction_pair.second.header.stamp].emplace_back(reaction_pair);
  }
  pipeline_map_vector_.emplace_back(pipeline_map);
  test_iteration_count_++;
}

void ReactionAnalyzerNode::init_analyzer_variables()
{
  entity_pose_ = create_entity_pose(node_params_.entity_params);
  entity_debug_marker_ = create_polyhedron_marker(node_params_.entity_params);
  goal_pose_.pose = pose_params_to_pose(node_params_.goal_pose);

  if (node_running_mode_ == RunningMode::PlanningControl) {
    init_pose_.pose.pose = pose_params_to_pose(node_params_.initial_pose);
  }
}

void ReactionAnalyzerNode::init_test_env(
  const LocalizationInitializationState::ConstSharedPtr & initialization_state_ptr,
  const RouteState::ConstSharedPtr & route_state_ptr,
  const OperationModeState::ConstSharedPtr & operation_mode_ptr,
  const PoseStamped::ConstSharedPtr & ground_truth_pose_ptr,
  const Odometry::ConstSharedPtr & odometry_ptr)
{
  const auto current_time = this->now();

  // Initialize the test environment
  constexpr double initialize_call_period = 1.0;  // sec
  if (
    !last_test_environment_init_request_time_ ||
    (current_time - last_test_environment_init_request_time_.value()).seconds() >=
      initialize_call_period) {
    last_test_environment_init_request_time_ = current_time;

    // Pose initialization of the ego
    is_initialization_requested = !is_initialization_requested
                                    ? init_ego(initialization_state_ptr, odometry_ptr, current_time)
                                    : is_initialization_requested;

    if (
      is_initialization_requested &&
      initialization_state_ptr->state != LocalizationInitializationState::INITIALIZED) {
      is_initialization_requested = false;
      return;
    }

    // Check is position initialized accurately, if node is running in perception_planning mode
    if (node_running_mode_ == RunningMode::PerceptionPlanning) {
      if (!check_ego_init_correctly(ground_truth_pose_ptr, odometry_ptr)) return;
    }

    // Set route
    is_route_set_ = !is_route_set_ ? set_route(route_state_ptr, current_time) : is_route_set_;

    if (!is_route_set_) {
      return;
    }

    // if node is running PlanningControl mode, change ego to Autonomous mode.
    if (node_running_mode_ == RunningMode::PlanningControl) {
      // change to autonomous
      if (operation_mode_ptr && operation_mode_ptr->mode != OperationModeState::AUTONOMOUS) {
        call_operation_mode_service_without_response();
      }
    }
    ego_initialized_ = true;

    const bool is_ready =
      (is_initialization_requested && is_route_set_ &&
       (operation_mode_ptr->mode == OperationModeState::AUTONOMOUS ||
        node_running_mode_ == RunningMode::PerceptionPlanning));
    if (is_ready) {
      test_environment_init_time_ = this->now();
    }
  }
}

void ReactionAnalyzerNode::call_operation_mode_service_without_response()
{
  auto req = std::make_shared<ChangeOperationMode::Request>();

  RCLCPP_INFO(this->get_logger(), "client request");

  if (!client_change_to_autonomous_->service_is_ready()) {
    RCLCPP_INFO(this->get_logger(), "client is unavailable");
    return;
  }

  client_change_to_autonomous_->async_send_request(
    req, [this](typename rclcpp::Client<ChangeOperationMode>::SharedFuture result) {
      RCLCPP_INFO(
        this->get_logger(), "Status: %d, %s", result.get()->status.code,
        result.get()->status.message.c_str());
    });
}

bool ReactionAnalyzerNode::init_ego(
  const LocalizationInitializationState::ConstSharedPtr & initialization_state_ptr,
  const Odometry::ConstSharedPtr & odometry_ptr, const rclcpp::Time & current_time)
{
  // Pose initialization of the ego
  if (initialization_state_ptr) {
    if (node_running_mode_ == RunningMode::PlanningControl) {
      // publish initial pose
      init_pose_.header.stamp = current_time;
      init_pose_.header.frame_id = "map";
      pub_initial_pose_->publish(init_pose_);
      RCLCPP_WARN_ONCE(get_logger(), "Initialization position is published. Waiting for init...");
    }
    // Wait until odometry_ptr is initialized
    if (!odometry_ptr) {
      RCLCPP_WARN_ONCE(get_logger(), "Odometry is not received. Waiting for odometry...");
      return false;
    }
    return true;
  }
  return false;
}

bool ReactionAnalyzerNode::set_route(
  const RouteState::ConstSharedPtr & route_state_ptr, const rclcpp::Time & current_time)
{
  if (route_state_ptr) {
    if (route_state_ptr->state != RouteState::SET) {
      // publish goal pose
      goal_pose_.header.stamp = current_time;
      goal_pose_.header.frame_id = "map";
      pub_goal_pose_->publish(goal_pose_);
      return false;
    }
    return true;
  }
  return false;
}

bool ReactionAnalyzerNode::check_ego_init_correctly(
  const PoseStamped::ConstSharedPtr & ground_truth_pose_ptr,
  const Odometry::ConstSharedPtr & odometry_ptr)
{
  if (!ground_truth_pose_ptr) {
    RCLCPP_WARN(
      get_logger(), "Ground truth pose is not received. Waiting for Ground truth pose...");
    return false;
  }
  if (!odometry_ptr) {
    RCLCPP_WARN(get_logger(), "Odometry is not received. Waiting for odometry...");
    return false;
  }

  constexpr double deviation_threshold = 0.1;
  const auto deviation = autoware::universe_utils::calcPoseDeviation(
    ground_truth_pose_ptr->pose, odometry_ptr->pose.pose);
  const bool is_position_initialized_correctly = deviation.longitudinal < deviation_threshold &&
                                                 deviation.lateral < deviation_threshold &&
                                                 deviation.yaw < deviation_threshold;
  if (!is_position_initialized_correctly) {
    RCLCPP_ERROR(
      get_logger(),
      "Deviation between ground truth position and ego position is high. Node is shutting "
      "down. Longitudinal deviation: %f, Lateral deviation: %f, Yaw deviation: %f",
      deviation.longitudinal, deviation.lateral, deviation.yaw);
    rclcpp::shutdown();
  }
  return true;
}

void ReactionAnalyzerNode::reset()
{
  if (test_iteration_count_ >= node_params_.test_iteration) {
    write_results(this, node_params_.output_file_path, node_running_mode_, pipeline_map_vector_);
    RCLCPP_INFO(get_logger(), "%zu Tests are finished. Node shutting down.", test_iteration_count_);
    rclcpp::shutdown();
    return;
  }
  // reset the variables
  is_initialization_requested = false;
  is_route_set_ = false;
  test_environment_init_time_ = std::nullopt;
  last_test_environment_init_request_time_ = std::nullopt;
  spawn_object_cmd_ = false;
  if (topic_publisher_ptr_) {
    topic_publisher_ptr_->reset();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  spawn_cmd_time_ = std::nullopt;
  subscriber_ptr_->reset();
  ego_initialized_ = false;
  RCLCPP_INFO(this->get_logger(), "Test - %zu is done, resetting..", test_iteration_count_);
}
}  // namespace reaction_analyzer

#include <rclcpp_components/register_node_macro.hpp>

#include <utility>

RCLCPP_COMPONENTS_REGISTER_NODE(reaction_analyzer::ReactionAnalyzerNode)
