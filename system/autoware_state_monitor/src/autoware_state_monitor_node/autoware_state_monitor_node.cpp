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

#include "autoware_state_monitor/autoware_state_monitor_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
template <class Config>
std::vector<Config> getConfigs(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr interface,
  const std::string & config_namespace)
{
  std::string names_key = config_namespace + ".names";
  interface->declare_parameter(names_key, rclcpp::PARAMETER_STRING_ARRAY);
  std::vector<std::string> config_names = interface->get_parameter(names_key).as_string_array();

  std::vector<Config> configs;
  configs.reserve(config_names.size());

  for (auto config_name : config_names) {
    configs.emplace_back(interface, config_namespace + ".configs." + config_name, config_name);
  }

  return configs;
}

double calcTopicRate(const std::deque<rclcpp::Time> & topic_received_time_buffer)
{
  assert(topic_received_time_buffer.size() >= 2);

  const auto & buf = topic_received_time_buffer;
  const auto time_diff = buf.back() - buf.front();

  return static_cast<double>(buf.size() - 1) / time_diff.seconds();
}

geometry_msgs::msg::PoseStamped::SharedPtr getCurrentPose(const tf2_ros::Buffer & tf_buffer)
{
  geometry_msgs::msg::TransformStamped tf_current_pose;

  try {
    tf_current_pose = tf_buffer.lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    return nullptr;
  }

  auto p = std::make_shared<geometry_msgs::msg::PoseStamped>();
  p->header = tf_current_pose.header;
  p->pose.orientation = tf_current_pose.transform.rotation;
  p->pose.position.x = tf_current_pose.transform.translation.x;
  p->pose.position.y = tf_current_pose.transform.translation.y;
  p->pose.position.z = tf_current_pose.transform.translation.z;

  return p;
}

}  // namespace

void AutowareStateMonitorNode::onAutowareEngage(
  const autoware_auto_vehicle_msgs::msg::Engage::ConstSharedPtr msg)
{
  state_input_.autoware_engage = msg;
}

void AutowareStateMonitorNode::onVehicleControlMode(
  const autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr msg)
{
  state_input_.control_mode_ = msg;
}

void AutowareStateMonitorNode::onRoute(
  const autoware_auto_planning_msgs::msg::HADMapRoute::ConstSharedPtr msg)
{
  state_input_.route = msg;

  // Get goal pose
  {
    geometry_msgs::msg::Pose::SharedPtr p = std::make_shared<geometry_msgs::msg::Pose>();
    *p = msg->goal_pose;
    state_input_.goal_pose = geometry_msgs::msg::Pose::ConstSharedPtr(p);
  }

  if (disengage_on_route_ && isEngaged()) {
    RCLCPP_INFO(this->get_logger(), "new route received and disengage Autoware");
    setDisengage();
  }
}

void AutowareStateMonitorNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  state_input_.odometry = msg;

  state_input_.odometry_buffer.push_back(msg);

  // Delete old data in buffer
  while (true) {
    const auto time_diff = rclcpp::Time(msg->header.stamp) -
                           rclcpp::Time(state_input_.odometry_buffer.front()->header.stamp);

    if (time_diff.seconds() < state_param_.th_stopped_time_sec) {
      break;
    }

    state_input_.odometry_buffer.pop_front();
  }
}

bool AutowareStateMonitorNode::onShutdownService(
  const std::shared_ptr<rmw_request_id_t> request_header,
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request_header;
  state_input_.is_finalizing = true;

  const auto t_start = this->get_clock()->now();
  constexpr double timeout = 3.0;
  while (rclcpp::ok()) {
    // rclcpp::spin_some(this->get_node_base_interface());

    if (state_machine_->getCurrentState() == AutowareState::Finalizing) {
      response->success = true;
      response->message = "Shutdown Autoware.";
      return true;
    }

    if ((this->get_clock()->now() - t_start).seconds() > timeout) {
      response->success = false;
      response->message = "Shutdown timeout.";
      return true;
    }

    rclcpp::Rate(10.0).sleep();
  }

  response->success = false;
  response->message = "Shutdown failure.";
  return true;
}

bool AutowareStateMonitorNode::onResetRouteService(
  [[maybe_unused]] const std::shared_ptr<rmw_request_id_t> request_header,
  [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (state_machine_->getCurrentState() != AutowareState::WaitingForEngage) {
    response->success = false;
    response->message = "Reset route can be accepted only under WaitingForEngage.";
    return true;
  }

  state_input_.is_route_reset_required = true;

  const auto t_start = this->now();
  constexpr double timeout = 3.0;
  while (rclcpp::ok()) {
    if (state_machine_->getCurrentState() == AutowareState::WaitingForRoute) {
      state_input_.is_route_reset_required = false;
      response->success = true;
      response->message = "Reset route.";
      return true;
    }

    if ((this->now() - t_start).seconds() > timeout) {
      response->success = false;
      response->message = "Reset route timeout.";
      return true;
    }

    rclcpp::Rate(10.0).sleep();
  }

  response->success = false;
  response->message = "Reset route failure.";
  return true;
}

void AutowareStateMonitorNode::onTimer()
{
  // Prepare state input
  state_input_.current_pose = getCurrentPose(tf_buffer_);
  if (state_input_.current_pose == nullptr) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */,
      "Fail lookupTransform base_link to map");
  }

  state_input_.topic_stats = getTopicStats();
  state_input_.param_stats = getParamStats();
  state_input_.tf_stats = getTfStats();
  state_input_.current_time = this->now();
  // Update state
  const auto prev_autoware_state = state_machine_->getCurrentState();
  const auto autoware_state = state_machine_->updateState(state_input_);

  if (autoware_state != prev_autoware_state) {
    RCLCPP_INFO(
      this->get_logger(), "state changed: %i -> %i", toMsg(prev_autoware_state),
      toMsg(autoware_state));
  }

  // Disengage on event
  if (disengage_on_goal_ && isEngaged() && autoware_state == AutowareState::ArrivedGoal) {
    RCLCPP_INFO(this->get_logger(), "arrived goal and disengage Autoware");
    setDisengage();
  }

  // Publish state message
  {
    autoware_auto_system_msgs::msg::AutowareState autoware_state_msg;
    autoware_state_msg.state = toMsg(autoware_state);

    pub_autoware_state_->publish(autoware_state_msg);
  }

  // Publish diag message
  updater_.force_update();
}

// TODO(jilaada): Use generic subscription base
void AutowareStateMonitorNode::onTopic(
  [[maybe_unused]] const std::shared_ptr<rclcpp::SerializedMessage> msg,
  const std::string & topic_name)
{
  const auto now = this->now();

  auto & buf = topic_received_time_buffer_.at(topic_name);
  buf.push_back(now);

  constexpr size_t topic_received_time_buffer_size = 10;
  if (buf.size() > topic_received_time_buffer_size) {
    buf.pop_front();
  }
}

void AutowareStateMonitorNode::registerTopicCallback(
  const std::string & topic_name, const std::string & topic_type, const bool transient_local,
  const bool best_effort)
{
  // Initialize buffer
  topic_received_time_buffer_[topic_name] = {};

  // Register callback
  using Callback = std::function<void(const std::shared_ptr<rclcpp::SerializedMessage>)>;
  const auto callback = static_cast<Callback>(
    std::bind(&AutowareStateMonitorNode::onTopic, this, std::placeholders::_1, topic_name));
  auto qos = rclcpp::QoS{1};
  if (transient_local) {
    qos.transient_local();
  }
  if (best_effort) {
    qos.best_effort();
  }

  auto subscriber_option = rclcpp::SubscriptionOptions();
  subscriber_option.callback_group = callback_group_subscribers_;

  sub_topic_map_[topic_name] =
    this->create_generic_subscription(topic_name, topic_type, qos, callback, subscriber_option);
}

TopicStats AutowareStateMonitorNode::getTopicStats() const
{
  TopicStats topic_stats;
  topic_stats.checked_time = this->now();

  for (const auto & topic_config : topic_configs_) {
    // Alias
    const auto & buf = topic_received_time_buffer_.at(topic_config.name);

    // Check at least once received
    if (buf.empty()) {
      topic_stats.non_received_list.push_back(topic_config);
      continue;
    }

    // Check timeout
    const auto last_received_time = buf.back();
    const auto time_diff = (topic_stats.checked_time - last_received_time).seconds();
    const auto is_timeout = (topic_config.timeout != 0) && (time_diff > topic_config.timeout);
    if (is_timeout) {
      topic_stats.timeout_list.emplace_back(topic_config, last_received_time);
      continue;
    }

    // Check topic rate
    if (!is_timeout && buf.size() >= 2) {
      const auto topic_rate = calcTopicRate(buf);
      if (topic_config.warn_rate != 0 && topic_rate < topic_config.warn_rate) {
        topic_stats.slow_rate_list.emplace_back(topic_config, topic_rate);
        continue;
      }
    }

    // No error
    topic_stats.ok_list.push_back(topic_config);
  }

  return topic_stats;
}

ParamStats AutowareStateMonitorNode::getParamStats() const
{
  ParamStats param_stats;
  param_stats.checked_time = this->now();

  for (const auto & param_config : param_configs_) {
    const bool result = this->has_parameter("param_configs.configs." + param_config.name);
    if (!result) {
      param_stats.non_set_list.push_back(param_config);
      continue;
    }

    // No error
    param_stats.ok_list.push_back(param_config);
  }

  return param_stats;
}

TfStats AutowareStateMonitorNode::getTfStats() const
{
  TfStats tf_stats;
  tf_stats.checked_time = this->now();

  for (const auto & tf_config : tf_configs_) {
    try {
      const auto transform =
        tf_buffer_.lookupTransform(tf_config.from, tf_config.to, tf2::TimePointZero);

      const auto last_received_time = transform.header.stamp;
      const auto time_diff = (tf_stats.checked_time - last_received_time).seconds();
      if (time_diff > tf_config.timeout) {
        tf_stats.timeout_list.emplace_back(tf_config, last_received_time);
        continue;
      }
    } catch (tf2::TransformException & ex) {
      tf_stats.non_received_list.push_back(tf_config);
      continue;
    }

    // No error
    tf_stats.ok_list.push_back(tf_config);
  }

  return tf_stats;
}

bool AutowareStateMonitorNode::isEngaged()
{
  if (!state_input_.autoware_engage) {
    return false;
  }

  return state_input_.autoware_engage->engage;
}

void AutowareStateMonitorNode::setDisengage()
{
  autoware_auto_vehicle_msgs::msg::Engage msg;
  msg.stamp = this->now();
  msg.engage = false;
  pub_autoware_engage_->publish(msg);
}

AutowareStateMonitorNode::AutowareStateMonitorNode()
: Node("autoware_state_monitor"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  updater_(this)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  // Parameter
  update_rate_ = this->declare_parameter("update_rate", 10.0);
  disengage_on_route_ = this->declare_parameter("disengage_on_route", true);
  disengage_on_goal_ = this->declare_parameter("disengage_on_goal", true);

  // Parameter for StateMachine
  state_param_.th_arrived_distance_m = this->declare_parameter("th_arrived_distance_m", 1.0);
  state_param_.th_arrived_angle =
    this->declare_parameter("th_arrived_angle_deg", autoware_utils::deg2rad(45.0));
  state_param_.th_stopped_time_sec = this->declare_parameter("th_stopped_time_sec", 1.0);
  state_param_.th_stopped_velocity_mps = this->declare_parameter("th_stopped_velocity_mps", 0.01);

  // State Machine
  state_machine_ = std::make_shared<StateMachine>(state_param_);

  // Config
  topic_configs_ = getConfigs<TopicConfig>(this->get_node_parameters_interface(), "topic_configs");
  tf_configs_ = getConfigs<TfConfig>(this->get_node_parameters_interface(), "tf_configs");

  // Callback Groups
  callback_group_subscribers_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group_services_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subscriber_option = rclcpp::SubscriptionOptions();
  subscriber_option.callback_group = callback_group_subscribers_;

  // Topic Callback
  for (const auto & topic_config : topic_configs_) {
    registerTopicCallback(
      topic_config.name, topic_config.type, topic_config.transient_local, topic_config.best_effort);
  }

  // Subscriber
  sub_autoware_engage_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::Engage>(
    "input/autoware_engage", 1, std::bind(&AutowareStateMonitorNode::onAutowareEngage, this, _1),
    subscriber_option);
  sub_control_mode_ = this->create_subscription<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
    "input/control_mode", 1, std::bind(&AutowareStateMonitorNode::onVehicleControlMode, this, _1),
    subscriber_option);
  sub_route_ = this->create_subscription<autoware_auto_planning_msgs::msg::HADMapRoute>(
    "input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&AutowareStateMonitorNode::onRoute, this, _1), subscriber_option);
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "input/odometry", 100, std::bind(&AutowareStateMonitorNode::onOdometry, this, _1),
    subscriber_option);

  // Service
  srv_shutdown_ = this->create_service<std_srvs::srv::Trigger>(
    "service/shutdown", std::bind(&AutowareStateMonitorNode::onShutdownService, this, _1, _2, _3),
    rmw_qos_profile_services_default, callback_group_services_);
  srv_reset_route_ = this->create_service<std_srvs::srv::Trigger>(
    "service/reset_route",
    std::bind(&AutowareStateMonitorNode::onResetRouteService, this, _1, _2, _3),
    rmw_qos_profile_services_default, callback_group_services_);

  // Publisher
  pub_autoware_state_ = this->create_publisher<autoware_auto_system_msgs::msg::AutowareState>(
    "output/autoware_state", 1);
  pub_autoware_engage_ =
    this->create_publisher<autoware_auto_vehicle_msgs::msg::Engage>("output/autoware_engage", 1);

  // Diagnostic Updater
  setupDiagnosticUpdater();

  // Wait for first topics
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // Timer
  auto timer_callback = std::bind(&AutowareStateMonitorNode::onTimer, this);
  auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / update_rate_));

  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, callback_group_subscribers_);
}
