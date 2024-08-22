// Copyright 2023 Autoware Foundation
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

#include "pose_estimator_arbiter.hpp"
#include "pose_estimator_type.hpp"
#include "stopper/stopper_artag.hpp"
#include "stopper/stopper_eagleye.hpp"
#include "stopper/stopper_ndt.hpp"
#include "stopper/stopper_yabloc.hpp"
#include "switch_rule/enable_all_rule.hpp"

#include <magic_enum.hpp>

namespace autoware::pose_estimator_arbiter
{
// Parses ros param to get the estimator set that is running
static std::unordered_set<PoseEstimatorType> parse_estimator_name_args(
  const std::vector<std::string> & arg, const rclcpp::Logger & logger)
{
  std::unordered_set<PoseEstimatorType> running_estimator_list;
  for (const auto & estimator_name : arg) {
    const auto estimator = magic_enum::enum_cast<PoseEstimatorType>(estimator_name);

    if (estimator.has_value()) {
      running_estimator_list.insert(estimator.value());
    } else {
      RCLCPP_ERROR_STREAM(logger, "invalid pose_estimator_name is specified: " << estimator_name);
    }
  }

  return running_estimator_list;
}

PoseEstimatorArbiter::PoseEstimatorArbiter(const rclcpp::NodeOptions & options)
: rclcpp::Node("pose_estimator_arbiter", options),
  running_estimator_list_(parse_estimator_name_args(
    declare_parameter<std::vector<std::string>>("pose_sources"), get_logger())),
  logger_configure_(std::make_unique<autoware::universe_utils::LoggerLevelConfigure>(this))
{
  // Shared data
  shared_data_ = std::make_shared<SharedData>();

  // Publisher
  pub_diag_ = create_publisher<DiagnosticArray>("/diagnostics", 10);
  pub_debug_string_ = create_publisher<String>("~/debug/string", 10);
  pub_debug_marker_array_ = create_publisher<MarkerArray>("~/debug/marker_array", 10);

  // Define function to get running pose_estimator
  const std::set<PoseEstimatorType> running_estimator_set(
    running_estimator_list_.begin(), running_estimator_list_.end());
  const auto is_running = [running_estimator_set](const PoseEstimatorType type) -> bool {
    return running_estimator_set.count(type) != 0;
  };

  // QoS
  const rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();
  const rclcpp::QoS latch_qos = rclcpp::QoS(1).transient_local().reliable();

  // Create stoppers & subscribers
  if (is_running(PoseEstimatorType::ndt)) {
    stoppers_.emplace(
      PoseEstimatorType::ndt, std::make_shared<stopper::StopperNdt>(this, shared_data_));
    sub_ndt_input_ = create_subscription<PointCloud2>(
      "~/input/ndt/pointcloud", sensor_qos, shared_data_->ndt_input_points.create_callback());
  }
  if (is_running(PoseEstimatorType::yabloc)) {
    stoppers_.emplace(
      PoseEstimatorType::yabloc, std::make_shared<stopper::StopperYabLoc>(this, shared_data_));
    sub_yabloc_input_ = create_subscription<Image>(
      "~/input/yabloc/image", sensor_qos, shared_data_->yabloc_input_image.create_callback());
  }
  if (is_running(PoseEstimatorType::eagleye)) {
    stoppers_.emplace(
      PoseEstimatorType::eagleye, std::make_shared<stopper::StopperEagleye>(this, shared_data_));
    sub_eagleye_output_ = create_subscription<PoseCovStamped>(
      "~/input/eagleye/pose_with_covariance", 5, /* this is not sensor topic */
      shared_data_->eagleye_output_pose_cov.create_callback());
  }
  if (is_running(PoseEstimatorType::artag)) {
    stoppers_.emplace(
      PoseEstimatorType::artag, std::make_shared<stopper::StopperArTag>(this, shared_data_));
    sub_artag_input_ = create_subscription<Image>(
      "~/input/artag/image", sensor_qos, shared_data_->artag_input_image.create_callback());
  }

  // Subscribers for switch rule
  {
    sub_localization_pose_cov_ = create_subscription<PoseCovStamped>(
      "~/input/pose_with_covariance", 5, shared_data_->localization_pose_cov.create_callback());
    sub_point_cloud_map_ = create_subscription<PointCloud2>(
      "~/input/pointcloud_map", latch_qos, shared_data_->point_cloud_map.create_callback());
    sub_vector_map_ = create_subscription<HADMapBin>(
      "~/input/vector_map", latch_qos, shared_data_->vector_map.create_callback());
    sub_initialization_state_ = create_subscription<InitializationState>(
      "~/input/initialization_state", latch_qos,
      shared_data_->initialization_state.create_callback());
  }

  // Load switching rule
  load_switch_rule();

  // Timer callback
  auto on_timer_callback = std::bind(&PoseEstimatorArbiter::on_timer, this);
  timer_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Rate(1).period(), std::move(on_timer_callback));

  // Enable all pose estimators at the first
  toggle_all(true);
}

void PoseEstimatorArbiter::load_switch_rule()
{
  // NOTE: In the future, some rule will be laid below
  RCLCPP_INFO_STREAM(get_logger(), "load default switching rule");
  switch_rule_ =
    std::make_shared<switch_rule::EnableAllRule>(*this, running_estimator_list_, shared_data_);
}

void PoseEstimatorArbiter::toggle_each(
  const std::unordered_map<PoseEstimatorType, bool> & toggle_list)
{
  for (auto [type, stopper] : stoppers_) {
    RCLCPP_DEBUG_STREAM(
      get_logger(), magic_enum::enum_name(type) << " : " << std::boolalpha << toggle_list.at(type));

    // If the rule implementation is perfect, toggle_list should contains all pose_estimator_type.
    if (toggle_list.count(type) == 0) {
      RCLCPP_ERROR_STREAM(
        get_logger(), magic_enum::enum_name(type) << " is not included in toggle_list.");
      continue;
    }

    // Enable or disable according to toggle_list
    if (toggle_list.at(type)) {
      stopper->enable();
    } else {
      stopper->disable();
    }
  }
}

void PoseEstimatorArbiter::toggle_all(bool enabled)
{
  // Create toggle_list
  std::unordered_map<PoseEstimatorType, bool> toggle_list;
  for (auto [type, stopper] : stoppers_) {
    toggle_list.emplace(type, enabled);
  }

  // Apply toggle_list
  toggle_each(toggle_list);
}

void PoseEstimatorArbiter::publish_diagnostics() const
{
  diagnostic_msgs::msg::DiagnosticStatus diag_status;

  // Temporary implementation
  {
    diag_status.name = "localization: " + std::string(this->get_name());
    diag_status.hardware_id = this->get_name();

    diag_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diag_status.message = "OK";

    // TODO(KYabuuchi) : Add more details
    diagnostic_msgs::msg::KeyValue key_value_msg;
    key_value_msg.key = "state";
    key_value_msg.value = "Further details have not been implemented yet.";
    diag_status.values.push_back(key_value_msg);
  }

  DiagnosticArray diag_msg;
  diag_msg.header.stamp = this->now();
  diag_msg.status.push_back(diag_status);

  pub_diag_->publish(diag_msg);
}

void PoseEstimatorArbiter::on_timer()
{
  // Toggle each stopper status
  if (switch_rule_) {
    const auto toggle_list = switch_rule_->update();
    toggle_each(toggle_list);

    // Publish std_msg::String for debug
    pub_debug_string_->publish(String().set__data(switch_rule_->debug_string()));
    // Publish visualization_msgs::MarkerArray for debug
    pub_debug_marker_array_->publish(switch_rule_->debug_marker_array());

  } else {
    RCLCPP_WARN_STREAM(
      get_logger(), "switch_rule is not activated. Therefore, enable all pose_estimators");
    toggle_all(true);
  }

  // Publish diagnostic results periodically
  publish_diagnostics();
}

}  // namespace autoware::pose_estimator_arbiter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pose_estimator_arbiter::PoseEstimatorArbiter)
