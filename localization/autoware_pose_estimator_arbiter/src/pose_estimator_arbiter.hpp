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

#ifndef POSE_ESTIMATOR_ARBITER_HPP_
#define POSE_ESTIMATOR_ARBITER_HPP_

#include "shared_data.hpp"
#include "stopper/base_stopper.hpp"
#include "switch_rule/base_switch_rule.hpp"

#include <autoware/universe_utils/ros/logger_level_configure.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <unordered_map>
#include <unordered_set>

namespace autoware::pose_estimator_arbiter
{
class PoseEstimatorArbiter : public rclcpp::Node
{
  using SetBool = std_srvs::srv::SetBool;
  using String = std_msgs::msg::String;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Image = sensor_msgs::msg::Image;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using HADMapBin = autoware_map_msgs::msg::LaneletMapBin;
  using InitializationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
  using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;

public:
  explicit PoseEstimatorArbiter(const rclcpp::NodeOptions & options);

private:
  // Set of running pose estimators specified by ros param `pose_sources`
  const std::unordered_set<PoseEstimatorType> running_estimator_list_;
  // Configuration to allow changing the log level by service
  const std::unique_ptr<autoware::universe_utils::LoggerLevelConfigure> logger_configure_;

  // This is passed to several modules (stoppers & rule) so that all modules can access common data
  // without passing them as arguments. Also, modules can register subscriber callbacks through
  // shared_data, avoiding the need to define duplicate subscribers for each module.
  std::shared_ptr<SharedData> shared_data_{nullptr};

  // Timer callback
  rclcpp::TimerBase::SharedPtr timer_;
  // Publishers
  rclcpp::Publisher<DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_debug_marker_array_;
  rclcpp::Publisher<String>::SharedPtr pub_debug_string_;
  // Subscribers for stoppers
  rclcpp::Subscription<Image>::SharedPtr sub_yabloc_input_;
  rclcpp::Subscription<Image>::SharedPtr sub_artag_input_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_ndt_input_;
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_eagleye_output_;
  // Subscribers for switch rules
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_localization_pose_cov_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_point_cloud_map_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_vector_map_;
  rclcpp::Subscription<InitializationState>::SharedPtr sub_initialization_state_;

  // Stoppers which enable/disable pose estimators
  std::unordered_map<PoseEstimatorType, stopper::BaseStopper::SharedPtr> stoppers_;
  // Abstract class to determine which pose estimator should be used
  std::shared_ptr<switch_rule::BaseSwitchRule> switch_rule_{nullptr};

  // Instruct all stopper to enable/disable
  void toggle_all(bool enabled);
  // Instruct each stopper to enable/disable
  void toggle_each(const std::unordered_map<PoseEstimatorType, bool> & toggle_list);

  // Load switching rule according to the condition
  void load_switch_rule();
  // Publish diagnostic messages
  void publish_diagnostics() const;

  // Timer callback
  void on_timer();
};
}  // namespace autoware::pose_estimator_arbiter

#endif  // POSE_ESTIMATOR_ARBITER_HPP_
