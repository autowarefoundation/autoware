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

#ifndef REACTION_ANALYZER_NODE_HPP_
#define REACTION_ANALYZER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <subscriber.hpp>
#include <topic_publisher.hpp>
#include <utils.hpp>

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <algorithm>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace reaction_analyzer
{
using autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_adapi_v1_msgs::msg::RouteState;
using autoware_adapi_v1_msgs::srv::ChangeOperationMode;
using autoware_internal_msgs::msg::PublishedTime;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;

struct NodeParams
{
  std::string running_mode;
  double timer_period;
  std::string output_file_path;
  size_t test_iteration;
  double spawn_time_after_init;
  double spawn_distance_threshold;
  PoseParams initial_pose;
  PoseParams goal_pose;
  EntityParams entity_params;
};

class ReactionAnalyzerNode : public rclcpp::Node
{
public:
  explicit ReactionAnalyzerNode(rclcpp::NodeOptions node_options);

  Odometry::ConstSharedPtr odometry_ptr_;

private:
  std::mutex mutex_;
  RunningMode node_running_mode_;

  // Parameters
  NodeParams node_params_;

  // Initialization Variables
  geometry_msgs::msg::Pose entity_pose_;
  geometry_msgs::msg::PoseWithCovarianceStamped init_pose_;
  geometry_msgs::msg::PoseStamped goal_pose_;

  // Subscribers
  rclcpp::Subscription<Odometry>::SharedPtr sub_kinematics_;
  rclcpp::Subscription<RouteState>::SharedPtr sub_route_state_;
  rclcpp::Subscription<LocalizationInitializationState>::SharedPtr sub_localization_init_state_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_;
  rclcpp::Subscription<PoseStamped>::SharedPtr
    sub_ground_truth_pose_;  // Only for perception_planning mode

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initial_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;

  // Variables
  std::vector<PipelineMap> pipeline_map_vector_;
  std::optional<rclcpp::Time> last_test_environment_init_request_time_;
  std::optional<rclcpp::Time> test_environment_init_time_;
  std::optional<rclcpp::Time> spawn_cmd_time_;
  std::atomic<bool> spawn_object_cmd_{false};
  std::atomic<bool> ego_initialized_{false};
  bool is_initialization_requested{false};
  bool is_route_set_{false};
  size_t test_iteration_count_{0};
  visualization_msgs::msg::Marker entity_debug_marker_;

  // Functions
  void init_analyzer_variables();
  void init_test_env(
    const LocalizationInitializationState::ConstSharedPtr & initialization_state_ptr,
    const RouteState::ConstSharedPtr & route_state_ptr,
    const OperationModeState::ConstSharedPtr & operation_mode_ptr,
    const PoseStamped::ConstSharedPtr & ground_truth_pose_ptr,
    const Odometry::ConstSharedPtr & odometry_ptr);
  bool init_ego(
    const LocalizationInitializationState::ConstSharedPtr & initialization_state_ptr,
    const Odometry::ConstSharedPtr & odometry_ptr, const rclcpp::Time & current_time);
  bool set_route(
    const RouteState::ConstSharedPtr & route_state_ptr, const rclcpp::Time & current_time);
  bool check_ego_init_correctly(
    const PoseStamped::ConstSharedPtr & ground_truth_pose_ptr,
    const Odometry::ConstSharedPtr & odometry_ptr);
  void call_operation_mode_service_without_response();
  void spawn_obstacle(const geometry_msgs::msg::Point & ego_pose);
  void calculate_results(
    const std::map<std::string, subscriber::MessageBufferVariant> & message_buffers,
    const rclcpp::Time & spawn_cmd_time);
  void on_timer();
  void reset();

  // Callbacks
  void on_vehicle_pose(Odometry::ConstSharedPtr msg_ptr);
  void on_localization_initialization_state(
    LocalizationInitializationState::ConstSharedPtr msg_ptr);
  void on_route_state(RouteState::ConstSharedPtr msg_ptr);
  void on_operation_mode_state(OperationModeState::ConstSharedPtr msg_ptr);
  void on_ground_truth_pose(PoseStamped::ConstSharedPtr msg_ptr);

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Client
  rclcpp::Client<ChangeOperationMode>::SharedPtr client_change_to_autonomous_;

  // Pointers
  std::unique_ptr<topic_publisher::TopicPublisher> topic_publisher_ptr_;
  std::unique_ptr<subscriber::SubscriberBase> subscriber_ptr_;
  LocalizationInitializationState::ConstSharedPtr initialization_state_ptr_;
  RouteState::ConstSharedPtr current_route_state_ptr_;
  OperationModeState::ConstSharedPtr operation_mode_ptr_;
  PoseStamped::ConstSharedPtr ground_truth_pose_ptr_;
};
}  // namespace reaction_analyzer

#endif  // REACTION_ANALYZER_NODE_HPP_
