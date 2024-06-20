// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_NODE_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_NODE_HPP_

#include "autoware/trajectory_follower_base/lateral_controller_base.hpp"
#include "autoware/trajectory_follower_base/longitudinal_controller_base.hpp"
#include "autoware/trajectory_follower_node/visibility_control.hpp"
#include "autoware/universe_utils/ros/logger_level_configure.hpp"
#include "autoware/universe_utils/ros/polling_subscriber.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware/universe_utils/ros/published_time_publisher.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_control_msgs/msg/longitudinal.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tier4_debug_msgs/msg/float64_stamped.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::motion::control
{
using trajectory_follower::LateralOutput;
using trajectory_follower::LongitudinalOutput;
namespace trajectory_follower_node
{

using autoware::universe_utils::StopWatch;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using tier4_debug_msgs::msg::Float64Stamped;

namespace trajectory_follower = ::autoware::motion::control::trajectory_follower;

/// \classController
/// \brief The node class used for generating longitudinal control commands (velocity/acceleration)
class TRAJECTORY_FOLLOWER_PUBLIC Controller : public rclcpp::Node
{
public:
  explicit Controller(const rclcpp::NodeOptions & node_options);
  virtual ~Controller() {}

private:
  rclcpp::TimerBase::SharedPtr timer_control_;
  double timeout_thr_sec_;
  boost::optional<LongitudinalOutput> longitudinal_output_{boost::none};

  std::shared_ptr<diagnostic_updater::Updater> diag_updater_ =
    std::make_shared<diagnostic_updater::Updater>(
      this);  // Diagnostic updater for publishing diagnostic data.

  std::shared_ptr<trajectory_follower::LongitudinalControllerBase> longitudinal_controller_;
  std::shared_ptr<trajectory_follower::LateralControllerBase> lateral_controller_;

  // Subscribers
  autoware::universe_utils::InterProcessPollingSubscriber<autoware_planning_msgs::msg::Trajectory>
    sub_ref_path_{this, "~/input/reference_trajectory"};

  autoware::universe_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry> sub_odometry_{
    this, "~/input/current_odometry"};

  autoware::universe_utils::InterProcessPollingSubscriber<
    autoware_vehicle_msgs::msg::SteeringReport>
    sub_steering_{this, "~/input/current_steering"};

  autoware::universe_utils::InterProcessPollingSubscriber<
    geometry_msgs::msg::AccelWithCovarianceStamped>
    sub_accel_{this, "~/input/current_accel"};

  autoware::universe_utils::InterProcessPollingSubscriber<OperationModeState> sub_operation_mode_{
    this, "~/input/current_operation_mode"};

  // Publishers
  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_pub_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr pub_processing_time_lat_ms_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr pub_processing_time_lon_ms_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;

  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr current_trajectory_ptr_;
  nav_msgs::msg::Odometry::ConstSharedPtr current_odometry_ptr_;
  autoware_vehicle_msgs::msg::SteeringReport::ConstSharedPtr current_steering_ptr_;
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr current_accel_ptr_;
  OperationModeState::ConstSharedPtr current_operation_mode_ptr_;

  enum class LateralControllerMode {
    INVALID = 0,
    MPC = 1,
    PURE_PURSUIT = 2,
  };
  enum class LongitudinalControllerMode {
    INVALID = 0,
    PID = 1,
  };

  /**
   * @brief compute control command, and publish periodically
   */
  boost::optional<trajectory_follower::InputData> createInputData(rclcpp::Clock & clock);
  void callbackTimerControl();
  bool processData(rclcpp::Clock & clock);
  bool isTimeOut(const LongitudinalOutput & lon_out, const LateralOutput & lat_out);
  LateralControllerMode getLateralControllerMode(const std::string & algorithm_name) const;
  LongitudinalControllerMode getLongitudinalControllerMode(
    const std::string & algorithm_name) const;
  void publishDebugMarker(
    const trajectory_follower::InputData & input_data,
    const trajectory_follower::LateralOutput & lat_out) const;

  std::unique_ptr<autoware::universe_utils::LoggerLevelConfigure> logger_configure_;

  std::unique_ptr<autoware::universe_utils::PublishedTimePublisher> published_time_publisher_;

  void publishProcessingTime(
    const double t_ms, const rclcpp::Publisher<Float64Stamped>::SharedPtr pub);
  StopWatch<std::chrono::milliseconds> stop_watch_;

  static constexpr double logger_throttle_interval = 5000;
};
}  // namespace trajectory_follower_node
}  // namespace autoware::motion::control

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_NODE__CONTROLLER_NODE_HPP_
