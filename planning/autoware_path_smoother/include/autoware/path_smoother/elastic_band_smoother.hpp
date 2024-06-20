// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__PATH_SMOOTHER__ELASTIC_BAND_SMOOTHER_HPP_
#define AUTOWARE__PATH_SMOOTHER__ELASTIC_BAND_SMOOTHER_HPP_

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/path_smoother/common_structs.hpp"
#include "autoware/path_smoother/elastic_band.hpp"
#include "autoware/path_smoother/replan_checker.hpp"
#include "autoware/path_smoother/type_alias.hpp"
#include "autoware/universe_utils/ros/logger_level_configure.hpp"
#include "autoware/universe_utils/ros/polling_subscriber.hpp"
#include "rclcpp/rclcpp.hpp"

#include <autoware/universe_utils/ros/published_time_publisher.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::path_smoother
{
class ElasticBandSmoother : public rclcpp::Node
{
public:
  explicit ElasticBandSmoother(const rclcpp::NodeOptions & node_options);

  // NOTE: This is for the static_centerline_generator package which utilizes the following
  // instance.
  std::shared_ptr<EBPathSmoother> getElasticBandSmoother() const { return eb_path_smoother_ptr_; }

private:
  class DrivingDirectionChecker
  {
  public:
    bool isDrivingForward(const std::vector<PathPoint> & path_points)
    {
      const auto is_driving_forward = autoware::motion_utils::isDrivingForward(path_points);
      is_driving_forward_ = is_driving_forward ? is_driving_forward.value() : is_driving_forward_;
      return is_driving_forward_;
    }

  private:
    bool is_driving_forward_{true};
  };
  DrivingDirectionChecker driving_direction_checker_{};

  // argument variables
  mutable std::shared_ptr<TimeKeeper> time_keeper_ptr_{nullptr};

  // flags for some functions
  bool enable_debug_info_;

  // algorithms
  std::shared_ptr<EBPathSmoother> eb_path_smoother_ptr_{nullptr};
  std::shared_ptr<ReplanChecker> replan_checker_ptr_{nullptr};

  // parameters
  CommonParam common_param_{};
  EgoNearestParam ego_nearest_param_{};

  // variables for previous information
  std::shared_ptr<std::vector<TrajectoryPoint>> prev_optimized_traj_points_ptr_;

  // interface publisher
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<Path>::SharedPtr path_pub_;

  // interface subscriber
  rclcpp::Subscription<Path>::SharedPtr path_sub_;
  autoware::universe_utils::InterProcessPollingSubscriber<Odometry> odom_sub_{
    this, "~/input/odometry"};

  // debug publisher
  rclcpp::Publisher<Trajectory>::SharedPtr debug_extended_traj_pub_;
  rclcpp::Publisher<StringStamped>::SharedPtr debug_calculation_time_str_pub_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr debug_calculation_time_float_pub_;

  // parameter callback
  rcl_interfaces::msg::SetParametersResult onParam(
    const std::vector<rclcpp::Parameter> & parameters);
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  // subscriber callback function
  void onPath(const Path::ConstSharedPtr path_ptr);

  // reset functions
  void initializePlanning();
  void resetPreviousData();

  // main functions
  bool isDataReady(
    const Path & path, const Odometry::ConstSharedPtr ego_state_ptr, rclcpp::Clock clock) const;
  void applyInputVelocity(
    std::vector<TrajectoryPoint> & output_traj_points,
    const std::vector<TrajectoryPoint> & input_traj_points,
    const geometry_msgs::msg::Pose & ego_pose) const;
  std::vector<TrajectoryPoint> extendTrajectory(
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & optimized_points) const;

  std::unique_ptr<autoware::universe_utils::LoggerLevelConfigure> logger_configure_;

  std::unique_ptr<autoware::universe_utils::PublishedTimePublisher> published_time_publisher_;
};
}  // namespace autoware::path_smoother

#endif  // AUTOWARE__PATH_SMOOTHER__ELASTIC_BAND_SMOOTHER_HPP_
