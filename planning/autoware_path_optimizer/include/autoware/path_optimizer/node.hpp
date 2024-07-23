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

#ifndef AUTOWARE__PATH_OPTIMIZER__NODE_HPP_
#define AUTOWARE__PATH_OPTIMIZER__NODE_HPP_

#include "autoware/motion_utils/trajectory/trajectory.hpp"
#include "autoware/path_optimizer/common_structs.hpp"
#include "autoware/path_optimizer/mpt_optimizer.hpp"
#include "autoware/path_optimizer/replan_checker.hpp"
#include "autoware/path_optimizer/type_alias.hpp"
#include "autoware/universe_utils/ros/logger_level_configure.hpp"
#include "autoware/universe_utils/ros/polling_subscriber.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"
#include "autoware/universe_utils/system/time_keeper.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"

#include <autoware/universe_utils/ros/published_time_publisher.hpp>
#include <rclcpp/publisher.hpp>

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::path_optimizer
{
class PathOptimizer : public rclcpp::Node
{
public:
  explicit PathOptimizer(const rclcpp::NodeOptions & node_options);

  // NOTE: This is for the static_centerline_generator package which utilizes the following
  // instance.
  std::shared_ptr<MPTOptimizer> getMPTOptimizer() const { return mpt_optimizer_ptr_; }

  // private:
protected:  // for the static_centerline_generator package
  // TODO(murooka) move this node to common
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
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_{};
  mutable std::shared_ptr<DebugData> debug_data_ptr_{nullptr};
  mutable std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper_{nullptr};

  // flags for some functions
  bool enable_pub_debug_marker_;
  bool enable_pub_extra_debug_marker_;
  bool enable_debug_info_;
  bool enable_outside_drivable_area_stop_;
  bool enable_skip_optimization_;
  bool enable_reset_prev_optimization_;
  bool use_footprint_polygon_for_outside_drivable_area_check_;

  // core algorithms
  std::shared_ptr<ReplanChecker> replan_checker_ptr_{nullptr};
  std::shared_ptr<MPTOptimizer> mpt_optimizer_ptr_{nullptr};

  // parameters
  TrajectoryParam traj_param_{};
  EgoNearestParam ego_nearest_param_{};

  // interface publisher
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr virtual_wall_pub_;

  // interface subscriber
  rclcpp::Subscription<Path>::SharedPtr path_sub_;
  autoware::universe_utils::InterProcessPollingSubscriber<Odometry> ego_odom_sub_{
    this, "~/input/odometry"};

  // debug publisher
  rclcpp::Publisher<Trajectory>::SharedPtr debug_extended_traj_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_markers_pub_;
  rclcpp::Publisher<StringStamped>::SharedPtr debug_calculation_time_str_pub_;
  rclcpp::Publisher<Float64Stamped>::SharedPtr debug_calculation_time_float_pub_;
  rclcpp::Publisher<autoware::universe_utils::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;

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
  bool checkInputPath(const Path & path, rclcpp::Clock clock) const;
  PlannerData createPlannerData(
    const Path & path, const Odometry::ConstSharedPtr ego_odom_ptr) const;
  std::vector<TrajectoryPoint> generateOptimizedTrajectory(const PlannerData & planner_data);
  std::vector<TrajectoryPoint> extendTrajectory(
    const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & optimized_points) const;
  void publishDebugData(const Header & header) const;

  // functions in generateOptimizedTrajectory
  std::vector<TrajectoryPoint> optimizeTrajectory(const PlannerData & planner_data);
  std::vector<TrajectoryPoint> getPrevOptimizedTrajectory(
    const std::vector<TrajectoryPoint> & traj_points) const;
  void applyInputVelocity(
    std::vector<TrajectoryPoint> & output_traj_points,
    const std::vector<TrajectoryPoint> & input_traj_points,
    const geometry_msgs::msg::Pose & ego_pose) const;
  void insertZeroVelocityOutsideDrivableArea(
    const PlannerData & planner_data, std::vector<TrajectoryPoint> & traj_points) const;
  void publishVirtualWall(const geometry_msgs::msg::Pose & stop_pose) const;
  void publishDebugMarkerOfOptimization(const std::vector<TrajectoryPoint> & traj_points) const;

private:
  double vehicle_stop_margin_outside_drivable_area_;

  std::unique_ptr<autoware::universe_utils::LoggerLevelConfigure> logger_configure_;

  std::unique_ptr<autoware::universe_utils::PublishedTimePublisher> published_time_publisher_;

  autoware::universe_utils::StopWatch<std::chrono::milliseconds> stop_watch_;
};
}  // namespace autoware::path_optimizer

#endif  // AUTOWARE__PATH_OPTIMIZER__NODE_HPP_
