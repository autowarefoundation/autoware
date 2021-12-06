// Copyright 2021 Tier IV, Inc.
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

#ifndef MOTION_VELOCITY_SMOOTHER__SMOOTHER__SMOOTHER_BASE_HPP_
#define MOTION_VELOCITY_SMOOTHER__SMOOTHER__SMOOTHER_BASE_HPP_

#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/trajectory/trajectory.hpp"
#include "motion_velocity_smoother/resample.hpp"
#include "motion_velocity_smoother/trajectory_utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"

#include "boost/optional.hpp"

#include <limits>
#include <vector>

namespace motion_velocity_smoother
{
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

class SmootherBase
{
public:
  struct BaseParam
  {
    double max_accel;   // max acceleration in planning [m/s2] > 0
    double min_decel;   // min deceleration in planning [m/s2] < 0
    double stop_decel;  // deceleration at a stop point [m/s2] <= 0
    double max_jerk;
    double min_jerk;
    double max_lateral_accel;            // max lateral acceleration [m/ss] > 0
    double min_curve_velocity;           // min velocity at curve [m/s]
    double decel_distance_before_curve;  // distance before slow down for lateral acc at a curve
    double decel_distance_after_curve;   // distance after slow down for lateral acc at a curve
    resampling::ResampleParam resample_param;
  };

  virtual ~SmootherBase() = default;
  virtual bool apply(
    const double initial_vel, const double initial_acc, const TrajectoryPoints & input,
    TrajectoryPoints & output, std::vector<TrajectoryPoints> & debug_trajectories) = 0;

  virtual boost::optional<TrajectoryPoints> resampleTrajectory(
    const TrajectoryPoints & input, const double v_current, const int closest_id) const = 0;

  virtual boost::optional<TrajectoryPoints> applyLateralAccelerationFilter(
    const TrajectoryPoints & input) const;

  double getMaxAccel() const;
  double getMinDecel() const;
  double getMaxJerk() const;
  double getMinJerk() const;

  void setParam(const BaseParam & param);

protected:
  BaseParam base_param_;
};
}  // namespace motion_velocity_smoother

#endif  // MOTION_VELOCITY_SMOOTHER__SMOOTHER__SMOOTHER_BASE_HPP_
