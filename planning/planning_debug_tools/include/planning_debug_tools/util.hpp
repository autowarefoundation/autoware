// Copyright 2022 Tier IV, Inc.
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

#ifndef PLANNING_DEBUG_TOOLS__UTIL_HPP_
#define PLANNING_DEBUG_TOOLS__UTIL_HPP_

#include "motion_utils/trajectory/trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/geometry/path_with_lane_id_geometry.hpp"

#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

#include <vector>

namespace planning_debug_tools
{

using autoware_auto_planning_msgs::msg::PathPoint;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::getPoint;
using tier4_autoware_utils::getRPY;

double getVelocity(const PathPoint & p) { return p.longitudinal_velocity_mps; }
double getVelocity(const PathPointWithLaneId & p) { return p.point.longitudinal_velocity_mps; }
double getVelocity(const TrajectoryPoint & p) { return p.longitudinal_velocity_mps; }

double getYaw(const PathPoint & p) { return getRPY(p.pose.orientation).z; }
double getYaw(const PathPointWithLaneId & p) { return getRPY(p.point.pose.orientation).z; }
double getYaw(const TrajectoryPoint & p) { return getRPY(p.pose.orientation).z; }

template <class T>
inline std::vector<double> getYawArray(const T & points)
{
  std::vector<double> yaw_arr;
  for (const auto & p : points) {
    yaw_arr.push_back(getYaw(p));
  }
  return yaw_arr;
}
template <class T>
inline std::vector<double> getVelocityArray(const T & points)
{
  std::vector<double> v_arr;
  for (const auto & p : points) {
    v_arr.push_back(getVelocity(p));
  }
  return v_arr;
}

template <class T>
inline std::vector<double> getAccelerationArray(const T & points)
{
  std::vector<double> segment_wise_a_arr;
  for (size_t i = 0; i < points.size() - 1; ++i) {
    const auto & prev_point = points.at(i);
    const auto & next_point = points.at(i + 1);

    const double delta_s = tier4_autoware_utils::calcDistance2d(prev_point, next_point);
    if (delta_s == 0.0) {
      segment_wise_a_arr.push_back(0.0);
    } else {
      const double prev_vel = getVelocity(prev_point);
      const double next_vel = getVelocity(next_point);

      const double acc = (std::pow(next_vel, 2) - std::pow(prev_vel, 2)) / 2.0 / delta_s;

      segment_wise_a_arr.push_back(acc);
    }
  }

  std::vector<double> point_wise_a_arr;
  for (size_t i = 0; i < points.size(); ++i) {
    if (i == 0) {
      point_wise_a_arr.push_back(segment_wise_a_arr.at(i));
    } else if (i == points.size() - 1 || i == points.size() - 2) {
      // Ignore the last two acceleration values which are negative infinity since the path end
      // velocity is always 0 by motion_velocity_smoother. NOTE: Path end velocity affects the last
      // two acceleration values.
      point_wise_a_arr.push_back(0.0);
    } else {
      point_wise_a_arr.push_back((segment_wise_a_arr.at(i - 1) + segment_wise_a_arr.at(i)) / 2.0);
    }
  }

  return point_wise_a_arr;
}

template <typename T>
std::vector<double> calcPathArcLengthArray(const T & points, const double offset)
{
  std::vector<double> out;
  out.push_back(offset);
  double sum = offset;
  for (size_t i = 1; i < points.size(); ++i) {
    sum += calcDistance2d(getPoint(points.at(i)), getPoint(points.at(i - 1)));
    out.push_back(sum);
  }
  return out;
}

template <class T>
inline std::vector<double> calcCurvature(const T & points)
{
  std::vector<double> curvature_arr;
  curvature_arr.push_back(0.0);
  for (size_t i = 1; i < points.size() - 1; ++i) {
    const auto p1 = getPoint(points.at(i - 1));
    const auto p2 = getPoint(points.at(i));
    const auto p3 = getPoint(points.at(i + 1));
    curvature_arr.push_back(tier4_autoware_utils::calcCurvature(p1, p2, p3));
  }
  curvature_arr.push_back(0.0);
  return curvature_arr;
}

}  // namespace planning_debug_tools

#endif  // PLANNING_DEBUG_TOOLS__UTIL_HPP_
