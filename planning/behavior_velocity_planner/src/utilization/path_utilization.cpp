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

#include <interpolation/linear_interpolation.hpp>
#include <interpolation/spline_interpolation.hpp>
#include <interpolation/zero_order_hold.hpp>
#include <motion_utils/resample/resample.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utilization/path_utilization.hpp>

#include <tf2/LinearMath/Quaternion.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <algorithm>
#include <memory>
#include <vector>

constexpr double DOUBLE_EPSILON = 1e-6;

namespace behavior_velocity_planner
{
bool splineInterpolate(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input, const double interval,
  autoware_auto_planning_msgs::msg::PathWithLaneId & output, const rclcpp::Logger logger)
{
  if (input.points.size() < 2) {
    RCLCPP_DEBUG(logger, "Do not interpolate because path size is 1.");
    return false;
  }

  output = motion_utils::resamplePath(input, interval, false, true, true, false);

  return true;
}

/*
 * Interpolate the path with a fixed interval by spline.
 * In order to correctly inherit the position of the planned velocity points, the position of the
 * existing points in the input path are kept in the interpolated path.
 * The velocity is interpolated by zero-order hold, that is, the velocity of the interpolated point
 * is the velocity of the closest point for the input "sub-path" which consists of the points before
 * the interpolated point.
 */
autoware_auto_planning_msgs::msg::Path interpolatePath(
  const autoware_auto_planning_msgs::msg::Path & path, const double length, const double interval)
{
  const auto logger{rclcpp::get_logger("behavior_velocity_planner").get_child("path_utilization")};

  const double epsilon = 0.01;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
  std::vector<double> v;
  std::vector<double> s_in;
  if (2000 < path.points.size()) {
    RCLCPP_WARN(
      logger, "because path size is too large, calculation cost is high. size is %d.",
      (int)path.points.size());
  }
  if (path.points.size() < 2) {
    RCLCPP_WARN(logger, "Do not interpolate because path size is 1.");
    return path;
  }

  double path_len = std::min(length, motion_utils::calcArcLength(path.points));
  {
    double s = 0.0;
    for (size_t idx = 0; idx < path.points.size(); ++idx) {
      const auto path_point = path.points.at(idx);
      x.push_back(path_point.pose.position.x);
      y.push_back(path_point.pose.position.y);
      z.push_back(path_point.pose.position.z);
      v.push_back(path_point.longitudinal_velocity_mps);
      if (idx != 0) {
        const auto path_point_prev = path.points.at(idx - 1);
        s += tier4_autoware_utils::calcDistance2d(path_point_prev.pose, path_point.pose);
      }
      if (s > path_len) {
        break;
      }
      s_in.push_back(s);
    }

    // update path length
    path_len = std::min(path_len, s_in.back());

    // Check Terminal Points
    if (std::fabs(s_in.back() - path_len) < epsilon) {
      s_in.back() = path_len;
    } else {
      s_in.push_back(path_len);
    }
  }

  // Calculate query points
  // Use all values of s_in to inherit the velocity-planned point, and add some points for
  // interpolation with a constant interval if the point is not closed to the original s_in points.
  // (because if this interval is very short, the interpolation will be less accurate and may fail.)
  std::vector<double> s_out = s_in;

  const auto has_almost_same_value = [&](const auto & vec, const auto x) {
    if (vec.empty()) return false;
    const auto has_close = [&](const auto v) { return std::abs(v - x) < epsilon; };
    return std::find_if(vec.begin(), vec.end(), has_close) != vec.end();
  };
  for (double s = 0.0; s < path_len; s += interval) {
    if (!has_almost_same_value(s_out, s)) {
      s_out.push_back(s);
    }
  }

  std::sort(s_out.begin(), s_out.end());

  if (s_out.empty()) {
    RCLCPP_WARN(logger, "Do not interpolate because s_out is empty.");
    return path;
  }

  return motion_utils::resamplePath(path, s_out);
}

autoware_auto_planning_msgs::msg::Path filterLitterPathPoint(
  const autoware_auto_planning_msgs::msg::Path & path)
{
  autoware_auto_planning_msgs::msg::Path filtered_path;

  const double epsilon = 0.01;
  size_t latest_id = 0;
  for (size_t i = 0; i < path.points.size(); ++i) {
    double dist = 0.0;
    if (i != 0) {
      const double x =
        path.points.at(i).pose.position.x - path.points.at(latest_id).pose.position.x;
      const double y =
        path.points.at(i).pose.position.y - path.points.at(latest_id).pose.position.y;
      dist = std::sqrt(x * x + y * y);
    }
    if (i == 0 || epsilon < dist /*init*/) {
      latest_id = i;
      filtered_path.points.push_back(path.points.at(latest_id));
    } else {
      filtered_path.points.back().longitudinal_velocity_mps = std::min(
        filtered_path.points.back().longitudinal_velocity_mps,
        path.points.at(i).longitudinal_velocity_mps);
    }
  }

  return filtered_path;
}
autoware_auto_planning_msgs::msg::Path filterStopPathPoint(
  const autoware_auto_planning_msgs::msg::Path & path)
{
  autoware_auto_planning_msgs::msg::Path filtered_path = path;
  bool found_stop = false;
  for (size_t i = 0; i < filtered_path.points.size(); ++i) {
    if (std::fabs(filtered_path.points.at(i).longitudinal_velocity_mps) < 0.01) {
      found_stop = true;
    }
    if (found_stop) {
      filtered_path.points.at(i).longitudinal_velocity_mps = 0.0;
    }
  }
  return filtered_path;
}
}  // namespace behavior_velocity_planner
