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

#include <interpolation/spline_interpolation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utilization/path_utilization.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <memory>
#include <vector>

namespace behavior_velocity_planner
{
autoware_auto_planning_msgs::msg::Path interpolatePath(
  const autoware_auto_planning_msgs::msg::Path & path, const double length, const double interval)
{
  const auto logger{rclcpp::get_logger("behavior_velocity_planner").get_child("path_utilization")};
  autoware_auto_planning_msgs::msg::Path interpolated_path;

  std::vector<double> x, x_interp;
  std::vector<double> y, y_interp;
  std::vector<double> z, z_interp;
  std::vector<double> v, v_interp;
  std::vector<double> s_in, s_out;
  if (2000 < path.points.size()) {
    RCLCPP_WARN(
      logger, "because path size is too large, calculation cost is high. size is %d.",
      (int)path.points.size());
  }
  if (path.points.size() < 2) {
    RCLCPP_WARN(logger, "Do not interpolate because path size is 1.");
    return path;
  }

  // Calculate sample points
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
        s += tier4_autoware_utils::calcDistance3d(path_point_prev.pose, path_point.pose);
      }
      s_in.push_back(s);
    }
  }

  // Calculate query points
  // Remove query point if query point is near input path point
  const double epsilon = 0.01;
  const double interpolation_interval = interval;
  size_t checkpoint_idx = 1;
  for (double s = interpolation_interval; s < std::min(length, s_in.back());
       s += interpolation_interval) {
    while (checkpoint_idx < s_in.size() && s_in.at(checkpoint_idx) < s) {
      s_out.push_back(s_in.at(checkpoint_idx));
      v_interp.push_back(v.at(checkpoint_idx));
      ++checkpoint_idx;
    }
    if (
      std::fabs(s - s_in.at(checkpoint_idx - 1)) > epsilon &&
      std::fabs(s - s_in.at(checkpoint_idx)) > epsilon) {
      s_out.push_back(s);
      v_interp.push_back(v.at(checkpoint_idx - 1));
    }
  }

  if (s_out.empty()) {
    RCLCPP_WARN(logger, "Do not interpolate because s_out is empty.");
    return path;
  }

  // Interpolate
  x_interp = interpolation::slerp(s_in, x, s_out);
  y_interp = interpolation::slerp(s_in, y, s_out);
  z_interp = interpolation::slerp(s_in, z, s_out);

  // Insert boundary points
  x_interp.insert(x_interp.begin(), x.front());
  y_interp.insert(y_interp.begin(), y.front());
  z_interp.insert(z_interp.begin(), z.front());
  v_interp.insert(v_interp.begin(), v.front());

  x_interp.push_back(x.back());
  y_interp.push_back(y.back());
  z_interp.push_back(z.back());
  v_interp.push_back(v.back());

  // Insert path point to interpolated_path
  for (size_t idx = 0; idx < v_interp.size() - 1; ++idx) {
    autoware_auto_planning_msgs::msg::PathPoint path_point;
    path_point.pose.position.x = x_interp.at(idx);
    path_point.pose.position.y = y_interp.at(idx);
    path_point.pose.position.z = z_interp.at(idx);
    path_point.longitudinal_velocity_mps = v_interp.at(idx);
    const double yaw =
      std::atan2(y_interp.at(idx + 1) - y_interp.at(idx), x_interp.at(idx + 1) - x_interp.at(idx));
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw);
    path_point.pose.orientation = tf2::toMsg(quat);
    interpolated_path.points.push_back(path_point);
  }
  interpolated_path.points.push_back(path.points.back());

  return interpolated_path;
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
