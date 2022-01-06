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

constexpr double DOUBLE_EPSILON = 1e-6;

namespace behavior_velocity_planner
{
autoware_auto_planning_msgs::msg::Path interpolatePath(
  const autoware_auto_planning_msgs::msg::Path & path, const double length, const double interval)
{
  const auto logger{rclcpp::get_logger("behavior_velocity_planner").get_child("path_utilization")};
  autoware_auto_planning_msgs::msg::Path interpolated_path;

  const double epsilon = 0.01;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
  std::vector<double> v;
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

  double path_len = length;
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
  // Remove query point if query point is near input path point
  std::vector<double> s_tmp = s_in;
  for (double s = 0.0; s < path_len; s += interval) {
    s_tmp.push_back(s);
  }
  std::sort(s_tmp.begin(), s_tmp.end());

  for (const double s : s_tmp) {
    if (!s_out.empty() && std::fabs(s_out.back() - s) < epsilon) {
      continue;
    }
    s_out.push_back(s);
  }

  if (s_out.empty()) {
    RCLCPP_WARN(logger, "Do not interpolate because s_out is empty.");
    return path;
  }

  // Interpolate
  const auto x_interp = interpolation::slerp(s_in, x, s_out);
  const auto y_interp = interpolation::slerp(s_in, y, s_out);
  const auto z_interp = interpolation::slerp(s_in, z, s_out);

  std::vector<double> v_interp;
  size_t closest_segment_idx = 0;
  for (size_t i = 0; i < s_out.size() - 1; ++i) {
    for (size_t j = closest_segment_idx; j < s_in.size() - 1; ++j) {
      if (s_in.at(j) < s_out.at(i) + DOUBLE_EPSILON && s_out.at(i) < s_in.at(j + 1)) {
        // find closest segment in s_in
        closest_segment_idx = j;
      }
    }
    v_interp.push_back(v.at(closest_segment_idx));
  }
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
