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
bool splineInterpolate(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input, const double interval,
  autoware_auto_planning_msgs::msg::PathWithLaneId * output, const rclcpp::Logger logger)
{
  *output = input;

  if (input.points.size() <= 1) {
    RCLCPP_DEBUG(logger, "Do not interpolate because path size is 1.");
    return false;
  }

  static constexpr double ep = 1.0e-8;

  // calc arclength for path
  std::vector<double> base_x;
  std::vector<double> base_y;
  std::vector<double> base_z;
  std::vector<double> base_v;
  for (const auto & p : input.points) {
    base_x.push_back(p.point.pose.position.x);
    base_y.push_back(p.point.pose.position.y);
    base_z.push_back(p.point.pose.position.z);
    base_v.push_back(p.point.longitudinal_velocity_mps);
  }
  std::vector<double> base_s = calcEuclidDist(base_x, base_y);

  // remove duplicating sample points
  {
    size_t Ns = base_s.size();
    size_t i = 1;
    while (i < Ns) {
      if (std::fabs(base_s[i - 1] - base_s[i]) < ep) {
        base_s.erase(base_s.begin() + i);
        base_x.erase(base_x.begin() + i);
        base_y.erase(base_y.begin() + i);
        base_z.erase(base_z.begin() + i);
        base_v.erase(base_v.begin() + i);
        Ns -= 1;
        i -= 1;
      }
      ++i;
    }
  }

  std::vector<double> resampled_s;
  for (double d = 0.0; d < base_s.back() - ep; d += interval) {
    resampled_s.push_back(d);
  }

  // do spline for xy
  const std::vector<double> resampled_x = ::interpolation::slerp(base_s, base_x, resampled_s);
  const std::vector<double> resampled_y = ::interpolation::slerp(base_s, base_y, resampled_s);
  const std::vector<double> resampled_z = ::interpolation::slerp(base_s, base_z, resampled_s);
  const std::vector<double> resampled_v = ::interpolation::slerp(base_s, base_v, resampled_s);

  // set xy
  output->points.clear();
  for (size_t i = 0; i < resampled_s.size(); i++) {
    autoware_auto_planning_msgs::msg::PathPointWithLaneId p;
    p.point.pose.position.x = resampled_x.at(i);
    p.point.pose.position.y = resampled_y.at(i);
    p.point.pose.position.z = resampled_z.at(i);
    p.point.longitudinal_velocity_mps = resampled_v.at(i);
    output->points.push_back(p);
  }

  // set yaw
  for (int i = 1; i < static_cast<int>(resampled_s.size()) - 1; i++) {
    auto p = output->points.at(i - 1).point.pose.position;
    auto n = output->points.at(i + 1).point.pose.position;
    double yaw = std::atan2(n.y - p.y, n.x - p.x);
    output->points.at(i).point.pose.orientation =
      tier4_autoware_utils::createQuaternionFromYaw(yaw);
  }
  if (output->points.size() > 1) {
    size_t l = output->points.size();
    output->points.front().point.pose.orientation = output->points.at(1).point.pose.orientation;
    output->points.back().point.pose.orientation = output->points.at(l - 2).point.pose.orientation;
  }

  // insert final point on path if its position is not same point as interpolated
  const auto & op = output->points.back().point.pose.position;
  const auto & ip = input.points.back().point.pose.position;
  if (std::hypot(op.x - ip.x, op.y - ip.y) > ep) {
    output->points.emplace_back(input.points.back());
  }
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
  autoware_auto_planning_msgs::msg::Path interpolated_path;

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

  // Interpolate
  const auto x_interp = interpolation::slerp(s_in, x, s_out);
  const auto y_interp = interpolation::slerp(s_in, y, s_out);
  const auto z_interp = interpolation::slerp(s_in, z, s_out);

  // Find a nearest segment for each point in s_out and use the velocity of the segment's beginning
  // point. Note that if s_out is almost the same value as s_in within DOUBLE_EPSILON range, the
  // velocity of s_out should be same as the one of s_in.
  std::vector<double> v_interp;
  size_t closest_segment_idx = 0;
  for (size_t i = 0; i < s_out.size() - 1; ++i) {
    for (size_t j = closest_segment_idx; j < s_in.size() - 1; ++j) {
      if (s_in.at(j) - DOUBLE_EPSILON < s_out.at(i) && s_out.at(i) < s_in.at(j + 1)) {
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

std::vector<double> calcEuclidDist(const std::vector<double> & x, const std::vector<double> & y)
{
  if (x.size() != y.size()) {
    std::cerr << "x y vector size should be the same." << std::endl;
  }

  std::vector<double> dist_v;
  dist_v.push_back(0.0);
  for (unsigned int i = 0; i < x.size() - 1; ++i) {
    const double dx = x.at(i + 1) - x.at(i);
    const double dy = y.at(i + 1) - y.at(i);
    dist_v.push_back(dist_v.at(i) + std::hypot(dx, dy));
  }

  return dist_v;
}
}  // namespace behavior_velocity_planner
