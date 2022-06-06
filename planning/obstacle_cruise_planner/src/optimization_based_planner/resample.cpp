// Copyright 2022 TIER IV, Inc.
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

#include "obstacle_cruise_planner/optimization_based_planner/resample.hpp"

#include "interpolation/linear_interpolation.hpp"
#include "interpolation/spline_interpolation.hpp"

#include <vector>

namespace
{
[[maybe_unused]] rclcpp::Duration safeSubtraction(const rclcpp::Time & t1, const rclcpp::Time & t2)
{
  rclcpp::Duration duration = rclcpp::Duration::from_seconds(0.0);
  try {
    duration = t1 - t2;
  } catch (std::runtime_error & err) {
    if (t1 > t2) {
      duration = rclcpp::Duration::max() * -1.0;
    } else {
      duration = rclcpp::Duration::max();
    }
  }
  return duration;
}

// tf2::toMsg does not have this type of function
geometry_msgs::msg::Point toMsg(tf2::Vector3 vec)
{
  geometry_msgs::msg::Point point;
  point.x = vec.x();
  point.y = vec.y();
  point.z = vec.z();
  return point;
}
}  // namespace

namespace resampling
{
std::vector<rclcpp::Duration> resampledValidRelativeTimeVector(
  const rclcpp::Time & start_time, const rclcpp::Time & obj_base_time,
  const std::vector<double> & rel_time_vec, const double duration)
{
  const auto prediction_duration = rclcpp::Duration::from_seconds(duration);
  const auto end_time = start_time + prediction_duration;

  // NOTE: rel_time_vec is relative time to start_time.
  //       rel_valid_time_vec is relative to obj_base_time, which is time stamp in predicted object.
  std::vector<rclcpp::Duration> rel_valid_time_vec;
  for (const auto & time : rel_time_vec) {
    // absolute target time
    const auto target_time = start_time + rclcpp::Duration::from_seconds(time);
    if (target_time > end_time) {
      break;
    }

    // relative target time
    const auto rel_target_time = target_time - obj_base_time;
    if (rel_target_time < rclcpp::Duration::from_seconds(0.0)) {
      continue;
    }

    rel_valid_time_vec.push_back(rel_target_time);
  }

  return rel_valid_time_vec;
}

autoware_auto_perception_msgs::msg::PredictedPath resamplePredictedPath(
  const autoware_auto_perception_msgs::msg::PredictedPath & input_path,
  const std::vector<rclcpp::Duration> & rel_time_vec)
{
  autoware_auto_perception_msgs::msg::PredictedPath resampled_path;
  resampled_path.time_step = input_path.time_step;

  for (const auto & rel_time : rel_time_vec) {
    const auto opt_pose = lerpByTimeStamp(input_path, rel_time);
    if (!opt_pose) {
      continue;
    }

    resampled_path.path.push_back(opt_pose.get());
  }

  return resampled_path;
}

geometry_msgs::msg::Pose lerpByPose(
  const geometry_msgs::msg::Pose & p1, const geometry_msgs::msg::Pose & p2, const double t)
{
  tf2::Transform tf_transform1, tf_transform2;
  tf2::fromMsg(p1, tf_transform1);
  tf2::fromMsg(p2, tf_transform2);
  const auto & tf_point = tf2::lerp(tf_transform1.getOrigin(), tf_transform2.getOrigin(), t);
  const auto & tf_quaternion =
    tf2::slerp(tf_transform1.getRotation(), tf_transform2.getRotation(), t);

  geometry_msgs::msg::Pose pose;
  pose.position = ::toMsg(tf_point);
  pose.orientation = tf2::toMsg(tf_quaternion);
  return pose;
}

boost::optional<geometry_msgs::msg::Pose> lerpByTimeStamp(
  const autoware_auto_perception_msgs::msg::PredictedPath & path, const rclcpp::Duration & rel_time)
{
  auto clock{rclcpp::Clock{RCL_ROS_TIME}};
  if (path.path.empty()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      rclcpp::get_logger("DynamicAvoidance.resample"), clock, 1000,
      "Empty path. Failed to interpolate path by time!");
    return {};
  }
  if (rel_time < rclcpp::Duration::from_seconds(0.0)) {
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger("DynamicAvoidance.resample"), "failed to interpolate path by time!"
                                                         << std::endl
                                                         << "query time: " << rel_time.seconds());

    return {};
  }

  if (rel_time > rclcpp::Duration(path.time_step) * (static_cast<double>(path.path.size()) - 1)) {
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger("DynamicAvoidance.resample"),
      "failed to interpolate path by time!"
        << std::endl
        << "path max duration: " << path.path.size() * rclcpp::Duration(path.time_step).seconds()
        << std::endl
        << "query time       : " << rel_time.seconds());

    return {};
  }

  for (size_t i = 1; i < path.path.size(); ++i) {
    const auto & pt = path.path.at(i);
    const auto & prev_pt = path.path.at(i - 1);
    if (rel_time <= rclcpp::Duration(path.time_step) * static_cast<double>(i)) {
      const auto offset = rel_time - rclcpp::Duration(path.time_step) * static_cast<double>(i - 1);
      const auto ratio = offset.seconds() / rclcpp::Duration(path.time_step).seconds();
      return lerpByPose(prev_pt, pt, ratio);
    }
  }

  RCLCPP_ERROR_STREAM(
    rclcpp::get_logger("DynamicAvoidance.resample"), "Something failed in function: " << __func__);
  return {};
}

inline void convertEulerAngleToMonotonic(std::vector<double> & a)
{
  for (unsigned int i = 1; i < a.size(); ++i) {
    const double da = a[i] - a[i - 1];
    a[i] = a[i - 1] + tier4_autoware_utils::normalizeRadian(da);
  }
}

autoware_auto_planning_msgs::msg::Trajectory applyLinearInterpolation(
  const std::vector<double> & base_index,
  const autoware_auto_planning_msgs::msg::Trajectory & base_trajectory,
  const std::vector<double> & out_index, const bool use_spline_for_pose)
{
  std::vector<double> px, py, pz, pyaw, tlx, taz, alx;
  for (const auto & p : base_trajectory.points) {
    px.push_back(p.pose.position.x);
    py.push_back(p.pose.position.y);
    pz.push_back(p.pose.position.z);
    pyaw.push_back(tf2::getYaw(p.pose.orientation));
    tlx.push_back(p.longitudinal_velocity_mps);
    taz.push_back(p.heading_rate_rps);
    alx.push_back(p.acceleration_mps2);
  }

  convertEulerAngleToMonotonic(pyaw);

  std::vector<double> px_p, py_p, pz_p, pyaw_p;
  if (use_spline_for_pose) {
    px_p = interpolation::slerp(base_index, px, out_index);
    py_p = interpolation::slerp(base_index, py, out_index);
    pz_p = interpolation::slerp(base_index, pz, out_index);
    pyaw_p = interpolation::slerp(base_index, pyaw, out_index);
  } else {
    px_p = interpolation::lerp(base_index, px, out_index);
    py_p = interpolation::lerp(base_index, py, out_index);
    pz_p = interpolation::lerp(base_index, pz, out_index);
    pyaw_p = interpolation::lerp(base_index, pyaw, out_index);
  }
  const auto tlx_p = interpolation::lerp(base_index, tlx, out_index);
  const auto taz_p = interpolation::lerp(base_index, taz, out_index);
  const auto alx_p = interpolation::lerp(base_index, alx, out_index);

  autoware_auto_planning_msgs::msg::Trajectory out_trajectory;
  out_trajectory.header = base_trajectory.header;
  autoware_auto_planning_msgs::msg::TrajectoryPoint point;
  for (unsigned int i = 0; i < out_index.size(); ++i) {
    point.pose.position.x = px_p.at(i);
    point.pose.position.y = py_p.at(i);
    point.pose.position.z = pz_p.at(i);
    point.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(pyaw_p.at(i));

    point.longitudinal_velocity_mps = tlx_p.at(i);
    point.heading_rate_rps = taz_p.at(i);
    point.acceleration_mps2 = alx_p.at(i);
    out_trajectory.points.push_back(point);
  }
  return out_trajectory;
}
}  // namespace resampling
