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

#ifndef PURE_PURSUIT__UTIL__TF_UTILS_HPP_
#define PURE_PURSUIT__UTIL__TF_UTILS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <boost/optional.hpp>  // To be replaced by std::optional in C++17

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/transform_listener.h>

#include <string>

#define TF_UTILS_LOGGER "tf_utils"

namespace pure_pursuit
{
namespace tf_utils
{
rclcpp::Logger logger = rclcpp::get_logger(TF_UTILS_LOGGER);
inline boost::optional<geometry_msgs::msg::TransformStamped> getTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & from, const std::string & to,
  const rclcpp::Time & time, const rclcpp::Duration & duration)
{
  try {
    return tf_buffer.lookupTransform(from, to, time, duration);
  } catch (tf2::TransformException & ex) {
    return {};
  }
}

inline geometry_msgs::msg::TransformStamped waitForTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & from, const std::string & to)
{
  while (rclcpp::ok()) {
    try {
      const auto transform = tf_buffer.lookupTransform(from, to, tf2::TimePointZero);
      return transform;
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(logger, "waiting for transform from `%s` to `%s` ...", from.c_str(), to.c_str());
      rclcpp::sleep_for(std::chrono::milliseconds(5000));
    }
  }
  return geometry_msgs::msg::TransformStamped();
}

inline geometry_msgs::msg::PoseStamped transform2pose(
  const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = transform.header;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

inline boost::optional<geometry_msgs::msg::PoseStamped> getCurrentPose(
  const tf2_ros::Buffer & tf_buffer, const double timeout = 1.0)
{
  const auto tf_current_pose = getTransform(
    tf_buffer, "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.0));
  if (!tf_current_pose) {
    return {};
  }

  const auto time_diff = tf2::timeToSec(tf2::get_now()) - (tf_current_pose->header.stamp.sec);
  if (std::abs(time_diff) > timeout) {
    return {};
  }

  return transform2pose(*tf_current_pose);
}

}  // namespace tf_utils
}  // namespace pure_pursuit

#endif  // PURE_PURSUIT__UTIL__TF_UTILS_HPP_
