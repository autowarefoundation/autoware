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

#ifndef PLANNING_DEBUG_TOOLS__TRAJECTORY_ANALYZER_HPP_
#define PLANNING_DEBUG_TOOLS__TRAJECTORY_ANALYZER_HPP_

#include "motion_utils/trajectory/trajectory.hpp"
#include "planning_debug_tools/msg/trajectory_debug_info.hpp"
#include "planning_debug_tools/util.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/geometry/path_with_lane_id_geometry.hpp"

#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tier4_debug_msgs/msg/float64_multi_array_stamped.hpp"

#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace planning_debug_tools
{
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;
using planning_debug_tools::msg::TrajectoryDebugInfo;

template <typename T>
class TrajectoryAnalyzer
{
  using SubscriberType = typename rclcpp::Subscription<T>::SharedPtr;
  using PublisherType = rclcpp::Publisher<TrajectoryDebugInfo>::SharedPtr;
  using T_ConstSharedPtr = typename T::ConstSharedPtr;

public:
  TrajectoryAnalyzer(rclcpp::Node * node, const std::string & sub_name)
  : node_(node), name_(sub_name)
  {
    const auto pub_name = sub_name + "/debug_info";
    pub_ = node->create_publisher<TrajectoryDebugInfo>(pub_name, 1);
    sub_ = node->create_subscription<T>(
      sub_name, 1, [this](const T_ConstSharedPtr msg) { run(msg->points); });
  }
  ~TrajectoryAnalyzer() = default;

  void setKinematics(const Odometry::ConstSharedPtr input) { ego_kinematics_ = input; }

  // Note: the lambda used in the subscriber captures "this", so any operations that change the
  // address of "this" are prohibited.
  TrajectoryAnalyzer(const TrajectoryAnalyzer &) = delete;                      // copy
  TrajectoryAnalyzer(TrajectoryAnalyzer &&) = delete;                           // move
  auto operator=(const TrajectoryAnalyzer &) -> TrajectoryAnalyzer & = delete;  // copy assignment
  auto operator=(TrajectoryAnalyzer &&) -> TrajectoryAnalyzer & = delete;       // move assignment

public:
  std::shared_ptr<rclcpp::Node> node_;
  std::string name_;
  PublisherType pub_;
  SubscriberType sub_;
  Odometry::ConstSharedPtr ego_kinematics_;

  template <typename P>
  void run(const P & points)
  {
    if (!ego_kinematics_) return;
    if (points.size() < 3) return;

    const auto & ego_p = ego_kinematics_->pose.pose.position;

    TrajectoryDebugInfo data;
    data.stamp = node_->now();
    data.size = points.size();
    data.curvature = calcCurvature(points);
    const auto arclength_offset = motion_utils::calcSignedArcLength(points, 0, ego_p);
    data.arclength = calcPathArcLengthArray(points, -arclength_offset);
    data.velocity = getVelocityArray(points);
    data.acceleration = getAccelerationArray(points);
    data.yaw = getYawArray(points);

    if (
      data.size != data.arclength.size() || data.size != data.velocity.size() ||
      data.size != data.yaw.size()) {
      RCLCPP_ERROR(node_->get_logger(), "computation failed.");
      return;
    }

    pub_->publish(data);
  }
};

class TrajectoryAnalyzerNode : public rclcpp::Node
{
public:
  explicit TrajectoryAnalyzerNode(const rclcpp::NodeOptions & options);
  ~TrajectoryAnalyzerNode() = default;

private:
  rclcpp::Subscription<Odometry>::SharedPtr sub_ego_kinematics_;
  void onEgoKinematics(const Odometry::ConstSharedPtr msg);

  std::vector<std::shared_ptr<TrajectoryAnalyzer<Path>>> path_analyzers_;
  std::vector<std::shared_ptr<TrajectoryAnalyzer<PathWithLaneId>>> path_with_lane_id_analyzers_;
  std::vector<std::shared_ptr<TrajectoryAnalyzer<Trajectory>>> trajectory_analyzers_;
};

}  // namespace planning_debug_tools

#endif  // PLANNING_DEBUG_TOOLS__TRAJECTORY_ANALYZER_HPP_
