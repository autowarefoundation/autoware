// Copyright 2024 Tier IV, Inc.
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

#include "autoware/control_evaluator/control_evaluator_node.hpp"

#include "autoware/evaluator_utils/evaluator_utils.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <algorithm>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace control_diagnostics
{
ControlEvaluatorNode::ControlEvaluatorNode(const rclcpp::NodeOptions & node_options)
: Node("control_evaluator", node_options)
{
  using std::placeholders::_1;
  control_diag_sub_ = create_subscription<DiagnosticArray>(
    "~/input/diagnostics", 1, std::bind(&ControlEvaluatorNode::onDiagnostics, this, _1));

  // Publisher
  metrics_pub_ = create_publisher<DiagnosticArray>("~/metrics", 1);

  // Timer callback to publish evaluator diagnostics
  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&ControlEvaluatorNode::onTimer, this));
}

void ControlEvaluatorNode::getRouteData()
{
  // route
  {
    const auto msg = route_subscriber_.takeNewData();
    if (msg) {
      if (msg->segments.empty()) {
        RCLCPP_ERROR(get_logger(), "input route is empty. ignored");
      } else {
        route_handler_.setRoute(*msg);
      }
    }
  }

  // map
  {
    const auto msg = vector_map_subscriber_.takeNewData();
    if (msg) {
      route_handler_.setMap(*msg);
    }
  }
}

void ControlEvaluatorNode::onDiagnostics(const DiagnosticArray::ConstSharedPtr diag_msg)
{
  // add target diagnostics to the queue and remove old ones
  for (const auto & function : target_functions_) {
    autoware::evaluator_utils::updateDiagnosticQueue(*diag_msg, function, now(), diag_queue_);
  }
}

DiagnosticStatus ControlEvaluatorNode::generateAEBDiagnosticStatus(const DiagnosticStatus & diag)
{
  DiagnosticStatus status;
  status.level = status.OK;
  status.name = diag.name;
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "decision";
  const bool is_emergency_brake = (diag.level == DiagnosticStatus::ERROR);
  key_value.value = (is_emergency_brake) ? "deceleration" : "none";
  status.values.push_back(key_value);
  return status;
}

DiagnosticStatus ControlEvaluatorNode::generateLaneletDiagnosticStatus(const Pose & ego_pose) const
{
  const auto current_lanelets = [&]() {
    lanelet::ConstLanelet closest_route_lanelet;
    route_handler_.getClosestLaneletWithinRoute(ego_pose, &closest_route_lanelet);
    const auto shoulder_lanelets = route_handler_.getShoulderLaneletsAtPose(ego_pose);
    lanelet::ConstLanelets closest_lanelets{closest_route_lanelet};
    closest_lanelets.insert(
      closest_lanelets.end(), shoulder_lanelets.begin(), shoulder_lanelets.end());
    return closest_lanelets;
  }();
  const auto arc_coordinates = lanelet::utils::getArcCoordinates(current_lanelets, ego_pose);
  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(current_lanelets, ego_pose, &current_lane);

  DiagnosticStatus status;
  status.name = "ego_lane_info";
  status.level = status.OK;
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "lane_id";
  key_value.value = std::to_string(current_lane.id());
  status.values.push_back(key_value);
  key_value.key = "s";
  key_value.value = std::to_string(arc_coordinates.length);
  status.values.push_back(key_value);
  key_value.key = "t";
  key_value.value = std::to_string(arc_coordinates.distance);
  status.values.push_back(key_value);
  return status;
}

DiagnosticStatus ControlEvaluatorNode::generateKinematicStateDiagnosticStatus(
  const Odometry & odom, const AccelWithCovarianceStamped & accel_stamped)
{
  DiagnosticStatus status;
  status.name = "kinematic_state";
  status.level = status.OK;
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "vel";
  key_value.value = std::to_string(odom.twist.twist.linear.x);
  status.values.push_back(key_value);
  key_value.key = "acc";
  const auto & acc = accel_stamped.accel.accel.linear.x;
  key_value.value = std::to_string(acc);
  status.values.push_back(key_value);
  key_value.key = "jerk";
  const auto jerk = [&]() {
    if (!prev_acc_stamped_.has_value()) {
      prev_acc_stamped_ = accel_stamped;
      return 0.0;
    }
    const auto t = static_cast<double>(accel_stamped.header.stamp.sec) +
                   static_cast<double>(accel_stamped.header.stamp.nanosec) * 1e-9;
    const auto prev_t = static_cast<double>(prev_acc_stamped_.value().header.stamp.sec) +
                        static_cast<double>(prev_acc_stamped_.value().header.stamp.nanosec) * 1e-9;
    const auto dt = t - prev_t;
    if (dt < std::numeric_limits<double>::epsilon()) return 0.0;

    const auto prev_acc = prev_acc_stamped_.value().accel.accel.linear.x;
    prev_acc_stamped_ = accel_stamped;
    return (acc - prev_acc) / dt;
  }();
  key_value.value = std::to_string(jerk);
  status.values.push_back(key_value);
  return status;
}

DiagnosticStatus ControlEvaluatorNode::generateLateralDeviationDiagnosticStatus(
  const Trajectory & traj, const Point & ego_point)
{
  const double lateral_deviation = metrics::calcLateralDeviation(traj, ego_point);

  DiagnosticStatus status;
  status.level = status.OK;
  status.name = "lateral_deviation";
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "metric_value";
  key_value.value = std::to_string(lateral_deviation);
  status.values.push_back(key_value);

  return status;
}

DiagnosticStatus ControlEvaluatorNode::generateYawDeviationDiagnosticStatus(
  const Trajectory & traj, const Pose & ego_pose)
{
  const double yaw_deviation = metrics::calcYawDeviation(traj, ego_pose);

  DiagnosticStatus status;
  status.level = status.OK;
  status.name = "yaw_deviation";
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "metric_value";
  key_value.value = std::to_string(yaw_deviation);
  status.values.push_back(key_value);

  return status;
}

void ControlEvaluatorNode::onTimer()
{
  DiagnosticArray metrics_msg;
  const auto traj = traj_sub_.takeData();
  const auto odom = odometry_sub_.takeData();
  const auto acc = accel_sub_.takeData();

  // generate decision diagnostics from input diagnostics
  for (const auto & function : target_functions_) {
    const auto it = std::find_if(
      diag_queue_.begin(), diag_queue_.end(),
      [&function](const std::pair<diagnostic_msgs::msg::DiagnosticStatus, rclcpp::Time> & p) {
        return p.first.name.find(function) != std::string::npos;
      });
    if (it == diag_queue_.end()) {
      continue;
    }
    // generate each decision diagnostics
    // - AEB decision
    if (it->first.name.find("autonomous_emergency_braking") != std::string::npos) {
      metrics_msg.status.push_back(generateAEBDiagnosticStatus(it->first));
    }
  }

  // calculate deviation metrics
  if (odom && traj && !traj->points.empty()) {
    const Pose ego_pose = odom->pose.pose;
    metrics_msg.status.push_back(
      generateLateralDeviationDiagnosticStatus(*traj, ego_pose.position));
    metrics_msg.status.push_back(generateYawDeviationDiagnosticStatus(*traj, ego_pose));
  }

  getRouteData();
  if (odom && route_handler_.isHandlerReady()) {
    const Pose ego_pose = odom->pose.pose;
    metrics_msg.status.push_back(generateLaneletDiagnosticStatus(ego_pose));
  }

  if (odom && acc) {
    metrics_msg.status.push_back(generateKinematicStateDiagnosticStatus(*odom, *acc));
  }

  metrics_msg.header.stamp = now();
  metrics_pub_->publish(metrics_msg);
}
}  // namespace control_diagnostics

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(control_diagnostics::ControlEvaluatorNode)
