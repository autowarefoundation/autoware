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

#ifndef SURROUND_OBSTACLE_CHECKER__DEBUG_MARKER_HPP_
#define SURROUND_OBSTACLE_CHECKER__DEBUG_MARKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_adapi_v1_msgs/msg/planning_behavior.hpp>
#include <autoware_adapi_v1_msgs/msg/velocity_factor_array.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tier4_planning_msgs/msg/stop_reason_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/geometry.hpp>

#include <memory>
#include <string>

namespace surround_obstacle_checker
{

using autoware_adapi_v1_msgs::msg::PlanningBehavior;
using autoware_adapi_v1_msgs::msg::VelocityFactor;
using autoware_adapi_v1_msgs::msg::VelocityFactorArray;
using geometry_msgs::msg::PolygonStamped;
using tier4_planning_msgs::msg::StopFactor;
using tier4_planning_msgs::msg::StopReason;
using tier4_planning_msgs::msg::StopReasonArray;
using vehicle_info_util::VehicleInfo;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace bg = boost::geometry;
using Point2d = bg::model::d2::point_xy<double>;
using Polygon2d = bg::model::polygon<Point2d>;

enum class PoseType : int8_t { NoStart = 0 };
enum class PointType : int8_t { NoStart = 0 };

class SurroundObstacleCheckerDebugNode
{
public:
  explicit SurroundObstacleCheckerDebugNode(
    const vehicle_info_util::VehicleInfo & vehicle_info, const double base_link2front,
    const std::string & object_label, const double & surround_check_front_distance,
    const double & surround_check_side_distance, const double & surround_check_back_distance,
    const double & surround_check_hysteresis_distance, const geometry_msgs::msg::Pose & self_pose,
    const rclcpp::Clock::SharedPtr clock, rclcpp::Node & node);

  bool pushPose(const geometry_msgs::msg::Pose & pose, const PoseType & type);
  bool pushObstaclePoint(const geometry_msgs::msg::Point & obstacle_point, const PointType & type);
  void publish();
  void publishFootprints();
  void updateFootprintMargin(
    const std::string & object_label, const double front_distance, const double side_distance,
    const double back_distance);

private:
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_viz_pub_;
  rclcpp::Publisher<StopReasonArray>::SharedPtr stop_reason_pub_;
  rclcpp::Publisher<VelocityFactorArray>::SharedPtr velocity_factor_pub_;

  rclcpp::Publisher<PolygonStamped>::SharedPtr vehicle_footprint_pub_;
  rclcpp::Publisher<PolygonStamped>::SharedPtr vehicle_footprint_offset_pub_;
  rclcpp::Publisher<PolygonStamped>::SharedPtr vehicle_footprint_recover_offset_pub_;

  vehicle_info_util::VehicleInfo vehicle_info_;
  double base_link2front_;
  std::string object_label_;
  double surround_check_front_distance_;
  double surround_check_side_distance_;
  double surround_check_back_distance_;
  double surround_check_hysteresis_distance_;
  geometry_msgs::msg::Pose self_pose_;

  MarkerArray makeVisualizationMarker();
  StopReasonArray makeStopReasonArray();
  VelocityFactorArray makeVelocityFactorArray();

  PolygonStamped boostPolygonToPolygonStamped(const Polygon2d & boost_polygon, const double & z);

  std::shared_ptr<geometry_msgs::msg::Point> stop_obstacle_point_ptr_;
  std::shared_ptr<geometry_msgs::msg::Pose> stop_pose_ptr_;
  rclcpp::Clock::SharedPtr clock_;
};
}  // namespace surround_obstacle_checker
#endif  // SURROUND_OBSTACLE_CHECKER__DEBUG_MARKER_HPP_
