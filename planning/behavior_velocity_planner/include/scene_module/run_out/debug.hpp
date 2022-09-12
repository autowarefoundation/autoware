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
#ifndef SCENE_MODULE__RUN_OUT__DEBUG_HPP_
#define SCENE_MODULE__RUN_OUT__DEBUG_HPP_

#include "scene_module/run_out/dynamic_obstacle.hpp"

#include <string>
#include <vector>
namespace behavior_velocity_planner
{
using autoware_auto_planning_msgs::msg::Trajectory;
using tier4_debug_msgs::msg::Float32MultiArrayStamped;
using tier4_debug_msgs::msg::Int32Stamped;

class DebugValues
{
public:
  enum class TYPE {
    CALCULATION_TIME = 0,
    CALCULATION_TIME_COLLISION_CHECK = 1,
    LATERAL_DIST = 2,
    LONGITUDINAL_DIST_OBSTACLE = 3,
    LONGITUDINAL_DIST_COLLISION = 4,
    COLLISION_POS_FROM_EGO_FRONT = 5,
    STOP_DISTANCE = 6,
    NUM_OBSTACLES = 7,
    LATERAL_PASS_DIST = 8,
    SIZE,  // this is the number of enum elements
  };

  /**
   * @brief get the index corresponding to the given value TYPE
   * @param [in] type the TYPE enum for which to get the index
   * @return index of the type
   */
  int getValuesIdx(const TYPE type) const { return static_cast<int>(type); }
  /**
   * @brief get all the debug values as an std::array
   * @return array of all debug values
   */
  std::array<float, static_cast<int>(TYPE::SIZE)> getValues() const { return values_; }
  /**
   * @brief set the given type to the given value
   * @param [in] type TYPE of the value
   * @param [in] value value to set
   */
  void setValues(const TYPE type, const float val) { values_.at(static_cast<int>(type)) = val; }
  /**
   * @brief set the given type to the given value
   * @param [in] type index of the type
   * @param [in] value value to set
   */
  void setValues(const int type, const float val) { values_.at(type) = val; }

private:
  static constexpr int num_debug_values_ = static_cast<int>(TYPE::SIZE);
  std::array<float, static_cast<int>(TYPE::SIZE)> values_;
};

class RunOutDebug
{
public:
  enum class AccelReason {
    STOP = 0,
    NO_OBSTACLE = 1,
    PASS = 2,
    LOW_JERK = 3,
  };

  struct TextWithPosition
  {
    std::string text;
    geometry_msgs::msg::Point position;
  };

  explicit RunOutDebug(rclcpp::Node & node);
  ~RunOutDebug() {}

  void setDebugValues(const DebugValues::TYPE type, const double val)
  {
    debug_values_.setValues(type, val);
  }

  void pushCollisionPoints(const geometry_msgs::msg::Point & point);
  void pushCollisionPoints(const std::vector<geometry_msgs::msg::Point> & points);
  void pushNearestCollisionPoint(const geometry_msgs::msg::Point & point);
  void pushStopPose(const geometry_msgs::msg::Pose & pose);
  void pushPredictedVehiclePolygons(const std::vector<geometry_msgs::msg::Point> & polygon);
  void pushPredictedObstaclePolygons(const std::vector<geometry_msgs::msg::Point> & polygon);
  void pushCollisionObstaclePolygons(const std::vector<geometry_msgs::msg::Point> & polygon);
  void pushDetectionAreaPolygons(const Polygon2d & debug_polygon);
  void pushTravelTimeTexts(
    const double travel_time, const geometry_msgs::msg::Pose pose, const float lateral_offset);
  void setAccelReason(const AccelReason & accel_reason);
  void publishDebugValue();
  void publishDebugTrajectory(const Trajectory & trajectory);
  visualization_msgs::msg::MarkerArray createVisualizationMarkerArray();
  void setHeight(const double height);
  visualization_msgs::msg::MarkerArray createVirtualWallMarkerArray();

private:
  visualization_msgs::msg::MarkerArray createVisualizationMarkerArrayFromDebugData(
    const builtin_interfaces::msg::Time & current_time);
  void clearDebugMarker();

  rclcpp::Node & node_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr pub_debug_values_;
  rclcpp::Publisher<Int32Stamped>::SharedPtr pub_accel_reason_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_debug_trajectory_;
  std::vector<geometry_msgs::msg::Point> collision_points_;
  std::vector<geometry_msgs::msg::Point> nearest_collision_point_;
  std::vector<geometry_msgs::msg::Pose> stop_pose_;
  std::vector<std::vector<geometry_msgs::msg::Point>> predicted_vehicle_polygons_;
  std::vector<std::vector<geometry_msgs::msg::Point>> predicted_obstacle_polygons_;
  std::vector<std::vector<geometry_msgs::msg::Point>> collision_obstacle_polygons_;
  std::vector<std::vector<geometry_msgs::msg::Point>> detection_area_polygons_;
  std::vector<TextWithPosition> travel_time_texts_;
  DebugValues debug_values_;
  AccelReason accel_reason_;
  double height_{0};
};

}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__RUN_OUT__DEBUG_HPP_
