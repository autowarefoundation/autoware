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
#ifndef DEBUG_HPP_
#define DEBUG_HPP_

#include "utils.hpp"

#include <autoware/motion_utils/marker/virtual_wall_marker_creator.hpp>

#include <tier4_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/int32_stamped.hpp>

#include <memory>
#include <string>
#include <vector>
namespace autoware::behavior_velocity_planner
{
using sensor_msgs::msg::PointCloud2;
using tier4_debug_msgs::msg::Float32MultiArrayStamped;
using tier4_debug_msgs::msg::Int32Stamped;

class DebugValues
{
public:
  enum class TYPE {
    CALCULATION_TIME = 0,
    CALCULATION_TIME_PATH_PROCESSING = 1,
    CALCULATION_TIME_OBSTACLE_CREATION = 2,
    CALCULATION_TIME_COLLISION_CHECK = 3,
    CALCULATION_TIME_PATH_PLANNING = 4,
    LATERAL_DIST = 5,
    LONGITUDINAL_DIST_OBSTACLE = 6,
    LONGITUDINAL_DIST_COLLISION = 7,
    COLLISION_POS_FROM_EGO_FRONT = 8,
    STOP_DISTANCE = 9,
    NUM_OBSTACLES = 10,
    LATERAL_PASS_DIST = 11,
    SIZE,  // this is the number of enum elements
  };

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
    UNKNOWN = 4,
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
  void pushEgoCutLine(const std::vector<geometry_msgs::msg::Point> & line);
  void pushDetectionAreaPolygons(const Polygon2d & debug_polygon);
  void pushMandatoryDetectionAreaPolygons(const Polygon2d & debug_polygon);
  void pushTravelTimeTexts(
    const double travel_time, const geometry_msgs::msg::Pose pose, const float lateral_offset);
  void setAccelReason(const AccelReason & accel_reason);
  void publishDebugValue();
  void publishFilteredPointCloud(const PointCloud2 & pointcloud);
  void publishFilteredPointCloud(
    const pcl::PointCloud<pcl::PointXYZ> & pointcloud, const std_msgs::msg::Header header);
  void publishEmptyPointCloud();
  visualization_msgs::msg::MarkerArray createVisualizationMarkerArray();
  void setHeight(const double height);
  autoware::motion_utils::VirtualWalls createVirtualWalls();

private:
  visualization_msgs::msg::MarkerArray createVisualizationMarkerArrayFromDebugData(
    const builtin_interfaces::msg::Time & current_time) const;
  void clearDebugMarker();

  rclcpp::Node & node_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr pub_debug_values_;
  rclcpp::Publisher<Int32Stamped>::SharedPtr pub_accel_reason_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_debug_pointcloud_;
  std::vector<geometry_msgs::msg::Point> collision_points_;
  std::vector<geometry_msgs::msg::Point> nearest_collision_point_;
  std::vector<geometry_msgs::msg::Point> ego_cut_line_;
  std::vector<geometry_msgs::msg::Pose> stop_pose_;
  std::vector<std::vector<geometry_msgs::msg::Point>> predicted_vehicle_polygons_;
  std::vector<std::vector<geometry_msgs::msg::Point>> predicted_obstacle_polygons_;
  std::vector<std::vector<geometry_msgs::msg::Point>> collision_obstacle_polygons_;
  std::vector<std::vector<geometry_msgs::msg::Point>> detection_area_polygons_;
  std::vector<std::vector<geometry_msgs::msg::Point>> mandatory_detection_area_polygons_;
  std::vector<TextWithPosition> travel_time_texts_;
  DebugValues debug_values_;
  AccelReason accel_reason_;
  double height_{0};
};

}  // namespace autoware::behavior_velocity_planner

#endif  // DEBUG_HPP_
