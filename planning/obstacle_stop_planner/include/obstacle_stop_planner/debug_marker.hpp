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
#ifndef OBSTACLE_STOP_PLANNER__DEBUG_MARKER_HPP_
#define OBSTACLE_STOP_PLANNER__DEBUG_MARKER_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tier4_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <tier4_planning_msgs/msg/stop_reason_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_types.h>

#include <memory>
#include <string>
#include <vector>
#define EIGEN_MPL2_ONLY
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
namespace motion_planning
{

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using std_msgs::msg::Header;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

using tier4_debug_msgs::msg::Float32MultiArrayStamped;
using tier4_planning_msgs::msg::StopFactor;
using tier4_planning_msgs::msg::StopReason;
using tier4_planning_msgs::msg::StopReasonArray;

enum class PolygonType : int8_t { Vehicle = 0, Collision, SlowDownRange, SlowDown };

enum class PointType : int8_t { Stop = 0, SlowDown };

enum class PoseType : int8_t { Stop = 0, TargetStop, SlowDownStart, SlowDownEnd };

class DebugValues
{
public:
  enum class TYPE {
    CURRENT_VEL = 0,
    CURRENT_ACC = 1,
    CURRENT_FORWARD_MARGIN = 2,
    SLOWDOWN_OBSTACLE_DISTANCE = 3,
    COLLISION_OBSTACLE_DISTANCE = 4,
    FLAG_FIND_COLLISION_OBSTACLE = 5,
    FLAG_FIND_SLOW_DOWN_OBSTACLE = 6,
    FLAG_ADAPTIVE_CRUISE = 7,
    FLAG_EXTERNAL = 8,
    SIZE
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
  std::array<double, static_cast<int>(TYPE::SIZE)> getValues() const { return values_; }
  /**
   * @brief set the given type to the given value
   * @param [in] type TYPE of the value
   * @param [in] value value to set
   */
  void setValues(const TYPE type, const double val) { values_.at(static_cast<int>(type)) = val; }
  /**
   * @brief set the given type to the given value
   * @param [in] type index of the type
   * @param [in] value value to set
   */
  void setValues(const int type, const double val) { values_.at(type) = val; }

private:
  static constexpr int num_debug_values_ = static_cast<int>(TYPE::SIZE);
  std::array<double, static_cast<int>(TYPE::SIZE)> values_;
};

class ObstacleStopPlannerDebugNode
{
public:
  explicit ObstacleStopPlannerDebugNode(rclcpp::Node * node, const double base_link2front);
  ~ObstacleStopPlannerDebugNode() {}
  bool pushPolygon(
    const std::vector<cv::Point2d> & polygon, const double z, const PolygonType & type);
  bool pushPolygon(const std::vector<Eigen::Vector3d> & polygon, const PolygonType & type);
  bool pushPose(const Pose & pose, const PoseType & type);
  bool pushObstaclePoint(const Point & obstacle_point, const PointType & type);
  bool pushObstaclePoint(const pcl::PointXYZ & obstacle_point, const PointType & type);
  MarkerArray makeVirtualWallMarker();
  MarkerArray makeVisualizationMarker();
  StopReasonArray makeStopReasonArray();

  void setDebugValues(const DebugValues::TYPE type, const double val)
  {
    debug_values_.setValues(type, val);
  }
  void publish();

private:
  rclcpp::Publisher<MarkerArray>::SharedPtr virtual_wall_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_viz_pub_;
  rclcpp::Publisher<StopReasonArray>::SharedPtr stop_reason_pub_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr pub_debug_values_;
  rclcpp::Node * node_;
  double base_link2front_;

  std::shared_ptr<Pose> stop_pose_ptr_;
  std::shared_ptr<Pose> target_stop_pose_ptr_;
  std::shared_ptr<Pose> slow_down_start_pose_ptr_;
  std::shared_ptr<Pose> slow_down_end_pose_ptr_;
  std::shared_ptr<Point> stop_obstacle_point_ptr_;
  std::shared_ptr<Point> slow_down_obstacle_point_ptr_;
  std::vector<std::vector<Eigen::Vector3d>> vehicle_polygons_;
  std::vector<std::vector<Eigen::Vector3d>> slow_down_range_polygons_;
  std::vector<std::vector<Eigen::Vector3d>> collision_polygons_;
  std::vector<std::vector<Eigen::Vector3d>> slow_down_polygons_;

  DebugValues debug_values_;
};

}  // namespace motion_planning

#endif  // OBSTACLE_STOP_PLANNER__DEBUG_MARKER_HPP_
