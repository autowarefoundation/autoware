// Copyright 2023 LeoDrive A.Ş. All rights reserved.
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

#ifndef PREDICTED_PATH_CHECKER__COLLISION_CHECKER_HPP_
#define PREDICTED_PATH_CHECKER__COLLISION_CHECKER_HPP_

#include <autoware/motion_utils/trajectory/conversion.hpp>
#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <autoware/universe_utils/ros/transform_listener.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <predicted_path_checker/debug_marker.hpp>
#include <predicted_path_checker/utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::motion::control::predicted_path_checker
{
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using autoware::universe_utils::Point2d;
using autoware::universe_utils::Polygon2d;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::TransformStamped;
using PointArray = std::vector<geometry_msgs::msg::Point>;

namespace bg = boost::geometry;

struct CollisionCheckerParam
{
  double width_margin;
  double z_axis_filtering_buffer;
  bool enable_z_axis_obstacle_filtering;
  double chattering_threshold;
};

struct PredictedObjectWithDetectionTime
{
  explicit PredictedObjectWithDetectionTime(
    const rclcpp::Time & t, geometry_msgs::msg::Point & p, PredictedObject obj)
  : detection_time(t), point(p), object(std::move(obj))
  {
  }

  rclcpp::Time detection_time;
  geometry_msgs::msg::Point point;
  PredictedObject object;
};

class CollisionChecker
{
public:
  explicit CollisionChecker(
    rclcpp::Node * node, std::shared_ptr<PredictedPathCheckerDebugNode> debug_ptr);

  boost::optional<std::pair<geometry_msgs::msg::Point, PredictedObject>>
  checkTrajectoryForCollision(
    TrajectoryPoints & predicted_trajectory_array,
    PredictedObjects::ConstSharedPtr dynamic_objects);

  void setParam(const CollisionCheckerParam & param);

private:
  // Functions

  boost::optional<std::pair<geometry_msgs::msg::Point, PredictedObject>> checkObstacleHistory(
    const Pose & base_pose, const Polygon2d & one_step_move_vehicle_polygon2d, const double z_min,
    const double z_max);

  boost::optional<std::pair<geometry_msgs::msg::Point, PredictedObject>> checkDynamicObjects(
    const Pose & base_pose, PredictedObjects::ConstSharedPtr dynamic_objects,
    const Polygon2d & one_step_move_vehicle_polygon2d, const double z_min, const double z_max);

  void updatePredictedObjectHistory(const rclcpp::Time & now)
  {
    for (auto itr = predicted_object_history_.begin(); itr != predicted_object_history_.end();) {
      const auto expired = (now - itr->detection_time).seconds() > param_.chattering_threshold;

      if (expired) {
        itr = predicted_object_history_.erase(itr);
        continue;
      }

      itr++;
    }
  }

  // Parameter
  CollisionCheckerParam param_;

  // Variables
  std::shared_ptr<PredictedPathCheckerDebugNode> debug_ptr_;
  rclcpp::Node * node_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  std::vector<PredictedObjectWithDetectionTime> predicted_object_history_{};
};
}  // namespace autoware::motion::control::predicted_path_checker

#endif  // PREDICTED_PATH_CHECKER__COLLISION_CHECKER_HPP_
