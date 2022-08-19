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

#ifndef SCENE_MODULE__RUN_OUT__DYNAMIC_OBSTACLE_HPP_
#define SCENE_MODULE__RUN_OUT__DYNAMIC_OBSTACLE_HPP_

#include "behavior_velocity_planner/planner_data.hpp"
#include "utilization/path_utilization.hpp"
#include "utilization/util.hpp"

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <vector>

namespace behavior_velocity_planner
{
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::Shape;
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using PathPointsWithLaneId = std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>;

struct DynamicObstacleParam
{
  float min_vel_kmph{0.0};
  float max_vel_kmph{5.0};

  // parameter to convert points to dynamic obstacle
  float diameter{0.1};              // [m]
  float height{2.0};                // [m]
  float max_prediction_time{10.0};  // [sec]
  float time_step{0.5};             // [sec]
  float points_interval{0.1};       // [m]
};

struct PoseWithRange
{
  geometry_msgs::msg::Pose pose_min;
  geometry_msgs::msg::Pose pose_max;
};

// since we use the minimum and maximum velocity,
// define the PredictedPath without time_step
struct PredictedPath
{
  std::vector<geometry_msgs::msg::Pose> path;
  float confidence;
};

// abstracted obstacle information
struct DynamicObstacle
{
  geometry_msgs::msg::Pose pose;
  std::vector<geometry_msgs::msg::Point> collision_points;
  geometry_msgs::msg::Point nearest_collision_point;
  float min_velocity_mps;
  float max_velocity_mps;
  std::vector<ObjectClassification> classifications;
  Shape shape;
  std::vector<PredictedPath> predicted_paths;
};

struct DynamicObstacleData
{
  PredictedObjects predicted_objects;
  pcl::PointCloud<pcl::PointXYZ> obstacle_points;
  PathWithLaneId path;
  Polygons2d detection_area_polygon;
};

/**
 * @brief base class for creating dynamic obstacles from multiple types of input
 */
class DynamicObstacleCreator
{
public:
  explicit DynamicObstacleCreator(rclcpp::Node & node) : node_(node) {}
  virtual ~DynamicObstacleCreator() = default;
  virtual std::vector<DynamicObstacle> createDynamicObstacles() = 0;
  void setParam(const DynamicObstacleParam & param) { param_ = param; }
  void setData(
    const PlannerData & planner_data, const PathWithLaneId & path, const Polygons2d & poly)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // compare map filtered points are subscribed in derived class that needs points
    dynamic_obstacle_data_.predicted_objects = *planner_data.predicted_objects;
    dynamic_obstacle_data_.path = path;
    dynamic_obstacle_data_.detection_area_polygon = poly;
  }

protected:
  DynamicObstacleParam param_;
  rclcpp::Node & node_;
  DynamicObstacleData dynamic_obstacle_data_;

  // mutex for dynamic_obstacle_data_
  std::mutex mutex_;
};

/**
 * @brief create dynamic obstacles from predicted objects
 */
class DynamicObstacleCreatorForObject : public DynamicObstacleCreator
{
public:
  explicit DynamicObstacleCreatorForObject(rclcpp::Node & node);
  std::vector<DynamicObstacle> createDynamicObstacles() override;
};

/**
 * @brief create dynamic obstacles from predicted objects, but overwrite the path to be normal to
 *        the path of ego vehicle.
 */
class DynamicObstacleCreatorForObjectWithoutPath : public DynamicObstacleCreator
{
public:
  explicit DynamicObstacleCreatorForObjectWithoutPath(rclcpp::Node & node);
  std::vector<DynamicObstacle> createDynamicObstacles() override;
};

/**
 * @brief create dynamic obstacles from points.
 *        predicted path is created to be normal to the path of ego vehicle.
 */
class DynamicObstacleCreatorForPoints : public DynamicObstacleCreator
{
public:
  explicit DynamicObstacleCreatorForPoints(rclcpp::Node & node);
  std::vector<DynamicObstacle> createDynamicObstacles() override;

private:
  void onCompareMapFilteredPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
    sub_compare_map_filtered_pointcloud_;

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__RUN_OUT__DYNAMIC_OBSTACLE_HPP_
