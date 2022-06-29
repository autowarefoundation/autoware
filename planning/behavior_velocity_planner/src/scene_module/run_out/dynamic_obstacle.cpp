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

#include "scene_module/run_out/dynamic_obstacle.hpp"

#include <pcl/filters/voxel_grid.h>

namespace behavior_velocity_planner
{
namespace
{
// create quaternion facing to the nearest trajectory point
geometry_msgs::msg::Quaternion createQuaternionFacingToTrajectory(
  const PathPointsWithLaneId & path_points, const geometry_msgs::msg::Point & point)
{
  const auto nearest_idx = tier4_autoware_utils::findNearestIndex(path_points, point);
  const auto & nearest_pose = path_points.at(nearest_idx).point.pose;

  const auto longitudinal_offset =
    tier4_autoware_utils::calcLongitudinalDeviation(nearest_pose, point);
  const auto vertical_point =
    tier4_autoware_utils::calcOffsetPose(nearest_pose, longitudinal_offset, 0, 0).position;
  const auto azimuth_angle = tier4_autoware_utils::calcAzimuthAngle(point, vertical_point);

  return tier4_autoware_utils::createQuaternionFromYaw(azimuth_angle);
}

// create predicted path assuming that obstacles move with constant velocity
std::vector<geometry_msgs::msg::Pose> createPredictedPath(
  const geometry_msgs::msg::Pose & initial_pose, const float time_step,
  const float max_velocity_mps, const float max_prediction_time)
{
  const size_t path_size = max_prediction_time / time_step;
  std::vector<geometry_msgs::msg::Pose> path_points;
  for (size_t i = 0; i < path_size; i++) {
    const float travel_dist = max_velocity_mps * time_step * i;
    const auto predicted_pose =
      tier4_autoware_utils::calcOffsetPose(initial_pose, travel_dist, 0, 0);
    path_points.emplace_back(predicted_pose);
  }

  return path_points;
}

pcl::PointCloud<pcl::PointXYZ> applyVoxelGridFilter(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & input_points)
{
  auto no_height_points = *input_points;
  for (auto & p : no_height_points) {
    p.z = 0.0;
  }

  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(no_height_points));
  filter.setLeafSize(0.05f, 0.05f, 100000.0f);

  pcl::PointCloud<pcl::PointXYZ> output_points;
  filter.filter(output_points);

  return output_points;
}
}  // namespace

DynamicObstacleCreatorForObject::DynamicObstacleCreatorForObject(rclcpp::Node & node)
: DynamicObstacleCreator(node)
{
}

std::vector<DynamicObstacle> DynamicObstacleCreatorForObject::createDynamicObstacles()
{
  // create dynamic obstacles from predicted objects
  std::vector<DynamicObstacle> dynamic_obstacles;
  for (const auto & predicted_object : dynamic_obstacle_data_.predicted_objects.objects) {
    DynamicObstacle dynamic_obstacle;
    dynamic_obstacle.pose = predicted_object.kinematics.initial_pose_with_covariance.pose;

    // TODO(Tomohito Ando): calculate velocity from covariance of predicted_object
    dynamic_obstacle.min_velocity_mps = tier4_autoware_utils::kmph2mps(param_.min_vel_kmph);
    dynamic_obstacle.max_velocity_mps = tier4_autoware_utils::kmph2mps(param_.max_vel_kmph);
    dynamic_obstacle.classifications = predicted_object.classification;
    dynamic_obstacle.shape = predicted_object.shape;

    // get predicted paths of predicted_objects
    for (const auto & path : predicted_object.kinematics.predicted_paths) {
      PredictedPath predicted_path;
      predicted_path.confidence = path.confidence;
      predicted_path.path.resize(path.path.size());
      std::copy(path.path.cbegin(), path.path.cend(), predicted_path.path.begin());

      dynamic_obstacle.predicted_paths.emplace_back(predicted_path);
    }

    dynamic_obstacles.emplace_back(dynamic_obstacle);
  }

  return dynamic_obstacles;
}

DynamicObstacleCreatorForObjectWithoutPath::DynamicObstacleCreatorForObjectWithoutPath(
  rclcpp::Node & node)
: DynamicObstacleCreator(node)
{
}

std::vector<DynamicObstacle> DynamicObstacleCreatorForObjectWithoutPath::createDynamicObstacles()
{
  std::vector<DynamicObstacle> dynamic_obstacles;

  for (const auto & predicted_object : dynamic_obstacle_data_.predicted_objects.objects) {
    DynamicObstacle dynamic_obstacle;
    dynamic_obstacle.pose.position =
      predicted_object.kinematics.initial_pose_with_covariance.pose.position;
    dynamic_obstacle.pose.orientation = createQuaternionFacingToTrajectory(
      dynamic_obstacle_data_.path.points, dynamic_obstacle.pose.position);

    dynamic_obstacle.min_velocity_mps = tier4_autoware_utils::kmph2mps(param_.min_vel_kmph);
    dynamic_obstacle.max_velocity_mps = tier4_autoware_utils::kmph2mps(param_.max_vel_kmph);
    dynamic_obstacle.classifications = predicted_object.classification;
    dynamic_obstacle.shape = predicted_object.shape;

    // replace predicted path with path that runs straight to lane
    PredictedPath predicted_path;
    predicted_path.path = createPredictedPath(
      dynamic_obstacle.pose, param_.time_step, dynamic_obstacle.max_velocity_mps,
      param_.max_prediction_time);
    predicted_path.confidence = 1.0;
    dynamic_obstacle.predicted_paths.emplace_back(predicted_path);

    dynamic_obstacles.emplace_back(dynamic_obstacle);
  }

  return dynamic_obstacles;
}

DynamicObstacleCreatorForPoints::DynamicObstacleCreatorForPoints(rclcpp::Node & node)
: DynamicObstacleCreator(node), tf_buffer_(node.get_clock()), tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;
  sub_compare_map_filtered_pointcloud_ = node.create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/compare_map_filtered_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&DynamicObstacleCreatorForPoints::onCompareMapFilteredPointCloud, this, _1));
}

std::vector<DynamicObstacle> DynamicObstacleCreatorForPoints::createDynamicObstacles()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<DynamicObstacle> dynamic_obstacles;
  for (const auto & point : dynamic_obstacle_data_.compare_map_filtered_pointcloud) {
    DynamicObstacle dynamic_obstacle;

    // create pose facing the direction of the lane
    dynamic_obstacle.pose.position = tier4_autoware_utils::createPoint(point.x, point.y, point.z);
    dynamic_obstacle.pose.orientation = createQuaternionFacingToTrajectory(
      dynamic_obstacle_data_.path.points, dynamic_obstacle.pose.position);

    dynamic_obstacle.min_velocity_mps = tier4_autoware_utils::kmph2mps(param_.min_vel_kmph);
    dynamic_obstacle.max_velocity_mps = tier4_autoware_utils::kmph2mps(param_.max_vel_kmph);

    // create classification of points as pedestrian
    ObjectClassification classification;
    classification.label = ObjectClassification::PEDESTRIAN;
    classification.probability = 1.0;
    dynamic_obstacle.classifications.emplace_back(classification);

    // create shape of points as cylinder
    dynamic_obstacle.shape.type = Shape::CYLINDER;
    dynamic_obstacle.shape.dimensions.x = param_.diameter;
    dynamic_obstacle.shape.dimensions.y = param_.diameter;
    dynamic_obstacle.shape.dimensions.z = param_.height;

    // create predicted path of points
    PredictedPath predicted_path;
    predicted_path.path = createPredictedPath(
      dynamic_obstacle.pose, param_.time_step, dynamic_obstacle.max_velocity_mps,
      param_.max_prediction_time);
    predicted_path.confidence = 1.0;
    dynamic_obstacle.predicted_paths.emplace_back(predicted_path);

    dynamic_obstacles.emplace_back(dynamic_obstacle);
  }

  return dynamic_obstacles;
}

void DynamicObstacleCreatorForPoints::onCompareMapFilteredPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (msg->data.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    dynamic_obstacle_data_.compare_map_filtered_pointcloud.clear();
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
      "map", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(node_.get_logger(), "no transform found for no_ground_pointcloud: %s", e.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> pc;
  pcl::fromROSMsg(*msg, pc);

  Eigen::Affine3f affine = tf2::transformToEigen(transform.transform).cast<float>();
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_transformed(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(pc, *pc_transformed, affine);

  std::lock_guard<std::mutex> lock(mutex_);
  dynamic_obstacle_data_.compare_map_filtered_pointcloud = applyVoxelGridFilter(pc_transformed);
}
}  // namespace behavior_velocity_planner
