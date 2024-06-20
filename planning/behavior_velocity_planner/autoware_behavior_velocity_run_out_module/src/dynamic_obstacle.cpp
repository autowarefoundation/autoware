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

#include "dynamic_obstacle.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/geometry/pose_deviation.hpp>
#include <autoware/universe_utils/math/unit_conversion.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>
#include <autoware/universe_utils/transform/transforms.hpp>

#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/envelope.hpp>

#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <limits>
#include <string>

namespace autoware::behavior_velocity_planner
{
namespace
{
// create quaternion facing to the nearest trajectory point
geometry_msgs::msg::Quaternion createQuaternionFacingToTrajectory(
  const PathPointsWithLaneId & path_points, const geometry_msgs::msg::Point & point)
{
  const auto nearest_idx = autoware::motion_utils::findNearestIndex(path_points, point);
  const auto & nearest_pose = path_points.at(nearest_idx).point.pose;

  const auto longitudinal_offset =
    autoware::universe_utils::calcLongitudinalDeviation(nearest_pose, point);
  const auto vertical_point =
    autoware::universe_utils::calcOffsetPose(nearest_pose, longitudinal_offset, 0, 0).position;
  const auto azimuth_angle = autoware::universe_utils::calcAzimuthAngle(point, vertical_point);

  return autoware::universe_utils::createQuaternionFromYaw(azimuth_angle);
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
      autoware::universe_utils::calcOffsetPose(initial_pose, travel_dist, 0, 0);
    path_points.emplace_back(predicted_pose);
  }

  return path_points;
}

pcl::PointCloud<pcl::PointXYZ> applyVoxelGridFilter(
  const pcl::PointCloud<pcl::PointXYZ> & input_points)
{
  auto no_height_points = input_points;
  for (auto & p : no_height_points) {
    p.z = 0.0;
  }

  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>(no_height_points));
  filter.setLeafSize(0.05f, 0.05f, 100000.0f);

  pcl::PointCloud<pcl::PointXYZ> output_points;
  filter.filter(output_points);
  output_points.header = input_points.header;

  return output_points;
}

pcl::PointCloud<pcl::PointXYZ> applyVoxelGridFilter(
  const sensor_msgs::msg::PointCloud2 & input_points)
{
  if (input_points.data.empty()) {
    return pcl::PointCloud<pcl::PointXYZ>();
  }

  pcl::PointCloud<pcl::PointXYZ> input_points_pcl;
  pcl::fromROSMsg(input_points, input_points_pcl);

  auto no_height_points = input_points_pcl;
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

bool isAheadOf(
  const geometry_msgs::msg::Point & target_point, const geometry_msgs::msg::Pose & base_pose)
{
  const auto longitudinal_deviation =
    autoware::universe_utils::calcLongitudinalDeviation(base_pose, target_point);
  const bool is_ahead = longitudinal_deviation > 0;
  return is_ahead;
}

pcl::PointCloud<pcl::PointXYZ> extractObstaclePointsWithinPolygon(
  const pcl::PointCloud<pcl::PointXYZ> & input_points, const Polygons2d & polys)
{
  namespace bg = boost::geometry;

  if (polys.empty()) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("run_out"), "detection area polygon is empty. return empty points.");

    const pcl::PointCloud<pcl::PointXYZ> empty_points;
    return empty_points;
  }

  pcl::PointCloud<pcl::PointXYZ> output_points;
  output_points.header = input_points.header;
  for (const auto & poly : polys) {
    const auto bounding_box = bg::return_envelope<autoware::universe_utils::Box2d>(poly);
    for (const auto & p : input_points) {
      Point2d point(p.x, p.y);

      // filter with bounding box to reduce calculation time
      if (!bg::covered_by(point, bounding_box)) {
        continue;
      }

      if (!bg::covered_by(point, poly)) {
        continue;
      }

      output_points.push_back(p);
    }
  }

  return output_points;
}

// group points with its nearest segment of path points
std::vector<pcl::PointCloud<pcl::PointXYZ>> groupPointsWithNearestSegmentIndex(
  const pcl::PointCloud<pcl::PointXYZ> & input_points, const PathPointsWithLaneId & path_points)
{
  // assign nearest segment index to each point
  std::vector<pcl::PointCloud<pcl::PointXYZ>> points_with_index;
  points_with_index.resize(path_points.size());

  for (const auto & p : input_points.points) {
    const auto ros_point = autoware::universe_utils::createPoint(p.x, p.y, p.z);
    const size_t nearest_seg_idx =
      autoware::motion_utils::findNearestSegmentIndex(path_points, ros_point);

    // if the point is ahead of end of the path, index should be path.size() - 1
    if (
      nearest_seg_idx == path_points.size() - 2 &&
      isAheadOf(ros_point, path_points.back().point.pose)) {
      points_with_index.back().push_back(p);
      continue;
    }

    points_with_index.at(nearest_seg_idx).push_back(p);
  }

  return points_with_index;
}

// calculate lateral nearest point from base_pose
pcl::PointXYZ calculateLateralNearestPoint(
  const pcl::PointCloud<pcl::PointXYZ> & input_points, const geometry_msgs::msg::Pose & base_pose)
{
  const auto lateral_nearest_point = std::min_element(
    input_points.points.begin(), input_points.points.end(), [&](const auto & p1, const auto & p2) {
      const auto lateral_deviation_p1 = std::abs(autoware::universe_utils::calcLateralDeviation(
        base_pose, autoware::universe_utils::createPoint(p1.x, p1.y, 0)));
      const auto lateral_deviation_p2 = std::abs(autoware::universe_utils::calcLateralDeviation(
        base_pose, autoware::universe_utils::createPoint(p2.x, p2.y, 0)));

      return lateral_deviation_p1 < lateral_deviation_p2;
    });

  return *lateral_nearest_point;
}

pcl::PointCloud<pcl::PointXYZ> selectLateralNearestPoints(
  const std::vector<pcl::PointCloud<pcl::PointXYZ>> & points_with_index,
  const PathPointsWithLaneId & path_points)
{
  pcl::PointCloud<pcl::PointXYZ> lateral_nearest_points;
  for (size_t idx = 0; idx < points_with_index.size(); idx++) {
    if (points_with_index.at(idx).points.empty()) {
      continue;
    }

    lateral_nearest_points.push_back(
      calculateLateralNearestPoint(points_with_index.at(idx), path_points.at(idx).point.pose));
  }

  return lateral_nearest_points;
}

// extract lateral nearest points for nearest segment of the path
// path is interpolated with given interval
pcl::PointCloud<pcl::PointXYZ> extractLateralNearestPoints(
  const pcl::PointCloud<pcl::PointXYZ> & input_points, const PathWithLaneId & path,
  const float interval)
{
  // interpolate path points with given interval
  PathWithLaneId interpolated_path;
  if (!splineInterpolate(
        path, interval, interpolated_path, rclcpp::get_logger("dynamic_obstacle_creator"))) {
    return input_points;
  }

  // divide points into groups according to nearest segment index
  const auto points_with_index =
    groupPointsWithNearestSegmentIndex(input_points, interpolated_path.points);

  // select the lateral nearest point for each group
  const auto lateral_nearest_points =
    selectLateralNearestPoints(points_with_index, interpolated_path.points);

  return lateral_nearest_points;
}

std::optional<Eigen::Affine3f> getTransformMatrix(
  const tf2_ros::Buffer & tf_buffer, const std::string & target_frame_id,
  const std::string & source_frame_id, const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer.lookupTransform(target_frame_id, source_frame_id, stamp);
  } catch (tf2::TransformException & e) {
    RCLCPP_WARN(rclcpp::get_logger("dynamic_obstacle_creator"), "no transform found: %s", e.what());
    return {};
  }

  Eigen::Affine3f transform_matrix = tf2::transformToEigen(transform.transform).cast<float>();
  return transform_matrix;
}

pcl::PointCloud<pcl::PointXYZ> transformPointCloud(
  const PointCloud2 & input_pointcloud, const Eigen::Affine3f & transform_matrix)
{
  if (input_pointcloud.data.empty()) {
    return pcl::PointCloud<pcl::PointXYZ>();
  }

  pcl::PointCloud<pcl::PointXYZ> pointcloud_pcl;
  pcl::fromROSMsg(input_pointcloud, pointcloud_pcl);
  pcl::PointCloud<pcl::PointXYZ> pointcloud_pcl_transformed;
  autoware::universe_utils::transformPointCloud(
    pointcloud_pcl, pointcloud_pcl_transformed, transform_matrix);

  return pointcloud_pcl_transformed;
}

PointCloud2 concatPointCloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud1, const pcl::PointCloud<pcl::PointXYZ> & cloud2)
{
  // convert to ROS pointcloud to concatenate points
  PointCloud2 cloud1_ros;
  PointCloud2 cloud2_ros;
  pcl::toROSMsg(cloud1, cloud1_ros);
  pcl::toROSMsg(cloud2, cloud2_ros);

  // concatenate two clouds
  PointCloud2 concat_points;
  pcl::concatenatePointCloud(cloud1_ros, cloud2_ros, concat_points);
  concat_points.header = cloud1_ros.header;

  return concat_points;
}

void calculateMinAndMaxVelFromCovariance(
  const geometry_msgs::msg::TwistWithCovariance & twist_with_covariance,
  const double std_dev_multiplier, run_out_utils::DynamicObstacle & dynamic_obstacle)
{
  const double x_velocity = std::abs(twist_with_covariance.twist.linear.x);
  const double y_velocity = std::abs(twist_with_covariance.twist.linear.y);
  const double x_variance = twist_with_covariance.covariance.at(0);
  const double y_variance = twist_with_covariance.covariance.at(7);
  const double x_std_dev = std::sqrt(x_variance);
  const double y_std_dev = std::sqrt(y_variance);

  // calculate the min and max velocity using the standard deviation of twist
  // note that this assumes the covariance of x and y is zero
  const double min_x = std::max(0.0, x_velocity - std_dev_multiplier * x_std_dev);
  const double min_y = std::max(0.0, y_velocity - std_dev_multiplier * y_std_dev);
  const double min_velocity = std::hypot(min_x, min_y);

  const double max_x = x_velocity + std_dev_multiplier * x_std_dev;
  const double max_y = y_velocity + std_dev_multiplier * y_std_dev;
  const double max_velocity = std::hypot(max_x, max_y);

  dynamic_obstacle.min_velocity_mps = min_velocity;
  dynamic_obstacle.max_velocity_mps = max_velocity;
}

double convertDurationToDouble(const builtin_interfaces::msg::Duration & duration)
{
  return duration.sec + duration.nanosec / 1e9;
}

// Create a path leading up to a specified prediction time
std::vector<geometry_msgs::msg::Pose> createPathToPredictionTime(
  const autoware_perception_msgs::msg::PredictedPath & predicted_path, double prediction_time)
{
  // Calculate the number of poses to include based on the prediction time and the time step between
  // poses
  const double time_step_seconds = convertDurationToDouble(predicted_path.time_step);
  if (time_step_seconds < std::numeric_limits<double>::epsilon()) {
    // Handle the case where time_step_seconds is zero or too close to zero
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("run_out: createPathToPredictionTime"),
      "time_step of the path is too close to zero. Use the input path");
    const std::vector<geometry_msgs::msg::Pose> input_path(
      predicted_path.path.begin(), predicted_path.path.end());
    return input_path;
  }
  const size_t poses_to_include =
    std::min(static_cast<size_t>(prediction_time / time_step_seconds), predicted_path.path.size());

  // Construct the path to the specified prediction time
  std::vector<geometry_msgs::msg::Pose> path_to_prediction_time;
  path_to_prediction_time.reserve(poses_to_include);
  for (size_t i = 0; i < poses_to_include; ++i) {
    path_to_prediction_time.push_back(predicted_path.path[i]);
  }

  return path_to_prediction_time;
}

}  // namespace

DynamicObstacleCreatorForObject::DynamicObstacleCreatorForObject(
  rclcpp::Node & node, std::shared_ptr<RunOutDebug> & debug_ptr, const DynamicObstacleParam & param)
: DynamicObstacleCreator(node, debug_ptr, param)
{
}

std::vector<DynamicObstacle> DynamicObstacleCreatorForObject::createDynamicObstacles()
{
  // create dynamic obstacles from predicted objects
  std::vector<DynamicObstacle> dynamic_obstacles;
  for (const auto & predicted_object : dynamic_obstacle_data_.predicted_objects.objects) {
    DynamicObstacle dynamic_obstacle;
    dynamic_obstacle.pose = predicted_object.kinematics.initial_pose_with_covariance.pose;

    if (param_.assume_fixed_velocity) {
      dynamic_obstacle.min_velocity_mps = autoware::universe_utils::kmph2mps(param_.min_vel_kmph);
      dynamic_obstacle.max_velocity_mps = autoware::universe_utils::kmph2mps(param_.max_vel_kmph);
    } else {
      calculateMinAndMaxVelFromCovariance(
        predicted_object.kinematics.initial_twist_with_covariance, param_.std_dev_multiplier,
        dynamic_obstacle);
    }
    dynamic_obstacle.classifications = predicted_object.classification;
    dynamic_obstacle.shape = predicted_object.shape;
    dynamic_obstacle.uuid = predicted_object.object_id;

    // get predicted paths of predicted_objects
    for (const auto & path : predicted_object.kinematics.predicted_paths) {
      PredictedPath predicted_path;
      predicted_path.confidence = path.confidence;
      predicted_path.path = createPathToPredictionTime(path, param_.max_prediction_time);

      dynamic_obstacle.predicted_paths.emplace_back(predicted_path);
    }

    dynamic_obstacles.emplace_back(dynamic_obstacle);
  }

  return dynamic_obstacles;
}

DynamicObstacleCreatorForObjectWithoutPath::DynamicObstacleCreatorForObjectWithoutPath(
  rclcpp::Node & node, std::shared_ptr<RunOutDebug> & debug_ptr, const DynamicObstacleParam & param)
: DynamicObstacleCreator(node, debug_ptr, param)
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

    dynamic_obstacle.min_velocity_mps = autoware::universe_utils::kmph2mps(param_.min_vel_kmph);
    dynamic_obstacle.max_velocity_mps = autoware::universe_utils::kmph2mps(param_.max_vel_kmph);
    dynamic_obstacle.classifications = predicted_object.classification;
    dynamic_obstacle.shape = predicted_object.shape;

    // replace predicted path with path that runs straight to lane
    PredictedPath predicted_path;
    predicted_path.path = createPredictedPath(
      dynamic_obstacle.pose, param_.time_step, dynamic_obstacle.max_velocity_mps,
      param_.max_prediction_time);
    predicted_path.confidence = 1.0;
    dynamic_obstacle.predicted_paths.emplace_back(predicted_path);
    dynamic_obstacle.uuid = predicted_object.object_id;

    dynamic_obstacles.emplace_back(dynamic_obstacle);
  }

  return dynamic_obstacles;
}

DynamicObstacleCreatorForPoints::DynamicObstacleCreatorForPoints(
  rclcpp::Node & node, std::shared_ptr<RunOutDebug> & debug_ptr, const DynamicObstacleParam & param)
: DynamicObstacleCreator(node, debug_ptr, param),
  tf_buffer_(node.get_clock()),
  tf_listener_(tf_buffer_)
{
  if (param_.use_mandatory_area) {
    // Subscribe the input using message filter
    const size_t max_queue_size = 1;
    sub_compare_map_filtered_pointcloud_sync_.subscribe(
      &node, "~/input/compare_map_filtered_pointcloud",
      rclcpp::SensorDataQoS().keep_last(max_queue_size).get_rmw_qos_profile());
    sub_vector_map_inside_area_filtered_pointcloud_sync_.subscribe(
      &node, "~/input/vector_map_inside_area_filtered_pointcloud",
      rclcpp::SensorDataQoS().keep_last(max_queue_size).get_rmw_qos_profile());

    // sync subscribers with ExactTime Sync Policy
    exact_time_synchronizer_ = std::make_unique<ExactTimeSynchronizer>(max_queue_size);
    exact_time_synchronizer_->connectInput(
      sub_compare_map_filtered_pointcloud_sync_,
      sub_vector_map_inside_area_filtered_pointcloud_sync_);
    exact_time_synchronizer_->registerCallback(
      &DynamicObstacleCreatorForPoints::onSynchronizedPointCloud, this);
  } else {
    using std::placeholders::_1;
    sub_compare_map_filtered_pointcloud_ = node.create_subscription<sensor_msgs::msg::PointCloud2>(
      "~/input/vector_map_inside_area_filtered_pointcloud", rclcpp::SensorDataQoS(),
      std::bind(&DynamicObstacleCreatorForPoints::onCompareMapFilteredPointCloud, this, _1));
  }
}

std::vector<DynamicObstacle> DynamicObstacleCreatorForPoints::createDynamicObstacles()
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<DynamicObstacle> dynamic_obstacles;
  for (const auto & point : obstacle_points_map_filtered_) {
    DynamicObstacle dynamic_obstacle;

    // create pose facing the direction of the lane
    dynamic_obstacle.pose.position =
      autoware::universe_utils::createPoint(point.x, point.y, point.z);
    dynamic_obstacle.pose.orientation = createQuaternionFacingToTrajectory(
      dynamic_obstacle_data_.path.points, dynamic_obstacle.pose.position);

    dynamic_obstacle.min_velocity_mps = autoware::universe_utils::kmph2mps(param_.min_vel_kmph);
    dynamic_obstacle.max_velocity_mps = autoware::universe_utils::kmph2mps(param_.max_vel_kmph);

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
    dynamic_obstacle.uuid = autoware::universe_utils::generateUUID();
    dynamic_obstacles.emplace_back(dynamic_obstacle);
  }

  return dynamic_obstacles;
}

void DynamicObstacleCreatorForPoints::onCompareMapFilteredPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (msg->data.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacle_points_map_filtered_.clear();
    return;
  }

  // transform and convert to pcl points for easier handling
  const auto transform_matrix =
    getTransformMatrix(tf_buffer_, "map", msg->header.frame_id, msg->header.stamp);
  if (!transform_matrix) {
    return;
  }
  const pcl::PointCloud<pcl::PointXYZ> pc_transformed =
    transformPointCloud(*msg, *transform_matrix);

  // apply voxel grid filter to reduce calculation cost
  const auto voxel_grid_filtered_points = applyVoxelGridFilter(pc_transformed);

  // these variables are written in another callback
  mutex_.lock();
  const auto detection_area_polygon = dynamic_obstacle_data_.detection_area;
  const auto path = dynamic_obstacle_data_.path;
  mutex_.unlock();

  // filter obstacle points within detection area polygon
  const auto detection_area_filtered_points =
    extractObstaclePointsWithinPolygon(voxel_grid_filtered_points, detection_area_polygon);

  // filter points that have lateral nearest distance
  const auto lateral_nearest_points =
    extractLateralNearestPoints(detection_area_filtered_points, path, param_.points_interval);

  std::lock_guard<std::mutex> lock(mutex_);
  obstacle_points_map_filtered_ = lateral_nearest_points;
}

void DynamicObstacleCreatorForPoints::onSynchronizedPointCloud(
  const PointCloud2::ConstSharedPtr compare_map_filtered_points,
  const PointCloud2::ConstSharedPtr vector_map_filtered_points)
{
  if (compare_map_filtered_points->data.empty() && vector_map_filtered_points->data.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacle_points_map_filtered_.clear();
    debug_ptr_->publishEmptyPointCloud();
    return;
  }

  // transform pointcloud and convert to pcl points for easier handling
  const auto transform_matrix = getTransformMatrix(
    tf_buffer_, "map", compare_map_filtered_points->header.frame_id,
    compare_map_filtered_points->header.stamp);
  if (!transform_matrix) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZ> compare_map_filtered_points_transformed =
    transformPointCloud(*compare_map_filtered_points, *transform_matrix);
  pcl::PointCloud<pcl::PointXYZ> vector_map_filtered_points_transformed =
    transformPointCloud(*vector_map_filtered_points, *transform_matrix);
  compare_map_filtered_points_transformed.header.frame_id = "map";
  vector_map_filtered_points_transformed.header.frame_id = "map";

  // apply voxel grid filter to reduce calculation cost
  const auto voxel_grid_filtered_compare_map_points =
    applyVoxelGridFilter(compare_map_filtered_points_transformed);
  const auto voxel_grid_filtered_vector_map_points =
    applyVoxelGridFilter(vector_map_filtered_points_transformed);

  // these variables are written in another callback
  mutex_.lock();
  const auto mandatory_detection_area = dynamic_obstacle_data_.mandatory_detection_area;
  const auto detection_area = dynamic_obstacle_data_.detection_area;
  const auto path = dynamic_obstacle_data_.path;
  mutex_.unlock();

  // filter obstacle points within detection area polygon
  const auto detection_area_filtered_compare_map_points = extractObstaclePointsWithinPolygon(
    voxel_grid_filtered_compare_map_points, mandatory_detection_area);
  const auto detection_area_filtered_vector_map_points =
    extractObstaclePointsWithinPolygon(voxel_grid_filtered_vector_map_points, detection_area);

  // concatenate two filtered pointclouds
  const auto concat_points = concatPointCloud(
    detection_area_filtered_compare_map_points, detection_area_filtered_vector_map_points);

  // remove overlap points
  const auto concat_points_no_overlap = applyVoxelGridFilter(concat_points);

  // filter points that have lateral nearest distance
  const auto lateral_nearest_points =
    extractLateralNearestPoints(concat_points_no_overlap, path, param_.points_interval);

  // publish filtered pointcloud for debug
  debug_ptr_->publishFilteredPointCloud(lateral_nearest_points, concat_points.header);

  std::lock_guard<std::mutex> lock(mutex_);
  obstacle_points_map_filtered_ = lateral_nearest_points;
}
}  // namespace autoware::behavior_velocity_planner
