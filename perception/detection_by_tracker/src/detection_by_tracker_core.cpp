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

#include "detection_by_tracker/detection_by_tracker_core.hpp"

#include "detection_by_tracker/utils.hpp"

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace
{
std::optional<geometry_msgs::msg::Transform> getTransform(
  const tf2_ros::Buffer & tf_buffer, const std::string & source_frame_id,
  const std::string & target_frame_id, const rclcpp::Time & time)
{
  try {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf_buffer.lookupTransform(
      /*target*/ target_frame_id, /*src*/ source_frame_id, time,
      rclcpp::Duration::from_seconds(0.5));
    return transform_stamped.transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("detection_by_tracker"), ex.what());
    return std::nullopt;
  }
}

bool transformTrackedObjects(
  const autoware_auto_perception_msgs::msg::TrackedObjects & input_msg,
  const std::string & target_frame_id, const tf2_ros::Buffer & tf_buffer,
  autoware_auto_perception_msgs::msg::TrackedObjects & output_msg)
{
  output_msg = input_msg;

  // Transform to world coordinate
  if (output_msg.header.frame_id != target_frame_id) {
    output_msg.header.frame_id = target_frame_id;
    tf2::Transform tf_target2objects_world;
    tf2::Transform tf_target2objects;
    tf2::Transform tf_objects_world2objects;
    {
      const auto ros_target2objects_world =
        getTransform(tf_buffer, input_msg.header.frame_id, target_frame_id, input_msg.header.stamp);
      if (!ros_target2objects_world) {
        return false;
      }
      tf2::fromMsg(*ros_target2objects_world, tf_target2objects_world);
    }
    for (auto & object : output_msg.objects) {
      tf2::fromMsg(object.kinematics.pose_with_covariance.pose, tf_objects_world2objects);
      tf_target2objects = tf_target2objects_world * tf_objects_world2objects;
      tf2::toMsg(tf_target2objects, object.kinematics.pose_with_covariance.pose);
    }
  }
  return true;
}

void setClusterInObjectWithFeature(
  const std_msgs::msg::Header & header, const pcl::PointCloud<pcl::PointXYZ> & cluster,
  tier4_perception_msgs::msg::DetectedObjectWithFeature & feature_object)
{
  sensor_msgs::msg::PointCloud2 ros_pointcloud;
  pcl::toROSMsg(cluster, ros_pointcloud);
  ros_pointcloud.header = header;
  feature_object.feature.cluster = ros_pointcloud;
}
autoware_auto_perception_msgs::msg::Shape extendShape(
  const autoware_auto_perception_msgs::msg::Shape & shape, const float scale)
{
  autoware_auto_perception_msgs::msg::Shape output = shape;
  output.dimensions.x *= scale;
  output.dimensions.y *= scale;
  output.dimensions.z *= scale;
  for (auto & point : output.footprint.points) {
    point.x *= scale;
    point.y *= scale;
    point.z *= scale;
  }
  return output;
}

autoware_auto_perception_msgs::msg::DetectedObjects toDetectedObjects(
  const autoware_auto_perception_msgs::msg::TrackedObjects & tracked_objects)
{
  autoware_auto_perception_msgs::msg::DetectedObjects out_objects;
  out_objects.header = tracked_objects.header;
  for (auto & tracked_object : tracked_objects.objects) {
    autoware_auto_perception_msgs::msg::DetectedObject object;
    object.existence_probability = tracked_object.existence_probability;
    object.classification = tracked_object.classification;
    object.kinematics.pose_with_covariance = tracked_object.kinematics.pose_with_covariance;
    object.kinematics.has_position_covariance = true;
    object.kinematics.orientation_availability = tracked_object.kinematics.orientation_availability;
    object.kinematics.twist_with_covariance = tracked_object.kinematics.twist_with_covariance;
    object.kinematics.has_twist = true;
    object.kinematics.has_twist_covariance = true;
    object.shape = tracked_object.shape;
    out_objects.objects.push_back(object);
  }
  return out_objects;
}

}  // namespace

void TrackerHandler::onTrackedObjects(
  const autoware_auto_perception_msgs::msg::TrackedObjects::ConstSharedPtr msg)
{
  constexpr size_t max_buffer_size = 10;

  // Add tracked objects to buffer
  objects_buffer_.push_front(*msg);

  // Remove old data
  while (max_buffer_size < objects_buffer_.size()) {
    objects_buffer_.pop_back();
  }
}

bool TrackerHandler::estimateTrackedObjects(
  const rclcpp::Time & time, autoware_auto_perception_msgs::msg::TrackedObjects & output)
{
  if (objects_buffer_.empty()) {
    return false;
  }

  // Get the objects closest to the target time.
  const auto target_objects_iter = std::min_element(
    objects_buffer_.cbegin(), objects_buffer_.cend(),
    [&time](
      autoware_auto_perception_msgs::msg::TrackedObjects first,
      autoware_auto_perception_msgs::msg::TrackedObjects second) {
      return std::fabs((time - first.header.stamp).seconds()) <
             std::fabs((time - second.header.stamp).seconds());
    });

  // Estimate the pose of the object at the target time
  const auto dt = time - target_objects_iter->header.stamp;
  output.header.frame_id = target_objects_iter->header.frame_id;
  output.header.stamp = time;
  for (const auto & object : target_objects_iter->objects) {
    const auto & pose_with_covariance = object.kinematics.pose_with_covariance;
    const auto & x = pose_with_covariance.pose.position.x;
    const auto & y = pose_with_covariance.pose.position.y;
    const auto & z = pose_with_covariance.pose.position.z;
    const float yaw = tf2::getYaw(pose_with_covariance.pose.orientation);
    const auto & twist = object.kinematics.twist_with_covariance.twist;
    const float & vx = twist.linear.x;
    const float & wz = twist.angular.z;

    // build output
    autoware_auto_perception_msgs::msg::TrackedObject estimated_object;
    estimated_object.object_id = object.object_id;
    estimated_object.existence_probability = object.existence_probability;
    estimated_object.classification = object.classification;
    estimated_object.shape = object.shape;
    estimated_object.kinematics.pose_with_covariance.pose.position.x =
      x + vx * std::cos(yaw) * dt.seconds();
    estimated_object.kinematics.pose_with_covariance.pose.position.y =
      y + vx * std::sin(yaw) * dt.seconds();
    estimated_object.kinematics.pose_with_covariance.pose.position.z = z;
    const float yaw_hat = autoware_utils::normalizeRadian(yaw + wz * dt.seconds());
    estimated_object.kinematics.pose_with_covariance.pose.orientation =
      autoware_utils::createQuaternionFromYaw(yaw_hat);
    output.objects.push_back(estimated_object);
  }
  return true;
}

DetectionByTracker::DetectionByTracker(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("detection_by_tracker", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Create publishers and subscribers
  trackers_sub_ = create_subscription<autoware_auto_perception_msgs::msg::TrackedObjects>(
    "~/input/tracked_objects", rclcpp::QoS{1},
    std::bind(&TrackerHandler::onTrackedObjects, &tracker_handler_, std::placeholders::_1));
  initial_objects_sub_ =
    create_subscription<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
      "~/input/initial_objects", rclcpp::QoS{1},
      std::bind(&DetectionByTracker::onObjects, this, std::placeholders::_1));
  objects_pub_ = create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "~/output", rclcpp::QoS{1});

  shape_estimator_ = std::make_shared<ShapeEstimator>(true, true);
  cluster_ = std::make_shared<euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    false, 10, 10000, 0.7, 0.3, 0);
}

void DetectionByTracker::onObjects(
  const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_msg)
{
  autoware_auto_perception_msgs::msg::DetectedObjects detected_objects;
  detected_objects.header = input_msg->header;

  // get objects from tracking module
  autoware_auto_perception_msgs::msg::TrackedObjects tracked_objects;
  {
    autoware_auto_perception_msgs::msg::TrackedObjects objects;
    const bool available_trackers =
      tracker_handler_.estimateTrackedObjects(input_msg->header.stamp, objects);
    if (
      !available_trackers ||
      !transformTrackedObjects(objects, input_msg->header.frame_id, tf_buffer_, tracked_objects)) {
      objects_pub_->publish(detected_objects);
      return;
    }
  }

  // to simplify post processes, convert tracked_objects to DetectedObjects message.
  autoware_auto_perception_msgs::msg::DetectedObjects tracked_objects_dt;
  tracked_objects_dt = toDetectedObjects(tracked_objects);

  // merge over segmented objects
  tier4_perception_msgs::msg::DetectedObjectsWithFeature merged_objects;
  autoware_auto_perception_msgs::msg::DetectedObjects no_found_tracked_objects;
  mergeOverSegmentedObjects(
    tracked_objects_dt, *input_msg, no_found_tracked_objects, merged_objects);

  // divide under segmented objects
  tier4_perception_msgs::msg::DetectedObjectsWithFeature divided_objects;
  autoware_auto_perception_msgs::msg::DetectedObjects temp_no_found_tracked_objects;
  divideUnderSegmentedObjects(
    no_found_tracked_objects, *input_msg, temp_no_found_tracked_objects, divided_objects);

  // merge under/over segmented objects to build output objects
  for (const auto & merged_object : merged_objects.feature_objects) {
    detected_objects.objects.push_back(merged_object.object);
  }
  for (const auto & divided_object : divided_objects.feature_objects) {
    detected_objects.objects.push_back(divided_object.object);
  }

  objects_pub_->publish(detected_objects);
}

void DetectionByTracker::divideUnderSegmentedObjects(
  const autoware_auto_perception_msgs::msg::DetectedObjects & tracked_objects,
  const tier4_perception_msgs::msg::DetectedObjectsWithFeature & in_cluster_objects,
  autoware_auto_perception_msgs::msg::DetectedObjects & out_no_found_tracked_objects,
  tier4_perception_msgs::msg::DetectedObjectsWithFeature & out_objects)
{
  constexpr float recall_min_threshold = 0.4;
  constexpr float precision_max_threshold = 0.5;
  constexpr float max_search_range = 6.0;
  constexpr float min_score_threshold = 0.4;

  out_objects.header = in_cluster_objects.header;
  out_no_found_tracked_objects.header = tracked_objects.header;

  for (const auto & tracked_object : tracked_objects.objects) {
    std::optional<tier4_perception_msgs::msg::DetectedObjectWithFeature>
      highest_score_divided_object = std::nullopt;
    float highest_score = 0.0;

    for (const auto & initial_object : in_cluster_objects.feature_objects) {
      // search near object
      const float distance = autoware_utils::calcDistance2d(
        tracked_object.kinematics.pose_with_covariance.pose,
        initial_object.object.kinematics.pose_with_covariance.pose);
      if (max_search_range < distance) {
        continue;
      }
      // detect under segmented cluster
      const float recall = utils::get2dRecall(initial_object.object, tracked_object);
      const float precision = utils::get2dPrecision(initial_object.object, tracked_object);
      const bool is_under_segmented =
        (recall_min_threshold < recall && precision < precision_max_threshold);
      if (!is_under_segmented) {
        continue;
      }
      // optimize clustering
      tier4_perception_msgs::msg::DetectedObjectWithFeature divided_object;
      float score = optimizeUnderSegmentedObject(
        tracked_object, initial_object.feature.cluster, divided_object);
      if (score < min_score_threshold) {
        continue;
      }

      if (highest_score < score) {
        highest_score = score;
        highest_score_divided_object = divided_object;
      }
    }
    if (highest_score_divided_object) {  // found
      out_objects.feature_objects.push_back(highest_score_divided_object.value());
    } else {  // not found
      out_no_found_tracked_objects.objects.push_back(tracked_object);
    }
  }
}

float DetectionByTracker::optimizeUnderSegmentedObject(
  const autoware_auto_perception_msgs::msg::DetectedObject & target_object,
  const sensor_msgs::msg::PointCloud2 & under_segmented_cluster,
  tier4_perception_msgs::msg::DetectedObjectWithFeature & output)
{
  constexpr float iter_rate = 0.8;
  constexpr int iter_max_count = 5;
  constexpr float initial_cluster_range = 0.7;
  float cluster_range = initial_cluster_range;
  constexpr float initial_voxel_size = initial_cluster_range / 2.0f;
  float voxel_size = initial_voxel_size;

  // initialize clustering parameters
  euclidean_cluster::VoxelGridBasedEuclideanCluster cluster(
    false, 4, 10000, initial_cluster_range, initial_voxel_size, 0);

  // convert to pcl
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cluster(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(under_segmented_cluster, *pcl_cluster);

  // iterate to find best fit divided object
  float highest_iou = 0.0;
  tier4_perception_msgs::msg::DetectedObjectWithFeature highest_iou_object;
  for (int iter_count = 0; iter_count < iter_max_count;
       ++iter_count, cluster_range *= iter_rate, voxel_size *= iter_rate) {
    // divide under segmented cluster
    std::vector<pcl::PointCloud<pcl::PointXYZ>> divided_clusters;
    cluster.setTolerance(cluster_range);
    cluster.setVoxelLeafSize(voxel_size);
    cluster.cluster(pcl_cluster, divided_clusters);

    // find highest iou object in divided clusters
    float highest_iou_in_current_iter = 0.0f;
    tier4_perception_msgs::msg::DetectedObjectWithFeature highest_iou_object_in_current_iter;
    highest_iou_object_in_current_iter.object.classification = target_object.classification;
    for (const auto & divided_cluster : divided_clusters) {
      bool is_shape_estimated = shape_estimator_->estimateShapeAndPose(
        target_object.classification.front().label, divided_cluster,
        tf2::getYaw(target_object.kinematics.pose_with_covariance.pose.orientation),
        highest_iou_object_in_current_iter.object.shape,
        highest_iou_object_in_current_iter.object.kinematics.pose_with_covariance.pose);
      if (!is_shape_estimated) {
        continue;
      }
      const float iou = utils::get2dIoU(highest_iou_object_in_current_iter.object, target_object);
      if (highest_iou_in_current_iter < iou) {
        highest_iou_in_current_iter = iou;
        setClusterInObjectWithFeature(
          under_segmented_cluster.header, divided_cluster, highest_iou_object_in_current_iter);
      }
    }

    // finish iteration when current score is under previous score
    if (highest_iou_in_current_iter < highest_iou) {
      break;
    }

    // copy for next iteration
    highest_iou = highest_iou_in_current_iter;
    highest_iou_object = highest_iou_object_in_current_iter;
  }

  // build output
  highest_iou_object.object.classification = target_object.classification;
  // TODO(yukkysaito): It is necessary to consider appropriate values in the future.
  highest_iou_object.object.existence_probability = 0.1f;
  output = highest_iou_object;
  return highest_iou;
}

void DetectionByTracker::mergeOverSegmentedObjects(
  const autoware_auto_perception_msgs::msg::DetectedObjects & tracked_objects,
  const tier4_perception_msgs::msg::DetectedObjectsWithFeature & in_cluster_objects,
  autoware_auto_perception_msgs::msg::DetectedObjects & out_no_found_tracked_objects,
  tier4_perception_msgs::msg::DetectedObjectsWithFeature & out_objects)
{
  constexpr float precision_threshold = 0.5;
  constexpr float max_search_range = 5.0;
  out_objects.header = in_cluster_objects.header;
  out_no_found_tracked_objects.header = tracked_objects.header;

  for (const auto & tracked_object : tracked_objects.objects) {
    // extend shape
    autoware_auto_perception_msgs::msg::DetectedObject extended_tracked_object = tracked_object;
    extended_tracked_object.shape = extendShape(tracked_object.shape, /*scale*/ 1.1);

    pcl::PointCloud<pcl::PointXYZ> pcl_merged_cluster;
    for (const auto & initial_object : in_cluster_objects.feature_objects) {
      const float distance = autoware_utils::calcDistance2d(
        tracked_object.kinematics.pose_with_covariance.pose,
        initial_object.object.kinematics.pose_with_covariance.pose);

      if (max_search_range < distance) {
        continue;
      }

      // If there is an initial object in the tracker, it will be merged.
      const float precision = utils::get2dPrecision(initial_object.object, extended_tracked_object);
      if (precision < precision_threshold) {
        continue;
      }
      pcl::PointCloud<pcl::PointXYZ> pcl_cluster;
      pcl::fromROSMsg(initial_object.feature.cluster, pcl_cluster);
      pcl_merged_cluster += pcl_cluster;
    }

    if (pcl_merged_cluster.points.empty()) {  // if clusters aren't found
      out_no_found_tracked_objects.objects.push_back(tracked_object);
      continue;
    }

    // build output clusters
    tier4_perception_msgs::msg::DetectedObjectWithFeature feature_object;
    feature_object.object.existence_probability = 0.1f;
    feature_object.object.classification = tracked_object.classification;

    bool is_shape_estimated = shape_estimator_->estimateShapeAndPose(
      tracked_object.classification.front().label, pcl_merged_cluster,
      tf2::getYaw(tracked_object.kinematics.pose_with_covariance.pose.orientation),
      feature_object.object.shape, feature_object.object.kinematics.pose_with_covariance.pose);
    if (!is_shape_estimated) {
      out_no_found_tracked_objects.objects.push_back(tracked_object);
      continue;
    }

    setClusterInObjectWithFeature(in_cluster_objects.header, pcl_merged_cluster, feature_object);
    out_objects.feature_objects.push_back(feature_object);
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(DetectionByTracker)
