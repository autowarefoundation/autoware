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

#include "object_recognition_utils/object_recognition_utils.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

using Label = autoware_auto_perception_msgs::msg::ObjectClassification;
namespace
{
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

boost::optional<ReferenceYawInfo> getReferenceYawInfo(const uint8_t label, const float yaw)
{
  const bool is_vehicle =
    Label::CAR == label || Label::TRUCK == label || Label::BUS == label || Label::TRAILER == label;
  if (is_vehicle) {
    return ReferenceYawInfo{yaw, tier4_autoware_utils::deg2rad(30)};
  } else {
    return boost::none;
  }
}

boost::optional<ReferenceShapeSizeInfo> getReferenceShapeSizeInfo(
  const uint8_t label, const autoware_auto_perception_msgs::msg::Shape & shape)
{
  const bool is_vehicle =
    Label::CAR == label || Label::TRUCK == label || Label::BUS == label || Label::TRAILER == label;
  if (is_vehicle) {
    return ReferenceShapeSizeInfo{shape, ReferenceShapeSizeInfo::Mode::Min};
  } else {
    return boost::none;
  }
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
    const float yaw_hat = tier4_autoware_utils::normalizeRadian(yaw + wz * dt.seconds());
    estimated_object.kinematics.pose_with_covariance.pose.orientation =
      tier4_autoware_utils::createQuaternionFromYaw(yaw_hat);
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

  // Set parameters
  tracker_ignore_.UNKNOWN = declare_parameter<bool>("tracker_ignore_label.UNKNOWN");
  tracker_ignore_.CAR = declare_parameter<bool>("tracker_ignore_label.CAR");
  tracker_ignore_.TRUCK = declare_parameter<bool>("tracker_ignore_label.TRUCK");
  tracker_ignore_.BUS = declare_parameter<bool>("tracker_ignore_label.BUS");
  tracker_ignore_.TRAILER = declare_parameter<bool>("tracker_ignore_label.TRAILER");
  tracker_ignore_.MOTORCYCLE = declare_parameter<bool>("tracker_ignore_label.MOTORCYCLE");
  tracker_ignore_.BICYCLE = declare_parameter<bool>("tracker_ignore_label.BICYCLE");
  tracker_ignore_.PEDESTRIAN = declare_parameter<bool>("tracker_ignore_label.PEDESTRIAN");

  // set maximum search setting for merger/divider
  setMaxSearchRange();

  shape_estimator_ = std::make_shared<ShapeEstimator>(true, true);
  cluster_ = std::make_shared<euclidean_cluster::VoxelGridBasedEuclideanCluster>(
    false, 10, 10000, 0.7, 0.3, 0);
  debugger_ = std::make_shared<Debugger>(this);
}

void DetectionByTracker::setMaxSearchRange()
{
  using Label = autoware_auto_perception_msgs::msg::ObjectClassification;
  // set max search distance for merger
  max_search_distance_for_merger_[Label::UNKNOWN] = 5.0;
  max_search_distance_for_merger_[Label::CAR] = 5.0;
  max_search_distance_for_merger_[Label::TRUCK] = 8.0;
  max_search_distance_for_merger_[Label::BUS] = 8.0;
  max_search_distance_for_merger_[Label::TRAILER] = 10.0;
  max_search_distance_for_merger_[Label::MOTORCYCLE] = 2.0;
  max_search_distance_for_merger_[Label::BICYCLE] = 1.0;
  max_search_distance_for_merger_[Label::PEDESTRIAN] = 1.0;

  // set max search distance for divider
  max_search_distance_for_divider_[Label::UNKNOWN] = 6.0;
  max_search_distance_for_divider_[Label::CAR] = 6.0;
  max_search_distance_for_divider_[Label::TRUCK] = 9.0;
  max_search_distance_for_divider_[Label::BUS] = 9.0;
  max_search_distance_for_divider_[Label::TRAILER] = 11.0;
  max_search_distance_for_divider_[Label::MOTORCYCLE] = 3.0;
  max_search_distance_for_divider_[Label::BICYCLE] = 2.0;
  max_search_distance_for_divider_[Label::PEDESTRIAN] = 2.0;
}

void DetectionByTracker::onObjects(
  const tier4_perception_msgs::msg::DetectedObjectsWithFeature::ConstSharedPtr input_msg)
{
  autoware_auto_perception_msgs::msg::DetectedObjects detected_objects;
  detected_objects.header = input_msg->header;

  // get objects from tracking module
  autoware_auto_perception_msgs::msg::DetectedObjects tracked_objects;
  {
    autoware_auto_perception_msgs::msg::TrackedObjects objects, transformed_objects;
    const bool available_trackers =
      tracker_handler_.estimateTrackedObjects(input_msg->header.stamp, objects);
    if (
      !available_trackers ||
      !object_recognition_utils::transformObjects(
        objects, input_msg->header.frame_id, tf_buffer_, transformed_objects)) {
      objects_pub_->publish(detected_objects);
      return;
    }
    // to simplify post processes, convert tracked_objects to DetectedObjects message.
    tracked_objects = object_recognition_utils::toDetectedObjects(transformed_objects);
  }
  debugger_->publishInitialObjects(*input_msg);
  debugger_->publishTrackedObjects(tracked_objects);

  // merge over segmented objects
  tier4_perception_msgs::msg::DetectedObjectsWithFeature merged_objects;
  autoware_auto_perception_msgs::msg::DetectedObjects no_found_tracked_objects;
  mergeOverSegmentedObjects(tracked_objects, *input_msg, no_found_tracked_objects, merged_objects);
  debugger_->publishMergedObjects(merged_objects);

  // divide under segmented objects
  tier4_perception_msgs::msg::DetectedObjectsWithFeature divided_objects;
  autoware_auto_perception_msgs::msg::DetectedObjects temp_no_found_tracked_objects;
  divideUnderSegmentedObjects(
    no_found_tracked_objects, *input_msg, temp_no_found_tracked_objects, divided_objects);
  debugger_->publishDividedObjects(divided_objects);

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
  constexpr float min_score_threshold = 0.4;

  out_objects.header = in_cluster_objects.header;
  out_no_found_tracked_objects.header = tracked_objects.header;

  for (const auto & tracked_object : tracked_objects.objects) {
    const auto & label = tracked_object.classification.front().label;
    if (tracker_ignore_.isIgnore(label)) continue;

    // change search range according to label type
    const float max_search_range = max_search_distance_for_divider_[label];

    std::optional<tier4_perception_msgs::msg::DetectedObjectWithFeature>
      highest_score_divided_object = std::nullopt;
    float highest_score = 0.0;

    for (const auto & initial_object : in_cluster_objects.feature_objects) {
      // search near object
      const float distance = tier4_autoware_utils::calcDistance2d(
        tracked_object.kinematics.pose_with_covariance.pose,
        initial_object.object.kinematics.pose_with_covariance.pose);
      if (max_search_range < distance) {
        continue;
      }
      // detect under segmented cluster
      const float recall =
        object_recognition_utils::get2dRecall(initial_object.object, tracked_object);
      const float precision =
        object_recognition_utils::get2dPrecision(initial_object.object, tracked_object);
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

  const auto & label = target_object.classification.front().label;

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
        label, divided_cluster,
        getReferenceYawInfo(
          label, tf2::getYaw(target_object.kinematics.pose_with_covariance.pose.orientation)),
        getReferenceShapeSizeInfo(label, target_object.shape),
        highest_iou_object_in_current_iter.object.shape,
        highest_iou_object_in_current_iter.object.kinematics.pose_with_covariance.pose);
      if (!is_shape_estimated) {
        continue;
      }
      const float iou = object_recognition_utils::get2dIoU(
        highest_iou_object_in_current_iter.object, target_object);
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
  highest_iou_object.object.existence_probability =
    object_recognition_utils::get2dIoU(target_object, highest_iou_object.object);

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
  out_objects.header = in_cluster_objects.header;
  out_no_found_tracked_objects.header = tracked_objects.header;

  for (const auto & tracked_object : tracked_objects.objects) {
    const auto & label = tracked_object.classification.front().label;
    if (tracker_ignore_.isIgnore(label)) continue;

    // change search range according to label type
    const float max_search_range = max_search_distance_for_merger_[label];

    // extend shape
    autoware_auto_perception_msgs::msg::DetectedObject extended_tracked_object = tracked_object;
    extended_tracked_object.shape = extendShape(tracked_object.shape, /*scale*/ 1.1);

    pcl::PointCloud<pcl::PointXYZ> pcl_merged_cluster;
    for (const auto & initial_object : in_cluster_objects.feature_objects) {
      const float distance = tier4_autoware_utils::calcDistance2d(
        tracked_object.kinematics.pose_with_covariance.pose,
        initial_object.object.kinematics.pose_with_covariance.pose);

      if (max_search_range < distance) {
        continue;
      }

      // If there is an initial object in the tracker, it will be merged.
      const float precision =
        object_recognition_utils::get2dPrecision(initial_object.object, extended_tracked_object);
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
    feature_object.object.classification = tracked_object.classification;

    bool is_shape_estimated = shape_estimator_->estimateShapeAndPose(
      label, pcl_merged_cluster,
      getReferenceYawInfo(
        label, tf2::getYaw(tracked_object.kinematics.pose_with_covariance.pose.orientation)),
      getReferenceShapeSizeInfo(label, tracked_object.shape), feature_object.object.shape,
      feature_object.object.kinematics.pose_with_covariance.pose);
    if (!is_shape_estimated) {
      out_no_found_tracked_objects.objects.push_back(tracked_object);
      continue;
    }

    feature_object.object.existence_probability =
      object_recognition_utils::get2dIoU(tracked_object, feature_object.object);
    setClusterInObjectWithFeature(in_cluster_objects.header, pcl_merged_cluster, feature_object);
    out_objects.feature_objects.push_back(feature_object);
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(DetectionByTracker)
