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

#include "image_projection_based_fusion/utils/utils.hpp"
namespace image_projection_based_fusion
{

std::optional<geometry_msgs::msg::TransformStamped> getTransformStamped(
  const tf2_ros::Buffer & tf_buffer, const std::string & target_frame_id,
  const std::string & source_frame_id, const rclcpp::Time & time)
{
  try {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf_buffer.lookupTransform(
      target_frame_id, source_frame_id, time, rclcpp::Duration::from_seconds(0.01));
    return transform_stamped;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("image_projection_based_fusion"), ex.what());
    return std::nullopt;
  }
}

Eigen::Affine3d transformToEigen(const geometry_msgs::msg::Transform & t)
{
  Eigen::Affine3d a;
  a.matrix() = tf2::transformToEigen(t).matrix();
  return a;
}

void convertCluster2FeatureObject(
  const std_msgs::msg::Header & header, const PointCloud & cluster,
  DetectedObjectWithFeature & feature_obj)
{
  PointCloud2 ros_cluster;
  pcl::toROSMsg(cluster, ros_cluster);
  ros_cluster.header = header;
  feature_obj.feature.cluster = ros_cluster;
  feature_obj.object.kinematics.pose_with_covariance.pose.position = getCentroid(ros_cluster);
  feature_obj.object.existence_probability = 1.0f;
}

PointCloud closest_cluster(
  PointCloud & cluster, const double cluster_2d_tolerance, const int min_cluster_size,
  const pcl::PointXYZ & center)
{
  // sort point by distance to camera origin

  auto func = [center](const pcl::PointXYZ & p1, const pcl::PointXYZ & p2) {
    return tier4_autoware_utils::calcDistance2d(center, p1) <
           tier4_autoware_utils::calcDistance2d(center, p2);
  };
  std::sort(cluster.begin(), cluster.end(), func);
  PointCloud out_cluster;
  for (auto & point : cluster) {
    if (out_cluster.empty()) {
      out_cluster.push_back(point);
      continue;
    }
    if (tier4_autoware_utils::calcDistance2d(out_cluster.back(), point) < cluster_2d_tolerance) {
      out_cluster.push_back(point);
      continue;
    }
    if (out_cluster.size() >= static_cast<std::size_t>(min_cluster_size)) {
      return out_cluster;
    }
    out_cluster.clear();
    out_cluster.push_back(point);
  }
  return out_cluster;
}

void updateOutputFusedObjects(
  std::vector<DetectedObjectWithFeature> & output_objs, const std::vector<PointCloud> & clusters,
  const std_msgs::msg::Header & in_cloud_header, const std_msgs::msg::Header & in_roi_header,
  const tf2_ros::Buffer & tf_buffer, const int min_cluster_size, const float cluster_2d_tolerance,
  std::vector<DetectedObjectWithFeature> & output_fused_objects)
{
  if (output_objs.size() != clusters.size()) {
    return;
  }
  Eigen::Vector3d orig_camera_frame, orig_point_frame;
  Eigen::Affine3d camera2lidar_affine;
  orig_camera_frame << 0.0, 0.0, 0.0;
  {
    const auto transform_stamped_optional = getTransformStamped(
      tf_buffer, in_cloud_header.frame_id, in_roi_header.frame_id, in_roi_header.stamp);
    if (!transform_stamped_optional) {
      return;
    }
    camera2lidar_affine = transformToEigen(transform_stamped_optional.value().transform);
  }
  orig_point_frame = camera2lidar_affine * orig_camera_frame;
  pcl::PointXYZ camera_orig_point_frame =
    pcl::PointXYZ(orig_point_frame.x(), orig_point_frame.y(), orig_point_frame.z());

  for (std::size_t i = 0; i < output_objs.size(); ++i) {
    PointCloud cluster = clusters.at(i);
    auto & feature_obj = output_objs.at(i);
    if (cluster.points.size() < std::size_t(min_cluster_size)) {
      continue;
    }

    // TODO(badai-nguyen): change to interface to select refine criteria like closest, largest
    //  to output refine cluster and centroid
    auto refine_cluster =
      closest_cluster(cluster, cluster_2d_tolerance, min_cluster_size, camera_orig_point_frame);
    if (refine_cluster.points.size() < std::size_t(min_cluster_size)) {
      continue;
    }

    refine_cluster.width = refine_cluster.points.size();
    refine_cluster.height = 1;
    refine_cluster.is_dense = false;
    // convert cluster to object
    convertCluster2FeatureObject(in_cloud_header, refine_cluster, feature_obj);
    output_fused_objects.push_back(feature_obj);
  }
}

void addShapeAndKinematic(
  const pcl::PointCloud<pcl::PointXYZ> & cluster,
  tier4_perception_msgs::msg::DetectedObjectWithFeature & feature_obj)
{
  if (cluster.empty()) {
    return;
  }
  pcl::PointXYZ centroid = pcl::PointXYZ(0.0, 0.0, 0.0);
  float max_z = -1e6;
  float min_z = 1e6;
  for (const auto & point : cluster) {
    centroid.x += point.x;
    centroid.y += point.y;
    centroid.z += point.z;
    max_z = max_z < point.z ? point.z : max_z;
    min_z = min_z > point.z ? point.z : min_z;
  }
  centroid.x = centroid.x / static_cast<double>(cluster.size());
  centroid.y = centroid.y / static_cast<double>(cluster.size());
  centroid.z = centroid.z / static_cast<double>(cluster.size());

  std::vector<cv::Point> cluster2d;
  std::vector<cv::Point> cluster2d_convex;

  for (size_t i = 0; i < cluster.size(); ++i) {
    cluster2d.push_back(
      cv::Point((cluster.at(i).x - centroid.x) * 1000.0, (cluster.at(i).y - centroid.y) * 1000.));
  }
  cv::convexHull(cluster2d, cluster2d_convex);
  if (cluster2d_convex.empty()) {
    return;
  }
  pcl::PointXYZ polygon_centroid = pcl::PointXYZ(0.0, 0.0, 0.0);
  for (size_t i = 0; i < cluster2d_convex.size(); ++i) {
    polygon_centroid.x += static_cast<double>(cluster2d_convex.at(i).x) / 1000.0;
    polygon_centroid.y += static_cast<double>(cluster2d_convex.at(i).y) / 1000.0;
  }
  polygon_centroid.x = polygon_centroid.x / static_cast<double>(cluster2d_convex.size());
  polygon_centroid.y = polygon_centroid.y / static_cast<double>(cluster2d_convex.size());

  autoware_auto_perception_msgs::msg::Shape shape;
  for (size_t i = 0; i < cluster2d_convex.size(); ++i) {
    geometry_msgs::msg::Point32 point;
    point.x = cluster2d_convex.at(i).x / 1000.0;
    point.y = cluster2d_convex.at(i).y / 1000.0;
    point.z = 0.0;
    shape.footprint.points.push_back(point);
  }
  shape.type = autoware_auto_perception_msgs::msg::Shape::POLYGON;
  constexpr float eps = 0.01;
  shape.dimensions.x = 0;
  shape.dimensions.y = 0;
  shape.dimensions.z = std::max((max_z - min_z), eps);
  feature_obj.object.shape = shape;
  feature_obj.object.kinematics.pose_with_covariance.pose.position.x =
    centroid.x + polygon_centroid.x;
  feature_obj.object.kinematics.pose_with_covariance.pose.position.y =
    centroid.y + polygon_centroid.y;
  feature_obj.object.kinematics.pose_with_covariance.pose.position.z =
    min_z + shape.dimensions.z * 0.5;
  feature_obj.object.existence_probability = 1.0;
  feature_obj.object.kinematics.pose_with_covariance.pose.orientation.x = 0;
  feature_obj.object.kinematics.pose_with_covariance.pose.orientation.y = 0;
  feature_obj.object.kinematics.pose_with_covariance.pose.orientation.z = 0;
  feature_obj.object.kinematics.pose_with_covariance.pose.orientation.w = 1;
}

geometry_msgs::msg::Point getCentroid(const sensor_msgs::msg::PointCloud2 & pointcloud)
{
  geometry_msgs::msg::Point centroid;
  centroid.x = 0.0f;
  centroid.y = 0.0f;
  centroid.z = 0.0f;
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x"),
       iter_y(pointcloud, "y"), iter_z(pointcloud, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    centroid.x += *iter_x;
    centroid.y += *iter_y;
    centroid.z += *iter_z;
  }
  const size_t size = pointcloud.width * pointcloud.height;
  centroid.x = centroid.x / static_cast<float>(size);
  centroid.y = centroid.y / static_cast<float>(size);
  centroid.z = centroid.z / static_cast<float>(size);
  return centroid;
}

pcl::PointXYZ getClosestPoint(const pcl::PointCloud<pcl::PointXYZ> & cluster)
{
  pcl::PointXYZ closest_point;
  double min_dist = 1e6;
  pcl::PointXYZ orig_point = pcl::PointXYZ(0.0, 0.0, 0.0);
  for (std::size_t i = 0; i < cluster.points.size(); ++i) {
    pcl::PointXYZ point = cluster.points.at(i);
    double dist_closest_point = tier4_autoware_utils::calcDistance2d(point, orig_point);
    if (min_dist > dist_closest_point) {
      min_dist = dist_closest_point;
      closest_point = pcl::PointXYZ(point.x, point.y, point.z);
    }
  }
  return closest_point;
}

}  // namespace image_projection_based_fusion
