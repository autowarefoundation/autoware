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
Eigen::Vector2d calcRawImageProjectedPoint(
  const image_geometry::PinholeCameraModel & pinhole_camera_model, const cv::Point3d & point3d)
{
  const cv::Point2d rectified_image_point = pinhole_camera_model.project3dToPixel(point3d);

  const cv::Point2d raw_image_point = pinhole_camera_model.unrectifyPoint(rectified_image_point);

  return Eigen::Vector2d(raw_image_point.x, raw_image_point.y);
}

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

void closest_cluster(
  const PointCloud2 & cluster, const double cluster_2d_tolerance, const int min_cluster_size,
  const pcl::PointXYZ & center, PointCloud2 & out_cluster)
{
  // sort point by distance to camera origin

  int x_offset = cluster.fields[pcl::getFieldIndex(cluster, "x")].offset;
  int y_offset = cluster.fields[pcl::getFieldIndex(cluster, "y")].offset;
  int z_offset = cluster.fields[pcl::getFieldIndex(cluster, "z")].offset;
  int point_step = cluster.point_step;
  auto func = [](const PointData & p1, const PointData & p2) { return p1.distance < p2.distance; };
  // Create and sort points_data by distance to camera origin
  std::vector<PointData> points_data;
  for (size_t i = 0; i < cluster.data.size() / point_step; ++i) {
    PointData point_data;
    pcl::PointXYZ point;
    std::memcpy(&point.x, &cluster.data[i * point_step + x_offset], sizeof(float));
    std::memcpy(&point.y, &cluster.data[i * point_step + y_offset], sizeof(float));
    std::memcpy(&point.z, &cluster.data[i * point_step + z_offset], sizeof(float));

    point_data.distance = autoware::universe_utils::calcDistance2d(center, point);
    point_data.orig_index = i;
    points_data.push_back(point_data);
  }
  std::sort(points_data.begin(), points_data.end(), func);
  // extract closest points as output cluster
  out_cluster.data.resize(cluster.data.size());
  out_cluster.point_step = cluster.point_step;
  out_cluster.height = cluster.height;
  size_t out_cluster_size = 0;
  for (size_t i = 0; i < points_data.size(); ++i) {
    if (out_cluster_size == 0) {
      std::memcpy(
        &out_cluster.data[out_cluster_size],
        &cluster.data[points_data.at(i).orig_index * point_step], point_step);
      out_cluster_size += point_step;
      continue;
    }
    if (points_data.at(i).distance - points_data.at(i - 1).distance < cluster_2d_tolerance) {
      std::memcpy(
        &out_cluster.data[out_cluster_size],
        &cluster.data[points_data.at(i).orig_index * point_step], point_step);
      out_cluster_size += point_step;
      continue;
    }
    if (i >= static_cast<size_t>(min_cluster_size)) {
      out_cluster.data.resize(out_cluster_size);
      return;
    }
    std::memcpy(
      &out_cluster.data[0], &cluster.data[points_data.at(i).orig_index * point_step], point_step);
    out_cluster_size = point_step;
  }
  out_cluster.data.resize(out_cluster_size);
}

void updateOutputFusedObjects(
  std::vector<DetectedObjectWithFeature> & output_objs, std::vector<PointCloud2> & clusters,
  const std::vector<size_t> clusters_data_size, const PointCloud2 & in_cloud,
  const std_msgs::msg::Header & in_roi_header, const tf2_ros::Buffer & tf_buffer,
  const int min_cluster_size, const int max_cluster_size, const float cluster_2d_tolerance,
  std::vector<DetectedObjectWithFeature> & output_fused_objects)
{
  if (output_objs.size() != clusters.size()) {
    return;
  }
  Eigen::Vector3d orig_camera_frame, orig_point_frame;
  Eigen::Affine3d camera2lidar_affine;
  orig_camera_frame << 0.0, 0.0, 0.0;
  const std_msgs::msg::Header & in_cloud_header = in_cloud.header;
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
    auto & cluster = clusters.at(i);
    cluster.data.resize(clusters_data_size.at(i));
    auto & feature_obj = output_objs.at(i);
    if (
      cluster.data.size() < std::size_t(min_cluster_size * cluster.point_step) ||
      cluster.data.size() >= std::size_t(max_cluster_size * cluster.point_step)) {
      continue;
    }

    // TODO(badai-nguyen): change to interface to select refine criteria like closest, largest
    //  to output refine cluster and centroid
    sensor_msgs::msg::PointCloud2 refine_cluster;
    closest_cluster(
      cluster, cluster_2d_tolerance, min_cluster_size, camera_orig_point_frame, refine_cluster);
    if (refine_cluster.data.size() < std::size_t(min_cluster_size * cluster.point_step)) {
      continue;
    }

    refine_cluster.width = refine_cluster.data.size() / in_cloud.point_step / in_cloud.height;
    refine_cluster.row_step = refine_cluster.data.size() / in_cloud.height;
    refine_cluster.is_bigendian = in_cloud.is_bigendian;
    refine_cluster.is_dense = in_cloud.is_dense;
    refine_cluster.header = in_cloud.header;
    refine_cluster.fields = in_cloud.fields;
    feature_obj.object.kinematics.pose_with_covariance.pose.position = getCentroid(refine_cluster);
    feature_obj.object.existence_probability = 1.0f;
    feature_obj.feature.cluster = refine_cluster;
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

  autoware_perception_msgs::msg::Shape shape;
  for (size_t i = 0; i < cluster2d_convex.size(); ++i) {
    geometry_msgs::msg::Point32 point;
    point.x = cluster2d_convex.at(i).x / 1000.0;
    point.y = cluster2d_convex.at(i).y / 1000.0;
    point.z = 0.0;
    shape.footprint.points.push_back(point);
  }
  shape.type = autoware_perception_msgs::msg::Shape::POLYGON;
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
    double dist_closest_point = autoware::universe_utils::calcDistance2d(point, orig_point);
    if (min_dist > dist_closest_point) {
      min_dist = dist_closest_point;
      closest_point = pcl::PointXYZ(point.x, point.y, point.z);
    }
  }
  return closest_point;
}

}  // namespace image_projection_based_fusion
