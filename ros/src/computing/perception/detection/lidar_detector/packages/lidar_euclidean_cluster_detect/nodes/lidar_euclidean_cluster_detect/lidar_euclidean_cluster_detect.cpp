#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <limits>
#include <cmath>

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/common/common.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <pcl/segmentation/extract_clusters.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

#include "autoware_msgs/centroids.h"
#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>

#include <vector_map/vector_map.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_rviz_plugins/Pictogram.h>
#include <jsk_rviz_plugins/PictogramArray.h>

#include <tf/tf.h>

#include <yaml-cpp/yaml.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/version.hpp>

#if (CV_MAJOR_VERSION == 3)
#include "gencolors.cpp"
#else

#include <opencv2/contrib/contrib.hpp>
#include <autoware_msgs/DetectedObjectArray.h>

#endif

#include "cluster.h"

#ifdef GPU_CLUSTERING

#include "gpu_euclidean_clustering.h"

#endif

using namespace cv;

ros::Publisher _pub_cluster_cloud;
ros::Publisher _pub_ground_cloud;
ros::Publisher _centroid_pub;
ros::Publisher _marker_pub;
ros::Publisher _pub_clusters_message;
ros::Publisher _pub_text_pictogram;

ros::Publisher _pub_points_lanes_cloud;
ros::Publisher _pub_jsk_boundingboxes;
ros::Publisher _pub_jsk_hulls;

ros::Publisher _pub_grid_map;

ros::Publisher _pub_detected_objects;

ros::ServiceClient _vectormap_server;

std_msgs::Header _velodyne_header;

std::string _output_frame;
std::string _vectormap_frame;
std::string _gridmap_layer;

grid_map::GridMap _wayarea_gridmap;

const int _grid_min_value = 0;
const int _grid_max_value = 255;
static int _gridmap_no_road_value;

static bool _velodyne_transform_available;
static bool _downsample_cloud;
static bool _pose_estimation;
static double _leaf_size;
static int _cluster_size_min;
static int _cluster_size_max;

static bool _remove_ground;  // only ground

static bool _using_sensor_cloud;
static bool _use_diffnormals;
static bool _use_vector_map;

static double _clip_min_height;
static double _clip_max_height;

static bool _keep_lanes;
static double _keep_lane_left_distance;
static double _keep_lane_right_distance;

static double _max_boundingbox_side;
static double _remove_points_upto;
static double _cluster_merge_threshold;
static double _clustering_distance;

static bool _use_gpu;
static std::chrono::system_clock::time_point _start, _end;

std::vector<std::vector<geometry_msgs::Point>> _way_area_points;
std::vector<cv::Scalar> _colors;
pcl::PointCloud<pcl::PointXYZ> _sensor_cloud;
visualization_msgs::Marker _visualization_marker;

static bool _use_multiple_thres;
std::vector<double> _clustering_distances;
std::vector<double> _clustering_ranges;

tf::StampedTransform* _transform;
tf::StampedTransform* _velodyne_output_transform;
tf::TransformListener* _transform_listener;
tf::TransformListener* _vectormap_transform_listener;

tf::StampedTransform findTransform(const std::string& in_target_frame, const std::string& in_source_frame)
{
  tf::StampedTransform transform;

  try
  {
    // What time should we use?
    _vectormap_transform_listener->lookupTransform(in_target_frame, in_source_frame, ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return transform;
  }

  return transform;
}

geometry_msgs::Point transformPoint(const geometry_msgs::Point& point, const tf::Transform& tf)
{
  tf::Point tf_point;
  tf::pointMsgToTF(point, tf_point);

  tf_point = tf * tf_point;

  geometry_msgs::Point ros_point;
  tf::pointTFToMsg(tf_point, ros_point);

  return ros_point;
}

bool checkPointInGrid(const grid_map::GridMap& in_grid_map, const cv::Mat& in_grid_image,
                      const geometry_msgs::Point& in_point)
{
  // calculate out_grid_map position
  grid_map::Position map_pos = in_grid_map.getPosition();
  double origin_x_offset = in_grid_map.getLength().x() / 2.0 - map_pos.x();
  double origin_y_offset = in_grid_map.getLength().y() / 2.0 - map_pos.y();
  // coordinate conversion for cv image
  double cv_x = (in_grid_map.getLength().y() - origin_y_offset - in_point.y) / in_grid_map.getResolution();
  double cv_y = (in_grid_map.getLength().x() - origin_x_offset - in_point.x) / in_grid_map.getResolution();

  // check coords are inside the gridmap
  if (cv_x < 0 || cv_x > in_grid_image.cols || cv_y < 0 || cv_y > in_grid_image.rows)
  {
    return false;
  }

  //_gridmap_no_road_value if road
  if (in_grid_image.at<uchar>(cv_y, cv_x) != _gridmap_no_road_value)
  {
    return true;
  }

  return false;
}

void transformBoundingBox(const jsk_recognition_msgs::BoundingBox& in_boundingbox,
                          jsk_recognition_msgs::BoundingBox& out_boundingbox, const std::string& in_target_frame,
                          const std_msgs::Header& in_header)
{
  geometry_msgs::PoseStamped pose_in, pose_out;
  pose_in.header = in_header;
  pose_in.pose = in_boundingbox.pose;
  try
  {
    _transform_listener->transformPose(in_target_frame, ros::Time(), pose_in, in_header.frame_id, pose_out);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("transformBoundingBox: %s", ex.what());
  }
  out_boundingbox.pose = pose_out.pose;
  out_boundingbox.header = in_header;
  out_boundingbox.header.frame_id = in_target_frame;
  out_boundingbox.dimensions = in_boundingbox.dimensions;
  out_boundingbox.value = in_boundingbox.value;
  out_boundingbox.label = in_boundingbox.label;
}

void publishDetectedObjects(const autoware_msgs::CloudClusterArray& in_clusters)
{
  autoware_msgs::DetectedObjectArray detected_objects;
  detected_objects.header = in_clusters.header;

  for (size_t i = 0; i < in_clusters.clusters.size(); i++)
  {
    autoware_msgs::DetectedObject detected_object;
    detected_object.header = in_clusters.header;
    detected_object.label = "unknown";
    detected_object.id = i;
    detected_object.score = 1.;
    detected_object.space_frame = in_clusters.header.frame_id;
    detected_object.pose = in_clusters.clusters[i].bounding_box.pose;
    detected_object.dimensions = in_clusters.clusters[i].dimensions;
    detected_object.pointcloud = in_clusters.clusters[i].cloud;
    detected_object.convex_hull = in_clusters.clusters[i].convex_hull;
    detected_object.color.r = 0.;
    detected_object.color.g = 1.;
    detected_object.color.b = 0.;
    detected_object.color.a = 1.;

    detected_objects.objects.push_back(detected_object);
  }
  _pub_detected_objects.publish(detected_objects);
}

void publishCloudClusters(const ros::Publisher* in_publisher, const autoware_msgs::CloudClusterArray& in_clusters,
                          const std::string& in_target_frame, const std_msgs::Header& in_header)
{
  if (in_target_frame != in_header.frame_id)
  {
    autoware_msgs::CloudClusterArray clusters_transformed;
    clusters_transformed.header = in_header;
    clusters_transformed.header.frame_id = in_target_frame;
    for (auto i = in_clusters.clusters.begin(); i != in_clusters.clusters.end(); i++)
    {
      autoware_msgs::CloudCluster cluster_transformed;
      cluster_transformed.header = in_header;
      try
      {
        _transform_listener->lookupTransform(in_target_frame, _velodyne_header.frame_id, ros::Time(), *_transform);
        pcl_ros::transformPointCloud(in_target_frame, *_transform, i->cloud, cluster_transformed.cloud);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), i->min_point, in_header.frame_id,
                                            cluster_transformed.min_point);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), i->max_point, in_header.frame_id,
                                            cluster_transformed.max_point);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), i->avg_point, in_header.frame_id,
                                            cluster_transformed.avg_point);
        _transform_listener->transformPoint(in_target_frame, ros::Time(), i->centroid_point, in_header.frame_id,
                                            cluster_transformed.centroid_point);

        cluster_transformed.dimensions = i->dimensions;
        cluster_transformed.eigen_values = i->eigen_values;
        cluster_transformed.eigen_vectors = i->eigen_vectors;

        transformBoundingBox(i->bounding_box, cluster_transformed.bounding_box, in_target_frame, in_header);

        clusters_transformed.clusters.push_back(cluster_transformed);
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR("publishCloudClusters: %s", ex.what());
      }
    }
    in_publisher->publish(clusters_transformed);
    publishDetectedObjects(clusters_transformed);
  }
  else
  {
    in_publisher->publish(in_clusters);
    publishDetectedObjects(in_clusters);
  }
}

void publishCentroids(const ros::Publisher* in_publisher, const autoware_msgs::centroids& in_centroids,
                      const std::string& in_target_frame, const std_msgs::Header& in_header)
{
  if (in_target_frame != in_header.frame_id)
  {
    autoware_msgs::centroids centroids_transformed;
    centroids_transformed.header = in_header;
    centroids_transformed.header.frame_id = in_target_frame;
    for (auto i = centroids_transformed.points.begin(); i != centroids_transformed.points.end(); i++)
    {
      geometry_msgs::PointStamped centroid_in, centroid_out;
      centroid_in.header = in_header;
      centroid_in.point = *i;
      try
      {
        _transform_listener->transformPoint(in_target_frame, ros::Time(), centroid_in, in_header.frame_id,
                                            centroid_out);

        centroids_transformed.points.push_back(centroid_out.point);
      }
      catch (tf::TransformException& ex)
      {
        ROS_ERROR("publishCentroids: %s", ex.what());
      }
    }
    in_publisher->publish(centroids_transformed);
  }
  else
  {
    in_publisher->publish(in_centroids);
  }
}

void publishBoundingBoxArray(const ros::Publisher* in_publisher,
                             const jsk_recognition_msgs::BoundingBoxArray& in_boundingbox_array,
                             const std::string& in_target_frame, const std_msgs::Header& in_header)
{
  if (in_target_frame != in_header.frame_id)
  {
    jsk_recognition_msgs::BoundingBoxArray boundingboxes_transformed;
    boundingboxes_transformed.header = in_header;
    boundingboxes_transformed.header.frame_id = in_target_frame;
    for (auto i = in_boundingbox_array.boxes.begin(); i != in_boundingbox_array.boxes.end(); i++)
    {
      jsk_recognition_msgs::BoundingBox boundingbox_transformed;
      transformBoundingBox(*i, boundingbox_transformed, in_target_frame, in_header);
      boundingboxes_transformed.boxes.push_back(boundingbox_transformed);
    }
    in_publisher->publish(boundingboxes_transformed);
  }
  else
  {
    in_publisher->publish(in_boundingbox_array);
  }
}

void publishCloud(const ros::Publisher* in_publisher, const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header = _velodyne_header;
  in_publisher->publish(cloud_msg);
}

void publishColorCloud(const ros::Publisher* in_publisher,
                       const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header = _velodyne_header;
  in_publisher->publish(cloud_msg);
}

void keepLanePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_left_lane_threshold = 1.5,
                    float in_right_lane_threshold = 1.5)
{
  pcl::PointIndices::Ptr far_indices(new pcl::PointIndices);
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    pcl::PointXYZ current_point;
    current_point.x = in_cloud_ptr->points[i].x;
    current_point.y = in_cloud_ptr->points[i].y;
    current_point.z = in_cloud_ptr->points[i].z;

    if (current_point.y > (in_left_lane_threshold) || current_point.y < -1.0 * in_right_lane_threshold)
    {
      far_indices->indices.push_back(i);
    }
  }
  out_cloud_ptr->points.clear();
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(in_cloud_ptr);
  extract.setIndices(far_indices);
  extract.setNegative(true);  // true removes the indices, false leaves only the indices
  extract.filter(*out_cloud_ptr);
}

#ifdef GPU_CLUSTERING

std::vector<ClusterPtr> clusterAndColorGpu(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                                           jsk_recognition_msgs::BoundingBoxArray& in_out_boundingbox_array,
                                           autoware_msgs::centroids& in_out_centroids,
                                           double in_max_cluster_distance = 0.5)
{
  std::vector<ClusterPtr> clusters;

  // Convert input point cloud to vectors of x, y, and z

  int size = in_cloud_ptr->points.size();

  if (size == 0)
    return clusters;

  float *tmp_x, *tmp_y, *tmp_z;

  tmp_x = (float*)malloc(sizeof(float) * size);
  tmp_y = (float*)malloc(sizeof(float) * size);
  tmp_z = (float*)malloc(sizeof(float) * size);

  for (int i = 0; i < size; i++)
  {
    pcl::PointXYZ tmp_point = in_cloud_ptr->at(i);

    tmp_x[i] = tmp_point.x;
    tmp_y[i] = tmp_point.y;
    tmp_z[i] = tmp_point.z;
  }

  GpuEuclideanCluster gecl_cluster;

  gecl_cluster.setInputPoints(tmp_x, tmp_y, tmp_z, size);
  gecl_cluster.setThreshold(in_max_cluster_distance);
  gecl_cluster.setMinClusterPts(_cluster_size_min);
  gecl_cluster.setMaxClusterPts(_cluster_size_max);
  gecl_cluster.extractClusters();
  std::vector<GpuEuclideanCluster::GClusterIndex> cluster_indices = gecl_cluster.getOutput();

  unsigned int k = 0;

  for (auto it = cluster_indices.begin(); it != cluster_indices.end(); it++)
  {
    ClusterPtr cluster(new Cluster());
    cluster->SetCloud(in_cloud_ptr, it->points_in_cluster, _velodyne_header, k, (int)_colors[k].val[0],
                      (int)_colors[k].val[1], (int)_colors[k].val[2], "", _pose_estimation);
    clusters.push_back(cluster);

    k++;
  }

  free(tmp_x);
  free(tmp_y);
  free(tmp_z);

  return clusters;
}

#endif

std::vector<ClusterPtr> clusterAndColor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                                        jsk_recognition_msgs::BoundingBoxArray& in_out_boundingbox_array,
                                        autoware_msgs::centroids& in_out_centroids,
                                        double in_max_cluster_distance = 0.5)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  // create 2d pc
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*in_cloud_ptr, *cloud_2d);
  // make it flat
  for (size_t i = 0; i < cloud_2d->points.size(); i++)
  {
    cloud_2d->points[i].z = 0;
  }

  if (cloud_2d->points.size() > 0)
    tree->setInputCloud(cloud_2d);

  std::vector<pcl::PointIndices> cluster_indices;

  // perform clustering on 2d cloud
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(in_max_cluster_distance);  //
  ec.setMinClusterSize(_cluster_size_min);
  ec.setMaxClusterSize(_cluster_size_max);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_2d);
  ec.extract(cluster_indices);
  // use indices on 3d cloud

  /*pcl::ConditionalEuclideanClustering<pcl::PointXYZ> cec (true);
  cec.setInputCloud (in_cloud_ptr);
  cec.setConditionFunction (&independentDistance);
  cec.setMinClusterSize (cluster_size_min);
  cec.setMaxClusterSize (cluster_size_max);
  cec.setClusterTolerance (_distance*2.0f);
  cec.segment (cluster_indices);*/

  /////////////////////////////////
  //---	3. Color clustered points
  /////////////////////////////////
  unsigned int k = 0;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

  std::vector<ClusterPtr> clusters;
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);//coord + color
  // cluster
  for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    ClusterPtr cluster(new Cluster());
    cluster->SetCloud(in_cloud_ptr, it->indices, _velodyne_header, k, (int)_colors[k].val[0], (int)_colors[k].val[1],
                      (int)_colors[k].val[2], "", _pose_estimation);
    clusters.push_back(cluster);

    k++;
  }
  // std::cout << "Clusters: " << k << std::endl;
  return clusters;
}

void checkClusterMerge(size_t in_cluster_id, std::vector<ClusterPtr>& in_clusters,
                       std::vector<bool>& in_out_visited_clusters, std::vector<size_t>& out_merge_indices,
                       double in_merge_threshold)
{
  // std::cout << "checkClusterMerge" << std::endl;
  pcl::PointXYZ point_a = in_clusters[in_cluster_id]->GetCentroid();
  for (size_t i = 0; i < in_clusters.size(); i++)
  {
    if (i != in_cluster_id && !in_out_visited_clusters[i])
    {
      pcl::PointXYZ point_b = in_clusters[i]->GetCentroid();
      double distance = sqrt(pow(point_b.x - point_a.x, 2) + pow(point_b.y - point_a.y, 2));
      if (distance <= in_merge_threshold)
      {
        in_out_visited_clusters[i] = true;
        out_merge_indices.push_back(i);
        // std::cout << "Merging " << in_cluster_id << " with " << i << " dist:" << distance << std::endl;
        checkClusterMerge(i, in_clusters, in_out_visited_clusters, out_merge_indices, in_merge_threshold);
      }
    }
  }
}

void mergeClusters(const std::vector<ClusterPtr>& in_clusters, std::vector<ClusterPtr>& out_clusters,
                   std::vector<size_t> in_merge_indices, const size_t& current_index,
                   std::vector<bool>& in_out_merged_clusters)
{
  // std::cout << "mergeClusters:" << in_merge_indices.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZRGB> sum_cloud;
  pcl::PointCloud<pcl::PointXYZ> mono_cloud;
  ClusterPtr merged_cluster(new Cluster());
  for (size_t i = 0; i < in_merge_indices.size(); i++)
  {
    sum_cloud += *(in_clusters[in_merge_indices[i]]->GetCloud());
    in_out_merged_clusters[in_merge_indices[i]] = true;
  }
  std::vector<int> indices(sum_cloud.points.size(), 0);
  for (size_t i = 0; i < sum_cloud.points.size(); i++)
  {
    indices[i] = i;
  }

  if (sum_cloud.points.size() > 0)
  {
    pcl::copyPointCloud(sum_cloud, mono_cloud);
    // std::cout << "mergedClusters " << sum_cloud.points.size() << " mono:" << mono_cloud.points.size() << std::endl;
    // cluster->SetCloud(in_cloud_ptr, it->indices, _velodyne_header, k, (int)_colors[k].val[0], (int)_colors[k].val[1],
    // (int)_colors[k].val[2], "", _pose_estimation);
    merged_cluster->SetCloud(mono_cloud.makeShared(), indices, _velodyne_header, current_index,
                             (int)_colors[current_index].val[0], (int)_colors[current_index].val[1],
                             (int)_colors[current_index].val[2], "", _pose_estimation);
    out_clusters.push_back(merged_cluster);
  }
}

void checkAllForMerge(std::vector<ClusterPtr>& in_clusters, std::vector<ClusterPtr>& out_clusters,
                      float in_merge_threshold)
{
  // std::cout << "checkAllForMerge" << std::endl;
  std::vector<bool> visited_clusters(in_clusters.size(), false);
  std::vector<bool> merged_clusters(in_clusters.size(), false);
  size_t current_index = 0;
  for (size_t i = 0; i < in_clusters.size(); i++)
  {
    if (!visited_clusters[i])
    {
      visited_clusters[i] = true;
      std::vector<size_t> merge_indices;
      checkClusterMerge(i, in_clusters, visited_clusters, merge_indices, in_merge_threshold);
      mergeClusters(in_clusters, out_clusters, merge_indices, current_index++, merged_clusters);
    }
  }
  for (size_t i = 0; i < in_clusters.size(); i++)
  {
    // check for clusters not merged, add them to the output
    if (!merged_clusters[i])
    {
      out_clusters.push_back(in_clusters[i]);
    }
  }

  // ClusterPtr cluster(new Cluster());
}

void segmentByDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                       jsk_recognition_msgs::BoundingBoxArray& in_out_boundingbox_array,
                       autoware_msgs::centroids& in_out_centroids, autoware_msgs::CloudClusterArray& in_out_clusters,
                       jsk_recognition_msgs::PolygonArray& in_out_polygon_array,
                       jsk_rviz_plugins::PictogramArray& in_out_pictogram_array)
{
  // cluster the pointcloud according to the distance of the points using different thresholds (not only one for the
  // entire pc)
  // in this way, the points farther in the pc will also be clustered

  // 0 => 0-15m d=0.5
  // 1 => 15-30 d=1
  // 2 => 30-45 d=1.6
  // 3 => 45-60 d=2.1
  // 4 => >60   d=2.6

  std::vector<ClusterPtr> all_clusters;

  if (!_use_multiple_thres)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
    {
      pcl::PointXYZ current_point;
      current_point.x = in_cloud_ptr->points[i].x;
      current_point.y = in_cloud_ptr->points[i].y;
      current_point.z = in_cloud_ptr->points[i].z;

      cloud_ptr->points.push_back(current_point);
    }
#ifdef GPU_CLUSTERING
    if (_use_gpu)
    {
      all_clusters = clusterAndColorGpu(cloud_ptr, out_cloud_ptr, in_out_boundingbox_array, in_out_centroids,
                                        _clustering_distance);
    }
    else
    {
      all_clusters =
          clusterAndColor(cloud_ptr, out_cloud_ptr, in_out_boundingbox_array, in_out_centroids, _clustering_distance);
    }
#else
    std::vector<ClusterPtr> all_clusters =
        clusterAndColor(cloud_ptr, out_cloud_ptr, in_out_boundingbox_array, in_out_centroids, _clustering_distance);
#endif
  }
  else
  {
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_segments_array(5);
    for (unsigned int i = 0; i < cloud_segments_array.size(); i++)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      cloud_segments_array[i] = tmp_cloud;
    }

    for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
    {
      pcl::PointXYZ current_point;
      current_point.x = in_cloud_ptr->points[i].x;
      current_point.y = in_cloud_ptr->points[i].y;
      current_point.z = in_cloud_ptr->points[i].z;

      float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

      if (origin_distance < _clustering_ranges[0])
      {
        cloud_segments_array[0]->points.push_back(current_point);
      }
      else if (origin_distance < _clustering_ranges[1])
      {
        cloud_segments_array[1]->points.push_back(current_point);
      }
      else if (origin_distance < _clustering_ranges[2])
      {
        cloud_segments_array[2]->points.push_back(current_point);
      }
      else if (origin_distance < _clustering_ranges[3])
      {
        cloud_segments_array[3]->points.push_back(current_point);
      }
      else
      {
        cloud_segments_array[4]->points.push_back(current_point);
      }
    }

    for (unsigned int i = 0; i < cloud_segments_array.size(); i++)
    {
#ifdef GPU_CLUSTERING
      std::vector<ClusterPtr> local_clusters;
      if (_use_gpu)
      {
        local_clusters = clusterAndColorGpu(cloud_segments_array[i], out_cloud_ptr, in_out_boundingbox_array,
                                            in_out_centroids, _clustering_distances[i]);
      }
      else
      {
        local_clusters = clusterAndColor(cloud_segments_array[i], out_cloud_ptr, in_out_boundingbox_array,
                                         in_out_centroids, _clustering_distances[i]);
      }
#else
      std::vector<ClusterPtr> local_clusters = clusterAndColor(
          cloud_segments_array[i], out_cloud_ptr, in_out_boundingbox_array, in_out_centroids, _clustering_distances[i]);
#endif
      all_clusters.insert(all_clusters.end(), local_clusters.begin(), local_clusters.end());
    }
  }

  // Clusters can be merged or checked in here
  //....
  // check for mergable clusters
  std::vector<ClusterPtr> mid_clusters;
  std::vector<ClusterPtr> final_clusters;

  if (all_clusters.size() > 0)
    checkAllForMerge(all_clusters, mid_clusters, _cluster_merge_threshold);
  else
    mid_clusters = all_clusters;

  if (mid_clusters.size() > 0)
    checkAllForMerge(mid_clusters, final_clusters, _cluster_merge_threshold);
  else
    final_clusters = mid_clusters;

  tf::StampedTransform vectormap_transform;
  if (_use_vector_map)
  {
    if (_wayarea_gridmap.exists(_gridmap_layer))
    {
      // check if centroids are inside the drivable area
      cv::Mat grid_image;
      grid_map::GridMapCvConverter::toImage<unsigned char, 1>(_wayarea_gridmap, _gridmap_layer, CV_8UC1,
                                                              _grid_min_value, _grid_max_value, grid_image);

#pragma omp for
      for (unsigned int i = 0; i < final_clusters.size(); i++)
      {
        pcl::PointXYZ pcl_centroid = final_clusters[i]->GetCentroid();

        geometry_msgs::Point original_centroid_point, final_centroid_point;
        original_centroid_point.x = pcl_centroid.x;
        original_centroid_point.y = pcl_centroid.y;
        original_centroid_point.z = pcl_centroid.z;

        if (_wayarea_gridmap.getFrameId() != _velodyne_header.frame_id)
        {
          tf::StampedTransform grid_sensor_tf = findTransform(_wayarea_gridmap.getFrameId(), _velodyne_header.frame_id);
          final_centroid_point = transformPoint(original_centroid_point, grid_sensor_tf);
        }
        else
        {
          final_centroid_point = original_centroid_point;
        }

        bool point_in_grid = checkPointInGrid(_wayarea_gridmap, grid_image, final_centroid_point);
        final_clusters[i]->SetValidity(point_in_grid);
      }
      // timer.stop();
      // std::cout << "vectormap filtering took " << timer.getTimeMilli() << " ms to check " << final_clusters.size() <<
      // std::endl;
    }
    else
    {
      ROS_INFO("%s layer not contained in the OccupancyGrid", _gridmap_layer.c_str());
    }
  }
  // Get final PointCloud to be published
  in_out_polygon_array.header = _velodyne_header;
  in_out_pictogram_array.header = _velodyne_header;
  for (unsigned int i = 0; i < final_clusters.size(); i++)
  {
    *out_cloud_ptr = *out_cloud_ptr + *(final_clusters[i]->GetCloud());

    jsk_recognition_msgs::BoundingBox bounding_box = final_clusters[i]->GetBoundingBox();
    geometry_msgs::PolygonStamped polygon = final_clusters[i]->GetPolygon();
    jsk_rviz_plugins::Pictogram pictogram_cluster;
    pictogram_cluster.header = _velodyne_header;

    // PICTO
    pictogram_cluster.mode = pictogram_cluster.STRING_MODE;
    pictogram_cluster.pose.position.x = final_clusters[i]->GetMaxPoint().x;
    pictogram_cluster.pose.position.y = final_clusters[i]->GetMaxPoint().y;
    pictogram_cluster.pose.position.z = final_clusters[i]->GetMaxPoint().z;
    tf::Quaternion quat(0.0, -0.7, 0.0, 0.7);
    tf::quaternionTFToMsg(quat, pictogram_cluster.pose.orientation);
    pictogram_cluster.size = 4;
    std_msgs::ColorRGBA color;
    color.a = 1;
    color.r = 1;
    color.g = 1;
    color.b = 1;
    pictogram_cluster.color = color;
    pictogram_cluster.character = std::to_string(i);
    // PICTO

    // pcl::PointXYZ min_point = final_clusters[i]->GetMinPoint();
    // pcl::PointXYZ max_point = final_clusters[i]->GetMaxPoint();
    pcl::PointXYZ center_point = final_clusters[i]->GetCentroid();
    geometry_msgs::Point centroid;
    centroid.x = center_point.x;
    centroid.y = center_point.y;
    centroid.z = center_point.z;
    bounding_box.header = _velodyne_header;
    polygon.header = _velodyne_header;

    if (final_clusters[i]->IsValid()
        //&& bounding_box.dimensions.x >0 && bounding_box.dimensions.y >0 && bounding_box.dimensions.z > 0
        //&&	bounding_box.dimensions.x < _max_boundingbox_side && bounding_box.dimensions.y < _max_boundingbox_side
        )
    {
      in_out_boundingbox_array.boxes.push_back(bounding_box);
      in_out_centroids.points.push_back(centroid);
      _visualization_marker.points.push_back(centroid);

      in_out_polygon_array.polygons.push_back(polygon);
      in_out_pictogram_array.pictograms.push_back(pictogram_cluster);

      autoware_msgs::CloudCluster cloud_cluster;
      final_clusters[i]->ToRosMessage(_velodyne_header, cloud_cluster);
      in_out_clusters.clusters.push_back(cloud_cluster);
    }
  }

  for (size_t i = 0; i < in_out_polygon_array.polygons.size(); i++)
  {
    in_out_polygon_array.labels.push_back(i);
  }
}

void removeFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr out_nofloor_cloud_ptr,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr out_onlyfloor_cloud_ptr, float in_max_height = 0.2,
                 float in_floor_max_angle = 0.1)
{
  /*pcl::PointIndicesPtr ground (new pcl::PointIndices);
  // Create the filtering object
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInputCloud (in_cloud_ptr);
  pmf.setMaxWindowSize (20);
  pmf.setSlope (1.0f);
  pmf.setInitialDistance (0.5f);
  pmf.setMaxDistance (3.0f);
  pmf.extract (ground->indices);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (in_cloud_ptr);
  extract.setIndices (ground);
  extract.setNegative(true);//true removes the indices, false leaves only the indices
  extract.filter(*out_nofloor_cloud_ptr);

  //EXTRACT THE FLOOR FROM THE CLOUD
  extract.setNegative(false);//true removes the indices, false leaves only the indices
  extract.filter(*out_onlyfloor_cloud_ptr);*/

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setAxis(Eigen::Vector3f(0, 0, 1));
  seg.setEpsAngle(in_floor_max_angle);

  seg.setDistanceThreshold(in_max_height);  // floor distance
  seg.setOptimizeCoefficients(true);
  seg.setInputCloud(in_cloud_ptr);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0)
  {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  // REMOVE THE FLOOR FROM THE CLOUD
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(in_cloud_ptr);
  extract.setIndices(inliers);
  extract.setNegative(true);  // true removes the indices, false leaves only the indices
  extract.filter(*out_nofloor_cloud_ptr);

  // EXTRACT THE FLOOR FROM THE CLOUD
  extract.setNegative(false);  // true removes the indices, false leaves only the indices
  extract.filter(*out_onlyfloor_cloud_ptr);
}

void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_leaf_size = 0.2)
{
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(in_cloud_ptr);
  sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
  sor.filter(*out_cloud_ptr);
}

void clipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
               pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, float in_min_height = -1.3, float in_max_height = 0.5)
{
  out_cloud_ptr->points.clear();
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    if (in_cloud_ptr->points[i].z >= in_min_height && in_cloud_ptr->points[i].z <= in_max_height)
    {
      out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
    }
  }
}

void differenceNormalsSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr)
{
  float small_scale = 0.5;
  float large_scale = 2.0;
  float angle_threshold = 0.5;
  pcl::search::Search<pcl::PointXYZ>::Ptr tree;
  if (in_cloud_ptr->isOrganized())
  {
    tree.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
  }
  else
  {
    tree.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
  }

  // Set the input pointcloud for the search tree
  tree->setInputCloud(in_cloud_ptr);

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> normal_estimation;
  // pcl::gpu::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimation;
  normal_estimation.setInputCloud(in_cloud_ptr);
  normal_estimation.setSearchMethod(tree);

  normal_estimation.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                                 std::numeric_limits<float>::max());

  pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<pcl::PointNormal>);

  normal_estimation.setRadiusSearch(small_scale);
  normal_estimation.compute(*normals_small_scale);

  normal_estimation.setRadiusSearch(large_scale);
  normal_estimation.compute(*normals_large_scale);

  pcl::PointCloud<pcl::PointNormal>::Ptr diffnormals_cloud(new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*in_cloud_ptr, *diffnormals_cloud);

  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> diffnormals_estimator;
  diffnormals_estimator.setInputCloud(in_cloud_ptr);
  diffnormals_estimator.setNormalScaleLarge(normals_large_scale);
  diffnormals_estimator.setNormalScaleSmall(normals_small_scale);

  diffnormals_estimator.initCompute();

  diffnormals_estimator.computeFeature(*diffnormals_cloud);

  pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond(new pcl::ConditionOr<pcl::PointNormal>());
  range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
      new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GT, angle_threshold)));
  // Build the filter
  pcl::ConditionalRemoval<pcl::PointNormal> cond_removal;
  cond_removal.setCondition(range_cond);
  cond_removal.setInputCloud(diffnormals_cloud);

  pcl::PointCloud<pcl::PointNormal>::Ptr diffnormals_cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);

  // Apply filter
  cond_removal.filter(*diffnormals_cloud_filtered);

  pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*diffnormals_cloud, *out_cloud_ptr);
}

void removePointsUpTo(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, const double in_distance)
{
  out_cloud_ptr->points.clear();
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++)
  {
    float origin_distance = sqrt(pow(in_cloud_ptr->points[i].x, 2) + pow(in_cloud_ptr->points[i].y, 2));
    if (origin_distance > in_distance)
    {
      out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
    }
  }
}

void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
{
  //_start = std::chrono::system_clock::now();

  if (!_using_sensor_cloud)
  {
    _using_sensor_cloud = true;

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removed_points_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlanes_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr nofloor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr onlyfloor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr diffnormals_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    autoware_msgs::centroids centroids;
    autoware_msgs::CloudClusterArray cloud_clusters;
    jsk_recognition_msgs::BoundingBoxArray boundingbox_array;
    jsk_recognition_msgs::PolygonArray polygon_array;
    jsk_rviz_plugins::PictogramArray pictograms_array;

    pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);

    _velodyne_header = in_sensor_cloud->header;

    if (_remove_points_upto > 0.0)
    {
      removePointsUpTo(current_sensor_cloud_ptr, removed_points_cloud_ptr, _remove_points_upto);
    }
    else
      removed_points_cloud_ptr = current_sensor_cloud_ptr;

    if (_downsample_cloud)
      downsampleCloud(removed_points_cloud_ptr, downsampled_cloud_ptr, _leaf_size);
    else
      downsampled_cloud_ptr = removed_points_cloud_ptr;

    clipCloud(downsampled_cloud_ptr, clipped_cloud_ptr, _clip_min_height, _clip_max_height);

    if (_keep_lanes)
      keepLanePoints(clipped_cloud_ptr, inlanes_cloud_ptr, _keep_lane_left_distance, _keep_lane_right_distance);
    else
      inlanes_cloud_ptr = clipped_cloud_ptr;

    if (_remove_ground)
    {
      removeFloor(inlanes_cloud_ptr, nofloor_cloud_ptr, onlyfloor_cloud_ptr);
      publishCloud(&_pub_ground_cloud, onlyfloor_cloud_ptr);
    }
    else
      nofloor_cloud_ptr = inlanes_cloud_ptr;

    publishCloud(&_pub_points_lanes_cloud, nofloor_cloud_ptr);

    if (_use_diffnormals)
      differenceNormalsSegmentation(nofloor_cloud_ptr, diffnormals_cloud_ptr);
    else
      diffnormals_cloud_ptr = nofloor_cloud_ptr;

    segmentByDistance(diffnormals_cloud_ptr, colored_clustered_cloud_ptr, boundingbox_array, centroids, cloud_clusters,
                      polygon_array, pictograms_array);

    publishColorCloud(&_pub_cluster_cloud, colored_clustered_cloud_ptr);

    // Publish BB
    boundingbox_array.header = _velodyne_header;

    _pub_jsk_hulls.publish(polygon_array);          // publish convex hulls
    _pub_text_pictogram.publish(pictograms_array);  // publish_ids

    publishBoundingBoxArray(&_pub_jsk_boundingboxes, boundingbox_array, _output_frame, _velodyne_header);
    centroids.header = _velodyne_header;

    publishCentroids(&_centroid_pub, centroids, _output_frame, _velodyne_header);

    _marker_pub.publish(_visualization_marker);
    _visualization_marker.points.clear();  // transform? is it used?
    cloud_clusters.header = _velodyne_header;

    publishCloudClusters(&_pub_clusters_message, cloud_clusters, _output_frame, _velodyne_header);

    _using_sensor_cloud = false;
  }
  //_end = std::chrono::system_clock::now();
  // double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(_end-_start).count();
  // ROS_INFO("Euclidean Clustering : %f", elapsed);
}

/*
void vectormap_callback(const visualization_msgs::MarkerArray::Ptr in_vectormap_markers)
{
  float min_x=std::numeric_limits<float>::max();float max_x=-std::numeric_limits<float>::max();
  float min_y=std::numeric_limits<float>::max();float max_y=-std::numeric_limits<float>::max();
  pcl::PointXYZ min_point;
  pcl::PointXYZ max_point;
  std::vector<geometry_msgs::Point> vectormap_points;
  std::string marker_frame;
  double map_scale = -10.0;
  for(auto i=in_vectormap_markers->markers.begin(); i!= in_vectormap_markers->markers.end(); i++)
  {
    visualization_msgs::Marker current_marker = *i;
    marker_frame = current_marker.header.frame_id;
    if (current_marker.ns == "road_edge")
    {
      for (unsigned int j=0; j< current_marker.points.size(); j++)
      {
        geometry_msgs::Point p = current_marker.points[j];
        p.x*=map_scale;
        p.y*=map_scale;
        if(p.x<min_x)	min_x = p.x;
        if(p.y<min_y)	min_y = p.y;
        if(p.x>max_x)	max_x = p.x;
        if(p.y>max_y)	max_y = p.y;
        vectormap_points.push_back(p);
      }
    }
  }
  min_point.x = min_x;	min_point.y = min_y;
  max_point.x = max_x;	max_point.y = max_y;

  min_point.x*=-1.0;
  min_point.y*=-1.0;
  //translate the points to the minimum point
  for (auto i=vectormap_points.begin(); i!=vectormap_points.end(); i++)
  {
    (*i).x+=min_point.x;
    (*i).y+=min_point.y;
  }
  max_point.x+=min_point.x;
  max_point.y+=min_point.y;
  //get world tf
  std::string error_transform_msg;
  tf::Vector3 map_origin_point;
  if(_transform_listener->waitForTransform("/map", marker_frame, ros::Time(0), ros::Duration(5), ros::Duration(0.1),
&error_transform_msg))
  {
    _transform_listener->lookupTransform("/map", marker_frame, ros::Time(0), *_transform);
    map_origin_point = _transform->getOrigin();
    map_origin_point.setX( map_origin_point.x() - min_point.x);
    map_origin_point.setY( map_origin_point.y() - min_point.y);
  }
  else
  {
    ROS_INFO("Euclidean Cluster (vectormap_callback): %s", error_transform_msg.c_str());
  }

  cv::Mat map_image = cv::Mat::zeros(max_point.y, max_point.x, CV_8UC3);

  std::cout << "W,H:" << max_point << std::endl;

  cv::Point image_start_point (vectormap_points[0].x, vectormap_points[0].y);
  cv::Point prev_point = image_start_point;
  for (auto i=vectormap_points.begin(); i!=vectormap_points.end(); i++)
  {
    cv::line(map_image, prev_point, cv::Point((int)(i->x), (int)(i->y)), cv::Scalar::all(255));

    prev_point.x = (int)(i->x);
    prev_point.y = (int)(i->y);
  }
  cv::circle(map_image, image_start_point, 3, cv::Scalar(255,0,0));
  cv::imshow("vectormap", map_image);
  cv::waitKey(0);
}*/

void wayarea_gridmap_callback(const grid_map_msgs::GridMap& message)
{
  grid_map::GridMapRosConverter::fromMessage(message, _wayarea_gridmap);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "euclidean_cluster");

  ros::NodeHandle h;
  ros::NodeHandle private_nh("~");

  tf::StampedTransform transform;
  tf::TransformListener listener;
  tf::TransformListener vectormap_tf_listener;

  _vectormap_transform_listener = &vectormap_tf_listener;
  _transform = &transform;
  _transform_listener = &listener;

#if (CV_MAJOR_VERSION == 3)
  generateColors(_colors, 255);
#else
  cv::generateColors(_colors, 255);
#endif

  _pub_cluster_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_cluster", 1);
  _pub_ground_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_ground", 1);
  _centroid_pub = h.advertise<autoware_msgs::centroids>("/cluster_centroids", 1);
  _marker_pub = h.advertise<visualization_msgs::Marker>("centroid_marker", 1);

  _pub_points_lanes_cloud = h.advertise<sensor_msgs::PointCloud2>("/points_lanes", 1);
  _pub_jsk_boundingboxes = h.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bounding_boxes", 1);
  _pub_jsk_hulls = h.advertise<jsk_recognition_msgs::PolygonArray>("/cluster_hulls", 1);
  _pub_clusters_message = h.advertise<autoware_msgs::CloudClusterArray>("/cloud_clusters", 1);
  _pub_detected_objects = h.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_objects", 1);
  _pub_text_pictogram = h.advertise<jsk_rviz_plugins::PictogramArray>("cluster_ids", 10);
  ROS_INFO("output pictograms topic: %s", "cluster_id");

  _pub_grid_map = h.advertise<grid_map_msgs::GridMap>("grid_map_wayarea", 1, true);

  std::string points_topic, gridmap_topic;

  _using_sensor_cloud = false;

  if (private_nh.getParam("points_node", points_topic))
  {
    ROS_INFO("euclidean_cluster > Setting points node to %s", points_topic.c_str());
  }
  else
  {
    ROS_INFO("euclidean_cluster > No points node received, defaulting to points_raw, you can use "
             "_points_node:=YOUR_TOPIC");
    points_topic = "/points_raw";
  }

  _use_diffnormals = false;
  if (private_nh.getParam("use_diffnormals", _use_diffnormals))
  {
    if (_use_diffnormals)
      ROS_INFO("Euclidean Clustering: Applying difference of normals on clustering pipeline");
    else
      ROS_INFO("Euclidean Clustering: Difference of Normals will not be used.");
  }

  /* Initialize tuning parameter */
  private_nh.param("downsample_cloud", _downsample_cloud, false);
  ROS_INFO("downsample_cloud: %d", _downsample_cloud);
  private_nh.param("remove_ground", _remove_ground, true);
  ROS_INFO("remove_ground: %d", _remove_ground);
  private_nh.param("leaf_size", _leaf_size, 0.1);
  ROS_INFO("leaf_size: %f", _leaf_size);
  private_nh.param("cluster_size_min", _cluster_size_min, 20);
  ROS_INFO("cluster_size_min %d", _cluster_size_min);
  private_nh.param("cluster_size_max", _cluster_size_max, 100000);
  ROS_INFO("cluster_size_max: %d", _cluster_size_max);
  private_nh.param("pose_estimation", _pose_estimation, false);
  ROS_INFO("pose_estimation: %d", _pose_estimation);
  private_nh.param("clip_min_height", _clip_min_height, -1.3);
  ROS_INFO("clip_min_height: %f", _clip_min_height);
  private_nh.param("clip_max_height", _clip_max_height, 0.5);
  ROS_INFO("clip_max_height: %f", _clip_max_height);
  private_nh.param("keep_lanes", _keep_lanes, false);
  ROS_INFO("keep_lanes: %d", _keep_lanes);
  private_nh.param("keep_lane_left_distance", _keep_lane_left_distance, 5.0);
  ROS_INFO("keep_lane_left_distance: %f", _keep_lane_left_distance);
  private_nh.param("keep_lane_right_distance", _keep_lane_right_distance, 5.0);
  ROS_INFO("keep_lane_right_distance: %f", _keep_lane_right_distance);
  private_nh.param("max_boundingbox_side", _max_boundingbox_side, 10.0);
  ROS_INFO("max_boundingbox_side: %f", _max_boundingbox_side);
  private_nh.param("cluster_merge_threshold", _cluster_merge_threshold, 1.5);
  ROS_INFO("cluster_merge_threshold: %f", _cluster_merge_threshold);
  private_nh.param<std::string>("output_frame", _output_frame, "velodyne");
  ROS_INFO("output_frame: %s", _output_frame.c_str());

  private_nh.param("use_vector_map", _use_vector_map, false);
  ROS_INFO("use_vector_map: %d", _use_vector_map);
  private_nh.param<std::string>("vectormap_frame", _vectormap_frame, "map");
  ROS_INFO("vectormap_frame: %s", _vectormap_frame.c_str());

  private_nh.param("remove_points_upto", _remove_points_upto, 0.0);
  ROS_INFO("remove_points_upto: %f", _remove_points_upto);

  private_nh.param("clustering_distance", _clustering_distance, 0.75);
  ROS_INFO("clustering_distance: %f", _clustering_distance);

  private_nh.param("use_gpu", _use_gpu, false);
  ROS_INFO("use_gpu: %d", _use_gpu);

  private_nh.param("use_multiple_thres", _use_multiple_thres, false);
  ROS_INFO("use_multiple_thres: %d", _use_multiple_thres);

  std::string str_distances;
  std::string str_ranges;
  private_nh.param("clustering_distances", str_distances, std::string("[0.5,1.1,1.6,2.1,2.6]"));
  ROS_INFO("clustering_distances: %s", str_distances.c_str());
  private_nh.param("clustering_ranges", str_ranges, std::string("[15,30,45,60]"));
  ROS_INFO("clustering_ranges: %s", str_ranges.c_str());

  YAML::Node distances = YAML::Load(str_distances);
  YAML::Node ranges = YAML::Load(str_ranges);
  size_t distances_size = distances.size();
  size_t ranges_size = ranges.size();
  if (distances_size == 0 || ranges_size == 0)
  {
    ROS_ERROR("Invalid size of clustering_ranges or/and clustering_distance. \
    The size of clustering distance and clustering_ranges shoule not be 0");
    ros::shutdown();
  }
  if ((distances_size - ranges_size) != 1)
  {
    ROS_ERROR("Invalid size of clustering_ranges or/and clustering_distance. \
    Expecting that (distances_size - ranges_size) == 1 ");
    ros::shutdown();
  }
  for (size_t i_distance = 0; i_distance < distances_size; i_distance++)
  {
    _clustering_distances.push_back(distances[i_distance].as<double>());
  }
  for (size_t i_range = 0; i_range < ranges_size; i_range++)
  {
    _clustering_ranges.push_back(ranges[i_range].as<double>());
  }

  _velodyne_transform_available = false;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = h.subscribe(points_topic, 1, velodyne_callback);

  private_nh.param<std::string>("wayarea_gridmap_topic", gridmap_topic, "grid_map_wayarea");
  ROS_INFO("wayarea_gridmap_topic: %s", gridmap_topic.c_str());
  private_nh.param<std::string>("wayarea_gridmap_layer", _gridmap_layer, "wayarea");
  ROS_INFO("wayarea_gridmap_layer: %s", _gridmap_layer.c_str());
  private_nh.param<int>("wayarea_no_road_value", _gridmap_no_road_value, _grid_max_value);
  ROS_INFO("wayarea_no_road_value: %ds", _gridmap_no_road_value);
  ros::Subscriber wayarea_sub = h.subscribe(gridmap_topic, 1, wayarea_gridmap_callback);

  _visualization_marker.header.frame_id = "velodyne";
  _visualization_marker.header.stamp = ros::Time();
  _visualization_marker.ns = "my_namespace";
  _visualization_marker.id = 0;
  _visualization_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  _visualization_marker.action = visualization_msgs::Marker::ADD;
  _visualization_marker.scale.x = 1.0;
  _visualization_marker.scale.y = 1.0;
  _visualization_marker.scale.z = 1.0;
  _visualization_marker.color.a = 1.0;
  _visualization_marker.color.r = 0.0;
  _visualization_marker.color.g = 0.0;
  _visualization_marker.color.b = 1.0;
  // marker.lifetime = ros::Duration(0.1);
  _visualization_marker.frame_locked = true;

  // Spin
  ros::spin();
}
