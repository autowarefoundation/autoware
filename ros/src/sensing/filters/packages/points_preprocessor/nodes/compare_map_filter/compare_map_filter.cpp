/*
 *  Copyright (c) 2018, TierIV, Inc
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <autoware_config_msgs/ConfigCompareMapFilter.h>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>

class CompareMapFilter
{
public:
  CompareMapFilter();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber config_sub_;
  ros::Subscriber sensor_points_sub_;
  ros::Subscriber map_sub_;
  ros::Publisher match_points_pub_;
  ros::Publisher unmatch_points_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  pcl::KdTreeFLANN<pcl::PointXYZI> tree_;

  double distance_threshold_;
  double min_clipping_height_;
  double max_clipping_height_;

  std::string map_frame_;

  void configCallback(const autoware_config_msgs::ConfigCompareMapFilter::ConstPtr& config_msg_ptr);
  void pointsMapCallback(const sensor_msgs::PointCloud2::ConstPtr& map_cloud_msg_ptr);
  void sensorPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& sensorTF_cloud_msg_ptr);
  void searchMatchingCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr match_cloud_ptr,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr unmatch_cloud_ptr);
  void transformAsMatrix(const geometry_msgs::TransformStamped &tf,
                        Eigen::Matrix4f &out_mat);
  void transformXYZICloud(
      const pcl::PointCloud<pcl::PointXYZI> &in_cloud,
      pcl::PointCloud<pcl::PointXYZI> &out_cloud,
      const geometry_msgs::TransformStamped &in_tf_stamped_transform);
};

CompareMapFilter::CompareMapFilter()
  : nh_()
  , nh_private_("~")
  , tf_listener_(tf_buffer_)
  , distance_threshold_(0.2)
  , min_clipping_height_(-2.0)
  , max_clipping_height_(0.5)
  , map_frame_("map")
{
  nh_private_.param("distance_threshold", distance_threshold_, distance_threshold_);
  nh_private_.param("min_clipping_height", min_clipping_height_, min_clipping_height_);
  nh_private_.param("max_clipping_height", max_clipping_height_, max_clipping_height_);

  config_sub_ = nh_.subscribe("/config/compare_map_filter", 10, &CompareMapFilter::configCallback, this);
  sensor_points_sub_ = nh_.subscribe("/points_raw", 1, &CompareMapFilter::sensorPointsCallback, this);
  map_sub_ = nh_.subscribe("/points_map", 10, &CompareMapFilter::pointsMapCallback, this);
  match_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_ground", 10);
  unmatch_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/points_no_ground", 10);
}

void CompareMapFilter::transformAsMatrix(const geometry_msgs::TransformStamped &tf,
                        Eigen::Matrix4f &out_mat) {
  double mv[12];
  tf2::Stamped<tf2::Transform> bt;
  tf2::fromMsg(tf, bt);
  bt.getBasis().getOpenGLSubMatrix(mv);
  tf2::Vector3 origin = bt.getOrigin();
  out_mat(0, 0) = mv[0];
  out_mat(0, 1) = mv[4];
  out_mat(0, 2) = mv[8];
  out_mat(1, 0) = mv[1];
  out_mat(1, 1) = mv[5];
  out_mat(1, 2) = mv[9];
  out_mat(2, 0) = mv[2];
  out_mat(2, 1) = mv[6];
  out_mat(2, 2) = mv[10];

  out_mat(3, 0) = out_mat(3, 1) = out_mat(3, 2) = 0;
  out_mat(3, 3) = 1;
  out_mat(0, 3) = origin.x();
  out_mat(1, 3) = origin.y();
  out_mat(2, 3) = origin.z();
}

  void CompareMapFilter::transformXYZICloud(
      const pcl::PointCloud<pcl::PointXYZI> &in_cloud,
      pcl::PointCloud<pcl::PointXYZI> &out_cloud,
      const geometry_msgs::TransformStamped &in_tf_stamped_transform) {
    Eigen::Matrix4f transform;
    transformAsMatrix(in_tf_stamped_transform, transform);

    if (&in_cloud != &out_cloud) {
      out_cloud.header.frame_id = in_tf_stamped_transform.header.frame_id;
      out_cloud.header.stamp = in_cloud.header.stamp;
      out_cloud.is_dense = in_cloud.is_dense;
      out_cloud.width = in_cloud.width;
      out_cloud.height = in_cloud.height;
      out_cloud.points.reserve(out_cloud.points.size());
      out_cloud.points.assign(in_cloud.points.begin(), in_cloud.points.end());
      out_cloud.sensor_orientation_ = in_cloud.sensor_orientation_;
      out_cloud.sensor_origin_ = in_cloud.sensor_origin_;
    }
    if (in_cloud.is_dense) {
      for (size_t i = 0; i < out_cloud.points.size(); ++i) {
        // out_cloud.points[i].getVector3fMap () = transform *
        // in_cloud.points[i].getVector3fMap ();
        Eigen::Matrix<float, 3, 1> pt(in_cloud[i].x, in_cloud[i].y,
                                      in_cloud[i].z);
        out_cloud[i].x = static_cast<float>(transform(0, 0) * pt.coeffRef(0) +
                                            transform(0, 1) * pt.coeffRef(1) +
                                            transform(0, 2) * pt.coeffRef(2) +
                                            transform(0, 3));
        out_cloud[i].y = static_cast<float>(transform(1, 0) * pt.coeffRef(0) +
                                            transform(1, 1) * pt.coeffRef(1) +
                                            transform(1, 2) * pt.coeffRef(2) +
                                            transform(1, 3));
        out_cloud[i].z = static_cast<float>(transform(2, 0) * pt.coeffRef(0) +
                                            transform(2, 1) * pt.coeffRef(1) +
                                            transform(2, 2) * pt.coeffRef(2) +
                                            transform(2, 3));
      }
    } else {
      // Dataset might contain NaNs and Infs, so check for them first,
      for (size_t i = 0; i < out_cloud.points.size(); ++i) {
        if (!pcl_isfinite(in_cloud.points[i].x) ||
            !pcl_isfinite(in_cloud.points[i].y) ||
            !pcl_isfinite(in_cloud.points[i].z)) {
          continue;
        }
        // out_cloud.points[i].getVector3fMap () = transform *
        // in_cloud.points[i].getVector3fMap ();
        Eigen::Matrix<float, 3, 1> pt(in_cloud[i].x, in_cloud[i].y,
                                      in_cloud[i].z);
        out_cloud[i].x = static_cast<float>(transform(0, 0) * pt.coeffRef(0) +
                                            transform(0, 1) * pt.coeffRef(1) +
                                            transform(0, 2) * pt.coeffRef(2) +
                                            transform(0, 3));
        out_cloud[i].y = static_cast<float>(transform(1, 0) * pt.coeffRef(0) +
                                            transform(1, 1) * pt.coeffRef(1) +
                                            transform(1, 2) * pt.coeffRef(2) +
                                            transform(1, 3));
        out_cloud[i].z = static_cast<float>(transform(2, 0) * pt.coeffRef(0) +
                                            transform(2, 1) * pt.coeffRef(1) +
                                            transform(2, 2) * pt.coeffRef(2) +
                                            transform(2, 3));
      }
    }
  }

void CompareMapFilter::configCallback(const autoware_config_msgs::ConfigCompareMapFilter::ConstPtr& config_msg_ptr)
{
  distance_threshold_ = config_msg_ptr->distance_threshold;
  min_clipping_height_ = config_msg_ptr->min_clipping_height;
  max_clipping_height_ = config_msg_ptr->max_clipping_height;
}

void CompareMapFilter::pointsMapCallback(const sensor_msgs::PointCloud2::ConstPtr& map_cloud_msg_ptr)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*map_cloud_msg_ptr, *map_cloud_ptr);
  tree_.setInputCloud(map_cloud_ptr);
  ROS_INFO_STREAM("map recieved");
  map_frame_ = map_cloud_msg_ptr->header.frame_id;
}

void CompareMapFilter::sensorPointsCallback(const sensor_msgs::PointCloud2::ConstPtr& sensorTF_cloud_msg_ptr)
{
  const ros::Time sensor_time = sensorTF_cloud_msg_ptr->header.stamp;
  const std::string sensor_frame = sensorTF_cloud_msg_ptr->header.frame_id;

  pcl::PointCloud<pcl::PointXYZI>::Ptr sensorTF_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*sensorTF_cloud_msg_ptr, *sensorTF_cloud_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr sensorTF_clipping_height_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  sensorTF_clipping_height_cloud_ptr->header = sensorTF_cloud_ptr->header;
  for (size_t i = 0; i < sensorTF_cloud_ptr->points.size(); ++i)
  {
    if (sensorTF_cloud_ptr->points[i].z > min_clipping_height_ &&
        sensorTF_cloud_ptr->points[i].z < max_clipping_height_)
    {
      sensorTF_clipping_height_cloud_ptr->points.push_back(sensorTF_cloud_ptr->points[i]);
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr mapTF_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_.lookupTransform(
        sensor_frame, map_frame_, sensor_time,
        ros::Duration(3.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  transformXYZICloud(*sensorTF_clipping_height_cloud_ptr, *mapTF_cloud_ptr, transform_stamped);
  ROS_WARN_STREAM(mapTF_cloud_ptr->header);
  pcl::PointCloud<pcl::PointXYZI>::Ptr mapTF_match_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr sensorTF_match_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr mapTF_unmatch_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr sensorTF_unmatch_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  searchMatchingCloud(mapTF_cloud_ptr, mapTF_match_cloud_ptr, mapTF_unmatch_cloud_ptr);
  sensor_msgs::PointCloud2 sensorTF_match_cloud_msg;
  try
  {
    transformXYZICloud(*mapTF_match_cloud_ptr, *sensorTF_match_cloud_ptr, transform_stamped);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("Transform error: %s", ex.what());
    return;
  }
  pcl::toROSMsg(*sensorTF_match_cloud_ptr, sensorTF_match_cloud_msg);
  match_points_pub_.publish(sensorTF_match_cloud_msg);
  sensor_msgs::PointCloud2 sensorTF_unmatch_cloud_msg;
  try
  {
    transformXYZICloud(*mapTF_unmatch_cloud_ptr, *sensorTF_unmatch_cloud_ptr, transform_stamped);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("Transform error: %s", ex.what());
    return;
  }
  sensor_msgs::PointCloud2 mapTF_unmatch_cloud_msg;
  pcl::toROSMsg(*sensorTF_unmatch_cloud_ptr, sensorTF_unmatch_cloud_msg);
  unmatch_points_pub_.publish(sensorTF_unmatch_cloud_msg);
}

void CompareMapFilter::searchMatchingCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr match_cloud_ptr,
                                           pcl::PointCloud<pcl::PointXYZI>::Ptr unmatch_cloud_ptr)
{
  match_cloud_ptr->points.clear();
  unmatch_cloud_ptr->points.clear();

  match_cloud_ptr->points.reserve(in_cloud_ptr->points.size());
  unmatch_cloud_ptr->points.reserve(in_cloud_ptr->points.size());

  std::vector<int> nn_indices(1);
  std::vector<float> nn_dists(1);
  for (size_t i = 0; i < in_cloud_ptr->points.size(); ++i)
  {
    tree_.nearestKSearch(in_cloud_ptr->points[i], 1, nn_indices, nn_dists);
    if (nn_dists[0] <= distance_threshold_)
    {
      match_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
    }
    else
    {
      unmatch_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "compare_map_filter");
  CompareMapFilter node;
  ros::spin();

  return 0;
}
