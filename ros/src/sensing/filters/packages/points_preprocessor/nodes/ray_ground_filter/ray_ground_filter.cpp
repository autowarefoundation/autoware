/*
 *  Copyright (c) 2017, Nagoya University
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
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 */
#include <iostream>
#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <velodyne_pointcloud/point_types.h>
#include "autoware_msgs/ConfigRayGroundFilter.h"

#include <opencv2/core/version.hpp>
#if (CV_MAJOR_VERSION == 3)
#include "gencolors.cpp"
#else
#include <opencv2/contrib/contrib.hpp>
#endif

#include "ray_ground_filter.h"

void RayGroundFilter::update_config_params(const autoware_msgs::ConfigRayGroundFilter::ConstPtr& param)
{
  sensor_height_          = param->sensor_height;
  general_max_slope_      = param->general_max_slope;
  local_max_slope_        = param->local_max_slope;
  radial_divider_angle_   = param->radial_divider_angle;
  concentric_divider_distance_ = param->concentric_divider_distance;
  min_height_threshold_   = param->min_height_threshold;
  clipping_height_        = param->clipping_height;
  min_point_distance_     = param->min_point_distance;
  reclass_distance_threshold_ = param->reclass_distance_threshold;
}

void RayGroundFilter::publish_cloud(const ros::Publisher& in_publisher,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
    const std_msgs::Header& in_header)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
  cloud_msg.header = in_header;
  in_publisher.publish(cloud_msg);
}

/*!
 *
 * @param[in] in_cloud Input Point Cloud to be organized in radial segments
 * @param[out] out_organized_points Custom Point Cloud filled with XYZRTZColor data
 * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each radial segment
 * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
 */
void RayGroundFilter::ConvertXYZIToRTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
    PointCloudXYZIRTColor& out_organized_points,
    std::vector<pcl::PointIndices>& out_radial_divided_indices,
    std::vector<PointCloudXYZIRTColor>& out_radial_ordered_clouds)
{
  out_organized_points.resize(in_cloud->points.size());
  out_radial_divided_indices.clear();
  out_radial_divided_indices.resize(radial_dividers_num_);
  out_radial_ordered_clouds.resize(radial_dividers_num_);

  for(size_t i=0; i< in_cloud->points.size(); i++)
  {
    PointXYZIRTColor new_point;
    auto radius         = (float) sqrt(
        in_cloud->points[i].x*in_cloud->points[i].x
        + in_cloud->points[i].y*in_cloud->points[i].y
        );
    auto theta          = (float) atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;
    if (theta < 0){ theta+=360; }

    auto radial_div     = (size_t) floor(theta/radial_divider_angle_);
    auto concentric_div = (size_t) floor(fabs(radius/concentric_divider_distance_));

    new_point.point    = in_cloud->points[i];
    new_point.radius   = radius;
    new_point.theta    = theta;
    new_point.radial_div = radial_div;
    new_point.concentric_div = concentric_div;
    new_point.red      = (size_t) colors_[new_point.radial_div % color_num_].val[0];
    new_point.green    = (size_t) colors_[new_point.radial_div % color_num_].val[1];
    new_point.blue     = (size_t) colors_[new_point.radial_div % color_num_].val[2];
    new_point.original_index = i;

    out_organized_points[i] = new_point;

    //radial divisions
    out_radial_divided_indices[radial_div].indices.push_back(i);

    out_radial_ordered_clouds[radial_div].push_back(new_point);

  }//end for

  //order radial points on each division
#pragma omp for
  for(size_t i=0; i< radial_dividers_num_; i++)
  {
    std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
        [](const PointXYZIRTColor& a, const PointXYZIRTColor& b){ return a.radius < b.radius; });
  }
}

/*!
 * Classifies Points in the PointCoud as Ground and Not Ground
 * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance from the origin
 * @param out_ground_indices Returns the indices of the points classified as ground in the original PointCloud
 * @param out_no_ground_indices Returns the indices of the points classified as not ground in the original PointCloud
 */
void RayGroundFilter::ClassifyPointCloud(std::vector<PointCloudXYZIRTColor>& in_radial_ordered_clouds,
    pcl::PointIndices& out_ground_indices,
    pcl::PointIndices& out_no_ground_indices)
{
  out_ground_indices.indices.clear();
  out_no_ground_indices.indices.clear();
#pragma omp for
  for (size_t i=0; i < in_radial_ordered_clouds.size(); i++)//sweep through each radial division
  {
    float prev_radius = 0.f;
    float prev_height = - sensor_height_;
    bool prev_ground = false;
    bool current_ground = false;
    for (size_t j=0; j < in_radial_ordered_clouds[i].size(); j++)//loop through each point in the radial div
    {
      float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
      float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance;
      float current_height = in_radial_ordered_clouds[i][j].point.z;
      float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius;

      //for points which are very close causing the height threshold to be tiny, set a minimum value
      if (points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_)
      { height_threshold = min_height_threshold_; }

      //check current point height against the LOCAL threshold (previous point)
      if (current_height <= (prev_height + height_threshold)
          && current_height >= (prev_height - height_threshold)
         )
      {
        //Check again using general geometry (radius from origin) if previous points wasn't ground
        if (!prev_ground)
        {
          if(current_height <= (-sensor_height_ + general_height_threshold)
              && current_height >= (-sensor_height_ - general_height_threshold))
          {
            current_ground = true;
          }
          else
          {current_ground = false;}
        }
        else
        {
          current_ground = true;
        }
      }
      else
      {
        //check if previous point is too far from previous one, if so classify again
        if (points_distance > reclass_distance_threshold_ &&
            (current_height <= (-sensor_height_ + height_threshold)
             && current_height >= (-sensor_height_ - height_threshold))
           )
        {
          current_ground = true;
        }
        else
        {current_ground = false;}
      }

      if (current_ground)
      {
        out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
        prev_ground=true;
      }
      else
      {
        out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
        prev_ground = false;
      }

      prev_radius = in_radial_ordered_clouds[i][j].radius;
      prev_height = in_radial_ordered_clouds[i][j].point.z;
    }
  }
}

/*!
 * Removes the points higher than a threshold
 * @param in_cloud_ptr PointCloud to perform Clipping
 * @param in_clip_height Maximum allowed height in the cloud
 * @param out_clipped_cloud_ptr Resultung PointCloud with the points removed
 */
void RayGroundFilter::ClipCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
    double in_clip_height,
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_clipped_cloud_ptr)
{
  pcl::ExtractIndices<pcl::PointXYZI> extractor;
  extractor.setInputCloud (in_cloud_ptr);
  pcl::PointIndices indices;

#pragma omp for
  for (size_t i=0; i< in_cloud_ptr->points.size(); i++)
  {
    if (in_cloud_ptr->points[i].z > in_clip_height)
    {
      indices.indices.push_back(i);
    }
  }
  extractor.setIndices(boost::make_shared<pcl::PointIndices>(indices));
  extractor.setNegative(true);//true removes the indices, false leaves only the indices
  extractor.filter(*out_clipped_cloud_ptr);
}

/*!
 * Returns the resulting complementary PointCloud, one with the points kept and the other removed as indicated
 * in the indices
 * @param in_cloud_ptr Input PointCloud to which the extraction will be performed
 * @param in_indices Indices of the points to be both removed and kept
 * @param out_only_indices_cloud_ptr Resulting PointCloud with the indices kept
 * @param out_removed_indices_cloud_ptr Resulting PointCloud with the indices removed
 */
void RayGroundFilter::ExtractPointsIndices(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
    const pcl::PointIndices& in_indices,
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_only_indices_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_removed_indices_cloud_ptr)
{
  pcl::ExtractIndices<pcl::PointXYZI> extract_ground;
  extract_ground.setInputCloud (in_cloud_ptr);
  extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(in_indices));

  extract_ground.setNegative(false);//true removes the indices, false leaves only the indices
  extract_ground.filter(*out_only_indices_cloud_ptr);

  extract_ground.setNegative(true);//true removes the indices, false leaves only the indices
  extract_ground.filter(*out_removed_indices_cloud_ptr);
}

/*!
 * Removes points up to a certain distance in the XY Plane
 * @param in_cloud_ptr Input PointCloud
 * @param in_min_distance Minimum valid distance, points closer than this will be removed.
 * @param out_filtered_cloud_ptr Resulting PointCloud with the invalid points removed.
 */
void RayGroundFilter::RemovePointsUpTo(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
    double in_min_distance,
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_filtered_cloud_ptr)
{
  pcl::ExtractIndices<pcl::PointXYZI> extractor;
  extractor.setInputCloud (in_cloud_ptr);
  pcl::PointIndices indices;

#pragma omp for
  for (size_t i=0; i< in_cloud_ptr->points.size(); i++)
  {
    if (sqrt(in_cloud_ptr->points[i].x*in_cloud_ptr->points[i].x +
          in_cloud_ptr->points[i].y*in_cloud_ptr->points[i].y)
        < in_min_distance)
    {
      indices.indices.push_back(i);
    }
  }
  extractor.setIndices(boost::make_shared<pcl::PointIndices>(indices));
  extractor.setNegative(true);//true removes the indices, false leaves only the indices
  extractor.filter(*out_filtered_cloud_ptr);
}

void RayGroundFilter::CloudCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr current_sensor_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr clipped_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  //remove points above certain point
  ClipCloud(current_sensor_cloud_ptr, clipping_height_, clipped_cloud_ptr);

  //remove closer points than a threshold
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  RemovePointsUpTo(clipped_cloud_ptr, min_point_distance_,filtered_cloud_ptr);

  //GetCloud Normals
  //pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals_ptr (new pcl::PointCloud<pcl::PointXYZINormal>);
  //GetCloudNormals(current_sensor_cloud_ptr, cloud_with_normals_ptr, 5.0);

  PointCloudXYZIRTColor organized_points;
  std::vector<pcl::PointIndices> radial_division_indices;
  std::vector<pcl::PointIndices> closest_indices;
  std::vector<PointCloudXYZIRTColor> radial_ordered_clouds;

  radial_dividers_num_ = ceil(360 / radial_divider_angle_);

  ConvertXYZIToRTZColor(filtered_cloud_ptr,
      organized_points,
      radial_division_indices,
      radial_ordered_clouds);

  pcl::PointIndices ground_indices, no_ground_indices;

  ClassifyPointCloud(radial_ordered_clouds, ground_indices, no_ground_indices);

  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  ExtractPointsIndices(filtered_cloud_ptr, ground_indices, ground_cloud_ptr, no_ground_cloud_ptr);

  publish_cloud(ground_points_pub_, ground_cloud_ptr, in_sensor_cloud->header);
  publish_cloud(groundless_points_pub_, no_ground_cloud_ptr, in_sensor_cloud->header);

}

RayGroundFilter::RayGroundFilter():node_handle_("~")
{
}

void RayGroundFilter::Run()
{
  //Model   |   Horizontal   |   Vertical   | FOV(Vertical)    degrees / rads
  //----------------------------------------------------------
  //HDL-64  |0.08-0.35(0.32) |     0.4      |  -24.9 <=x<=2.0   (26.9  / 0.47)
  //HDL-32  |     0.1-0.4    |     1.33     |  -30.67<=x<=10.67 (41.33 / 0.72)
  //VLP-16  |     0.1-0.4    |     2.0      |  -15.0<=x<=15.0   (30    / 0.52)
  //VLP-16HD|     0.1-0.4    |     1.33     |  -10.0<=x<=10.0   (20    / 0.35)
  ROS_INFO("Initializing Ground Filter, please wait...");
  node_handle_.param<std::string>("input_point_topic", input_point_topic_, "/points_raw");
  ROS_INFO("Input point_topic: %s", input_point_topic_.c_str());

  node_handle_.param("sensor_height", sensor_height_, 1.7);
  ROS_INFO("sensor_height[meters]: %f", sensor_height_);

  node_handle_.param("general_max_slope", general_max_slope_, 3.0);
  ROS_INFO("general_max_slope[deg]: %f", general_max_slope_);

  node_handle_.param("local_max_slope", local_max_slope_, 5.0);
  ROS_INFO("local_max_slope[deg]: %f", local_max_slope_);

  node_handle_.param("radial_divider_angle", radial_divider_angle_, 0.1); //1 degree default
  ROS_INFO("radial_divider_angle[deg]: %f", radial_divider_angle_);
  node_handle_.param("concentric_divider_distance", concentric_divider_distance_, 0.01);//0.1 meters default
  ROS_INFO("concentric_divider_distance[meters]: %f", concentric_divider_distance_);
  node_handle_.param("min_height_threshold", min_height_threshold_, 0.05);//0.05 meters default
  ROS_INFO("min_height_threshold[meters]: %f", min_height_threshold_);
  node_handle_.param("clipping_height", clipping_height_, 0.2);//0.2 meters default above the car
  ROS_INFO("clipping_height[meters]: %f", clipping_height_);
  node_handle_.param("min_point_distance", min_point_distance_, 1.85);//1.85 meters default
  ROS_INFO("min_point_distance[meters]: %f", min_point_distance_);
  node_handle_.param("reclass_distance_threshold", reclass_distance_threshold_, 0.2);//0.5 meters default
  ROS_INFO("reclass_distance_threshold[meters]: %f", reclass_distance_threshold_);


#if (CV_MAJOR_VERSION == 3)
  generateColors(colors_, color_num_);
#else
  cv::generateColors(colors_, color_num_);
#endif

  radial_dividers_num_ = ceil(360 / radial_divider_angle_);
  ROS_INFO("Radial Divisions: %d", (int)radial_dividers_num_);

  std::string no_ground_topic, ground_topic;
  node_handle_.param<std::string>("no_ground_point_topic", no_ground_topic, "/points_no_ground");
  ROS_INFO("No Ground Output Point Cloud no_ground_point_topic: %s", no_ground_topic.c_str());
  node_handle_.param<std::string>("ground_point_topic", ground_topic, "/points_ground");
  ROS_INFO("Only Ground Output Point Cloud ground_topic: %s", ground_topic.c_str());

  ROS_INFO("Subscribing to... %s", input_point_topic_.c_str());
  points_node_sub_ = node_handle_.subscribe(input_point_topic_, 1, &RayGroundFilter::CloudCallback, this);

  config_node_sub_ = node_handle_.subscribe("/config/ray_ground_filter", 1, &RayGroundFilter::update_config_params, this);

  groundless_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 2);
  ground_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(ground_topic, 2);

  ROS_INFO("Ready");

  ros::spin();

}

