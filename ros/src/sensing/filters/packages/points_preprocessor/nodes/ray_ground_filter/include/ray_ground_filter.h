/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 */
#ifndef RAY_GROUND_FILTER_H_
#define RAY_GROUND_FILTER_H_

#include <iostream>
#include <algorithm>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <velodyne_pointcloud/point_types.h>
#include "autoware_config_msgs/ConfigRayGroundFilter.h"

//headers in Autoware Health Checker
#include <autoware_health_checker/node_status_publisher.h>

#include <opencv2/core/version.hpp>
#if (CV_MAJOR_VERSION == 3)
	#include "gencolors.cpp"
#else
	#include <opencv2/contrib/contrib.hpp>
#endif

class RayGroundFilter
{
private:
	std::shared_ptr<autoware_health_checker::NodeStatusPublisher> node_status_pub_ptr_;
	ros::NodeHandle     node_handle_;
	ros::Subscriber     points_node_sub_;
	ros::Subscriber     config_node_sub_;
	ros::Publisher      groundless_points_pub_;
	ros::Publisher      ground_points_pub_;

	std::string         input_point_topic_;

	double              sensor_height_;//meters
	double              general_max_slope_;//degrees
	double              local_max_slope_;//degrees
	double              radial_divider_angle_;//distance in rads between dividers
	double              concentric_divider_distance_;//distance in meters between concentric divisions
	double              min_height_threshold_;//minimum height threshold regardless the slope, useful for close points
	double              clipping_height_; //the points higher than this will be removed from the input cloud.
	double              min_point_distance_;//minimum distance from the origin to consider a point as valid
	double              reclass_distance_threshold_;//distance between points at which re classification will occur

	size_t              radial_dividers_num_;
	size_t              concentric_dividers_num_;

	std::vector<cv::Scalar> colors_;
	const size_t        color_num_ = 60;//different number of color to generate

	struct PointXYZIRTColor
	{
		pcl::PointXYZI point;

		float radius;       //cylindric coords on XY Plane
		float theta;        //angle deg on XY plane

		size_t radial_div;  //index of the radial divsion to which this point belongs to
		size_t concentric_div;//index of the concentric division to which this points belongs to

		size_t red;         //Red component  [0-255]
		size_t green;       //Green Component[0-255]
		size_t blue;        //Blue component [0-255]

		size_t original_index; //index of this point in the source pointcloud
	};
	typedef std::vector<PointXYZIRTColor> PointCloudXYZIRTColor;

	void update_config_params(const autoware_config_msgs::ConfigRayGroundFilter::ConstPtr& param);

	void publish_cloud(const ros::Publisher& in_publisher,
	                         const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
	                         const std_msgs::Header& in_header);
	
	/*!
	 *
	 * @param[in] in_cloud Input Point Cloud to be organized in radial segments
	 * @param[out] out_organized_points Custom Point Cloud filled with XYZRTZColor data
	 * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each radial segment
	 * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
	 */
	void ConvertXYZIToRTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
	                           PointCloudXYZIRTColor& out_organized_points,
	                           std::vector<pcl::PointIndices>& out_radial_divided_indices,
	                           std::vector<PointCloudXYZIRTColor>& out_radial_ordered_clouds);


	/*!
	 * Classifies Points in the PointCoud as Ground and Not Ground
	 * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance from the origin
	 * @param out_ground_indices Returns the indices of the points classified as ground in the original PointCloud
	 * @param out_no_ground_indices Returns the indices of the points classified as not ground in the original PointCloud
	 */
	void ClassifyPointCloud(std::vector<PointCloudXYZIRTColor>& in_radial_ordered_clouds,
	                        pcl::PointIndices& out_ground_indices,
	                        pcl::PointIndices& out_no_ground_indices);
	

	/*!
	 * Removes the points higher than a threshold
	 * @param in_cloud_ptr PointCloud to perform Clipping
	 * @param in_clip_height Maximum allowed height in the cloud
	 * @param out_clipped_cloud_ptr Resultung PointCloud with the points removed
	 */
	void ClipCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
	               double in_clip_height,
	               pcl::PointCloud<pcl::PointXYZI>::Ptr out_clipped_cloud_ptr);
	

	/*!
	 * Returns the resulting complementary PointCloud, one with the points kept and the other removed as indicated
	 * in the indices
	 * @param in_cloud_ptr Input PointCloud to which the extraction will be performed
	 * @param in_indices Indices of the points to be both removed and kept
	 * @param out_only_indices_cloud_ptr Resulting PointCloud with the indices kept
	 * @param out_removed_indices_cloud_ptr Resulting PointCloud with the indices removed
	 */
	void ExtractPointsIndices(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
	                          const pcl::PointIndices& in_indices,
	                          pcl::PointCloud<pcl::PointXYZI>::Ptr out_only_indices_cloud_ptr,
	                          pcl::PointCloud<pcl::PointXYZI>::Ptr out_removed_indices_cloud_ptr);
	
	/*!
	 * Removes points up to a certain distance in the XY Plane
	 * @param in_cloud_ptr Input PointCloud
	 * @param in_min_distance Minimum valid distance, points closer than this will be removed.
	 * @param out_filtered_cloud_ptr Resulting PointCloud with the invalid points removed.
	 */
	void RemovePointsUpTo(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
	                      double in_min_distance,
	                      pcl::PointCloud<pcl::PointXYZI>::Ptr out_filtered_cloud_ptr);
	
	void CloudCallback(const sensor_msgs::PointCloud2ConstPtr &in_sensor_cloud);
	
friend class RayGroundFilter_clipCloud_Test;
public:
	RayGroundFilter();
  void Run();
};

#endif  // RAY_GROUND_FILTER_H_
