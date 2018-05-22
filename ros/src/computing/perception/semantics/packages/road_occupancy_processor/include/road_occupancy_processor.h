/*
 *  Copyright (c) 2018, Nagoya University
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
 *
 * road_occupancy_processor.cpp
 *
 *  Created on: Jan 29, 2018
 */

#ifndef PROJECT_ROAD_OCCUPANCY_PROCESSOR_H
#define PROJECT_ROAD_OCCUPANCY_PROCESSOR_H


#include <string>
#include <vector>
#include <chrono>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>

#include <tf/tf.h>

#include <vector_map/vector_map.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#if (CV_MAJOR_VERSION == 3)
#include <opencv2/imgcodecs.hpp>
#else
#include <opencv2/contrib/contrib.hpp>
#endif

#define __APP_NAME__ "road_occupancy_processor"

class RosRoadOccupancyProcessorApp

{
	ros::NodeHandle                     node_handle_;
	ros::Publisher                      publisher_occupancy_grid_;
	ros::Publisher                      publisher_grid_map_;

	cv::Mat                             road_wayarea_original_mat_;
	grid_map::GridMap                   gridmap_;

	tf::TransformListener*              transform_listener_;

	std::string                         wayarea_layer_name_;
	std::string                         output_layer_name_;//desired name to be published in the layer of the GridMap

	std::string                         input_gridmap_frame_;
	double                              input_gridmap_resolution_;
	grid_map::Length                    input_gridmap_length_;
	grid_map::Position                  input_gridmap_position_;

	typedef
	message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
			sensor_msgs::PointCloud2> SyncPolicyT;

	message_filters::Subscriber<sensor_msgs::PointCloud2>   *cloud_ground_subscriber_, *cloud_no_ground_subscriber_;
	//message_filters::Subscriber<grid_map_msgs::GridMap>     *gridmap_subscriber_;
	ros::Subscriber                                         gridmap_subscriber_;
	message_filters::Synchronizer<SyncPolicyT>              *cloud_synchronizer_;

	size_t                              radial_dividers_num_;

	const double                        radial_divider_angle_   = 0.1;
	const double                        concentric_divider_distance_ = 1;

	const int                           grid_min_value_         = 0;
	const int                           grid_max_value_         = 255;

	int                                 OCCUPANCY_ROAD_UNKNOWN  = 128;
	int                                 OCCUPANCY_ROAD_FREE     = 75;
	int                                 OCCUPANCY_ROAD_OCCUPIED = 0;
	int                                 OCCUPANCY_NO_ROAD       = 255;

	struct PointXYZIRT
	{
		pcl::PointXYZI  point;

		float           radius;       //cylindric coords on XY Plane
		float           theta;        //angle deg on XY plane

		size_t          radial_div;  //index of the radial divsion to which this point belongs to
		size_t          concentric_div;//index of the concentric division to which this points belongs to

		size_t          original_index; //index of this point in the source pointcloud
	};
	typedef std::vector<PointXYZIRT> PointCloudXYZIRTColor;

	/*!
	 *
	 * @param[in] in_cloud Input Point Cloud to be organized in radial segments
	 * @param[out] out_organized_points Custom Point Cloud filled with XYZRTZColor data
	 * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each radial segment
	 * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
	 */
	void ConvertXYZIToRTZ(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
	                      PointCloudXYZIRTColor& out_organized_points,
	                      std::vector<pcl::PointIndices>& out_radial_divided_indices,
	                      std::vector<PointCloudXYZIRTColor>& out_radial_ordered_clouds);

	/*!
	 * Resets road layer with in_grid_image
	 * @param in_grid_map Map to replace
	 * @param in_grid_image Image containing the desired data in the layer
	 * @return True if replacement was possible. False otherwise
	 */
	bool LoadRoadLayerFromMat(grid_map::GridMap &in_grid_map, cv::Mat &in_grid_image);

	/*!
	 * Publishes the GridMap object contained in the current instance
	 * @param[in] GridMap to Publish
	 * @param[in] Name of the layer to publish as OccupancyGrid
	 */
	void PublishGridMap(grid_map::GridMap& in_grid_map, const std::string& in_layer_publish);

	void Convert3dPointToOccupancy(grid_map::GridMap& in_grid_map,
	                               const geometry_msgs::Point& in_point,
	                               cv::Point& out_point);

	/*!
	 * Draws a line between two points in the bitmap
	 * @param in_grid_map GridMap object to modify
	 * @param in_grid_image Bitmap on which to draw the line
	 * @param in_start_point Initial point of the line
	 * @param in_end_point Final point of the line
	 * @param in_value Valid valid to assign to the points in the line
	 */
	void DrawLineInGridMap(grid_map::GridMap& in_grid_map,
	                       cv::Mat& in_grid_image,
	                       const geometry_msgs::Point& in_start_point,
	                       const geometry_msgs::Point& in_end_point,
	                       uchar in_value);

	/*!
	 * Draws a point in the occupancy grid
	 * @param in_grid_map GridMap object to modify
	 * @param in_grid_image Bitmap on which to draw the point
	 * @param in_point Point where to draw the point
	 * @param in_value Value to set in the whole point
	 */
	void SetPointInGridMap(grid_map::GridMap& in_grid_map,
	                       cv::Mat& in_grid_image,
	                       const geometry_msgs::Point& in_point,
	                       uchar in_value);

	/*!
	 * Receives the GridMap message and extract its geometry, occupancy bitmap
	 * @param in_message Received message
	 */
	void GridMapCallback(const grid_map_msgs::GridMap& in_message);
	/*!
	 * Receives 2 synchronized point cloud messages. in_ground_cloud_msg contains the points classified externally as
	 * ground, while in_no_ground_cloud_msg contains the points classified beloging to obstacle above the ground.
	 * @param[in] in_ground_cloud_msg Message containing pointcloud classified as ground.
	 * @param[in] in_no_ground_cloud_msg Message containing pointcloud classified as obstacle.
	 * @param[in] in_gridmap_msg Message containing the GridMap object with the road areas defined.
	 */
	void PointsCallback(const sensor_msgs::PointCloud2::ConstPtr& in_ground_cloud_msg,
	                    const sensor_msgs::PointCloud2::ConstPtr& in_no_ground_cloud_msg);

	/*!
	 * Obtains parameters from the command line, initializes subscribers and publishers.
	 * @param in_private_handle Ros private handle to get parameters for this node.
	 */
	void InitializeRosIo(ros::NodeHandle& in_private_handle);

	/*!
	 * Searches for the transformation between in_source_frame and in_target_frame in the TF tree
	 * @param in_target_frame Target Frame name
	 * @param in_source_frame Source Frame name
	 * @return The Transform, if any, to convert from in_source_frame to in_target_frame
	 */
	tf::StampedTransform FindTransform(const std::string& in_target_frame, const std::string& in_source_frame);

	/*!
	 * Transforms a pointcloud if the target frame is different
	 * @param in_pointcloud PointCloud to convert
	 * @param in_targetframe Target frame
	 * @param out_pointcloud Output pointcloud, if frame is the same, no transformation will be performed.
	 */
	void ConvertPointCloud(const pcl::PointCloud<pcl::PointXYZI>& in_pointcloud,
	                       const std::string& in_targetframe,
	                       pcl::PointCloud<pcl::PointXYZI>& out_pointcloud);
	/*!
	 * Transforms a Point using the specified transform
	 * @param in_point Point to transform
	 * @param in_transform Transformation data
	 * @return The transformed point
	 */
	geometry_msgs::Point TransformPoint(const geometry_msgs::Point &in_point, const tf::Transform &in_transform);

public:
	void Run();

	RosRoadOccupancyProcessorApp();
};

#endif //PROJECT_ROAD_OCCUPANCY_PROCESSOR_H
