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

#include "road_occupancy_processor.h"

void RosRoadOccupancyProcessorApp::ConvertXYZIToRTZ(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                                                    RosRoadOccupancyProcessorApp::PointCloudXYZIRTColor &out_organized_points,
                                                    std::vector<pcl::PointIndices> &out_radial_divided_indices,
                                                    std::vector<RosRoadOccupancyProcessorApp::PointCloudXYZIRTColor> &out_radial_ordered_clouds)
{
	out_organized_points.resize(in_cloud->points.size());
	out_radial_divided_indices.clear();
	out_radial_divided_indices.resize(radial_dividers_num_);
	out_radial_ordered_clouds.resize(radial_dividers_num_);

	for(size_t i=0; i< in_cloud->points.size(); i++)
	{
		PointXYZIRT new_point;
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
		          [](const PointXYZIRT& a, const PointXYZIRT& b){ return a.radius < b.radius; });
	}
}

void RosRoadOccupancyProcessorApp::PublishGridMap(grid_map::GridMap &in_grid_map, const std::string& in_layer_publish)
{
	if (in_grid_map.exists(in_layer_publish))
	{
		grid_map_msgs::GridMap ros_gridmap_message;
		nav_msgs::OccupancyGrid ros_occupancygrid_message;
		grid_map::GridMapRosConverter::toMessage(in_grid_map, ros_gridmap_message);
		grid_map::GridMapRosConverter::toOccupancyGrid(in_grid_map,
		                                               in_layer_publish,
		                                               grid_min_value_,
		                                               grid_max_value_,
		                                               ros_occupancygrid_message);
		publisher_grid_map_.publish(ros_gridmap_message);
		publisher_occupancy_grid_.publish(ros_occupancygrid_message);
	}
	else
	{
		ROS_INFO("[%s] Empty GridMap. It might still be loading or it does not contain valid data.", __APP_NAME__);
	}
}

bool RosRoadOccupancyProcessorApp::LoadRoadLayerFromMat(grid_map::GridMap &in_grid_map, cv::Mat &in_grid_image)
{
	if (!in_grid_image.empty())
	{
		grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 1>(in_grid_image,
		                                                                  output_layer_name_,
		                                                                  in_grid_map,
		                                                                  grid_min_value_,
		                                                                  grid_max_value_);

		return true;
	}

	ROS_INFO("[%s] Empty Image received.", __APP_NAME__);
	return false;
}

void RosRoadOccupancyProcessorApp::Convert3dPointToOccupancy(grid_map::GridMap &in_grid_map,
                                                             const geometry_msgs::Point &in_point, cv::Point &out_point)
{
	// calculate position
	grid_map::Position map_pos = in_grid_map.getPosition();
	double origin_x_offset = in_grid_map.getLength().x() / 2.0 - map_pos.x();
	double origin_y_offset = in_grid_map.getLength().y() / 2.0 - map_pos.y();
	// coordinate conversion for cv image
	out_point.x = (in_grid_map.getLength().y() - origin_y_offset - in_point.y) / in_grid_map.getResolution();
	out_point.y = (in_grid_map.getLength().x() - origin_x_offset - in_point.x) / in_grid_map.getResolution();
}

void RosRoadOccupancyProcessorApp::DrawLineInGridMap(grid_map::GridMap &in_grid_map, cv::Mat &in_grid_image,
                                                     const geometry_msgs::Point &in_start_point,
                                                     const geometry_msgs::Point &in_end_point, uchar in_value)
{
	cv::Point cv_start_point, cv_end_point;
	Convert3dPointToOccupancy(in_grid_map, in_start_point, cv_start_point);
	Convert3dPointToOccupancy(in_grid_map, in_end_point, cv_end_point);

	cv::Rect rect(cv::Point(), in_grid_image.size());

	if(!rect.contains(cv_start_point) || !rect.contains(cv_end_point))
	{
		return;
	}
	if (in_grid_image.at<uchar>(cv_start_point.y, cv_start_point.x) != OCCUPANCY_NO_ROAD
	    && in_grid_image.at<uchar>(cv_end_point.y, cv_end_point.x) != OCCUPANCY_NO_ROAD)
	{
		const int line_width = 3;
		cv::line(in_grid_image, cv_start_point, cv_end_point, cv::Scalar(in_value), line_width);
	}
}

void RosRoadOccupancyProcessorApp::SetPointInGridMap(grid_map::GridMap &in_grid_map, cv::Mat &in_grid_image,
                                                     const geometry_msgs::Point &in_point, uchar in_value)
{
	// calculate position
	cv::Point cv_point;
	Convert3dPointToOccupancy(in_grid_map, in_point, cv_point);

	cv::Rect rect(cv::Point(), in_grid_image.size());

	if(!rect.contains(cv_point))
		return;

	if (in_grid_image.at<uchar>(cv_point.y, cv_point.x) != OCCUPANCY_NO_ROAD)
	{
		const int radius = 2;
		const int fill = -1;
		cv::circle(in_grid_image, cv::Point(cv_point.x, cv_point.y), radius, cv::Scalar(in_value), fill);
	}
}

void RosRoadOccupancyProcessorApp::GridMapCallback(const grid_map_msgs::GridMap& in_message)
{
	grid_map::GridMap input_grid;
	grid_map::GridMapRosConverter::fromMessage(in_message, input_grid);

	grid_map::GridMapCvConverter::toImage<unsigned char, 1>(input_grid,
	                                                        wayarea_layer_name_,
	                                                        CV_8UC1,
	                                                        grid_min_value_,
	                                                        grid_max_value_,
	                                                        road_wayarea_original_mat_);

	input_gridmap_frame_        = input_grid.getFrameId();
	input_gridmap_length_       = input_grid.getLength();
	input_gridmap_resolution_   = input_grid.getResolution();
	input_gridmap_position_     = input_grid.getPosition();
}

void RosRoadOccupancyProcessorApp::ConvertPointCloud(const pcl::PointCloud<pcl::PointXYZI>& in_pointcloud,
                                                     const std::string& in_targetframe,
                                                     pcl::PointCloud<pcl::PointXYZI>& out_pointcloud)
{
	//check that both pointcloud and grid are in the same frame, otherwise transform
	if (in_pointcloud.header.frame_id != in_targetframe)
	{
		ROS_INFO("transformPointCloud");
		tf::Transform map2sensor_transform = FindTransform(in_targetframe, in_pointcloud.header.frame_id);
		pcl_ros::transformPointCloud(in_pointcloud, out_pointcloud, map2sensor_transform);
	}
	else
	{
		out_pointcloud = in_pointcloud;
	}
};

void RosRoadOccupancyProcessorApp::PointsCallback(const sensor_msgs::PointCloud2::ConstPtr &in_ground_cloud_msg,
                                                  const sensor_msgs::PointCloud2::ConstPtr &in_no_ground_cloud_msg)
{
	if(road_wayarea_original_mat_.empty())
		return;

	// timer start
	//auto start = std::chrono::system_clock::now();

	cv::Mat current_road_mat = road_wayarea_original_mat_.clone();
	cv::Mat original_road_mat = current_road_mat.clone();

	grid_map::GridMap output_gridmap;
	output_gridmap.setFrameId(input_gridmap_frame_);
	output_gridmap.setGeometry(input_gridmap_length_,
	                           input_gridmap_resolution_,
	                           input_gridmap_position_);

	//output_gridmap[output_layer_name_].setConstant(OCCUPANCY_NO_ROAD);

	pcl::PointCloud<pcl::PointXYZI>::Ptr in_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr in_no_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr final_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr final_no_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>);

	pcl::fromROSMsg(*in_ground_cloud_msg, *in_ground_cloud);
	pcl::fromROSMsg(*in_no_ground_cloud_msg, *in_no_ground_cloud);

	//check that both pointcloud and grid are in the same frame, otherwise transform
	ConvertPointCloud(*in_ground_cloud, output_gridmap.getFrameId(), *final_ground_cloud);
	ConvertPointCloud(*in_no_ground_cloud, output_gridmap.getFrameId(), *final_no_ground_cloud);

	//organize pointcloud in cylindrical coords
	PointCloudXYZIRTColor organized_points;
	std::vector<pcl::PointIndices> radial_division_indices;
	std::vector<pcl::PointIndices> closest_indices;
	std::vector<PointCloudXYZIRTColor> radial_ordered_clouds;

	ConvertXYZIToRTZ(final_ground_cloud,
	                 organized_points,
	                 radial_division_indices,
	                 radial_ordered_clouds);

	//draw lines between initial and last point of each ray
	for (size_t i = 0; i < radial_ordered_clouds.size(); i++)//sweep through each radial division
	{
		geometry_msgs::Point prev_point;
		for (size_t j = 0; j < radial_ordered_clouds[i].size(); j++)//loop through each point
		{
			geometry_msgs::Point current_point;
			current_point.x = radial_ordered_clouds[i][j].point.x;
			current_point.y = radial_ordered_clouds[i][j].point.y;
			current_point.z = radial_ordered_clouds[i][j].point.z;

			DrawLineInGridMap(output_gridmap, current_road_mat, prev_point, current_point, OCCUPANCY_ROAD_FREE);
		}
	}

	//process obstacle points
	for (const auto &point:final_no_ground_cloud->points)
	{
		geometry_msgs::Point sensor_point;
		sensor_point.x = point.x;
		sensor_point.y = point.y;
		sensor_point.z = point.z;
		SetPointInGridMap(output_gridmap, current_road_mat, sensor_point, OCCUPANCY_ROAD_OCCUPIED);
	}

#pragma omp for
	for(int row = 0; row < current_road_mat.rows; row++)
	{
		for(int col = 0; col < current_road_mat.cols; col++)
		{
			if(original_road_mat.at<uchar>(row,col) == OCCUPANCY_NO_ROAD)
			{
				current_road_mat.at<uchar>(row,col) = OCCUPANCY_NO_ROAD;
			}
		}
	}
	//cv::imshow("result", current_road_mat);
	//cv::waitKey(10);
	LoadRoadLayerFromMat(output_gridmap, current_road_mat);

	PublishGridMap(output_gridmap, output_layer_name_);

	// timer end
	//auto end = std::chrono::system_clock::now();
	//auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
	//std::cout << "time: " << usec / 1000.0 << " [msec]" << std::endl;

}

void RosRoadOccupancyProcessorApp::InitializeRosIo(ros::NodeHandle &in_private_handle)
{
	//get params
	std::string points_ground_topic_str, points_no_ground_topic_str, wayarea_topic_str;

	in_private_handle.param<std::string>("points_ground_src", points_ground_topic_str, "points_ground");
	ROS_INFO("[%s] points_ground_src: %s",__APP_NAME__, points_ground_topic_str.c_str());

	in_private_handle.param<std::string>("points_no_ground_src", points_no_ground_topic_str, "points_no_ground");
	ROS_INFO("[%s] points_no_ground_src: %s",__APP_NAME__, points_no_ground_topic_str.c_str());

	in_private_handle.param<std::string>("wayarea_topic_src", wayarea_topic_str, "grid_map_wayarea");
	ROS_INFO("[%s] wayarea_topic_src: %s",__APP_NAME__, wayarea_topic_str.c_str());

	in_private_handle.param<std::string>("wayarea_layer_name", wayarea_layer_name_, "wayarea");
	ROS_INFO("[%s] wayarea_layer_name: %s",__APP_NAME__, wayarea_layer_name_.c_str());

	in_private_handle.param<std::string>("output_layer_name", output_layer_name_, "road_status");
	ROS_INFO("[%s] output_layer_name: %s",__APP_NAME__, output_layer_name_.c_str());

	in_private_handle.param<int>("road_unknown_value", OCCUPANCY_ROAD_UNKNOWN, 128);
	ROS_INFO("[%s] road_unknown_value: %d",__APP_NAME__, OCCUPANCY_ROAD_UNKNOWN);

	in_private_handle.param<int>("road_free_value", OCCUPANCY_ROAD_FREE, 75);
	ROS_INFO("[%s] road_free_value: %d",__APP_NAME__, OCCUPANCY_ROAD_FREE);

	in_private_handle.param<int>("road_occupied_value", OCCUPANCY_ROAD_OCCUPIED, 0);
	ROS_INFO("[%s] road_occupied_value: %d",__APP_NAME__, OCCUPANCY_ROAD_OCCUPIED);

	in_private_handle.param<int>("no_road_value", OCCUPANCY_NO_ROAD, 255);
	ROS_INFO("[%s] no_road_value: %d",__APP_NAME__, OCCUPANCY_NO_ROAD);

	//generate subscribers and sychronizers
	cloud_ground_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
	                                                                                     points_ground_topic_str, 1);
	ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_ground_topic_str.c_str());
	cloud_no_ground_subscriber_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_handle_,
	                                                                                        points_no_ground_topic_str, 1);
	ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, points_no_ground_topic_str.c_str());

	/*gridmap_subscriber_ = new message_filters::Subscriber<grid_map_msgs::GridMap>(node_handle_,
	                                                                              wayarea_topic_str, 1);
	gridmap_subscriber_->registerCallback(boost::bind(&RosRoadOccupancyProcessorApp::PointsCallback, this));*/
	gridmap_subscriber_ = node_handle_.subscribe(wayarea_topic_str, 10,
	                                                             &RosRoadOccupancyProcessorApp::GridMapCallback, this);
	ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, wayarea_topic_str.c_str());

	cloud_synchronizer_ =
			new message_filters::Synchronizer<SyncPolicyT>(SyncPolicyT(100),
			                                               *cloud_ground_subscriber_,
			                                               *cloud_no_ground_subscriber_);
	cloud_synchronizer_->registerCallback(boost::bind(&RosRoadOccupancyProcessorApp::PointsCallback, this, _1, _2));

	//register publishers
	publisher_grid_map_= node_handle_.advertise<grid_map_msgs::GridMap>("gridmap_road_status", 1);
	ROS_INFO("[%s] Publishing GridMap in gridmap_road_status",__APP_NAME__);

	publisher_occupancy_grid_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("occupancy_road_status", 1);
	ROS_INFO("[%s] Publishing Occupancy grid in occupancy_road_status",__APP_NAME__);
}

tf::StampedTransform
RosRoadOccupancyProcessorApp::FindTransform(const std::string &in_target_frame, const std::string &in_source_frame)
{
	tf::StampedTransform transform;

	try
	{
		transform_listener_->lookupTransform(in_target_frame, in_source_frame, ros::Time(0), transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
	}

	return transform;
}

geometry_msgs::Point
RosRoadOccupancyProcessorApp::TransformPoint(const geometry_msgs::Point &in_point, const tf::Transform &in_transform)
{
	tf::Point tf_point;
	tf::pointMsgToTF(in_point, tf_point);

	tf_point = in_transform * tf_point;

	geometry_msgs::Point geometry_point;
	tf::pointTFToMsg(tf_point, geometry_point);

	return geometry_point;
}

void RosRoadOccupancyProcessorApp::Run()
{
	ros::NodeHandle private_node_handle("~");
	tf::TransformListener transform_listener;

	transform_listener_ = &transform_listener;

	InitializeRosIo(private_node_handle);

	ROS_INFO("[%s] Ready. Waiting for data...",__APP_NAME__);

	ros::spin();

	ROS_INFO("[%s] END",__APP_NAME__);
}

RosRoadOccupancyProcessorApp::RosRoadOccupancyProcessorApp()
{
	radial_dividers_num_ = ceil(360 / radial_divider_angle_);
}