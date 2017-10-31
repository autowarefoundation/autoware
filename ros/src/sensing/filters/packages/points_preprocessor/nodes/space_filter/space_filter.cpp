/*
 * space_filter.cpp
 *
 *  Created on: Nov 4, 2016
 *      Author: ne0
 */
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>


class SpaceFilter
{
public:
	SpaceFilter();

private:

	ros::NodeHandle node_handle_;
	ros::Subscriber cloud_sub_;
	ros::Publisher 	cloud_pub_;

	std::string 	subscribe_topic_;

	bool			lateral_removal_;
	bool			vertical_removal_;

	double 			left_distance_;
	double 			right_distance_;
	double 			below_distance_;
	double 			above_distance_;

	void VelodyneCallback(const sensor_msgs::PointCloud2::Ptr& in_sensor_cloud_ptr);
	void KeepLanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
							pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
							float in_left_lane_threshold,
							float in_right_lane_threshold);
	void ClipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
							pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
							float in_min_height,
							float in_max_height);
};

SpaceFilter::SpaceFilter() :
		node_handle_("~")
{

	node_handle_.param<std::string>("subscribe_topic",  subscribe_topic_,  "/points_raw");

	node_handle_.param("lateral_removal",  lateral_removal_,  true);
	node_handle_.param("left_distance",  left_distance_,  5.0);
	node_handle_.param("right_distance",  right_distance_,  5.0);

	node_handle_.param("vertical_removal",  vertical_removal_,  true);
	node_handle_.param("below_distance",  below_distance_,  -1.5);
	node_handle_.param("above_distance",  above_distance_,  0.5);

	cloud_sub_ = node_handle_.subscribe(subscribe_topic_, 10, &SpaceFilter::VelodyneCallback, this);
	cloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>( "/points_clipped", 10);
}

void SpaceFilter::KeepLanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
		pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
		float in_left_lane_threshold,
		float in_right_lane_threshold)
{
	pcl::PointIndices::Ptr far_indices (new pcl::PointIndices);
	for(unsigned int i=0; i< in_cloud_ptr->points.size(); i++)
	{
		pcl::PointXYZ current_point;
		current_point.x=in_cloud_ptr->points[i].x;
		current_point.y=in_cloud_ptr->points[i].y;
		current_point.z=in_cloud_ptr->points[i].z;

		if (
			current_point.y > (in_left_lane_threshold) || current_point.y < -1.0*in_right_lane_threshold
		)
		{
			far_indices->indices.push_back(i);
		}
	}
	out_cloud_ptr->points.clear();
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (in_cloud_ptr);
	extract.setIndices(far_indices);
	extract.setNegative(true);//true removes the indices, false leaves only the indices
	extract.filter(*out_cloud_ptr);
}

void SpaceFilter::ClipCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
		pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr,
		float in_min_height,
		float in_max_height)
{
	out_cloud_ptr->points.clear();
	for (unsigned int i=0; i<in_cloud_ptr->points.size(); i++)
	{
		if (in_cloud_ptr->points[i].z >= in_min_height &&
				in_cloud_ptr->points[i].z <= in_max_height)
		{
			out_cloud_ptr->points.push_back(in_cloud_ptr->points[i]);
		}
	}
}

void SpaceFilter::VelodyneCallback(const sensor_msgs::PointCloud2::Ptr& in_sensor_cloud_ptr)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr inlanes_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr clipped_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::fromROSMsg(*in_sensor_cloud_ptr, *current_sensor_cloud_ptr);

	if (lateral_removal_)
	{
		KeepLanes(current_sensor_cloud_ptr, inlanes_cloud_ptr, left_distance_, right_distance_);
	}
	else
	{
		inlanes_cloud_ptr = current_sensor_cloud_ptr;
	}
	if (vertical_removal_)
	{
		ClipCloud(inlanes_cloud_ptr, clipped_cloud_ptr, below_distance_, above_distance_);
	}
	else
	{
		clipped_cloud_ptr = inlanes_cloud_ptr;
	}

	sensor_msgs::PointCloud2 cloud_msg;

	pcl::toROSMsg(*clipped_cloud_ptr, cloud_msg);

	cloud_msg.header=in_sensor_cloud_ptr->header;
	cloud_pub_.publish(cloud_msg);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "space_filter");
	SpaceFilter node;
	ros::spin();

	return 0;
}




