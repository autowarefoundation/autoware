/*
 * ground_filter.cpp
 *
 *  Created on: Nov 4, 2016
 *      Author: ne0
 */
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>


class GroundFilter
{
public:
	GroundFilter();

private:

	ros::NodeHandle node_handle_;
	ros::Subscriber cloud_sub_;
	ros::Publisher 	cloud_lanes_pub_;
	ros::Publisher 	cloud_ground_pub_;

	std::string 	subscribe_topic_;

	bool			floor_removal_;

	double 			points_distance_;
	double 			angle_threshold_;


	void VelodyneCallback(const sensor_msgs::PointCloud2::Ptr& in_sensor_cloud_ptr);
	void RemoveFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
				pcl::PointCloud<pcl::PointXYZ>::Ptr out_nofloor_cloud_ptr,
				pcl::PointCloud<pcl::PointXYZ>::Ptr out_onlyfloor_cloud_ptr,
				float in_max_height,
				float in_floor_max_angle);

};

GroundFilter::GroundFilter() :
		node_handle_("~")
{

	node_handle_.param<std::string>("subscribe_topic",  subscribe_topic_,  "/points_clipped");

	node_handle_.param("remove_floor",  floor_removal_,  true);
	node_handle_.param("points_distance",  points_distance_,  0.2);
	node_handle_.param("angle_threshold",  angle_threshold_,  0.35);

	cloud_sub_ = node_handle_.subscribe(subscribe_topic_, 10, &GroundFilter::VelodyneCallback, this);
	cloud_lanes_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>( "/points_lanes", 10);
	cloud_ground_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>( "/points_ground", 10);
}

void GroundFilter::RemoveFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,
		pcl::PointCloud<pcl::PointXYZ>::Ptr out_nofloor_cloud_ptr,
		pcl::PointCloud<pcl::PointXYZ>::Ptr out_onlyfloor_cloud_ptr,
		float in_max_distance,
		float in_floor_max_angle)
{
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

	seg.setOptimizeCoefficients (true);
	seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setAxis(Eigen::Vector3f(0,0,1));
	seg.setEpsAngle(in_floor_max_angle);

	seg.setDistanceThreshold (in_max_distance);//floor distance
	seg.setOptimizeCoefficients(true);
	seg.setInputCloud(in_cloud_ptr);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size () == 0)
	{
		std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	}

	/*REMOVE THE FLOOR FROM THE CLOUD*/
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud (in_cloud_ptr);
	extract.setIndices(inliers);
	extract.setNegative(true);//true removes the indices, false leaves only the indices
	extract.filter(*out_nofloor_cloud_ptr);

	/*EXTRACT THE FLOOR FROM THE CLOUD*/
	extract.setNegative(false);//true removes the indices, false leaves only the indices
	extract.filter(*out_onlyfloor_cloud_ptr);
}

void GroundFilter::VelodyneCallback(const sensor_msgs::PointCloud2::Ptr& in_sensor_cloud_ptr)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr lanes_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::fromROSMsg(*in_sensor_cloud_ptr, *current_sensor_cloud_ptr);


	RemoveFloor(current_sensor_cloud_ptr, lanes_cloud_ptr, ground_cloud_ptr, points_distance_, angle_threshold_);

	if (!floor_removal_)
		output_cloud_ptr = current_sensor_cloud_ptr;
	else
		output_cloud_ptr = lanes_cloud_ptr;

	sensor_msgs::PointCloud2 cloud_ground_msg;
	sensor_msgs::PointCloud2 cloud_output_msg;

	pcl::toROSMsg(*ground_cloud_ptr, cloud_ground_msg);
	pcl::toROSMsg(*output_cloud_ptr, cloud_output_msg);

	cloud_ground_msg.header=in_sensor_cloud_ptr->header;
	cloud_ground_pub_.publish(cloud_ground_msg);

	cloud_output_msg.header=in_sensor_cloud_ptr->header;
	cloud_lanes_pub_.publish(cloud_output_msg);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "ground_filter");
	GroundFilter node;
	ros::spin();

	return 0;
}




