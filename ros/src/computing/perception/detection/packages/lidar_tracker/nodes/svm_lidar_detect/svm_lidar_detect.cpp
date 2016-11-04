/*
 * svm_detect.cpp
 *
 *  Created on: Nov 3, 2016
 *      Author: ne0
 */


#include "ros/ros.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <lidar_tracker/CloudCluster.h>
#include <lidar_tracker/CloudClusterArray.h>


class SvmDetect
{
public:
	SvmDetect();

private:

	ros::NodeHandle node_handle_;
	ros::Subscriber cloud_clusters_sub_;
	ros::Publisher cloud_clusters_pub_;

	void CloudClustersCallback(const lidar_tracker::CloudClusterArray::Ptr& in_cloud_cluster_array_ptr);
};

SvmDetect::SvmDetect() :
		node_handle_("~")
{
	cloud_clusters_sub_ = node_handle_.subscribe("/cloud_clusters", 10, &SvmDetect::CloudClustersCallback, this);
	cloud_clusters_pub_ = node_handle_.advertise<lidar_tracker::CloudClusterArray>( "/cloud_clusters_class", 10);
}

void SvmDetect::CloudClustersCallback(const lidar_tracker::CloudClusterArray::Ptr& in_cloud_cluster_array_ptr)
{
	cloud_clusters_pub_.publish(*in_cloud_cluster_array_ptr);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "svm_lidar_detect");
	SvmDetect node;
	ros::spin();

	return 0;
}
