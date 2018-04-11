#include <ros/ros.h>
#include <iostream>
#include "visualize_cloud_cluster.h"

#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"

int main(int argc, char **argv)
{

	// std::cout<< 122<<std::endl;
	ros::init(argc, argv, "visualize_cloud_cluster");
	VisualizeCloudCluster app;
	ros::spin();

	return 0;
}