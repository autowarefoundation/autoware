/*
 * kf_track.cpp
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
#include <lidar_tracker/DetectedObject.h>
#include <lidar_tracker/DetectedObjectArray.h>


class KfTrack
{
public:
	KfTrack();

private:

	ros::NodeHandle node_handle_;
	ros::Subscriber cloud_clusters_sub_;
	ros::Publisher detected_objects_pub_;

	void CloudClustersCallback(const lidar_tracker::CloudClusterArray::Ptr& in_cloud_cluster_array_ptr);
};

KfTrack::KfTrack() :
		node_handle_("~")
{
	cloud_clusters_sub_ = node_handle_.subscribe("/cloud_clusters_class", 10, &KfTrack::CloudClustersCallback, this);
	detected_objects_pub_ = node_handle_.advertise<lidar_tracker::DetectedObjectArray>( "/detected_objects", 10);
}

void KfTrack::CloudClustersCallback(const lidar_tracker::CloudClusterArray::Ptr& in_cloud_cluster_array_ptr)
{
	lidar_tracker::DetectedObjectArray detected_objects;
	detected_objects.header = in_cloud_cluster_array_ptr->header;
	for (auto i = in_cloud_cluster_array_ptr->clusters.begin(); i != in_cloud_cluster_array_ptr->clusters.end(); i++)
	{
		lidar_tracker::DetectedObject detected_object;
		detected_object.header 		= i->header;
		detected_object.id 			= i->id;
		detected_object.label 		= i->label;
		detected_object.dimensions 	= i->bounding_box.dimensions;
		detected_object.pose 		= i->bounding_box.pose;

		detected_objects.objects.push_back(detected_object);
	}
	detected_objects_pub_.publish(detected_objects);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "kf_lidar_track");
	KfTrack node;
	ros::spin();

	return 0;
}
