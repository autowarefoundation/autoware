#ifndef OBJECT_TRACKING_VisualizeCloudCluster_H
#define OBJECT_TRACKING_VisualizeCloudCluster_H

#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <pcl/io/io.h>

#include <vector>
#include <string>

#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"


// using namespace std;
// using namespace pcl;
// using namespace Eigen;

class VisualizeCloudCluster
{
private:
	std::string input_topic_;

	ros::NodeHandle node_handle_;
  ros::Subscriber sub_cloud_array_;
  ros::Publisher  pub_jsk_bb_;
  ros::Publisher  pub_arrow_;
	ros::Publisher  pub_id_;

	void getJskBBs(autoware_msgs::CloudClusterArray input,
				  jsk_recognition_msgs::BoundingBoxArray& jskBBs);
	void visArrows(autoware_msgs::CloudClusterArray input);
	void callBack(autoware_msgs::CloudClusterArray input);

public:
	VisualizeCloudCluster();
};

#endif //OBJECT_TRACKING_VisualizeCloudCluster_H
