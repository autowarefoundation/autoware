#include "visualize_cloud_cluster.h"

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>


#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <pcl/io/io.h>

#include <vector>


using namespace std;
using namespace pcl;
using namespace Eigen;

VisualizeCloudCluster::VisualizeCloudCluster()
{
	// sub_cloud_array_  = node_handle_.subscribe ("/bbox_cluster_array", 1, &VisualizeCloudCluster::callBack, this);
	sub_cloud_array_  = node_handle_.subscribe ("/tracking_cluster_array", 1, &VisualizeCloudCluster::callBack, this);
    pub_jsk_bb_       = node_handle_.advertise<jsk_recognition_msgs::BoundingBoxArray> ("/tracking_cluster_array/jsk_bb", 1);
    pub_arrow_        = node_handle_.advertise<visualization_msgs::Marker> ("/tracking_cluster_array/velocity_arrow", 1);
}

void VisualizeCloudCluster::callBack(autoware_msgs::CloudClusterArray input)
{
	jsk_recognition_msgs::BoundingBoxArray jsk_bbs;
	visualization_msgs::Marker arrows;
	getJskBB(input, jsk_bbs);
	pub_jsk_bb_.publish(jsk_bbs);
	visArrows(input);
	cout << "receive jsk call back" << endl;
}

void VisualizeCloudCluster::getJskBB(autoware_msgs::CloudClusterArray input,
							 jsk_recognition_msgs::BoundingBoxArray& jsk_bbs)
{
	jsk_bbs.header = input.header;
	
	for(size_t i = 0; i < input.clusters.size(); i++)
	{
		jsk_recognition_msgs::BoundingBox bb;
		bb = input.clusters[i].bounding_box;
		bb.header = input.header;
		string label = input.clusters[i].label;
		cout << label << endl;
		// bb.label = label;
		// ? jsk bb, how to find appropriate color
		if(label == "Stable")
		{
			bb.label = 2;
		}
		else if(label == "Static")
		{
			bb.label = 15;
		}

		jsk_bbs.boxes.push_back(bb);

	}
	// cout <<"cluster size " << jsk_bbs.boxes.size() << endl; 
}

void VisualizeCloudCluster::visArrows(autoware_msgs::CloudClusterArray input)
{
	for(size_t i = 0; i < input.clusters.size(); i++)
	{
		visualization_msgs::Marker arrows;
		arrows.lifetime = ros::Duration(0.1);
		string label = input.clusters[i].label;

		if(label == "None" || label == "Initialized" || label == "Lost" || label == "Static") 
		{
			continue;
		}
		
		arrows.header.frame_id = "/velodyne";
		arrows.header.stamp = input.header.stamp;
		arrows.ns = "arrows";
		arrows.action = visualization_msgs::Marker::ADD;
		arrows.type   = visualization_msgs::Marker::ARROW;
		// green
		arrows.color.g = 1.0f;
		arrows.color.a = 1.0;  
		arrows.id = i;
		geometry_msgs::Point p;
		// assert(targetPoints[i].size()==4);
		p.x = input.clusters[i].bounding_box.pose.position.x;
		p.y = input.clusters[i].bounding_box.pose.position.y;
		p.z = 0.5;
		double tv   = input.clusters[i].score;
		double tyaw = input.clusters[i].estimated_angle;

		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		arrows.pose.position.x = p.x;
		arrows.pose.position.y = p.y;
		arrows.pose.position.z = p.z;

		// convert from RPY to quartenion
		tf::Matrix3x3 obs_mat;
		obs_mat.setEulerYPR(tyaw, 0, 0); // yaw, pitch, roll
		tf::Quaternion q_tf;
		obs_mat.getRotation(q_tf);
		arrows.pose.orientation.x = q_tf.getX();
		arrows.pose.orientation.y = q_tf.getY();
		arrows.pose.orientation.z = q_tf.getZ();
		arrows.pose.orientation.w = q_tf.getW();

		// Set the scale of the arrows -- 1x1x1 here means 1m on a side
		arrows.scale.x = tv;
		arrows.scale.y = 0.1;
		arrows.scale.z = 0.1;

		pub_arrow_.publish(arrows);
	}
}
