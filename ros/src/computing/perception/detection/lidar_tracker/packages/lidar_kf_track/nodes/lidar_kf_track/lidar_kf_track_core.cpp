/*
 * kf_track.cpp
 *
 *  Created on: Nov 3, 2016
 *      Author: ne0
 */

#include <vector>

#include "ros/ros.h"

#include <pcl_ros/transforms.h>

#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_rviz_plugins/Pictogram.h>
#include <jsk_rviz_plugins/PictogramArray.h>
#include <jsk_recognition_msgs/PolygonArray.h>

#include "lidar_kf_track.h"

class KfLidarTrackNode
{
public:
	KfLidarTrackNode();
	~KfLidarTrackNode();

private:

	ros::NodeHandle node_handle_;
	ros::Subscriber cloud_clusters_sub_;
	ros::Publisher pub_jsk_hulls_;
	ros::Publisher pub_jsk_tracked_objects_;
	ros::Publisher pub_detected_objects_;
	ros::Publisher pub_jsk_boundingboxes_;
	ros::Publisher pub_jsk_pictograms_;

	bool pose_estimation_;
	int keep_alive_;
	int maximum_track_id_;

	boost::shared_ptr<KfLidarTracker> tracker_ptr;

	double distance_matching_threshold_;
	double tracker_merging_threshold_;

	void CloudClustersCallback(const autoware_msgs::CloudClusterArray::Ptr& in_cloud_cluster_array_ptr);
};

KfLidarTrackNode::KfLidarTrackNode() :
		node_handle_("~"),
		pose_estimation_(false)
{
	cloud_clusters_sub_ = node_handle_.subscribe("/cloud_clusters_class", 10, &KfLidarTrackNode::CloudClustersCallback, this);
	pub_detected_objects_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>( "/detected_objects", 10);
	pub_jsk_tracked_objects_ = node_handle_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bounding_boxes_tracked",1);
	pub_jsk_hulls_ = node_handle_.advertise<jsk_recognition_msgs::PolygonArray>("/cluster_hulls_tracked",1);
	pub_jsk_pictograms_ = node_handle_.advertise<jsk_rviz_plugins::PictogramArray>("/cluster_ids_tracked",1);

	node_handle_.param("distance_matching_threshold", distance_matching_threshold_, 1.5);
	ROS_INFO("distance_matching_threshold: %f", distance_matching_threshold_);// distance threshold to match objects between scans
	node_handle_.param("tracker_merging_threshold", tracker_merging_threshold_, 1.0);
	ROS_INFO("tracker_merging_threshold: %f", tracker_merging_threshold_);// distance threshold to match objects between scans
	node_handle_.param("pose_estimation", pose_estimation_, false);
	ROS_INFO("pose_estimation: %d", pose_estimation_);// whether or not to estimate pose
	node_handle_.param("keep_alive", keep_alive_, 2);
	ROS_INFO("keep_alive: %d", keep_alive_);// frames to keep an object
	node_handle_.param("maximum_track_id", maximum_track_id_, 200);
	ROS_INFO("maximum_track_id: %d", maximum_track_id_);// frames to keep an object



	tracker_ptr = boost::shared_ptr<KfLidarTracker>(new KfLidarTracker(0.2f,  //dt
							0.1f, 			//acceleration_noise
							distance_matching_threshold_, 			//matching distance threshold
							tracker_merging_threshold_, //tracker merging threshold
							keep_alive_, 				//life span
							keep_alive_,
							maximum_track_id_));			//trace length
}

KfLidarTrackNode::~KfLidarTrackNode()
{
}

void KfLidarTrackNode::CloudClustersCallback(const autoware_msgs::CloudClusterArray::Ptr& in_cloud_cluster_array_ptr)
{

	autoware_msgs::CloudClusterArray final_cloud_cluster_array;
	autoware_msgs::DetectedObjectArray detected_objects;
	detected_objects.header = in_cloud_cluster_array_ptr->header;


	//std::cout << "Update start" << std::endl;
	tracker_ptr->Update(*in_cloud_cluster_array_ptr, KfLidarTracker::CentersDist);
	//std::cout << "Update end" << std::endl;

	jsk_recognition_msgs::BoundingBoxArray tracked_boxes;
	jsk_recognition_msgs::PolygonArray tracked_hulls;
	jsk_rviz_plugins::PictogramArray tracked_ids;

	tracked_hulls.header = in_cloud_cluster_array_ptr->header;
	tracked_boxes.header = in_cloud_cluster_array_ptr->header;
	for (unsigned int i = 0; i < tracker_ptr->tracks_.size(); i++)
	{
		//BBOXES
		jsk_recognition_msgs::BoundingBox tracked_box;
		tracked_box = tracker_ptr->tracks_[i].GetCluster().bounding_box;
		tracked_box.header = in_cloud_cluster_array_ptr->header;
		tracked_box.label = tracker_ptr->tracks_[i].track_id;
		tracked_box.value = tracker_ptr->tracks_[i].track_id;
		//tracker_ptr->tracks_[i]->trace.end();//calculate orientation
		tracked_boxes.boxes.push_back(tracked_box);
		//END BBOXES

		//CONVEx HULL
		geometry_msgs::PolygonStamped hull;
		hull = tracker_ptr->tracks_[i].GetCluster().convex_hull;
		//std::cout << "hull size:" << hull.polygon.points.size() << std::endl;
		hull.header = in_cloud_cluster_array_ptr->header;
		tracked_hulls.polygons.push_back(hull);
		tracked_hulls.labels.push_back(tracker_ptr->tracks_[i].track_id);

		//END HULLS

		//PICTO
		jsk_rviz_plugins::Pictogram tracked_pictogram;
		tracked_pictogram.header = in_cloud_cluster_array_ptr->header;

		tracked_pictogram.mode = tracked_pictogram.STRING_MODE;
		tracked_pictogram.pose.position.x = tracker_ptr->tracks_[i].GetCluster().max_point.point.x;
		tracked_pictogram.pose.position.y = tracker_ptr->tracks_[i].GetCluster().max_point.point.y;
		tracked_pictogram.pose.position.z = tracker_ptr->tracks_[i].GetCluster().max_point.point.z;
		tf::Quaternion quat(0.0, -0.7, 0.0, 0.7);
		tf::quaternionTFToMsg(quat, tracked_pictogram.pose.orientation);
		tracked_pictogram.size = 4;
		std_msgs::ColorRGBA color;
		color.a = 1; color.r = 1; color.g = 1; color.b = 1;
		tracked_pictogram.color = color;
		tracked_pictogram.character = std::to_string( tracker_ptr->tracks_[i].track_id );
		tracked_ids.header = in_cloud_cluster_array_ptr->header;
		tracked_ids.pictograms.push_back(tracked_pictogram);
		//PICTO
	}

	pub_jsk_tracked_objects_.publish(tracked_boxes);
	pub_jsk_hulls_.publish(tracked_hulls);
	pub_jsk_pictograms_.publish(tracked_ids);

	//autoware_msgs::DetectedObjectArray detected_objects;
	//detected_objects.header = in_cloud_cluster_array_ptr->header;
	//for (auto i = in_cloud_cluster_array_ptr->clusters.begin(); i != in_cloud_cluster_array_ptr->clusters.end(); i++)
	//{
	//	autoware_msgs::DetectedObject detected_object;
	//	detected_object.header 		= i->header;
	//	detected_object.id 			= i->id;
	//	detected_object.label 		= i->label;
	//	detected_object.dimensions 	= i->bounding_box.dimensions;
	//	detected_object.pose 		= i->bounding_box.pose;

	//	detected_objects.objects.push_back(detected_object);
	//}
	//detected_objects_pub_.publish(detected_objects);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "kf_lidar_track");
	KfLidarTrackNode node;
	ros::spin();

	return 0;
}
