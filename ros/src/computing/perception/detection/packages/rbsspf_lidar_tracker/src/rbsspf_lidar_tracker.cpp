/*
 *  Copyright (c) 2015, Nagoya University
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
*/
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_rviz_plugins/Pictogram.h>
#include <jsk_rviz_plugins/PictogramArray.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#if (CV_MAJOR_VERSION == 3)
	#include <opencv2/imgcodecs.hpp>
#else
	#include <opencv2/contrib/contrib.hpp>
#endif

#include <cmath>

//#include"rbsspfvehicletracker.cuh"
#include"rbsspf_tracker.cuh"//LiDAR Tracker

class RosRbssPfTrackerNode
{
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, jsk_recognition_msgs::BoundingBoxArray> TrackerSyncPolicy;

	ros::NodeHandle node_handle_;

	//Topic Subscribers and synchronizer
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
	message_filters::Subscriber<jsk_recognition_msgs::BoundingBoxArray> boxes_sub_;
	message_filters::Synchronizer<TrackerSyncPolicy> sync_subs;

	//Output publishers
	ros::Publisher publisher_boxes_;
	ros::Publisher publisher_detected_objects_;

	void SyncedCallback(const sensor_msgs::LaserScan::ConstPtr& in_scan, const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& in_boxes)
	{
	}

	void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& in_scan)
	{
	}
	void BoxesCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& in_boxes)
	{
	}

public:
	RosRbssPfTrackerNode(ros::NodeHandle& in_handle, std::string in_scan_topic, std::string in_boxes_topic):
			node_handle_(in_handle),
			laser_sub_(node_handle_, in_scan_topic , 10),
			boxes_sub_(node_handle_, in_boxes_topic , 10),
			sync_subs(TrackerSyncPolicy(10),laser_sub_, boxes_sub_)

	{
		laser_sub_.registerCallback(&RosRbssPfTrackerNode::LaserScanCallback, this);
		boxes_sub_.registerCallback(&RosRbssPfTrackerNode::BoxesCallback, this);
		sync_subs.registerCallback(boost::bind(&RosRbssPfTrackerNode::SyncedCallback, this, _1, _2));

		publisher_boxes_ = node_handle_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bounding_boxes_tracked",1);

		//Initialize LiDARTracker
		cudaOpenTracker();

	}

	~RosRbssPfTrackerNode()
	{
		//destroy LiDAR Tracker
		cudaCloseTracker();
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_message_filters");
	ros::NodeHandle nh;

	ros::NodeHandle private_node_handle("~");//to receive args

	std::string scan_topic_str;
	std::string boxes_topic_str;

	private_node_handle.param<std::string>("scan_topic", scan_topic_str, "/scan");				ROS_INFO("scan_topic: %s", scan_topic_str.c_str());
	private_node_handle.param<std::string>("boxes_topic", boxes_topic_str, "/bounding_boxes");	ROS_INFO("boxes_topic: %s", boxes_topic_str.c_str());

	RosRbssPfTrackerNode lidar_tracker_node(nh, scan_topic_str, boxes_topic_str);

	ros::Rate loop_rate(10);
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

/*
class RosRbssPfTrackerNode
{
	ros::Publisher publisher_boxes_;
	ros::Publisher publisher_detected_objects_;

	ros::NodeHandle node_handle_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> TrackerSyncPolicy;

	message_filters::Subscriber<sensor_msgs::Image> *sub1_;
	message_filters::Subscriber<sensor_msgs::Image> *sub2_;

	message_filters::Synchronizer<TrackerSyncPolicy> *sync_;

	//receive LaserScan and BoundingBoxes synced
	void synced_callback(const sensor_msgs::ImagePtr& in_scan, const sensor_msgs::ImagePtr& in_boxes)
	{
		//sync TF, Scan and BBoxes


		//vehicletracker->addTrackerData(curscan,initstate);
	}//end synced_callback


public:
	void Run()
	{
		//ROS STUFF
		ros::NodeHandle private_node_handle("~");//to receive args

		//RECEIVE TOPIC NAME
		std::string scan_topic_str;
		std::string boxes_topic_str;

		private_node_handle.param<std::string>("scan_topic", scan_topic_str, "/scan");					ROS_INFO("scan_topic: %s", scan_topic_str.c_str());
		private_node_handle.param<std::string>("boxes_topic", boxes_topic_str, "/cloud_clusters_class");	ROS_INFO("boxes_topic: %s", boxes_topic_str.c_str());

		//Initialize LiDARTracker
		cudaOpenTracker();



		publisher_boxes_ 	= node_handle_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bounding_boxes_tracked",1);

		sub1_ = new message_filters::Subscriber<sensor_msgs::Image>(node_handle_, "topic1", 10);
		sub2_ = new message_filters::Subscriber<sensor_msgs::Image>(node_handle_, "topic2", 10);

		sync_ = new message_filters::Synchronizer<TrackerSyncPolicy>(TrackerS);

		sync_->registerCallback(boost::bind(&RosRbssPfTrackerNode::synced_callback, this, _1, _2));

		ros::spin();
		ROS_INFO("END Rbsspf Lidar Tracker");

	}

	~RosRbssPfTrackerNode()
	{
		delete sub1_;
		delete sub2_;
	}

	RosRbssPfTrackerNode()
	{
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rbsspf_lidar_tracker");

	RosRbssPfTrackerNode app;

	app.Run();

	return 0;
}
*/
