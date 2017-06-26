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

#include <tf/tf.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#if (CV_MAJOR_VERSION == 3)
	#include <opencv2/imgcodecs.hpp>
#else
	#include <opencv2/contrib/contrib.hpp>
#endif

#include <cmath>
#include <vector>
#include <cstddef>

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

	std::vector<Tracker> trackers_pool_;//contains all the trackers among the time

	std::size_t trackers_num_;//current number of trackers

	LaserScan current_scan_;//GPU LaserScan

	bool previous_scan_stored_;//Previous LaserScan is required to perform matching before tracking

	void ros_laserscan_to_tracker_laserscan(const sensor_msgs::LaserScan::ConstPtr& in_sensor_scan, LaserScan& out_tracker_scan)
	{
		//copy data from Ros Msg to Custom LaserScan object
		out_tracker_scan.timestamp = in_sensor_scan->header.stamp.toSec();;
		out_tracker_scan.beamnum = in_sensor_scan->ranges.size();
		for(int i=0; i < out_tracker_scan.beamnum; i++)
		{
			out_tracker_scan.length[i] = in_sensor_scan->ranges[i];
		}
	}

	void get_detections_from_boxes(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& in_boxes, std::vector<Tracker>& out_trackers)
	{
		for (std::size_t i=0; i<in_boxes->boxes.size(); i++)
		{
			Tracker current_detection;
			jsk_recognition_msgs::BoundingBox current_box = in_boxes->boxes[i];

			current_detection.mean.x = current_box.pose.position.x;
			current_detection.mean.y = current_box.pose.position.y;

			tf::Quaternion box_quat;

			tf::quaternionMsgToTF(current_box.pose.orientation, box_quat);
			current_detection.mean.theta = tf::getYaw(box_quat);

			current_detection.mean.wl = 1.5;
			current_detection.mean.wr = 1.5;
			current_detection.mean.lf = 2.5;
			current_detection.mean.lb = 2.5;

			current_detection.mean.a = 0;
			current_detection.mean.v = 10;
			current_detection.mean.k = 0;
			current_detection.mean.omega = 0;

			current_detection.pfcount = 0;

			current_detection.id = trackers_num_++;
			current_detection.status = StatusInitGeometry;

			out_trackers.push_back(current_detection);
		}
	}

	void match_current_trackers_pool(const std::vector<Tracker>& in_current_trackers, std::vector<Tracker>& out_trackers)
	{
		if(in_current_trackers.size() > 0
			|| out_trackers.size() > 0)
		{
			std::vector<Tracker> tmp_trackers;
			for(std::size_t i=0, j=0; i < out_trackers.size( )|| j < in_current_trackers.size();)
			{
				if( i >= trackers_pool_.size()) //current tracker number is larger than the pool, add it (as new)
				{
					tmp_trackers.push_back(in_current_trackers[j]);
					j++;
				}
				else if(j >= in_current_trackers.size())//
				{
					tmp_trackers.push_back(out_trackers[i]);
					i++;
				}
				else if(out_trackers[i].id < in_current_trackers[j].id)
				{
					tmp_trackers.push_back(out_trackers[i]);
					i++;
				}
				else if(out_trackers[i].id > in_current_trackers[j].id)
				{
					tmp_trackers.push_back(in_current_trackers[j]);
					j++;
				}
				else
				{
					tmp_trackers.push_back(out_trackers[i]);
					i++;
					j++;
				}
			}
			cudaUpdateTracker(tmp_trackers);
			out_trackers.clear();
			//add remaining trackers if space available
			for(std::size_t i=0; i <tmp_trackers.size(); i++)
			{
				if(tmp_trackers[i].pfcount<20)
				{
					out_trackers.push_back(tmp_trackers[i]);
				}
			}
		}//end pool update
	}

	void SyncedCallback(const sensor_msgs::LaserScan::ConstPtr& in_scan, const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& in_boxes)
	{
		if (previous_scan_stored_)
		{
			std::vector<Tracker> current_frame_trackers;

			///////////////////////////////////////////////
			//get all the detections from the BoundingBoxes
			get_detections_from_boxes(in_boxes, current_frame_trackers);

			//////////////////////////////////////////
			//Try to match current frame with trackers in the pool
			match_current_trackers_pool(current_frame_trackers, trackers_pool_);

			/////////////////////////////////////
			//update LaserScan from sensor
			ros_laserscan_to_tracker_laserscan(in_scan, current_scan_);

			//check if using global or local coords
			//if global coords, try to get transformation using tf, from velodyne->world
			////TODO///////
			//current_scan.x = tflist[0].second.x;
			//current_scan.y = tflist[0].second.y;
			//current_scan.theta = tflist[0].second.theta;
			cudaSetLaserScan(current_scan_);
			previous_scan_stored_ = true;
		}//if (previous_scan_stored_)

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
			sync_subs(TrackerSyncPolicy(10),laser_sub_, boxes_sub_),
			previous_scan_stored_(false)

	{
		laser_sub_.registerCallback(&RosRbssPfTrackerNode::LaserScanCallback, this);
		boxes_sub_.registerCallback(&RosRbssPfTrackerNode::BoxesCallback, this);

		sync_subs.registerCallback(boost::bind(&RosRbssPfTrackerNode::SyncedCallback, this, _1, _2));

		publisher_boxes_ = node_handle_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bounding_boxes_tracked",1);

		//Initialize LiDARTracker
		cudaOpenTracker();

		trackers_num_ = 0;

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
