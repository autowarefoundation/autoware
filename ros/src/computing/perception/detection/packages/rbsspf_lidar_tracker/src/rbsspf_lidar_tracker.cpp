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
#include <stdlib.h>

#include"rbsspfvehicletracker.cuh"
//#include"rbsspf_tracker.cuh"//LiDAR Tracker
enum class TrackerState
{
	NoLaserData,
	OneLaserData,
	ReadyForTracking,
	Processing
};
enum class ProcessingState
{
	Idle,
	InitGeometry,
	InitMotion,
	UpdateTracker
};

class LaserScanVehicleTracker
{
	ProcessingState			tracker_processing_state_;
	TrackerDataContainer 	tracker_data_container_;
	TrackerResultContainer 	tracker_result_container_;
	std::size_t 			lifespan_;
	unsigned long			tracker_id_;

public:
	TrackerDataContainer 	GetTrackerDataContainer()					{	return tracker_data_container_; 	}
	TrackerResultContainer 	GetTrackerResultContainer()				{	return tracker_result_container_;	}

	unsigned long			GetTrackerId()								{	return tracker_id_;					}

	std::size_t				GetLifespan()								{	return lifespan_;					}
	void					SetLifespan(std::size_t in_lifespan)		{	lifespan_ = in_lifespan;			}

	void Update(VehicleState& in_vehiclestate, bool in_real_detection)
	{
		std::cout << "Update Tracker " << tracker_id_ << ": ";
		switch(tracker_processing_state_)
		{
			case ProcessingState::InitGeometry:
				if(in_real_detection)
				{
					std::cout << "InitGeometry, Type:" << in_real_detection << ", (x,y,t)(" << in_vehiclestate.x << "," << in_vehiclestate.y << "," << in_vehiclestate.theta << ") " << std::endl;
					tracker_processing_state_ = ProcessingState::InitMotion;
					tracker_result_container_.estimate = in_vehiclestate;
					std::cout << "Container BEFORE (x,y,theta): (" << tracker_result_container_.estimate.x << "," << tracker_result_container_.estimate.y << "," << tracker_result_container_.estimate.theta << ")" << std::endl;
					cuda_InitGeometry(tracker_data_container_, tracker_result_container_);
					std::cout << "Container AFTER (x,y,theta): (" << tracker_result_container_.estimate.x << "," << tracker_result_container_.estimate.y << "," << tracker_result_container_.estimate.theta << ")" << std::endl;
				}
				break;
			case ProcessingState::InitMotion:
				std::cout << "InitMotion " << std::endl;
				tracker_processing_state_ = ProcessingState::UpdateTracker;
				std::cout << "Container BEFORE (x,y,theta): (" << tracker_result_container_.estimate.x << "," << tracker_result_container_.estimate.y << "," << tracker_result_container_.estimate.theta << ")" << std::endl;
				cuda_InitMotion(tracker_data_container_, tracker_result_container_);
				std::cout << "Container AFTER (x,y,theta): (" << tracker_result_container_.estimate.x << "," << tracker_result_container_.estimate.y << "," << tracker_result_container_.estimate.theta << ")" << std::endl;
				break;
			case ProcessingState::UpdateTracker:
				std::cout << "Container BEFORE (x,y,theta): (" << tracker_result_container_.estimate.x << "," << tracker_result_container_.estimate.y << "," << tracker_result_container_.estimate.theta << ")" << std::endl;
				std::cout << "UpdateTracker " << std::endl;
				cuda_UpdateTracker(tracker_data_container_, tracker_result_container_);
				std::cout << "Container AFTER (x,y,theta): (" << tracker_result_container_.estimate.x << "," << tracker_result_container_.estimate.y << "," << tracker_result_container_.estimate.theta << ")" << std::endl;
				break;
			default:
				std::cout << "OtherState " << std::endl;
				return;
		}
	}
	bool IsLifespanValid()
	{
		std::cout << "Tracker " << tracker_id_ << " ";
		if(    tracker_result_container_.estimate.dx > 1
			|| tracker_result_container_.estimate.dy > 1
			|| tracker_result_container_.estimate.dtheta > DEG2RAD(20)
			|| tracker_result_container_.estimate.count < 1
			)
		{
			lifespan_++;
			std::cout << " Increased>" << lifespan_;
		}
		else
		{
			lifespan_--;
			std::cout << " Decreased>" << lifespan_;
		}
		std::cout <<std::endl;

		if (lifespan_ > 10)//too old, mark as invalid
			return false;
		return true;
	}
	LaserScanVehicleTracker(unsigned long in_tracker_id)
	{
		lifespan_ 		= 0;
		tracker_id_		= in_tracker_id;
		tracker_processing_state_ = ProcessingState::InitGeometry;
		cuda_OpenTracker(tracker_data_container_);
	}

	~LaserScanVehicleTracker()
	{
		cuda_CloseTracker(tracker_data_container_);
	}
};

class RosRbssPfTrackerNode
{
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan,
														jsk_recognition_msgs::BoundingBoxArray> TrackerSyncPolicy;

	std::vector<LaserScanVehicleTracker> trackers_pool_;

	ros::NodeHandle*	node_handle_;
	//Output publishers
	ros::Publisher 		publisher_boxes_;
	ros::Publisher 		publisher_detected_objects_;

	LaserScan 			previous_scan_;//GPU LaserScan
	TrackerState 		tracker_state_;

	bool 				previous_scan_stored_;//Previous LaserScan is required to perform matching before tracking
	unsigned long		trackers_count_;//Number of CREATED trackers so far, not necessarily represents the number of EXISTENT trackers

	std::size_t			max_trackers_;

	//Topic Subscribers and synchronizer
	message_filters::Subscriber<sensor_msgs::LaserScan> 				laser_sub_;
	message_filters::Subscriber<jsk_recognition_msgs::BoundingBoxArray> boxes_sub_;
	message_filters::Synchronizer<TrackerSyncPolicy> 					sync_subs_;

	void ros_laserscan_to_tracker_laserscan(const sensor_msgs::LaserScan::ConstPtr& in_sensor_scan,
												LaserScan& out_tracker_scan)
	{
		//copy data from Ros Msg to Custom LaserScan object
		out_tracker_scan.timestamp = in_sensor_scan->header.stamp.toSec();;
		out_tracker_scan.beamnum = in_sensor_scan->ranges.size();
		for(int i=0; i < out_tracker_scan.beamnum; i++)
		{
			out_tracker_scan.length[i] = in_sensor_scan->ranges[i];
		}
		out_tracker_scan.x = 0.;
		out_tracker_scan.y = 0.;
		out_tracker_scan.theta = atan2(0.,1.);
		//ROS_INFO("Scan Beams: %d", out_tracker_scan.beamnum);
	}

	void get_detections_from_boxes(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& in_boxes,
												std::vector<VehicleState>& out_detections)
	{
		for (std::size_t i=0; i<in_boxes->boxes.size(); i++)
		{
			VehicleState vehicle_state;
			jsk_recognition_msgs::BoundingBox current_box = in_boxes->boxes[i];

			tf::Vector3 pt(current_box.pose.position.x, current_box.pose.position.y, current_box.pose.position.z);
			//tf::Vector3 converted = transformMap2Lidar * pt;
			//TODO transform to World Coords
			vehicle_state.x = pt.x();
			vehicle_state.y = pt.y();
			tf::Quaternion quat;
			tf::quaternionMsgToTF(current_box.pose.orientation, quat);
			vehicle_state.theta = tf::getYaw(quat);

			//initialize geometry width, length
			vehicle_state.wl=1.5;//left
			vehicle_state.wr=1.5;//right
			vehicle_state.lf=2.5;//front
			vehicle_state.lb=2.5;//back

			vehicle_state.a=0;//acceleration
			vehicle_state.v=10;//velocity
			vehicle_state.k=0;
			vehicle_state.omega=0;

			out_detections.push_back(vehicle_state);
		}
	}

	void update_tracker_with_detection(LaserScanVehicleTracker& in_tracker,
										VehicleState& in_detection,
										LaserScan& in_laserscan,
										bool in_real_detection)
	{
		switch(tracker_state_)
		{
			case TrackerState::NoLaserData:
				std::cout << in_tracker.GetTrackerId() << ": NoLaserData->OneLaserData";
				tracker_state_ = TrackerState::OneLaserData;
				//cuda_SetLaserScan(in_laserscan);
				break;
			case TrackerState::OneLaserData:
				std::cout << in_tracker.GetTrackerId() << ": OneLaserData->ReadyForTracking";
				tracker_state_ = TrackerState::ReadyForTracking;
				//cuda_SetLaserScan(in_laserscan);
				break;
			case TrackerState::ReadyForTracking:
				//cuda_SetLaserScan(in_laserscan);
				std::cout << in_tracker.GetTrackerId() << ": ReadyForTracking->Processing";
				if(in_real_detection)
					std::cout << ", (" << in_detection.x << "," << in_detection.y << "," << in_detection.theta << ") " << std::endl;

				tracker_state_ = TrackerState::Processing;
				in_tracker.Update(in_detection, in_real_detection);
				break;
			case TrackerState::Processing:
				break;
			default:
				std::cout << "update_tracker_with_detection unknown ";
				break;
		}
	}

	void match_trackers_detections(const std::vector<LaserScanVehicleTracker>& in_trackers,
										std::vector<VehicleState>& in_detections,
										std::vector<bool>& out_matched_detections,
										std::vector<bool>& out_updated_trackers,
										LaserScan& in_laserscan)
	{
		for (std::size_t j = 0; j < in_trackers.size(); j++)
		{
			LaserScanVehicleTracker cur_tracker = in_trackers[j];
			TrackerResultContainer tracker_result = cur_tracker.GetTrackerResultContainer();
			for(std::size_t i = 0; i < in_detections.size(); i++)
			{
				VehicleState cur_detection = in_detections[i];

				double tmp_dx = cur_detection.x - tracker_result.estimate.x;
				double tmp_dy = cur_detection.y - tracker_result.estimate.y;
				double dx = tmp_dx * cos( -tracker_result.estimate.theta ) -
							tmp_dy * sin( -tracker_result.estimate.theta );
				double dy = tmp_dx * sin( -tracker_result.estimate.theta ) +
							tmp_dy * cos( -tracker_result.estimate.theta );

				if(
						(dx <= tracker_result.estimate.wl + MARGIN2)
						&& (dx >= -tracker_result.estimate.wr - MARGIN2)
						&& (dy <= tracker_result.estimate.lf + MARGIN2)
						&& (dy >= -tracker_result.estimate.lb - MARGIN2)
					)//check overlap between tracker and detection
				{
					//overlap found, therefore update current tracker with detection
					out_matched_detections[i] = true;//mark this detection as matched

					if (!out_updated_trackers[j])//upddate tracker once
					{
						update_tracker_with_detection(cur_tracker, in_detections[i], in_laserscan, true);

						cur_tracker.SetLifespan(cur_tracker.GetLifespan()-1);
						ROS_INFO("Matched");
					}
					out_updated_trackers[j] = true;
				}
			}
		}
	}

	void trackers_to_boxes(const std::vector<LaserScanVehicleTracker>& in_trackers, const std_msgs::Header& in_header, jsk_recognition_msgs::BoundingBoxArray& out_boxes)
	{
		for (std::size_t  i = 0; i < in_trackers.size(); i++)
		{
			LaserScanVehicleTracker cur_tracker = in_trackers[i];
			if(tracker_state_ == TrackerState::ReadyForTracking)
			{
				TrackerResultContainer track = cur_tracker.GetTrackerResultContainer();
				jsk_recognition_msgs::BoundingBox bbox;
				bbox.header = in_header;
				bbox.pose.position.x = track.estimate.x;
				bbox.pose.position.y = track.estimate.y;
				bbox.pose.position.z = 0;

				bbox.dimensions.x = sqrt( pow(track.estimate.cx[2] - track.estimate.cx[1], 2) +
											pow(track.estimate.cy[2] - track.estimate.cy[1], 2)
										);
				bbox.dimensions.y = sqrt( pow(track.estimate.cx[1] - track.estimate.cx[0], 2) +
											pow(track.estimate.cy[1] - track.estimate.cy[0], 2)
										);
				bbox.dimensions.z = 2.0;

				tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, track.estimate.theta );
				tf::quaternionTFToMsg(quat, bbox.pose.orientation);

				bbox.value = 1.0;
				bbox.label = cur_tracker.GetTrackerId();

				out_boxes.boxes.push_back(bbox);
			}
		}
	}

	void SyncedCallback(const sensor_msgs::LaserScan::ConstPtr& in_scan,
							const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& in_boxes)
	{
		ROS_INFO("(1) RosRbssPfTrackerNode: Starting... Received %lu detections. %lu available trackers.",
						in_boxes->boxes.size(), trackers_pool_.size());
		std::vector<VehicleState> frame_detections;
		std::vector<bool> matched_detections, updated_trackers;
		matched_detections.resize(in_boxes->boxes.size(), false);
		updated_trackers.resize(trackers_pool_.size(), false);

		if (previous_scan_stored_)//Previous LaserScan required to start
		{
			//get vehicle states from bboxes
			get_detections_from_boxes(in_boxes, frame_detections);

			//all trackers use the same laserscan
			//try to match detections with trackers
			match_trackers_detections(trackers_pool_, frame_detections, matched_detections, updated_trackers, previous_scan_);

			//update all non-matched trackers, send empty detection
			for (std::size_t i = 0; i< trackers_pool_.size(); i++)
			{
				VehicleState empty_detection;
				update_tracker_with_detection(trackers_pool_[i], empty_detection, previous_scan_, false);
			}

			//create trackers for unassigned detections
			for(std::size_t i=0; i < frame_detections.size(); i++)
			{
				if(!matched_detections[i]
				                       && trackers_pool_.size()<=max_trackers_)
				{
					LaserScanVehicleTracker tracker(trackers_count_++);
					update_tracker_with_detection(tracker, frame_detections[i], previous_scan_, true);
					trackers_pool_.push_back(tracker);
				}
			}
			tracker_state_ = TrackerState::ReadyForTracking;

			//check lifespan, delete old trackers
			std::vector<LaserScanVehicleTracker>::iterator it;
			for(it = trackers_pool_.begin(); it != trackers_pool_.end();)
			{
				if ( !(it->IsLifespanValid()) ) //if not valid, delete
				{
					ROS_INFO("Removing tracker %lu", it->GetTrackerId());
					it = trackers_pool_.erase(it);
				}
				else
					it++;
			}
		}

		//store current laserscan
		ros_laserscan_to_tracker_laserscan(in_scan, previous_scan_);
		previous_scan_stored_ = true;

		cuda_SetLaserScan(previous_scan_);

		//output, publish
		jsk_recognition_msgs::BoundingBoxArray output_boxes;
		output_boxes.header = in_boxes->header;
		trackers_to_boxes(trackers_pool_, in_boxes->header, output_boxes);
		ROS_INFO("RosRbssPfTrackerNode: Publishing... %lu tracked objects", output_boxes.boxes.size());
		publisher_boxes_.publish(output_boxes);
	}

	void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& in_scan)
	{
	}
	void BoxesCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& in_boxes)
	{
	}

public:
	RosRbssPfTrackerNode(ros::NodeHandle* in_handle, std::string in_scan_topic, std::string in_boxes_topic, std::size_t in_max_trackers):
			node_handle_(in_handle),
			laser_sub_(*node_handle_, in_scan_topic , 10),
			boxes_sub_(*node_handle_, in_boxes_topic , 10),
			sync_subs_(TrackerSyncPolicy(10),laser_sub_, boxes_sub_),
			previous_scan_stored_(false)
	{
		laser_sub_.registerCallback(&RosRbssPfTrackerNode::LaserScanCallback, this);
		boxes_sub_.registerCallback(&RosRbssPfTrackerNode::BoxesCallback, this);

		tracker_state_ 	= TrackerState::NoLaserData;//initial state

		sync_subs_.registerCallback(boost::bind(&RosRbssPfTrackerNode::SyncedCallback, this, _1, _2));
		trackers_count_ = 0;
		max_trackers_ = in_max_trackers;
		publisher_boxes_ = node_handle_->advertise<jsk_recognition_msgs::BoundingBoxArray>("/bounding_boxes_tracked",1);
		cuda_InitLaserScan();
	}

	~RosRbssPfTrackerNode()
	{
		cuda_FreeLaserScan();
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rbsspf_lidar_tracker");
	ros::NodeHandle nh;

	ros::NodeHandle private_node_handle("~");//to receive args

	std::string 	scan_topic_str;
	std::string 	boxes_topic_str;
	int				max_trackers;
	const int 		MIN_TRACKERS = 5;

	private_node_handle.param<std::string>("scan_topic", scan_topic_str, "/scan");				ROS_INFO("scan_topic: %s", scan_topic_str.c_str());
	private_node_handle.param<std::string>("boxes_topic", boxes_topic_str, "/bounding_boxes");	ROS_INFO("boxes_topic: %s", boxes_topic_str.c_str());

	private_node_handle.param("max_trackers", max_trackers, MIN_TRACKERS);	ROS_INFO("max_trackers %ud", max_trackers);

	if (max_trackers <=MIN_TRACKERS ) max_trackers = MIN_TRACKERS;

	RosRbssPfTrackerNode lidar_tracker_node(&nh, scan_topic_str, boxes_topic_str, max_trackers);

	ros::Rate loop_rate(10);
	ROS_INFO("rbsspf_lidar_tracker: Waiting for %s and %s", scan_topic_str.c_str(), boxes_topic_str.c_str());
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
