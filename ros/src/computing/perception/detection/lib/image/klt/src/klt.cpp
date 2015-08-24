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

//ROS STUFF
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_tracker/image_obj.h>
#include <cv_tracker/image_obj_tracked.h>

//TRACKING STUFF
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <LkTracker.hpp>

#include <iostream>
#include <stdio.h>

#include <sstream>


class RosTrackerApp
{
	ros::Subscriber subscriber_image_raw_;
	ros::Subscriber subscriber_image_obj_;
	ros::Subscriber subscriber_klt_config_;
	ros::Publisher publisher_tracked_objects;//ROS
	ros::NodeHandle node_handle_;

	LkTracker obj_tracker_;

	std::vector<cv::LatentSvmDetector::ObjectDetection> dpm_detections_;
	bool ready_;

public:
	void image_callback(const sensor_msgs::Image& image_source)
	{
		//std::cout << "I" << std::endl;
		if (!ready_)
			return;

		cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::TYPE_8UC3);
		cv::Mat image_track = cv_image->image;

		if (!dpm_detections_.empty())
			obj_tracker_.Track(image_track, dpm_detections_[0].rect);
	}

	void detections_callback(cv_tracker::image_obj image_objects_msg)
	{
		//std::cout << "D" << std::endl;
		if(ready_)
				return;
		unsigned int num = image_objects_msg.obj.size();
		std::vector<cv_tracker::image_rect> objects = image_objects_msg.obj;
		//object_type = image_objects_msg.type;
		//points are X,Y,W,H and repeat for each instance
		dpm_detections_.clear();

		for (unsigned int i=0; i<num;i++)
		{
			cv::Rect tmp;
			tmp.x = objects.at(i).x;
			tmp.y = objects.at(i).y;
			tmp.width = objects.at(i).width;
			tmp.height = objects.at(i).height;
			dpm_detections_.push_back(cv::LatentSvmDetector::ObjectDetection(tmp, 0));
		}
		ready_ = true;
	}

	void klt_config_cb()
	{

	}


	void Run()
	{
		std::string image_raw_topic_str;
		std::string image_obj_topic_str;

		ros::NodeHandle private_node_handle("~");//to receive args

		if (private_node_handle.getParam("image_raw_node", image_raw_topic_str))
			{
				ROS_INFO("Setting image node to %s", image_raw_topic_str.c_str());
			}
		else
		{
			ROS_INFO("No image node received, defaulting to /image_raw, you can use _image_raw_node:=YOUR_TOPIC");
			image_raw_topic_str = "/image_raw";
		}
		if (private_node_handle.getParam(ros::this_node::getNamespace() + "img_obj_node", image_obj_topic_str))
			{
				ROS_INFO("Setting object node to %s", image_obj_topic_str.c_str());
			}
		else
		{
			ROS_INFO("No object node received, defaulting to image_obj, you can use _img_obj_node:=YOUR_TOPIC");
			image_obj_topic_str = "image_obj";
		}


		publisher_tracked_objects = node_handle_.advertise<cv_tracker::image_obj_tracked>("image_obj_tracked", 1);

		ROS_INFO("Subscribing to... %s", image_raw_topic_str.c_str());
		ROS_INFO("Subscribing to... %s", image_obj_topic_str.c_str());
		subscriber_image_raw_ = node_handle_.subscribe(image_raw_topic_str, 1, &RosTrackerApp::image_callback, this);
		subscriber_image_obj_ = node_handle_.subscribe(image_obj_topic_str, 1, &RosTrackerApp::detections_callback, this);

		std::string config_topic("/config");
		config_topic += ros::this_node::getNamespace() + "/klt";
		//node_handle.subscribe(config_topic, 1, &RosTrackerApp::klt_config_cb, this);

		ros::spin();
		ROS_INFO("END klt");
	}

	RosTrackerApp()
	{
		ready_ = false;
	}

};

int klt_main(int argc, char* argv[])
{
	ros::init(argc, argv, "klt");

	RosTrackerApp app;

	app.Run();

	return 0;
}
