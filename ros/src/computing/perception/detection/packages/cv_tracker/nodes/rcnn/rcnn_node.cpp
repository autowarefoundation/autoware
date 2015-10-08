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
#include <cv_bridge/cv_bridge.h>
#include <runtime_manager/ConfigRcnn.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_tracker/image_obj.h>

#include <rcnn_detector.h>
#include <rect_class_score.h>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>

class RosRcnnApp
{
	ros::Subscriber subscriber_image_raw_;
	ros::Subscriber subscriber_rcnn_config_;
	ros::Publisher publisher_car_objects_;
	ros::Publisher publisher_person_objects_;
	ros::NodeHandle node_handle_;

	//Caffe based Object Detection ConvNet
	RcnnDetector* rcnn_detector_;

	//The minimum score required to filter the detected objects by the ConvNet
	float score_threshold_;

	//The percentage area to group bounding boxes obtained by the ConvNet
	float group_threshold_;

	//Number of slices to use for the creation of proposals for the ConvNet
	float image_slices_;

	//percentage of overlapping between the slices
	float slices_overlap_;

	//If GPU is enabled, stores the GPU Device to use
	unsigned int gpu_device_id_;

	//Sets whether or not use GPU acceleration
	bool use_gpu_;

	//vector of indices of the classes to search for
	std::vector<unsigned int> detect_classes_;

	void convert_rect_to_image_obj(std::vector< RectClassScore<float> >& in_objects, cv_tracker::image_obj& out_message, cv::Mat& in_image, std::string in_class)
	{
		for (unsigned int i = 0; i < in_objects.size(); ++i)
		{
			if ( (in_objects[i].score > score_threshold_)
				&& (	(in_class == "car" && (in_objects[i].class_type == Rcnn::CAR || in_objects[i].class_type == Rcnn::BUS))
						|| (in_class == "person" && (in_objects[i].class_type == Rcnn::PERSON || in_objects[i].class_type == Rcnn::BICYCLE))
					)

				)//check if the score is larger than minimum required
			{
				//std::cout << in_objects[i].toString() << std::endl;
				cv_tracker::image_rect rect;

				rect.x = in_objects[i].x;
				rect.y = in_objects[i].y;
				rect.width = in_objects[i].w;
				rect.height = in_objects[i].h;
				if (in_objects[i].x < 0)
					rect.x = 0;
				if (in_objects[i].y < 0)
					rect.y = 0;
				if (in_objects[i].w < 0)
					rect.width = 0;
				if (in_objects[i].h < 0)
					rect.height = 0;

				rect.score = in_objects[i].score;

				out_message.obj.push_back(rect);

			}
		}
	}

	void image_callback(const sensor_msgs::Image& image_source)
	{
		//Receive Image, convert it to OpenCV Mat
		cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source, "bgr8");//toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
		cv::Mat image = cv_image->image;

		//Detect Object in image
		std::vector< RectClassScore<float> > detections;
		//cv::TickMeter timer; timer.start();
		//std::cout << "score:" << score_threshold_ << " slices:" << image_slices_ << " slices overlap:" << slices_overlap_ << "nms" << group_threshold_ << std::endl;
		detections = rcnn_detector_->Detect(image, detect_classes_, score_threshold_, image_slices_, slices_overlap_, group_threshold_);

		//timer.stop();
		//std::cout << "Detection took: " << timer.getTimeMilli() << std::endl;

		//Prepare Output message
		cv_tracker::image_obj output_car_message;
		cv_tracker::image_obj output_person_message;
		output_car_message.header = image_source.header;
		output_car_message.type = "car";

		output_person_message.header = image_source.header;
		output_person_message.type = "person";

		//Convert Objects to Message type
		//timer.reset(); timer.start();
		convert_rect_to_image_obj(detections, output_car_message, image, "car");
		convert_rect_to_image_obj(detections, output_person_message, image, "person");

		publisher_car_objects_.publish(output_car_message);
		publisher_person_objects_.publish(output_person_message);
	}

	void config_cb(const runtime_manager::ConfigRcnn::ConstPtr& param)
	{
		rcnn_detector_->SetPixelMean(cv::Scalar(param->b_mean, param->g_mean, param->r_mean));

		score_threshold_ 	= param->score_threshold;
		group_threshold_ 	= param->group_threshold;
		image_slices_		= param->image_slices;
		slices_overlap_		= param->slices_overlap;

		/*use_gpu_			= param->use_gpu;
		gpu_device_id_		= param->gpu_device_id;*/
	}
public:
	void Run()
	{
		//ROS STUFF
		ros::NodeHandle private_node_handle("~");//to receive args

		//RECEIVE IMAGE TOPIC NAME
		std::string image_raw_topic_str;
		if (private_node_handle.getParam("image_raw_node", image_raw_topic_str))
		{
			ROS_INFO("Setting image node to %s", image_raw_topic_str.c_str());
		}
		else
		{
			ROS_INFO("No image node received, defaulting to /image_raw, you can use _image_raw_node:=YOUR_TOPIC");
			image_raw_topic_str = "/image_raw";
		}

		//RECEIVE CONVNET FILENAMES
		std::string network_definition_file;
		std::string pretrained_model_file;
		if (private_node_handle.getParam("network_definition_file", network_definition_file))
		{
			ROS_INFO("Network Definition File: %s", network_definition_file.c_str());
		}
		else
		{
			ROS_INFO("No Network Definition File was received. Finishing execution.");
			return;
		}
		if (private_node_handle.getParam("pretrained_model_file", pretrained_model_file))
		{
			ROS_INFO("Pretrained Model File: %s", pretrained_model_file.c_str());
		}
		else
		{
			ROS_INFO("No Pretrained Model File was received. Finishing execution.");
			return;
		}

		if (private_node_handle.getParam("use_gpu", use_gpu_))
		{
			ROS_INFO("GPU Mode: %d", use_gpu_);
		}
		int gpu_id;
		if (private_node_handle.getParam("gpu_device_id", gpu_id ))
		{
			ROS_INFO("GPU Device ID: %d", gpu_id);
			gpu_device_id_ = (unsigned int) gpu_id;
		}

		detect_classes_.push_back(Rcnn::CAR);
		detect_classes_.push_back(Rcnn::PERSON);
		detect_classes_.push_back(Rcnn::BUS);
		//RCNN STUFF
		rcnn_detector_ = new RcnnDetector(network_definition_file, pretrained_model_file, use_gpu_, gpu_device_id_);

		if (NULL == rcnn_detector_)
		{
			ROS_INFO("Error while creating RCNN Object");
			return;
		}

		publisher_car_objects_ = node_handle_.advertise<cv_tracker::image_obj>("/obj_car/image_obj", 1);
		publisher_person_objects_ = node_handle_.advertise<cv_tracker::image_obj>("/obj_person/image_obj", 1);

		ROS_INFO("Subscribing to... %s", image_raw_topic_str.c_str());
		subscriber_image_raw_ = node_handle_.subscribe(image_raw_topic_str, 1, &RosRcnnApp::image_callback, this);

		std::string config_topic("/config");	config_topic += ros::this_node::getNamespace() + "/rcnn";
		subscriber_rcnn_config_ =node_handle_.subscribe(config_topic, 1, &RosRcnnApp::config_cb, this);

		ros::spin();
		ROS_INFO("END rcnn");

	}

	~RosRcnnApp()
	{
		if (NULL != rcnn_detector_)
			delete rcnn_detector_;
	}

	RosRcnnApp()
	{
		rcnn_detector_ 	= NULL;
		score_threshold_= 0.6;
		group_threshold_= 0.8;
		image_slices_ 	= 16;
		use_gpu_ 		= false;
		gpu_device_id_ 	= 0;
		slices_overlap_ = 0.7;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rcnn_node");

	RosRcnnApp app;

	app.Run();

	return 0;
}
