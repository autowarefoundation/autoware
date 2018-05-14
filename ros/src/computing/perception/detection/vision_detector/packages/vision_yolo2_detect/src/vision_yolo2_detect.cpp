#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <autoware_msgs/ConfigSsd.h>
#include <autoware_msgs/image_obj.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#if (CV_MAJOR_VERSION != 3)
#include <opencv2/contrib/contrib.hpp>
#endif
#include <opencv2/highgui/highgui.hpp>


#include <string>
#include <vector>

#include <math.h>
#include <stdlib.h>

#include <rect_class_score.h>

#include "darknet/yolo2.h"

namespace Yolo2
{
	enum YoloDetectorClasses//using coco for default cfg and weights
	{
		PERSON, BICYCLE, CAR, MOTORBIKE, AEROPLANE, BUS, TRAIN, TRUCK, BOAT, TRAFFIC_LIGHT,
		FIRE_HYDRANT, STOP_SIGN, PARKING_METER, BENCH, BIRD, CAT, DOG, HORSE, SHEEP, COW,
		ELEPHANT, BEAR, ZEBRA, GIRAFFE, BACKPACK, UMBRELLA, HANDBAG, TIE, SUITCASE, FRISBEE,
		SKIS, SNOWBOARD, SPORTS_BALL, KITE, BASEBALL_BAT, BASEBALL_GLOVE, SKATEBOARD, SURFBOARD, TENNIS_RACKET, BOTTLE,
		WINE_GLASS, CUP, FORK, KNIFE, SPOON, BOWL, BANANA, APPLE, SANDWICH, ORANGE,
		BROCCOLI, CARROT, HOT_DOG, PIZZA, DONUT, CAKE, CHAIR, SOFA, POTTEDPLANT, BED,
		DININGTABLE, TOILET, TVMONITOR, LAPTOP, MOUSE, REMOTE, KEYBOARD, CELL_PHONE, MICROWAVE, OVEN,
		TOASTER, SINK, REFRIGERATOR, BOOK, CLOCK, VASE, SCISSORS, TEDDY_BEAR, HAIR_DRIER, TOOTHBRUSH,
	};
}

class Yolo2DetectorNode
{
	ros::Subscriber subscriber_image_raw_;
	ros::Subscriber subscriber_yolo_config_;
	ros::Publisher publisher_car_objects_;
	ros::Publisher publisher_person_objects_;
	ros::NodeHandle node_handle_;

	darknet::Yolo2Detector yolo_detector_;

	image darknet_image = {};

	float score_threshold_;
	float nms_threshold_;
	double image_ratio_;//resdize ratio used to fit input image to network input size
	uint32_t image_top_bottom_border_;//black strips added to the input image to maintain aspect ratio while resizing it to fit the network input size
	uint32_t image_left_right_border_;

	void convert_rect_to_image_obj(std::vector< RectClassScore<float> >& in_objects, autoware_msgs::image_obj& out_message, std::string in_class)
	{
		for (unsigned int i = 0; i < in_objects.size(); ++i)
		{
			if ( (in_objects[i].score > score_threshold_)
				&& (	(in_class == "car"
							&& (in_objects[i].class_type == Yolo2::CAR
								|| in_objects[i].class_type == Yolo2::BUS
								|| in_objects[i].class_type == Yolo2::TRUCK
								|| in_objects[i].class_type == Yolo2::MOTORBIKE
								)
						)
					|| (in_class == "person"
							&& (in_objects[i].class_type == Yolo2::PERSON
								|| in_objects[i].class_type == Yolo2::BICYCLE
								|| in_objects[i].class_type == Yolo2::DOG
								|| in_objects[i].class_type == Yolo2::CAT
								|| in_objects[i].class_type == Yolo2::HORSE
								)
						)
					)
				)//check if the score is larger than minimum required
			{
				autoware_msgs::image_rect rect;

				rect.x = (in_objects[i].x * darknet_image.w /image_ratio_) - image_left_right_border_/image_ratio_;
				rect.y = (in_objects[i].y * darknet_image.h /image_ratio_) - image_top_bottom_border_/image_ratio_;
				rect.width = in_objects[i].w * darknet_image.w/image_ratio_;
				rect.height = in_objects[i].h * darknet_image.h/image_ratio_;
				if (in_objects[i].x < 0)
					rect.x = 0;
				if (in_objects[i].y < 0)
					rect.y = 0;
				if (in_objects[i].w < 0)
					rect.width = 0;
				if (in_objects[i].h < 0)
					rect.height = 0;

				rect.score = in_objects[i].score;

				//std::cout << "x "<< rect.x<< " y " << rect.y << " w "<< rect.width << " h "<< rect.height<< " s " << rect.score << " c " << in_objects[i].class_type << std::endl;

				out_message.obj.push_back(rect);

			}
		}
	}

	void rgbgr_image(image& im)
	{
		int i;
		for(i = 0; i < im.w*im.h; ++i)
		{
			float swap = im.data[i];
			im.data[i] = im.data[i+im.w*im.h*2];
			im.data[i+im.w*im.h*2] = swap;
		}
	}

	image convert_ipl_to_image(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, "bgr8");//toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
		cv::Mat mat_image = cv_image->image;

		uint32_t network_input_width = yolo_detector_.get_network_width();
		uint32_t network_input_height = yolo_detector_.get_network_height();

		uint32_t image_height = msg->height,
						image_width = msg->width;

		IplImage ipl_image;
		cv::Mat final_mat;

		//ROS_INFO("Before Network (%d,%d), Image (%d,%d)", network_input_width, network_input_height, image_width, image_height);
		if (network_input_width!=image_width
				|| network_input_height != image_height)
		{
			//final_mat = cv::Mat(network_input_width, network_input_height, CV_8UC3, cv::Scalar(0,0,0));
			image_ratio_ = (double ) network_input_width /  (double)mat_image.cols;
			//std::cout << "Ratio:" << image_ratio_ << std::endl;

			cv::resize(mat_image, final_mat, cv::Size(), image_ratio_, image_ratio_);
			image_top_bottom_border_ = abs(final_mat.rows-network_input_height)/2;
			image_left_right_border_ = abs(final_mat.cols-network_input_width)/2;
			cv::copyMakeBorder(final_mat, final_mat,
								image_top_bottom_border_, image_top_bottom_border_,
								image_left_right_border_, image_left_right_border_,
								cv::BORDER_CONSTANT, cv::Scalar(0,0,0));

			/*
			 //CROP CENTER
			 * uint32_t crop_x, crop_y;
			crop_x = (image_width-network_input_width)/2;
			crop_y = (image_height-network_input_height)/2;
			cv::Rect center_crop(crop_x, crop_y, network_input_width, network_input_height);
			std::cout << mat_image.cols << ", " << mat_image.rows << std::endl;
			cv::Mat cropped_mat = mat_image(center_crop);
			cropped_mat.copyTo(final_mat);
			*/

			//VILE RESIZE
			//cv::resize(mat_image, final_mat, cv::Size(network_input_width, network_input_height));
		}
		else
			final_mat = mat_image;

		//ROS_INFO("After Network (%d,%d), Image (%d,%d)", network_input_width, network_input_height, final_mat.cols, final_mat.rows);

		ipl_image = final_mat;

		unsigned char *data = (unsigned char *)ipl_image.imageData;
		int h = ipl_image.height;
		int w = ipl_image.width;
		int c = ipl_image.nChannels;
		int step = ipl_image.widthStep;
		int i, j, k;

		image darknet_image = make_image(w, h, c);

		for(i = 0; i < h; ++i){
			for(k= 0; k < c; ++k){
				for(j = 0; j < w; ++j){
					darknet_image.data[k*w*h + i*w + j] = data[i*step + j*c + k]/255.;
				}
			}
		}
		rgbgr_image(darknet_image);
		return darknet_image;
	}

	void image_callback(const sensor_msgs::ImageConstPtr& in_image_message)
	{
		std::vector< RectClassScore<float> > detections;
		//darknet_image_ = yolo_detector_.convert_image(in_image_message);

		darknet_image = convert_ipl_to_image(in_image_message);

		detections = yolo_detector_.detect(darknet_image);

		//ROS_INFO("Detections: %ud", (unsigned int)detections.size());

		//Prepare Output message
		autoware_msgs::image_obj output_car_message;
		autoware_msgs::image_obj output_person_message;
		output_car_message.header = in_image_message->header;
		output_car_message.type = "car";

		output_person_message.header = in_image_message->header;
		output_person_message.type = "person";

		convert_rect_to_image_obj(detections, output_car_message, "car");
		convert_rect_to_image_obj(detections, output_person_message, "person");

		publisher_car_objects_.publish(output_car_message);
		publisher_person_objects_.publish(output_person_message);

		free(darknet_image.data);
	}

	void config_cb(const autoware_msgs::ConfigSsd::ConstPtr& param)
	{
		score_threshold_ 	= param->score_threshold;
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

		std::string network_definition_file;
		std::string pretrained_model_file;
		if (private_node_handle.getParam("network_definition_file", network_definition_file))
		{
			ROS_INFO("Network Definition File (Config): %s", network_definition_file.c_str());
		}
		else
		{
			ROS_INFO("No Network Definition File was received. Finishing execution.");
			return;
		}
		if (private_node_handle.getParam("pretrained_model_file", pretrained_model_file))
		{
			ROS_INFO("Pretrained Model File (Weights): %s", pretrained_model_file.c_str());
		}
		else
		{
			ROS_INFO("No Pretrained Model File was received. Finishing execution.");
			return;
		}

		if (private_node_handle.getParam("score_threshold", score_threshold_))
		{
			ROS_INFO("Score Threshold: %f", score_threshold_);
		}
		if (private_node_handle.getParam("nms_threshold", nms_threshold_))
		{
			ROS_INFO("NMS Threshold: %f", nms_threshold_);
		}

		ROS_INFO("Initializing Yolo2 on Darknet...");
		yolo_detector_.load(network_definition_file, pretrained_model_file, score_threshold_, nms_threshold_);
		ROS_INFO("Initialization complete.");

		publisher_car_objects_ = node_handle_.advertise<autoware_msgs::image_obj>("/obj_car/image_obj", 1);
		publisher_person_objects_ = node_handle_.advertise<autoware_msgs::image_obj>("/obj_person/image_obj", 1);

		ROS_INFO("Subscribing to... %s", image_raw_topic_str.c_str());
		subscriber_image_raw_ = node_handle_.subscribe(image_raw_topic_str, 1, &Yolo2DetectorNode::image_callback, this);

		std::string config_topic("/config");
		config_topic += "/yolo2";
		subscriber_yolo_config_ = node_handle_.subscribe(config_topic, 1, &Yolo2DetectorNode::config_cb, this);

		ros::spin();
		ROS_INFO("END Yolo2");

	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ssd_unc");

	Yolo2DetectorNode app;

	app.Run();

	return 0;
}

