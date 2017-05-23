#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <runtime_manager/ConfigSsd.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_tracker/image_obj.h>

#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <string>
#include <vector>

#include <math.h>

#include <rect_class_score.h>

#include "darknet/yolo2.h"

namespace Yolo2
{
	enum YoloDetectorClasses
	{
		BACKGROUND,
		PLANE, BICYCLE, BIRD, BOAT,
		BOTTLE, BUS, CAR, CAT, CHAIR,
		COW, TABLE, DOG, HORSE,
		MOTORBIKE, PERSON, PLANT,
		SHEEP, SOFA, TRAIN, TV, NUM_CLASSES
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
	double image_ratio_;

	void convert_rect_to_image_obj(std::vector< RectClassScore<float> >& in_objects, cv_tracker::image_obj& out_message, std::string in_class)
	{
		for (unsigned int i = 0; i < in_objects.size(); ++i)
		{
			if ( (in_objects[i].score > score_threshold_)
				/*&& (	(in_class == "car" && (in_objects[i].class_type == Yolo2::CAR || in_objects[i].class_type == Yolo2::BUS))
						|| (in_class == "person" && (in_objects[i].class_type == Yolo2::PERSON || in_objects[i].class_type == Yolo2::BICYCLE))
					)*/

				)//check if the score is larger than minimum required
			{
				//std::cout << in_objects[i].toString() << std::endl;
				cv_tracker::image_rect rect;

				rect.x = in_objects[i].x * darknet_image.w;
				rect.y = in_objects[i].y * darknet_image.h;
				rect.width = in_objects[i].w * darknet_image.w;
				rect.height = in_objects[i].h * darknet_image.h;
				if (in_objects[i].x < 0)
					rect.x = 0;
				if (in_objects[i].y < 0)
					rect.y = 0;
				if (in_objects[i].w < 0)
					rect.width = 0;
				if (in_objects[i].h < 0)
					rect.height = 0;

				rect.score = in_objects[i].score;

				std::cout << "x"<< rect.x<< "y" << rect.y << "w"<< rect.width << "h"<< rect.height<< "s" << rect.score << std::endl;

				out_message.obj.push_back(rect);

			}
		}
	}

	image convert_ipl_to_image(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, "bgr8");//toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
		cv::Mat mat_image = cv_image->image;

		uint32_t network_input_width = yolo_detector_.get_network_width();
		uint32_t network_input_height = yolo_detector_.get_network_height();

		uint32_t height = msg->height,
						width = msg->width;

		IplImage ipl_image;
		cv::Mat final_mat;

		ROS_INFO("Before Network (%d,%d), Image (%d,%d)", network_input_width, network_input_height, width, height);
		if (network_input_width!=width
				|| network_input_height != height)
		{
			//final_mat = cv::Mat(network_input_width, network_input_height, CV_8UC3, cv::Scalar(0,0,0));
			image_ratio_ = (double ) network_input_width /  (double)mat_image.cols;
			std::cout << "Ratio:" << image_ratio_ << std::endl;

			cv::resize(mat_image, final_mat, cv::Size(), image_ratio_, image_ratio_);
			uint32_t top_bottom_border = abs(final_mat.rows-network_input_height)/2;
			uint32_t left_right_border = abs(final_mat.cols-network_input_width)/2;
			cv::copyMakeBorder(final_mat, final_mat, top_bottom_border, top_bottom_border, left_right_border, left_right_border, cv::BORDER_CONSTANT, cv::Scalar(0,0,0));
		}
		else
			final_mat = mat_image;

		ROS_INFO("After Network (%d,%d), Image (%d,%d)", network_input_width, network_input_height, final_mat.cols, final_mat.rows);

		cv::imshow("cropped", final_mat);
		cvWaitKey(10);

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

		return darknet_image;
	}

	void image_callback(const sensor_msgs::ImageConstPtr& in_image_message)
	{
		std::vector< RectClassScore<float> > detections;
		//darknet_image = yolo_detector_.convert_image(in_image_message);

		darknet_image = convert_ipl_to_image(in_image_message);

		detections = yolo_detector_.detect(darknet_image.data);

		//Prepare Output message
		cv_tracker::image_obj output_car_message;
		cv_tracker::image_obj output_person_message;
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

	void config_cb(const runtime_manager::ConfigSsd::ConstPtr& param)
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

		publisher_car_objects_ = node_handle_.advertise<cv_tracker::image_obj>("/obj_car/image_obj", 1);
		publisher_person_objects_ = node_handle_.advertise<cv_tracker::image_obj>("/obj_person/image_obj", 1);

		ROS_INFO("Subscribing to... %s", image_raw_topic_str.c_str());
		subscriber_image_raw_ = node_handle_.subscribe(image_raw_topic_str, 1, &Yolo2DetectorNode::image_callback, this);

		std::string config_topic("/config");
		config_topic += "/ssd";
		subscriber_yolo_config_ = node_handle_.subscribe(config_topic, 1, &Yolo2DetectorNode::config_cb, this);

		ros::spin();
		ROS_INFO("END Ssd");

	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ssd_unc");

	Yolo2DetectorNode app;

	app.Run();

	return 0;
}

/*
namespace
{
	darknet::Yolo2Detector yolo;
	ros::Publisher publisher;
	image im = {};
	float *image_data = nullptr;
	ros::Time timestamp;
	std::mutex mutex;
	std::condition_variable im_condition;

	void imageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
	  im = yolo.convert_image(msg);
	  std::unique_lock<std::mutex> lock(mutex);
	  if (image_data)
		free(image_data);
	  timestamp = msg->header.stamp;
	  image_data = im.data;
	  lock.unlock();
	  im_condition.notify_one();
	}
	}  // namespace

	namespace yolo2
	{
	class Yolo2Nodelet : public nodelet::Nodelet
	{
	 public:
	  virtual void onInit()
	  {
		ros::NodeHandle& node = getPrivateNodeHandle();
		const std::string NET_DATA = ros::package::getPath("yolo2") + "/data/";
		std::string config = NET_DATA + "yolo.cfg", weights = NET_DATA + "yolo.weights";
		double confidence, nms;
		node.param<double>("confidence", confidence, .8);
		node.param<double>("nms", nms, .4);
		yolo.load(config, weights, confidence, nms);

		image_transport::ImageTransport transport = image_transport::ImageTransport(node);
		subscriber = transport.subscribe("image", 1, imageCallback);
		publisher = node.advertise<yolo2::ImageDetections>("detections", 5);

		yolo_thread = new std::thread(run_yolo);
	  }

	  ~Yolo2Nodelet()
	  {
		yolo_thread->join();
		delete yolo_thread;
	  }

	 private:
	  image_transport::Subscriber subscriber;
	  std::thread *yolo_thread;

	  static void run_yolo()
	  {
		while (ros::ok())
		{
		  float *data;
		  ros::Time stamp;
		  {
			std::unique_lock<std::mutex> lock(mutex);
			while (!image_data)
			  im_condition.wait(lock);
			data = image_data;
			image_data = nullptr;
			stamp = timestamp;
		  }
		  boost::shared_ptr<yolo2::ImageDetections> detections(new yolo2::ImageDetections);
		  *detections = yolo.detect(data);
		  detections->header.stamp = stamp;
		  publisher.publish(detections);
		  free(data);
		}
	  }
	};
}  // namespace yolo2
*/
