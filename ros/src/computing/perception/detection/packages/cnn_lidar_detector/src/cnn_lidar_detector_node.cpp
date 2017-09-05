/*
 *  Copyright (c) 2017, Nagoya University
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
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

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

#include <string>
#include <cmath>

#include "cnn_lidar_detector.hpp"

#include "MatlabIO.hpp"
#include "MatlabIOContainer.hpp"

class RosLidarDetectorApp
{
	ros::Subscriber subscriber_image_raw_;
	ros::Publisher 	publisher_pointcloud_class_;
	ros::Publisher 	publisher_boxes_;
	ros::Publisher 	publisher_depth_image_;
	ros::Publisher 	publisher_height_image_;
	ros::Publisher 	publisher_intensity_image_;
	ros::Publisher 	publisher_objectness_image_;
	ros::NodeHandle node_handle_;

	//Caffe based Object Detection ConvNet
	CnnLidarDetector* lidar_detector_;

	//Sets whether or not use GPU acceleration
	bool 			use_gpu_;
	int 			gpu_device_id_;
	const double 	pi_ = 3.14159265;

	float 			horizontal_res_;
	float 			vertical_res_;
	unsigned int 	image_width_;
	unsigned int 	image_height_;
	double			score_threshold_;

	cv::Mat			projection_mean_depth_;//single channel float image subtrahend for depth projection
	cv::Mat			projection_mean_height_;//single channel float image subtrahend for height projection

	cv::Mat			projection_std_dev_depth_;//double divisor for depth channel
	cv::Mat			projection_std_dev_height_;//double divisor for height channel

	//Project 3D PointCloud to Image using a Cylindrical representation
	void project_pointcloud_to_image(
			pcl::PointCloud<pcl::PointXYZI>::Ptr in_point_cloud, //pointcloud to be projected to image
			cv::Mat& out_depth_image, //resulting depth projection image
			cv::Mat& out_height_image, //resulting height projection image
			cv::Mat& out_intensity_image, //resulting intensity image
			std::vector<cv::Point2d>& out_points2d, //resulting 2d points
			std::vector<pcl::PointXYZI>& out_points_3d//corresponding 3d points
			)
	{
		for(unsigned int i=0; i<in_point_cloud->points.size(); i++)
		{
			pcl::PointXYZI point = in_point_cloud->points[i];
			if (point.x > 0. && point.x <70.)
			{
				float theta = atan2(point.y, point.x);
				float length = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
				float angle = asin(point.z/length);
				float depth = sqrt(point.x*point.x + point.y*point.y);
				unsigned int image_x = floor((theta/horizontal_res_) + horizontal_res_/2);//250.5 = (501/2)
				unsigned int image_y = floor((angle/vertical_res_) + vertical_res_/2);//71.0 = 90*0.78

				if ( (image_x >= 0) && (image_x < image_width_) &&
					 (image_y >= 0) && (image_y < image_height_)
					) //check the projection is inside the image
				{
					out_depth_image.at<float>(image_y, image_x) = depth;
					out_height_image.at<float>(image_y, image_x) = point.z;
					out_intensity_image.at<float>(image_y, image_x) = point.intensity;
					//store correspondence between cloud and image coords
					out_points2d[i] = cv::Point2d(image_x, image_y);
					out_points_3d[i] = point;
				}
			}
		}
	}

	//Normalizes in_image and creates a ColorMap representation of it in out_image to be published
	void post_process_image(cv::Mat& in_image, cv::Mat& out_image)
	{
		cv::flip(in_image, in_image,-1);

		cv::Mat grayscale_cloud;
		double min, max;
		cv::minMaxIdx(in_image, &min, &max);
		in_image.convertTo(grayscale_cloud,CV_8UC1, 255 / (max-min), -min);

		//Apply colormap:
		applyColorMap(grayscale_cloud, out_image, cv::COLORMAP_JET);
	}

	void publish_image(ros::Publisher& in_publisher, cv::Mat& in_image, pcl::PCLHeader in_header)
	{
		sensor_msgs::ImagePtr pub_image_msg;
		pub_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", in_image).toImageMsg();
		pub_image_msg->header.seq = in_header.seq;
		pub_image_msg->header.frame_id = in_header.frame_id;
		pub_image_msg->header.stamp = ros::Time(in_header.stamp);
		in_publisher.publish(pub_image_msg);
	}

	void subtract_image(cv::Mat& in_out_minuend_image, float in_subtrahend)
	{
		for(unsigned int y = 0; y < in_out_minuend_image.rows; y++)
		{
			for(unsigned int x = 0; x < in_out_minuend_image.cols; x++)
			{
				in_out_minuend_image.at<float>(y, x) -= in_subtrahend;
			}
		}
	}

	void divide_image(cv::Mat& in_out_dividend, float in_divisor)
	{
		if(fabs(in_divisor) < 0.00001)
		{
			ROS_ERROR("Not performing division by small number, divisor is too small (%f). Check Mat file.", in_divisor);
			return;
		}
		for(unsigned int y = 0; y < in_out_dividend.rows; y++)
		{
			for(unsigned int x = 0; x < in_out_dividend.cols; x++)
			{
				in_out_dividend.at<float>(y, x) /= in_divisor;
			}
		}
	}

	void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr current_sensor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
		pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);
		//get cloud and project to image

		cv::Mat projected_cloud_depth(image_height_, image_width_, CV_32F, 0.);//depth image
		cv::Mat projected_cloud_height(image_height_, image_width_, CV_32F, 0.);//height image
		cv::Mat projected_cloud_intensity(image_height_, image_width_, CV_32F, 0.);//intensity image
		cv::Mat resulting_objectness;//objectness image

		std::vector<cv::Point2d> image_points;
		std::vector<pcl::PointXYZI> cloud_points;

		//store the 3d and 2d points relation for faster backprojection
		image_points.resize(current_sensor_cloud_ptr->points.size());
		cloud_points.resize(current_sensor_cloud_ptr->points.size());

		project_pointcloud_to_image(current_sensor_cloud_ptr,
								projected_cloud_depth,
								projected_cloud_height,
								projected_cloud_intensity,
								image_points,
								cloud_points);

		// subtract mean
		subtract_image(projected_cloud_depth, projection_mean_depth_.at<float>(0));
		subtract_image(projected_cloud_height, projection_mean_height_.at<float>(0));
		// divide std dev
		divide_image(projected_cloud_depth, projection_std_dev_depth_.at<float>(0));
		divide_image(projected_cloud_height, projection_std_dev_height_.at<float>(0));

		//use image for CNN forward

		lidar_detector_->Detect(projected_cloud_depth,
								projected_cloud_height,
								resulting_objectness);

		cv::Mat ros_depth_image, ros_height_image, ros_intensity_image; //mats for publishing

		//grayscale to colormap  representation
		post_process_image(projected_cloud_depth, ros_depth_image);
		post_process_image(projected_cloud_height, ros_height_image);
		post_process_image(projected_cloud_intensity, ros_intensity_image);

		publish_image(publisher_depth_image_, ros_depth_image, current_sensor_cloud_ptr->header);
		publish_image(publisher_height_image_, ros_height_image, current_sensor_cloud_ptr->header);
		publish_image(publisher_intensity_image_, ros_intensity_image, current_sensor_cloud_ptr->header);
		publish_image(publisher_objectness_image_, resulting_objectness, current_sensor_cloud_ptr->header);
		
	}//end cloud_callback

	bool LoadPreProcessingParamsFromMat(std::string in_matfile_path)
	{
		MatlabIO mat_io;
		if (!mat_io.open(in_matfile_path, "r"))//try to open specified file
			return false;

		std::vector<MatlabIOContainer> variables;
		variables = mat_io.read();

		mat_io.close();

		mat_io.whos(variables);

		std::size_t count = 0;
		for (unsigned int i = 0; i < variables.size(); i++)
		{
			if (variables[i].name().compare("mean_depth") == 0)
			{
				projection_mean_depth_ = variables[i].data<cv::Mat>();
				ROS_INFO("Successfully loaded Depth Channel Mean");
				count++;
			}
			if (variables[i].name().compare("mean_height") == 0)
			{
				projection_mean_height_ = variables[i].data<cv::Mat>();
				ROS_INFO("Successfully loaded Height Channel Mean");
				count++;
			}
			if (variables[i].name().compare("std_depth") == 0)
			{
				projection_std_dev_depth_ = variables[i].data<cv::Mat>();
				ROS_INFO("Successfully loaded Depth Channel Standard Deviation ");
				count++;
			}
			if (variables[i].name().compare("std_height") == 0)
			{
				projection_std_dev_height_ = variables[i].data<cv::Mat>();
				ROS_INFO("Successfully loaded Height Channel Standard Deviation");
				count++;
			}
		}
		if(count != 4) //4 expected vars to be contained in the Mat file
			return false;
		return true;
	}

public:
	void Run()
	{
		//ROS STUFF
		ros::NodeHandle private_node_handle("~");//to receive args

		//RECEIVE TOPIC NAME
		std::string cloud_topic_str;
		if (private_node_handle.getParam("points_node", cloud_topic_str))
		{
			ROS_INFO("Setting input cloud node to %s", cloud_topic_str.c_str());
		}
		else
		{
			ROS_INFO("No cloud node received, defaulting to /points_raw, you can use _points_raw_node:=YOUR_TOPIC");
			cloud_topic_str = "/points_raw";
		}

		std::string preprocess_mat_path;
		private_node_handle.param<std::string>("preprocess_mat_path", preprocess_mat_path, ros::package::getPath("cnn_lidar_detector")+"/data/preprocess.mat");
		ROS_INFO("preprocess_mat_path: %s", preprocess_mat_path.c_str());

		//Load Std Dev and Mean from Mat File
		if( !LoadPreProcessingParamsFromMat(preprocess_mat_path) )
			ROS_ERROR("cnn_lidar_detector. Unable to Load all the pre processing parameters from MAT file. Preprocessing of the image projection will not be executed, leading to an incorrect detection.");

		//RECEIVE CONVNET FILENAMES
		std::string network_definition_file;
		std::string pretrained_model_file;
		private_node_handle.param<std::string>("network_definition_file",
							network_definition_file,
							ros::package::getPath("cnn_lidar_detector")+"/data/lidar_detector.prototxt");
		ROS_INFO("preprocess_mat_path: %s", preprocess_mat_path.c_str());

		private_node_handle.param<std::string>("pretrained_model_file",
							pretrained_model_file,
							ros::package::getPath("cnn_lidar_detector")+"/data/lidar_detector.caffemodel");
		ROS_INFO("preprocess_mat_path: %s", preprocess_mat_path.c_str());

		private_node_handle.param("use_gpu", use_gpu_, true);					ROS_INFO("use_gpu: %d", use_gpu_);
		private_node_handle.param("gpu_device_id", gpu_device_id_, 0);			ROS_INFO("gpu_device_id: %d", gpu_device_id_);
		private_node_handle.param("score_threshold", score_threshold_, 0.9);	ROS_INFO("score_threshold: %f", score_threshold_);

		lidar_detector_ = new CnnLidarDetector(network_definition_file, pretrained_model_file, use_gpu_, gpu_device_id_, score_threshold_);

		if (NULL == lidar_detector_)
		{
			ROS_INFO("Error while creating LidarDetector Object");
			return;
		}
		ROS_INFO("CNN Lidar Detector initialized.");

		publisher_pointcloud_class_ = node_handle_.advertise<sensor_msgs::PointCloud2>("/points_class",1);
		publisher_boxes_ = node_handle_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bounding_boxes",1);
		publisher_depth_image_= node_handle_.advertise<sensor_msgs::Image>("/image_depth",1);
		publisher_height_image_= node_handle_.advertise<sensor_msgs::Image>("/image_height",1);
		publisher_intensity_image_= node_handle_.advertise<sensor_msgs::Image>("/image_intensity",1);
		publisher_objectness_image_= node_handle_.advertise<sensor_msgs::Image>("/image_objectness",1);

		ROS_INFO("Subscribing to... %s", cloud_topic_str.c_str());
		subscriber_image_raw_ = node_handle_.subscribe(cloud_topic_str, 1, &RosLidarDetectorApp::cloud_callback, this);

		ros::spin();
		ROS_INFO("END CNN Lidar Detector");

	}

	~RosLidarDetectorApp()
	{
		if (NULL != lidar_detector_)
			delete lidar_detector_;
	}

	RosLidarDetectorApp()
	{
		lidar_detector_ 	= NULL;
		use_gpu_ 		= true;

		gpu_device_id_ 	= 0;
		score_threshold_ = 0.5;

		//TODO: parametrize these to enable different lidar models to be projected.
		//Model   |   Horizontal   |   Vertical   | FOV(Vertical)    degrees / rads
		//----------------------------------------------------------
		//HDL-64  |0.08-0.35(0.32) |     0.4      |  -24.9 <=x<=2.0   (26.9  / 0.47)
		//HDL-32  |     0.1-0.4    |     1.33     |  -30.67<=x<=10.67 (41.33 / 0.72)
		//VLP-16  |     0.1-0.4    |     2.0      |  -15.0<=x<=15.0   (30    / 0.52)
		//VLP-16HD|     0.1-0.4    |     1.33     |  -10.0<=x<=10.0   (20    / 0.35)

		horizontal_res_ = 0.25 * pi_ /180.;//Angular Resolution (Horizontal/Azimuth)
		vertical_res_ = 2.0*pi_/180.;//Vertical Resolution HDL
		image_width_ = pi_ / horizontal_res_;//501
		image_height_ = 0.52 / vertical_res_;//90

	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cnn_lidar_detector");

	RosLidarDetectorApp app;

	app.Run();

	return 0;
}
