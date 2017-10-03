/*
 * cnn_lidar_detector.hpp
 *
 *  Created on: Jun 20, 2017
 *      Author: ne0
 */

#ifndef CNN_LIDAR_DETECTOR_HPP_
#define CNN_LIDAR_DETECTOR_HPP_

#include <string>
#include <vector>
#include <iostream>

#include <caffe/caffe.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>


class CnnLidarDetector
{
public:
	CnnLidarDetector(const std::string& in_network_definition_file,
			const std::string& in_pre_trained_model_file,
			bool in_use_gpu,
			unsigned int in_gpu_id,
			float in_score_threshold);

	void Detect(const cv::Mat& in_depth_image,
	            const cv::Mat& in_height_image,
	            cv::Mat& out_objectness_image,
	            jsk_recognition_msgs::BoundingBoxArray& out_boxes);

private:
	boost::shared_ptr<caffe::Net<float> > net_;
	cv::Size 	input_geometry_;
	int 		num_channels_;
	float		score_threshold_;

	void WrapInputLayer(std::vector<cv::Mat>* in_out_channels);//Point mat vector to the network input layer
	void PreProcess(const cv::Mat& in_depth_image,
	                const cv::Mat& in_height_image,
	                std::vector<cv::Mat>* in_out_channels);//match input images to input layer geometry
	void GetNetworkResults(cv::Mat& out_objectness_image,
	                       jsk_recognition_msgs::BoundingBoxArray& out_boxes);//get objectness image and bounding boxes
};

#endif /* CNN_LIDAR_DETECTOR_HPP_ */
