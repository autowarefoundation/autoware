/*
 * cnn_lidar_detector.hpp
 *
 *  Created on: Jun 20, 2017
 *      Author: ne0
 */

#ifndef CNN_LIDAR_DETECTOR_HPP_
#define CNN_LIDAR_DETECTOR_HPP_

#include <caffe/caffe.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>


class CnnLidarDetector
{
public:
	CnnLidarDetector(const std::string& in_network_definition_file,
			const std::string& in_pre_trained_model_file,
			bool in_use_gpu,
			unsigned int in_gpu_id);

	void Detect(const cv::Mat& in_depth_image, const cv::Mat& in_height_image, cv::Mat& out_objectness_image);



private:
	boost::shared_ptr<caffe::Net<float> > net_;
	cv::Size input_geometry_;
	int num_channels_;

	void WrapInputLayer(std::vector<cv::Mat>* in_out_channels);//Point mat vector to the network input layer
	void PreProcess(const cv::Mat& in_depth_image, const cv::Mat& in_height_image, std::vector<cv::Mat>* in_out_channels);//match input images to input layer geometry
};



#endif /* CNN_LIDAR_DETECTOR_HPP_ */
