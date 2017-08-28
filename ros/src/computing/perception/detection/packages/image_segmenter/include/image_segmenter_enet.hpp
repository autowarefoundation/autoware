/*
 * image_segmenter_enet.hpp
 *
 *  Created on: Aug 23, 2017
 *      Author: ne0
 */

#ifndef ENET_IMAGE_SEGMENTER_HPP_
#define ENET_IMAGE_SEGMENTER_HPP_

#include <algorithm>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <caffe/caffe.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class ENetSegmenter
{
public:
	ENetSegmenter(const std::string& in_model_file, const std::string& in_trained_file, const std::string& in_lookuptable_file);

	void Predict(const cv::Mat& in_img, cv::Mat& out_segmented);

private:
	void SetMean(const std::string& in_mean_file);

	void WrapInputLayer(std::vector<cv::Mat>* in_input_channels);

	void Preprocess(const cv::Mat& img, std::vector<cv::Mat>* in_input_channels);

	cv::Mat Visualization(cv::Mat in_prediction_map, std::string in_lookuptable_file);

private:
	std::shared_ptr<caffe::Net<float> > 	net_;
	cv::Size 					input_geometry_;
	int 						num_channels_;
	std::string 				lookuptable_file_;
	cv::Scalar 					pixel_mean_;

};

#endif /* ENET_IMAGE_SEGMENTER_HPP_ */
