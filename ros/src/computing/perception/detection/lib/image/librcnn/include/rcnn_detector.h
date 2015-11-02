#ifndef RCNN_DETECTOR_H_
#define RCNN_DETECTOR_H_

#include <opencv2/core/core.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <stdio.h>
#include <vector>

#include <sstream>
#include <string>
#include <algorithm>

#include "rect_class_score.h"

#include "caffe/caffe.hpp"


namespace Rcnn
{
	enum RcnnDetectorClasses
	{
		BACKGROUND,
		PLANE, BICYCLE, BIRD, BOAT,
		BOTTLE, BUS, CAR, CAT, CHAIR,
		COW, TABLE, DOG, HORSE,
		MOTORBIKE, PERSON, PLANT,
		SHEEP, SOFA, TRAIN, TV, NUM_CLASSES
	};
}

class RcnnDetector
{
	int counter_;
	//pointer to Caffe neural net object
	caffe::Net<float>* 			net_;
	//pointer to BlobProto to store the Generated Proposals
	caffe::BlobProto* 			rois_blob_proto_;
	//BGR color pixel mean to subtract during pre processing
	cv::Scalar 					pixel_mean_;
	//number of times to slices the image to generate proposals
	unsigned int 				image_slices_;
	//sets if the network is loaded and ready
	/**
	* @brief Generate Object proposals to feed the ConvNet
	*
	* The proposals' size is decided based on the image dimensions,
	* slices sets how many regions horizontally and vertically will be
	* created. Increasing this value, might improve detection but it
	* will impact performance.
	*/
	std::vector< cv::Scalar > GenerateProposals(unsigned int width,
												unsigned int height,
												unsigned int slices = 16,
												float in_box_overlap = 0.7);

	/**
	* @brief Pre process the image before feeding it to the ConvNet
	*
	* Subtracts the color pixel mean to the image, use SetPixelMean to change the default value
	*/
	void PreProcessImage(cv::Mat& in_image, cv::Mat& out_image);

	/**
	* @brief Converts an OpenCV Mat to Caffe BlobProto
	*
	* Fills a BlobProto out_blob object from an OpenCV Mat, so it can be fed into the network
	*/
	void ConvertImageToBlob(cv::Mat& in_image, caffe::BlobProto& out_blob);

	/**
	* @brief Converts a vector of scalars to Caffe BlobProto
	*
	* Fills a BlobProto out_blob object from a vector of scalars, so it can be fed into the network
	*/
	void ConvertRoisToBlob(std::vector< cv::Scalar  >& in_proposals, caffe::BlobProto& out_blob);

	/**
	* @brief Generates Class Scored Rectangles of the ROIS input proposals
	*
	*/
	std::vector< RectClassScore<float> > GetRectClassesScored(std::vector< cv::Scalar >& in_proposals,
								const std::vector<unsigned int>& in_classes,
								float in_score_threshold,
								const float* in_boxes,
								const float* in_probabilities,
								unsigned int width,
								unsigned int height);

	/**
	 * @brief Apply non maximum suppresion to the detections
	 */
	std::vector< RectClassScore<float> > ApplyNonMaximumSuppresion(std::vector< RectClassScore<float> > in_source, float in_nms_threshold);

	bool CheckClasses(unsigned int in_class, const std::vector<unsigned int>& in_classes);

public:
	/**
	* @brief FastRCNN Constructor, creates the Caffe network,  loads the definition(.proto) and pretrained model(.caffemodel), also sets GPU mode and  Device ID
	*/
	RcnnDetector(std::string& network_definition_file,
					std::string& pre_trained_model_file,
					bool use_gpu = true,
					unsigned int gpu_id = 0);

	/**
	* @brief Starts detection of the 21 classes on an image, using the parameters
	*/
	std::vector< RectClassScore<float> > Detect(cv::Mat& in_image,
											const std::vector<unsigned int>& in_classes,
											float in_threshold = 0.7,
											unsigned int in_slices=16,
											float in_box_overlap = 0.7,
											float in_nms_threshold = 0.3);

	void Sort(const std::vector<float> in_scores, std::vector<unsigned int>& in_out_indices);

	/**
	* @brief Changes the default pixel mean used by PreProcessImage
	*/
	inline void SetPixelMean(cv::Scalar new_pixel_mean) { pixel_mean_ = new_pixel_mean; }
};



#endif /* RCNN_DETECTOR_H_ */
