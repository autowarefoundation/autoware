#ifndef SSD_DETECTOR_H_
#define SSD_DETECTOR_H_

#include <caffe/caffe.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <iomanip>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rect_class_score.h"

namespace Ssd
{
	enum SsdDetectorClasses
	{
		BACKGROUND,
		PLANE, BICYCLE, BIRD, BOAT,
		BOTTLE, BUS, CAR, CAT, CHAIR,
		COW, TABLE, DOG, HORSE,
		MOTORBIKE, PERSON, PLANT,
		SHEEP, SOFA, TRAIN, TV, NUM_CLASSES
	};
}

class SsdDetector
{
public:
	SsdDetector(const std::string& in_network_definition_file, const std::string& in_pre_trained_model_file, const cv::Scalar& in_mean_value, bool in_use_gpu, unsigned int in_gpu_id);

	std::vector <  RectClassScore<float>  > Detect(const cv::Mat& img);

private:
	void SetMean(const cv::Scalar& in_mean_value);
	void WrapInputLayer(std::vector<cv::Mat>* input_channels);
	void Preprocess(const cv::Mat& img, std::vector<cv::Mat>* input_channels);

private:
	boost::shared_ptr<caffe::Net<float> > net_;
	cv::Size input_geometry_;
	int num_channels_;
	cv::Scalar mean_;
};

#endif //SSD_DETECTOR_H
