#ifndef MXNETTRAFFIC_LIGHT_RECOGNIZER_H
#define MXNETTRAFFIC_LIGHT_RECOGNIZER_H

#include <vector>
#include <string>

#include "mxnet-cpp/MxNetCpp.h"
#include <mxnet/c_predict_api.h>
#include <opencv2/opencv.hpp>

#include "Context.h"

class MxNetTrafficLightRecognizer {
	enum NetworkResults {Green, Yellow, Red, None};
	int width_;
	int height_;
	int num_channels_;
	PredictorHandle prediction_handle_;

	void PreProcessImage(const cv::Mat& in_image,
	                     mx_float* out_image_data,
	                     const int in_channels,
	                     const cv::Size in_resize_size);
public:
	MxNetTrafficLightRecognizer();
	~MxNetTrafficLightRecognizer();
	void Init(const char* in_network_definition_buffer,
	          const char* in_pretrained_model_buffer,
	          int in_pretrained_model_length,
	          const bool in_use_gpu,
	          const unsigned int in_gpu_id);

	LightState RecognizeLightState(const cv::Mat& in_image, double in_score_threshold);



};

#endif  // TRAFFIC_LIGHT_RECOGNIZER_H
