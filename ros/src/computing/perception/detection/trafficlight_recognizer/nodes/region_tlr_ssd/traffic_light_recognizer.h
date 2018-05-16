#ifndef TRAFFIC_LIGHT_RECOGNIZER_H
#define TRAFFIC_LIGHT_RECOGNIZER_H

#include <vector>
#include <string>

#include <caffe/caffe.hpp>
#include <opencv2/opencv.hpp>

#include "Context.h"

class TrafficLightRecognizer {
 public:
  TrafficLightRecognizer();
  ~TrafficLightRecognizer();
  void Init(const std::string& network_definition_file_name,
            const std::string& pretrained_model_file_name,
            const bool use_gpu,
            const unsigned int gpu_id);

  LightState RecognizeLightState(const cv::Mat& image);

 private:
  void WrapInputLayer(std::vector<cv::Mat>* input_channels);
  void Preprocess(const cv::Mat& image, std::vector<cv::Mat>* input_channels);

  boost::shared_ptr<caffe::Net<float> > network_;
  int num_channels_;
  cv::Size input_geometry_;
  cv::Scalar kPixelMean_;
};

#endif  // TRAFFIC_LIGHT_RECOGNIZER_H
