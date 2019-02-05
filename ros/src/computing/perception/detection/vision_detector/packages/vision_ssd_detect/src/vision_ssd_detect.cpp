/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "vision_ssd_detect.h"


SSDDetector::SSDDetector(const std::string& in_network_definition_file,
		const std::string& in_pre_trained_model_file,
		const cv::Scalar& in_mean_value,
		bool in_use_gpu, unsigned int in_gpu_id)
{
  if (in_use_gpu)
  {
    caffe::Caffe::set_mode(caffe::Caffe::GPU);
    caffe::Caffe::SetDevice(in_gpu_id);
  } else
    caffe::Caffe::set_mode(caffe::Caffe::CPU);

  /* Load the network. */
  net_.reset(new caffe::Net<float>(in_network_definition_file, caffe::TEST));
  net_->CopyTrainedLayersFrom(in_pre_trained_model_file);

  CHECK_EQ(net_->num_inputs(), 1) << "Network should have exactly one input.";
  CHECK_EQ(net_->num_outputs(), 1) << "Network should have exactly one output.";

  caffe::Blob<float> *input_layer = net_->input_blobs()[0];
  num_channels_ = input_layer->channels();
  CHECK(num_channels_ == 3 || num_channels_ == 1)
    << "Input layer should have 1 or 3 channels.";
  input_geometry_ = cv::Size(input_layer->width(), input_layer->height());

  SetMean(in_mean_value);
}

std::vector <  RectClassScore<float>  > SSDDetector::Detect(const cv::Mat& img)
{
  caffe::Blob<float> *input_layer = net_->input_blobs()[0];
  input_layer->Reshape(1, num_channels_, input_geometry_.height,
                       input_geometry_.width);
  /* Forward dimension change to all layers. */
  net_->Reshape();

  std::vector <cv::Mat> input_channels;
  WrapInputLayer(&input_channels);

  Preprocess(img, &input_channels);

  net_->Forward();

  /* Copy the output layer to a std::vector */
  caffe::Blob<float> *result_blob = net_->output_blobs()[0];
  const float *result = result_blob->cpu_data();
  const int num_det = result_blob->height();
  std::vector <RectClassScore<float> > detections;
  for (int k = 0; k < num_det; ++k)
  {
    if (result[0] == -1)
    {
      // Skip invalid detection.
      result += 7;
      continue;
    }
    // Detection format: [image_id (0), label(1), score(2), xmin(3), ymin(4), xmax(5), ymax(6)].
    RectClassScore<float> detection;
    detection.class_type = static_cast<int>(result[1]);
    detection.score = result[2];
    detection.x = result[3] * img.cols;
    detection.y = result[4] * img.rows;
    detection.w = result[5] * img.cols - detection.x;
    detection.h = result[6] * img.rows - detection.y;

    detection.enabled = true;

    detections.push_back(detection);
    result += 7;
  }
  return detections;
}


void SSDDetector::SetMean(const cv::Scalar& in_mean_value)
{
  mean_ = in_mean_value;
}

/* Wrap the input layer of the network in separate cv::Mat objects
 * (one per channel). This way we save one memcpy operation and we
 * don't need to rely on cudaMemcpy2D. The last preprocessing
 * operation will write the separate channels directly to the input
 * layer. */
void SSDDetector::WrapInputLayer(std::vector<cv::Mat>* input_channels)
{
  caffe::Blob<float> *input_layer = net_->input_blobs()[0];

  int width = input_layer->width();
  int height = input_layer->height();
  float *input_data = input_layer->mutable_cpu_data();
  for (int i = 0; i < input_layer->channels(); ++i)
  {
    cv::Mat channel(height, width, CV_32FC1, input_data);
    input_channels->push_back(channel);
    input_data += width * height;
  }
}

void SSDDetector::Preprocess(const cv::Mat& img,
		std::vector<cv::Mat>* input_channels)
{
  /* Convert the input image to the input image format of the network. */
  cv::Mat sample;
  if (img.channels() == 3 && num_channels_ == 1)
    cv::cvtColor(img, sample, cv::COLOR_BGR2GRAY);
  else if (img.channels() == 4 && num_channels_ == 1)
    cv::cvtColor(img, sample, cv::COLOR_BGRA2GRAY);
  else if (img.channels() == 4 && num_channels_ == 3)
    cv::cvtColor(img, sample, cv::COLOR_BGRA2BGR);
  else if (img.channels() == 1 && num_channels_ == 3)
    cv::cvtColor(img, sample, cv::COLOR_GRAY2BGR);
  else
    sample = img;

  cv::Mat sample_resized;
  if (sample.size() != input_geometry_)
    cv::resize(sample, sample_resized, input_geometry_);
  else
    sample_resized = sample;

  cv::Mat sample_float;
  cv::Mat mean_img;
  if (num_channels_ == 3)
  {
    sample_resized.convertTo(sample_float, CV_32FC3);
    mean_img = cv::Mat(input_geometry_, CV_32FC3, mean_);
  } else
  {
    sample_resized.convertTo(sample_float, CV_32FC1);
    mean_img = cv::Mat(input_geometry_, CV_32FC1, mean_);
  }

  cv::Mat sample_normalized;

  cv::subtract(sample_float, mean_img, sample_normalized);

  /* This operation will write the separate BGR planes directly to the
   * input layer of the network because it is wrapped by the cv::Mat
   * objects in input_channels. */
  cv::split(sample_normalized, *input_channels);

  CHECK(reinterpret_cast<float *>(input_channels->at(0).data)
        == net_->input_blobs()[0]->cpu_data())
    << "Input channels are not wrapping the input layer of the network.";
}
