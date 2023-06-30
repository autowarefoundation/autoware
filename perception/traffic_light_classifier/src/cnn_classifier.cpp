// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "traffic_light_classifier/cnn_classifier.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <memory>
#include <string>
#include <vector>

namespace traffic_light
{
CNNClassifier::CNNClassifier(rclcpp::Node * node_ptr) : node_ptr_(node_ptr)
{
  image_pub_ = image_transport::create_publisher(
    node_ptr_, "~/output/debug/image", rclcpp::QoS{1}.get_rmw_qos_profile());

  std::string precision;
  std::string label_file_path;
  std::string model_file_path;
  precision = node_ptr_->declare_parameter("classifier_precision", "fp16");
  label_file_path = node_ptr_->declare_parameter("classifier_label_path", "labels.txt");
  model_file_path = node_ptr_->declare_parameter("classifier_model_path", "model.onnx");
  apply_softmax_ = node_ptr_->declare_parameter("apply_softmax", false);
  mean_ =
    node_ptr->declare_parameter("classifier_mean", std::vector<double>{123.675, 116.28, 103.53});
  std_ = node_ptr->declare_parameter("classifier_std", std::vector<double>{58.395, 57.12, 57.375});
  if (mean_.size() != 3 || std_.size() != 3) {
    RCLCPP_ERROR(node_ptr->get_logger(), "classifier_mean and classifier_std must be of size 3");
    return;
  }

  readLabelfile(label_file_path, labels_);

  tensorrt_common::BatchConfig batch_config{1, 1, 1};
  size_t max_workspace_size = 1 << 30;

  trt_common_ = std::make_unique<tensorrt_common::TrtCommon>(
    model_file_path, precision, nullptr, batch_config, max_workspace_size);
  trt_common_->setup();
  if (!trt_common_->isInitialized()) {
    return;
  }
  const auto input_dims = trt_common_->getBindingDimensions(0);
  if (input_dims.nbDims != 4) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Model input dimension must be 4!");
  }
  batch_size_ = input_dims.d[0];
  input_c_ = input_dims.d[1];
  input_h_ = input_dims.d[2];
  input_w_ = input_dims.d[3];
  num_input_ = batch_size_ * input_c_ * input_h_ * input_w_;
  const auto output_dims = trt_common_->getBindingDimensions(1);
  num_output_ =
    std::accumulate(output_dims.d, output_dims.d + output_dims.nbDims, 1, std::multiplies<int>());
  if (node_ptr_->declare_parameter("build_only", false)) {
    RCLCPP_INFO(node_ptr_->get_logger(), "TensorRT engine is built and shutdown node.");
    rclcpp::shutdown();
  }
}

bool CNNClassifier::getTrafficSignal(
  const cv::Mat & input_image, tier4_perception_msgs::msg::TrafficSignal & traffic_signal)
{
  if (!trt_common_->isInitialized()) {
    RCLCPP_WARN(node_ptr_->get_logger(), "failed to init tensorrt");
    return false;
  }

  std::vector<float> input_data_host(num_input_);

  cv::Mat image = input_image.clone();
  preProcess(image, input_data_host, true);

  auto input_data_device = cuda_utils::make_unique<float[]>(num_input_);
  cudaMemcpy(
    input_data_device.get(), input_data_host.data(), num_input_ * sizeof(float),
    cudaMemcpyHostToDevice);

  auto output_data_device = cuda_utils::make_unique<float[]>(num_output_);

  // do inference
  std::vector<void *> bindings = {input_data_device.get(), output_data_device.get()};

  trt_common_->enqueueV2(bindings.data(), *stream_, nullptr);

  std::vector<float> output_data_host(num_output_);
  cudaMemcpy(
    output_data_host.data(), output_data_device.get(), num_output_ * sizeof(float),
    cudaMemcpyDeviceToHost);

  postProcess(output_data_host, traffic_signal, apply_softmax_);

  /* debug */
  if (0 < image_pub_.getNumSubscribers()) {
    cv::Mat debug_image = input_image.clone();
    outputDebugImage(debug_image, traffic_signal);
  }

  return true;
}

void CNNClassifier::outputDebugImage(
  cv::Mat & debug_image, const tier4_perception_msgs::msg::TrafficSignal & traffic_signal)
{
  float probability;
  std::string label;
  for (std::size_t i = 0; i < traffic_signal.elements.size(); i++) {
    auto light = traffic_signal.elements.at(i);
    const auto light_label = state2label_[light.color] + "-" + state2label_[light.shape];
    label += light_label;
    // all lamp confidence are the same
    probability = light.confidence;
    if (i < traffic_signal.elements.size() - 1) {
      label += ",";
    }
  }

  const int expand_w = 200;
  const int expand_h =
    std::max(static_cast<int>((expand_w * debug_image.rows) / debug_image.cols), 1);

  cv::resize(debug_image, debug_image, cv::Size(expand_w, expand_h));
  cv::Mat text_img(cv::Size(expand_w, 50), CV_8UC3, cv::Scalar(0, 0, 0));
  std::string text = label + " " + std::to_string(probability);
  cv::putText(
    text_img, text, cv::Point(5, 25), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
  cv::vconcat(debug_image, text_img, debug_image);

  const auto debug_image_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", debug_image).toImageMsg();
  image_pub_.publish(debug_image_msg);
}

void CNNClassifier::preProcess(cv::Mat & image, std::vector<float> & input_tensor, bool normalize)
{
  const float scale =
    std::min(static_cast<float>(input_w_) / image.cols, static_cast<float>(input_h_) / image.rows);
  const auto scale_size = cv::Size(image.cols * scale, image.rows * scale);
  cv::resize(image, image, scale_size, 0, 0, cv::INTER_CUBIC);
  const auto bottom = input_h_ - image.rows;
  const auto right = input_w_ - image.cols;
  copyMakeBorder(image, image, 0, bottom, 0, right, cv::BORDER_CONSTANT, {0, 0, 0});

  const size_t strides_cv[3] = {
    static_cast<size_t>(input_w_ * input_c_), static_cast<size_t>(input_c_), 1};
  const size_t strides[3] = {
    static_cast<size_t>(input_h_ * input_w_), static_cast<size_t>(input_w_), 1};

  for (int i = 0; i < input_h_; i++) {
    for (int j = 0; j < input_w_; j++) {
      for (int k = 0; k < input_c_; k++) {
        const size_t offset_cv = i * strides_cv[0] + j * strides_cv[1] + k * strides_cv[2];
        const size_t offset = k * strides[0] + i * strides[1] + j * strides[2];
        if (normalize) {
          input_tensor[offset] = (static_cast<float>(image.data[offset_cv]) - mean_[k]) / std_[k];
        } else {
          input_tensor[offset] = static_cast<float>(image.data[offset_cv]);
        }
      }
    }
  }
}

bool CNNClassifier::postProcess(
  std::vector<float> & output_tensor, tier4_perception_msgs::msg::TrafficSignal & traffic_signal,
  bool apply_softmax)
{
  std::vector<float> probs;
  if (apply_softmax) {
    calcSoftmax(output_tensor, probs, num_output_);
  }
  std::vector<size_t> sorted_indices = argsort(output_tensor, num_output_);

  size_t max_indice = sorted_indices.front();
  std::string match_label = labels_[max_indice];
  float probability = apply_softmax ? probs[max_indice] : output_tensor[max_indice];

  // label names are assumed to be comma-separated to represent each lamp
  // e.g.
  // match_label: "red","red-cross","right"
  // split_label: ["red","red-cross","right"]
  // if shape doesn't have color suffix, set GREEN to color state.
  // if color doesn't have shape suffix, set CIRCLE to shape state.
  std::vector<std::string> split_label;
  boost::algorithm::split(split_label, match_label, boost::is_any_of(","));
  for (auto label : split_label) {
    if (label2state_.find(label) == label2state_.end()) {
      RCLCPP_DEBUG(
        node_ptr_->get_logger(), "cnn_classifier does not have a key [%s]", label.c_str());
      continue;
    }
    tier4_perception_msgs::msg::TrafficLightElement element;
    if (label.find("-") != std::string::npos) {
      // found "-" delimiter in label string
      std::vector<std::string> color_and_shape;
      boost::algorithm::split(color_and_shape, label, boost::is_any_of("-"));
      element.color = label2state_[color_and_shape.at(0)];
      element.shape = label2state_[color_and_shape.at(1)];
    } else {
      if (label == state2label_[tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN]) {
        element.color = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
        element.shape = tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN;
      } else if (isColorLabel(label)) {
        element.color = label2state_[label];
        element.shape = tier4_perception_msgs::msg::TrafficLightElement::CIRCLE;
      } else {
        element.color = tier4_perception_msgs::msg::TrafficLightElement::GREEN;
        element.shape = label2state_[label];
      }
    }
    element.confidence = probability;
    traffic_signal.elements.push_back(element);
  }

  return true;
}

bool CNNClassifier::readLabelfile(std::string filepath, std::vector<std::string> & labels)
{
  std::ifstream labelsFile(filepath);
  if (!labelsFile.is_open()) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Could not open label file. [%s]", filepath.c_str());
    return false;
  }
  std::string label;
  while (getline(labelsFile, label)) {
    labels.push_back(label);
  }
  return true;
}

void CNNClassifier::calcSoftmax(
  std::vector<float> & data, std::vector<float> & probs, int num_output)
{
  float exp_sum = 0.0;
  for (int i = 0; i < num_output; ++i) {
    exp_sum += exp(data[i]);
  }

  for (int i = 0; i < num_output; ++i) {
    probs.push_back(exp(data[i]) / exp_sum);
  }
}

std::vector<size_t> CNNClassifier::argsort(std::vector<float> & tensor, int num_output)
{
  std::vector<size_t> indices(num_output);
  for (int i = 0; i < num_output; i++) {
    indices[i] = i;
  }
  std::sort(indices.begin(), indices.begin() + num_output, [tensor](size_t idx1, size_t idx2) {
    return tensor[idx1] > tensor[idx2];
  });

  return indices;
}

bool CNNClassifier::isColorLabel(const std::string label)
{
  using tier4_perception_msgs::msg::TrafficSignal;
  if (
    label == state2label_[tier4_perception_msgs::msg::TrafficLightElement::GREEN] ||
    label == state2label_[tier4_perception_msgs::msg::TrafficLightElement::AMBER] ||
    label == state2label_[tier4_perception_msgs::msg::TrafficLightElement::RED] ||
    label == state2label_[tier4_perception_msgs::msg::TrafficLightElement::WHITE]) {
    return true;
  }
  return false;
}

}  // namespace traffic_light
