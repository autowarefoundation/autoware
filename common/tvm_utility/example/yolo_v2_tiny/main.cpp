// Copyright 2021-2022 Arm Limited and Contributors.
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

#include "tvm_utility/pipeline.hpp"
#include "yolo_v2_tiny/inference_engine_tvm_config.hpp"

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

using model_zoo::perception::camera_obstacle_detection::yolo_v2_tiny::tensorflow_fp32_coco::config;

namespace tvm_utility
{
namespace yolo_v2_tiny
{

class PreProcessorYoloV2Tiny : public tvm_utility::pipeline::PreProcessor<std::string>
{
public:
  explicit PreProcessorYoloV2Tiny(tvm_utility::pipeline::InferenceEngineTVMConfig config)
  : network_input_width(config.network_inputs[0].node_shape[1]),
    network_input_height(config.network_inputs[0].node_shape[2]),
    network_input_depth(config.network_inputs[0].node_shape[3]),
    network_input_datatype_bytes(config.network_inputs[0].tvm_dtype_bits / 8)
  {
    // Allocate input variable
    std::vector<int64_t> shape_x{1, network_input_width, network_input_height, network_input_depth};
    tvm_utility::pipeline::TVMArrayContainer x{
      shape_x,
      config.network_inputs[0].tvm_dtype_code,
      config.network_inputs[0].tvm_dtype_bits,
      config.network_inputs[0].tvm_dtype_lanes,
      config.tvm_device_type,
      config.tvm_device_id};

    output = x;
  }

  // The cv::Mat can't be used as an input because it throws an exception when
  // passed as a constant reference
  tvm_utility::pipeline::TVMArrayContainerVector schedule(const std::string & input)
  {
    // Read input image
    auto image = cv::imread(input, cv::IMREAD_COLOR);
    if (!image.data) {
      throw std::runtime_error("File " + input + " not found");
    }

    // Compute the ratio for resizing and size for padding
    double scale_x = static_cast<double>(image.size().width) / network_input_width;
    double scale_y = static_cast<double>(image.size().height) / network_input_height;
    double scale = std::max(scale_x, scale_y);

    // Perform padding
    if (scale != 1) {
      cv::resize(image, image, cv::Size(), 1.0f / scale, 1.0f / scale);
    }

    size_t w_pad = network_input_width - image.size().width;
    size_t h_pad = network_input_height - image.size().height;

    if (w_pad || h_pad) {
      cv::copyMakeBorder(
        image, image, h_pad / 2, (h_pad - h_pad / 2), w_pad / 2, (w_pad - w_pad / 2),
        cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    }

    // Convert pixel values from int8 to float32. convert pixel value range from 0 - 255 to 0 - 1.
    cv::Mat3f image_3f{};
    image.convertTo(image_3f, CV_32FC3, 1 / 255.0f);

    // cv library uses BGR as a default color format, the network expects the data in RGB format
    cv::cvtColor(image_3f, image_3f, cv::COLOR_BGR2RGB);

    TVMArrayCopyFromBytes(
      output.getArray(), image_3f.data,
      network_input_width * network_input_height * network_input_depth *
        network_input_datatype_bytes);

    return {output};
  }

private:
  int64_t network_input_width;
  int64_t network_input_height;
  int64_t network_input_depth;
  int64_t network_input_datatype_bytes;
  tvm_utility::pipeline::TVMArrayContainer output;
};

class PostProcessorYoloV2Tiny : public tvm_utility::pipeline::PostProcessor<std::vector<float>>
{
public:
  explicit PostProcessorYoloV2Tiny(
    tvm_utility::pipeline::InferenceEngineTVMConfig config, std::string label_filename,
    std::string anchor_filename)
  : network_output_width(config.network_outputs[0].node_shape[1]),
    network_output_height(config.network_outputs[0].node_shape[2]),
    network_output_depth(config.network_outputs[0].node_shape[3]),
    network_output_datatype_bytes(config.network_outputs[0].tvm_dtype_bits / 8)
  {
    // Parse human readable names for the classes
    std::ifstream label_file{label_filename};
    if (!label_file.good()) {
      std::string label_filename = label_filename;
      throw std::runtime_error("unable to open label file:" + label_filename);
    }
    std::string line{};
    while (std::getline(label_file, line)) {
      labels.push_back(line);
    }

    // Get anchor values for this network from the anchor file
    std::ifstream anchor_file{anchor_filename};
    if (!anchor_file.good()) {
      std::string anchor_filename = anchor_filename;
      throw std::runtime_error("unable to open anchor file:" + anchor_filename);
    }
    std::string first{};
    std::string second{};
    while (std::getline(anchor_file, line)) {
      std::stringstream line_stream(line);
      std::getline(line_stream, first, ',');
      std::getline(line_stream, second, ',');
      anchors.push_back(std::make_pair(std::atof(first.c_str()), std::atof(second.c_str())));
    }
  }

  // Sigmoid function
  float sigmoid(float x) { return static_cast<float>(1.0 / (1.0 + std::exp(-x))); }

  std::vector<float> schedule(const tvm_utility::pipeline::TVMArrayContainerVector & input)
  {
    auto l_h = network_output_width;   // Layer height
    auto l_w = network_output_height;  // Layer width
    auto n_classes = labels.size();    // Total number of classes
    auto n_anchors = anchors.size();   // Total number of anchors
    const uint32_t n_coords = 4;       // Number of coordinates in a single anchor box

    // Assert data is stored row-majored in input and the dtype is float
    assert(input[0].getArray()->strides == nullptr);
    assert(input[0].getArray()->dtype.bits == sizeof(float) * 8);

    // Copy the inference data to CPU memory
    std::vector<float> infer(
      network_output_width * network_output_height * network_output_depth, 0);
    TVMArrayCopyToBytes(
      input[0].getArray(), infer.data(),
      network_output_width * network_output_height * network_output_depth *
        network_output_datatype_bytes);

    // Utility function to return data from y given index
    auto get_output_data = [this, infer, n_classes, n_anchors, n_coords](
                             auto row_i, auto col_j, auto anchor_k, auto offset) {
      auto box_index = (row_i * network_output_height + col_j) * network_output_depth;
      auto index = box_index + anchor_k * (n_classes + n_coords + 1);
      return infer[index + offset];
    };

    // Vector used to check if the result is accurate,
    // this is also the output of this (schedule) function
    std::vector<float> scores_above_threshold{};

    // Parse results into detections. Loop over each detection cell in the model output
    for (decltype(l_w) i = 0; i < l_w; i++) {
      for (decltype(l_h) j = 0; j < l_h; j++) {
        for (size_t anchor_k = 0; anchor_k < n_anchors; anchor_k++) {
          // Compute property index
          auto box_p = get_output_data(i, j, anchor_k, 4);

          // Decode the confidence of detection in this anchor box
          auto p_0 = sigmoid(box_p);

          // Find maximum probability of all classes
          float max_p = 0.0f;
          int max_ind = -1;
          for (size_t i_class = 0; i_class < n_classes; i_class++) {
            auto class_p = get_output_data(i, j, anchor_k, 5 + i_class);
            if (max_p < class_p) {
              max_p = class_p;
              max_ind = i_class;
            }
          }

          // Decode and copy class probabilities
          std::vector<float> class_probabilities{};
          float p_total = 0;
          for (size_t i_class = 0; i_class < n_classes; i_class++) {
            auto class_p = get_output_data(i, j, anchor_k, 5 + i_class);
            class_probabilities.push_back(std::exp(class_p - max_p));
            p_total += class_probabilities[i_class];
          }

          // Find the most likely score
          auto max_score = class_probabilities[max_ind] * p_0 / p_total;

          // Draw all detections with high scores
          if (max_score > 0.3) {
            scores_above_threshold.push_back(max_score);
          }
        }
      }
    }

    return scores_above_threshold;
  }

private:
  int64_t network_output_width;
  int64_t network_output_height;
  int64_t network_output_depth;
  int64_t network_output_datatype_bytes;
  std::vector<std::string> labels{};
  std::vector<std::pair<float, float>> anchors{};
};

}  // namespace yolo_v2_tiny
}  // namespace tvm_utility

bool check_near(double expected, double actual, double tolerance)
{
  return fabs(expected - actual) <= tolerance;
}

int main(int argc, char * argv[])
{
  // init node to use parameters
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("yolo_v2_tiny_example");
  // Filename of the image on which to run the inference
  node->declare_parameter<std::string>("image_filename");
  // Name of file containing the human readable names of the classes. One class
  // on each line.
  node->declare_parameter<std::string>("label_filename");
  // Name of file containing the anchor values for the network. Each line is one
  // anchor. each anchor has 2 comma separated floating point values.
  node->declare_parameter<std::string>("anchor_filename");
  // Packages data and artifacts directory path.
  node->declare_parameter<std::string>("data_path");

  RCLCPP_INFO(node->get_logger(), "Node started");

  // Instantiate the pipeline
  using PrePT = tvm_utility::yolo_v2_tiny::PreProcessorYoloV2Tiny;
  using IET = tvm_utility::pipeline::InferenceEngineTVM;
  using PostPT = tvm_utility::yolo_v2_tiny::PostProcessorYoloV2Tiny;

  PrePT PreP{config};
  IET IE{config, "tvm_utility", node->get_parameter("data_path").as_string()};
  PostPT PostP{
    config, node->get_parameter("label_filename").as_string(),
    node->get_parameter("anchor_filename").as_string()};

  tvm_utility::pipeline::Pipeline<PrePT, IET, PostPT> pipeline(PreP, IE, PostP);

  // Push data input the pipeline and get the output
  auto output = pipeline.schedule(node->get_parameter("image_filename").as_string());

  // Define reference vector containing expected values, expressed as hexadecimal integers
  std::vector<int32_t> int_output{0x3eb64594, 0x3f435656, 0x3ece1600, 0x3e99d381,
                                  0x3f1cd6bc, 0x3f14f4dd, 0x3ed8065f, 0x3ee9f4fa,
                                  0x3ec1b5e8, 0x3f4e7c6c, 0x3f136af1};

  std::vector<float> expected_output(int_output.size());

  // A memcpy means that the floats in expected_output have a well-defined binary value
  for (size_t i = 0; i < int_output.size(); i++) {
    memcpy(&expected_output[i], &int_output[i], sizeof(expected_output[i]));
  }

  // Test: check if the generated output is equal to the reference
  if (expected_output.size() == output.size()) {
    RCLCPP_INFO(node->get_logger(), "Model has proper output size");
  } else {
    RCLCPP_INFO(node->get_logger(), "Model has unexpected output size");
  }

  for (size_t i = 0; i < output.size(); ++i) {
    if (check_near(expected_output[i], output[i], 0.0001)) {
      std::cout << "Model has proper output at index: " << i << std::endl;
      RCLCPP_INFO(node->get_logger(), "Model has proper output at index: %zu", i);

    } else {
      RCLCPP_INFO(node->get_logger(), "Model has unexpected output at index: %zu", i);
    }
  }
  rclcpp::shutdown();
  return 0;
}
