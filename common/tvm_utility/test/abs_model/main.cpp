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

#include "gtest/gtest.h"
#include "tvm_utility/pipeline.hpp"
// file for current arch x86 or arm is chosen in cmake file
#include <inference_engine_tvm_config.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

using model_zoo::inf_test::engine_load::abs_model::config;

namespace tvm_utility
{
namespace abs_model
{

class PreProcessorLinearModel : public tvm_utility::pipeline::PreProcessor<std::vector<float>>
{
public:
  explicit PreProcessorLinearModel(tvm_utility::pipeline::InferenceEngineTVMConfig config)
  : network_input_a_width(config.network_inputs[0].node_shape[0]),
    network_input_a_height(config.network_inputs[0].node_shape[1]),
    network_input_datatype_bytes(config.network_inputs[0].tvm_dtype_bits / 8)
  {
    // Allocate input variable
    std::vector<int64_t> shape_a{network_input_a_width, network_input_a_height};
    tvm_utility::pipeline::TVMArrayContainer a{
      shape_a,
      config.network_inputs[0].tvm_dtype_code,
      config.network_inputs[0].tvm_dtype_bits,
      config.network_inputs[0].tvm_dtype_lanes,
      config.tvm_device_type,
      config.tvm_device_id};

    output = a;
  }

  // The cv::Mat can't be used as an input because it throws an exception when
  // passed as a constant reference
  tvm_utility::pipeline::TVMArrayContainerVector schedule(const std::vector<float> & input)
  {
    float input_mat[2][2];
    input_mat[0][0] = input[0];
    input_mat[0][1] = input[1];
    input_mat[1][0] = input[2];
    input_mat[1][1] = input[3];

    // Create cv::Mat from input array
    cv::Mat a_input = cv::Mat(2, 2, CV_32F, &input_mat);

    TVMArrayCopyFromBytes(
      output.getArray(), a_input.data,
      network_input_a_width * network_input_a_height * network_input_datatype_bytes);

    return {output};
  }

private:
  int64_t network_input_a_width;
  int64_t network_input_a_height;
  int64_t network_input_datatype_bytes;
  tvm_utility::pipeline::TVMArrayContainer output;
};

class PostProcessorLinearModel : public tvm_utility::pipeline::PostProcessor<std::vector<float>>
{
public:
  explicit PostProcessorLinearModel(tvm_utility::pipeline::InferenceEngineTVMConfig config)
  : network_output_width(config.network_outputs[0].node_shape[0]),
    network_output_height(config.network_outputs[0].node_shape[1]),
    network_output_datatype_bytes(config.network_outputs[0].tvm_dtype_bits / 8)
  {
  }

  std::vector<float> schedule(const tvm_utility::pipeline::TVMArrayContainerVector & input)
  {
    // Assert data is stored row-majored in input and the dtype is float
    assert(input[0].getArray()->strides == nullptr);
    assert(input[0].getArray()->dtype.bits == sizeof(float) * 8);

    // Copy the inference data to CPU memory
    std::vector<float> infer(network_output_width * network_output_height, 0.0f);

    TVMArrayCopyToBytes(
      input[0].getArray(), infer.data(),
      network_output_width * network_output_height * network_output_datatype_bytes);

    return infer;
  }

private:
  int64_t network_output_width;
  int64_t network_output_height;
  int64_t network_output_datatype_bytes;
};

TEST(PipelineExamples, SimplePipeline)
{
  // // Instantiate the pipeline
  using PrePT = PreProcessorLinearModel;
  using IET = tvm_utility::pipeline::InferenceEngineTVM;
  using PostPT = PostProcessorLinearModel;

  PrePT PreP{config};
  IET IE{config, "tvm_utility"};
  PostPT PostP{config};

  tvm_utility::pipeline::Pipeline<PrePT, IET, PostPT> pipeline(PreP, IE, PostP);

  auto version_status = IE.version_check({2, 0, 0});
  EXPECT_NE(version_status, tvm_utility::Version::Unsupported);

  // create input array
  std::vector<float> input_arr{-1., -2., -3., 4.};
  // send it to the model
  auto output = pipeline.schedule(input_arr);

  // define vector with expected values
  std::vector<float> expected_output{1., 2., 3., 4.};

  // // Test: check if the generated output is equal to the reference
  EXPECT_EQ(expected_output.size(), output.size()) << "Unexpected output size";
  for (size_t i = 0; i < output.size(); ++i) {
    EXPECT_NEAR(expected_output[i], output[i], 0.0001) << "at index: " << i;
  }
}

}  // namespace abs_model
}  // namespace tvm_utility
