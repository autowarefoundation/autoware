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

// Copyright (c) OpenMMLab. All rights reserved.

#ifndef TRT_PLUGIN_HELPER_HPP_
#define TRT_PLUGIN_HELPER_HPP_

#include <NvInferRuntime.h>
#include <cudnn.h>

#include <iostream>
#include <stdexcept>

cudnnStatus_t convert_trt2cudnn_dtype(nvinfer1::DataType trt_dtype, cudnnDataType_t * cudnn_dtype);

enum pluginStatus_t {
  STATUS_SUCCESS = 0,
  STATUS_FAILURE = 1,
  STATUS_BAD_PARAM = 2,
  STATUS_NOT_SUPPORTED = 3,
  STATUS_NOT_INITIALIZED = 4
};  // enum pluginStatus_t

#define ASSERT(assertion)                                                     \
  {                                                                           \
    if (!assertion) {                                                         \
      std::cerr << "#assertion" << __FILE__ << ", " << __LINE__ << std::endl; \
      abort();                                                                \
    }                                                                         \
  }

// cspell: ignore CUASSERT
#define CUASSERT(status_)                                                                       \
  {                                                                                             \
    auto s_ = status;                                                                           \
    if (s_ != cudaSuccess) {                                                                    \
      std::cerr << __FILE__ << ", " << __LINE__ << ", " << s_ << ", " << cudaGetErrorString(s_) \
                << std::endl;                                                                   \
    }                                                                                           \
  }

// cspell: ignore CUBLASASSERT
#define CUBLASASSERT(status_)                                               \
  {                                                                         \
    auto s_ = status_;                                                      \
    if (s_ != CUBLAS_STATUS_SUCCESS) {                                      \
      std::cerr << __FILE__ << ", " << __LINE__ << ", " << s_ << std::endl; \
    }                                                                       \
  }

// cspell: ignore CUERRORMSG
#define CUERRORMSG(status_)                                                            \
  {                                                                                    \
    auto s_ = status_;                                                                 \
    if (s_ != 0) std::cerr << __FILE__ << ", " << __LINE__ << ", " << s_ << std::endl; \
  }

#endif  // TRT_PLUGIN_HELPER_HPP_
