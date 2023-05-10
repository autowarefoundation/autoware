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

#ifndef TENSORRT_COMMON__SIMPLE_PROFILER_HPP_
#define TENSORRT_COMMON__SIMPLE_PROFILER_HPP_

#include <NvInfer.h>

#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace tensorrt_common
{
struct LayerInfo
{
  int in_c;
  int out_c;
  int w;
  int h;
  int k;
  int stride;
  int groups;
  nvinfer1::LayerType type;
};

/**
 * @class Profiler
 * @brief Collect per-layer profile information, assuming times are reported in the same order
 */
class SimpleProfiler : public nvinfer1::IProfiler
{
public:
  struct Record
  {
    float time{0};
    int count{0};
    float min_time{-1.0};
    int index;
  };
  SimpleProfiler(
    std::string name,
    const std::vector<SimpleProfiler> & src_profilers = std::vector<SimpleProfiler>());

  void reportLayerTime(const char * layerName, float ms) noexcept override;

  void setProfDict(nvinfer1::ILayer * layer) noexcept;

  friend std::ostream & operator<<(std::ostream & out, SimpleProfiler & value);

private:
  std::string m_name;
  std::map<std::string, Record> m_profile;
  int m_index;
  std::map<std::string, LayerInfo> m_layer_dict;
};
}  // namespace tensorrt_common
#endif  // TENSORRT_COMMON__SIMPLE_PROFILER_HPP_
