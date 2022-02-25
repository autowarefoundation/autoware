/*
 * MIT License
 *
 * Copyright (c) 2018 lewes6369
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <NvInferPlugin.h>
#include <cudnn.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#ifndef CUDA_CHECK

#define CUDA_CHECK(callstr)                                                              \
  {                                                                                      \
    cudaError_t error_code = callstr;                                                    \
    if (error_code != cudaSuccess) {                                                     \
      std::cerr << "CUDA error " << error_code << " at " << __FILE__ << ":" << __LINE__; \
      assert(0);                                                                         \
    }                                                                                    \
  }

#endif

namespace Tn
{
class Profiler : public nvinfer1::IProfiler
{
public:
  void printLayerTimes(int iterationsTimes)
  {
    float totalTime = 0;
    for (size_t i = 0; i < mProfile.size(); i++) {
      printf("%-40.40s %4.3fms\n", mProfile[i].first.c_str(), mProfile[i].second / iterationsTimes);
      totalTime += mProfile[i].second;
    }
    printf("Time over all layers: %4.3f\n", totalTime / iterationsTimes);
  }

private:
  typedef std::pair<std::string, float> Record;
  std::vector<Record> mProfile;

  void reportLayerTime(const char * layerName, float ms) noexcept override
  {
    auto record = std::find_if(
      mProfile.begin(), mProfile.end(), [&](const Record & r) { return r.first == layerName; });
    if (record == mProfile.end()) {
      mProfile.push_back(std::make_pair(layerName, ms));
    } else {
      record->second += ms;
    }
  }
};

// Logger for TensorRT info/warning/errors
class Logger : public nvinfer1::ILogger
{
public:
  Logger() : Logger(Severity::kWARNING) {}

  explicit Logger(Severity severity) : reportableSeverity(severity) {}

  void log(Severity severity, const char * msg) noexcept override
  {
    // suppress messages with severity enum value greater than the reportable
    if (severity > reportableSeverity) {
      return;
    }

    switch (severity) {
      case Severity::kINTERNAL_ERROR:
        std::cerr << "INTERNAL_ERROR: ";
        break;
      case Severity::kERROR:
        std::cerr << "ERROR: ";
        break;
      case Severity::kWARNING:
        std::cerr << "WARNING: ";
        break;
      case Severity::kINFO:
        std::cerr << "INFO: ";
        break;
      default:
        std::cerr << "UNKNOWN: ";
        break;
    }
    std::cerr << msg << std::endl;
  }

  Severity reportableSeverity{Severity::kWARNING};
};

template <typename T>
void write(char *& buffer, const T & val)
{
  *reinterpret_cast<T *>(buffer) = val;
  buffer += sizeof(T);
}

template <typename T>
void read(const char *& buffer, T & val)
{
  val = *reinterpret_cast<const T *>(buffer);
  buffer += sizeof(T);
}
}  // namespace Tn

#endif  // UTILS_HPP_
