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
#ifndef TRTNET_HPP_
#define TRTNET_HPP_

#include "Utils.hpp"

#include <NvCaffeParser.h>
#include <NvInferPlugin.h>

#include <algorithm>
#include <fstream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

namespace Tn
{
enum class RUN_MODE { FLOAT32 = 0, FLOAT16 = 1, INT8 = 2 };

class trtNet
{
public:
  // Load from engine file
  explicit trtNet(const std::string & engineFile);

  ~trtNet()
  {
    // Release the stream and the buffers
    cudaStreamSynchronize(mTrtCudaStream);
    cudaStreamDestroy(mTrtCudaStream);
    for (auto & item : mTrtCudaBuffer) {
      cudaFree(item);
    }

    if (!mTrtRunTime) {
      delete mTrtRunTime;
    }
    if (!mTrtContext) {
      delete mTrtContext;
    }
    if (!mTrtEngine) {
      delete mTrtEngine;
    }
  }

  void saveEngine(std::string fileName)
  {
    if (mTrtEngine) {
      nvinfer1::IHostMemory * data = mTrtEngine->serialize();
      std::ofstream file;
      file.open(fileName, std::ios::binary | std::ios::out);
      if (!file.is_open()) {
        std::cout << "read create engine file" << fileName << " failed" << std::endl;
        return;
      }

      file.write((const char *)data->data(), data->size());
      file.close();
    }
  }

  void doInference(const void * inputData, void * outputData);

  inline size_t getInputSize()
  {
    return std::accumulate(
      mTrtBindBufferSize.begin(), mTrtBindBufferSize.begin() + mTrtInputCount, 0);
  }

  inline size_t getOutputSize()
  {
    return std::accumulate(
      mTrtBindBufferSize.begin() + mTrtInputCount, mTrtBindBufferSize.end(), 0);
  }

private:
  void InitEngine();

  nvinfer1::IExecutionContext * mTrtContext;
  nvinfer1::ICudaEngine * mTrtEngine;
  nvinfer1::IRuntime * mTrtRunTime;
  cudaStream_t mTrtCudaStream;
  Profiler mTrtProfiler;
  RUN_MODE mTrtRunMode;

  std::vector<void *> mTrtCudaBuffer;
  std::vector<int64_t> mTrtBindBufferSize;
  int mTrtInputCount;
};
}  // namespace Tn

#endif  // TRTNET_HPP_
