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

#include <TrtNet.hpp>

#include <cublas_v2.h>
#include <cudnn.h>
#include <string.h>
#include <time.h>

#include <cassert>
#include <chrono>
#include <iostream>
#include <sstream>
#include <unordered_map>

using namespace nvinfer1;        // NOLINT
using namespace nvcaffeparser1;  // NOLINT
using namespace plugin;          // NOLINT

static Tn::Logger gLogger;

inline void * safeCudaMalloc(size_t memSize)
{
  void * deviceMem;
  CUDA_CHECK(cudaMalloc(&deviceMem, memSize));
  if (deviceMem == nullptr) {
    std::cerr << "Out of memory" << std::endl;
    exit(1);
  }
  return deviceMem;
}

inline int64_t volume(const nvinfer1::Dims & d)
{
  return std::accumulate(d.d, d.d + d.nbDims, 1, std::multiplies<int64_t>());
}

inline unsigned int getElementSize(nvinfer1::DataType t)
{
  switch (t) {
    case nvinfer1::DataType::kINT32:
      return 4;
    case nvinfer1::DataType::kFLOAT:
      return 4;
    case nvinfer1::DataType::kHALF:
      return 2;
    case nvinfer1::DataType::kINT8:
      return 1;
    default:
      throw std::runtime_error("Invalid DataType.");
      return 0;
  }
}

namespace Tn
{
trtNet::trtNet(const std::string & engineFile)
: mTrtContext(nullptr),
  mTrtEngine(nullptr),
  mTrtRunTime(nullptr),
  mTrtRunMode(RUN_MODE::FLOAT32),
  mTrtInputCount(0)
{
  using namespace std;  // NOLINT
  fstream file;

  file.open(engineFile, ios::binary | ios::in);
  if (!file.is_open()) {
    cout << "read engine file" << engineFile << " failed" << endl;
    return;
  }
  file.seekg(0, ios::end);
  int length = file.tellg();
  file.seekg(0, ios::beg);
  std::unique_ptr<char[]> data(new char[length]);
  file.read(data.get(), length);
  file.close();

  mTrtRunTime = createInferRuntime(gLogger);
  assert(mTrtRunTime != nullptr);
  mTrtEngine = mTrtRunTime->deserializeCudaEngine(data.get(), length);
  assert(mTrtEngine != nullptr);

  InitEngine();
}

void trtNet::InitEngine()
{
  const int maxBatchSize = 1;
  mTrtContext = mTrtEngine->createExecutionContext();
  assert(mTrtContext != nullptr);
  mTrtContext->setProfiler(&mTrtProfiler);

  int nbBindings = mTrtEngine->getNbBindings();

  mTrtCudaBuffer.resize(nbBindings);
  mTrtBindBufferSize.resize(nbBindings);
  for (int i = 0; i < nbBindings; ++i) {
    Dims dims = mTrtEngine->getBindingDimensions(i);
    DataType dtype = mTrtEngine->getBindingDataType(i);
    int64_t totalSize = volume(dims) * maxBatchSize * getElementSize(dtype);
    mTrtBindBufferSize[i] = totalSize;
    mTrtCudaBuffer[i] = safeCudaMalloc(totalSize);
    if (mTrtEngine->bindingIsInput(i)) {
      mTrtInputCount++;
    }
  }

  CUDA_CHECK(cudaStreamCreate(&mTrtCudaStream));
}

void trtNet::doInference(const void * inputData, void * outputData)
{
  static const int batchSize = 1;
  assert(mTrtInputCount == 1);

  int inputIndex = 0;
  CUDA_CHECK(cudaMemcpyAsync(
    mTrtCudaBuffer[inputIndex], inputData, mTrtBindBufferSize[inputIndex], cudaMemcpyHostToDevice,
    mTrtCudaStream));

  mTrtContext->execute(batchSize, &mTrtCudaBuffer[inputIndex]);

  for (size_t bindingIdx = mTrtInputCount; bindingIdx < mTrtBindBufferSize.size(); ++bindingIdx) {
    auto size = mTrtBindBufferSize[bindingIdx];
    CUDA_CHECK(cudaMemcpyAsync(
      outputData, mTrtCudaBuffer[bindingIdx], size, cudaMemcpyDeviceToHost, mTrtCudaStream));
  }
}
}  // namespace Tn
