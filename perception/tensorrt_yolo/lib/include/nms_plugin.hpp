// Copyright 2020 Tier IV, Inc.
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

/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef NMS_PLUGIN_HPP_
#define NMS_PLUGIN_HPP_

#include <NvInfer.h>

#include <cassert>
#include <vector>

namespace yolo
{
class NMSPlugin : public nvinfer1::IPluginV2DynamicExt
{
public:
  NMSPlugin(float nms_thresh, int detections_per_im);
  NMSPlugin(float nms_thresh, int detections_per_im, size_t count);
  NMSPlugin(const void * data, size_t length);

  // IPluginV2 methods
  const char * getPluginType() const noexcept override;
  const char * getPluginVersion() const noexcept override;
  int getNbOutputs() const noexcept override;
  int initialize() noexcept override;
  void terminate() noexcept override;
  size_t getSerializationSize() const noexcept override;
  void serialize(void * buffer) const noexcept override;
  void destroy() noexcept override;
  void setPluginNamespace(const char * libNamespace) noexcept override;
  const char * getPluginNamespace() const noexcept override;

  // IPluginV2Ext methods
  nvinfer1::DataType getOutputDataType(
    int index, const nvinfer1::DataType * inputType, int nbInputs) const noexcept override;

  // IPluginV2DynamicExt methods
  nvinfer1::IPluginV2DynamicExt * clone() const noexcept override;
  nvinfer1::DimsExprs getOutputDimensions(
    int outputIndex, const nvinfer1::DimsExprs * inputs, int nbInputs,
    nvinfer1::IExprBuilder & exprBuilder) noexcept override;
  bool supportsFormatCombination(
    int pos, const nvinfer1::PluginTensorDesc * inOut, int nbInputs,
    int nbOutputs) noexcept override;
  void configurePlugin(
    const nvinfer1::DynamicPluginTensorDesc * in, int nbInputs,
    const nvinfer1::DynamicPluginTensorDesc * out, int nbOutputs) noexcept override;
  size_t getWorkspaceSize(
    const nvinfer1::PluginTensorDesc * inputs, int nbInputs,
    const nvinfer1::PluginTensorDesc * outputs, int nbOutputs) const noexcept override;
  int enqueue(
    const nvinfer1::PluginTensorDesc * inputDesc, const nvinfer1::PluginTensorDesc * outputDesc,
    const void * const * inputs, void * const * outputs, void * workspace,
    cudaStream_t stream) noexcept override;

private:
  float nms_thresh_;
  int detections_per_im_;

  size_t count_;
  mutable int size = -1;
};

class NMSPluginCreator : public nvinfer1::IPluginCreator
{
public:
  NMSPluginCreator();

  const char * getPluginName() const noexcept override;

  const char * getPluginVersion() const noexcept override;

  const nvinfer1::PluginFieldCollection * getFieldNames() noexcept override;

  nvinfer1::IPluginV2DynamicExt * createPlugin(
    const char * name, const nvinfer1::PluginFieldCollection * fc) noexcept override;

  nvinfer1::IPluginV2DynamicExt * deserializePlugin(
    const char * name, const void * serialData, size_t serialLength) noexcept override;

  void setPluginNamespace(const char * libNamespace) noexcept override;

  const char * getPluginNamespace() const noexcept override;
};

REGISTER_TENSORRT_PLUGIN(NMSPluginCreator);

}  // namespace yolo

#endif  // NMS_PLUGIN_HPP_
