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
 * MIT License

 * Copyright (c) 2019-2020 Wang Xinyu

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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

#include <yolo_layer.hpp>
#include <yolo_layer_plugin.hpp>

#include <cuda_runtime_api.h>
#include <stdio.h>
#include <string.h>

#include <cassert>
#include <cmath>
#include <vector>

using nvinfer1::DataType;
using nvinfer1::DimsExprs;
using nvinfer1::DynamicPluginTensorDesc;
using nvinfer1::IExprBuilder;
using nvinfer1::IPluginV2DynamicExt;
using nvinfer1::PluginFieldCollection;
using nvinfer1::PluginFormat;
using nvinfer1::PluginTensorDesc;

namespace
{
const char * YOLO_LAYER_PLUGIN_VERSION{"1"};
const char * YOLO_LAYER_PLUGIN_NAME{"YOLO_Layer_TRT"};
const char * YOLO_LAYER_PLUGIN_NAMESPACE{""};

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
}  // namespace

namespace yolo
{
YoloLayerPlugin::YoloLayerPlugin(
  int width, int height, int num_anchors, std::vector<float> & anchors, float scale_x_y,
  float score_thresh, bool use_darknet_layer)
: width_(width),
  height_(height),
  num_anchors_(num_anchors),
  anchors_(anchors),
  scale_x_y_(scale_x_y),
  score_thresh_(score_thresh),
  use_darknet_layer_(use_darknet_layer)
{
}

// create the plugin at runtime from a byte stream
YoloLayerPlugin::YoloLayerPlugin(const void * data, size_t length)
{
  (void)length;

  const char * d = static_cast<const char *>(data);
  read(d, width_);
  read(d, height_);
  read(d, num_anchors_);
  int anchor_size = num_anchors_ * 2;
  while (anchor_size--) {
    float val;
    read(d, val);
    anchors_.push_back(val);
  }
  read(d, scale_x_y_);
  read(d, score_thresh_);
  read(d, use_darknet_layer_);
}
// IPluginV2 Methods

const char * YoloLayerPlugin::getPluginType() const noexcept { return YOLO_LAYER_PLUGIN_NAME; }

const char * YoloLayerPlugin::getPluginVersion() const noexcept
{
  return YOLO_LAYER_PLUGIN_VERSION;
}

int YoloLayerPlugin::getNbOutputs() const noexcept { return 3; }

int YoloLayerPlugin::initialize() noexcept { return 0; }

void YoloLayerPlugin::terminate() noexcept {}

size_t YoloLayerPlugin::getSerializationSize() const noexcept
{
  return sizeof(width_) + sizeof(height_) + sizeof(num_anchors_) +
         sizeof(float) * num_anchors_ * 2 + sizeof(scale_x_y_) + sizeof(score_thresh_) +
         sizeof(use_darknet_layer_);
}

void YoloLayerPlugin::serialize(void * buffer) const noexcept
{
  char * d = reinterpret_cast<char *>(buffer);
  write(d, width_);
  write(d, height_);
  write(d, num_anchors_);
  for (int i = 0; i < num_anchors_ * 2; ++i) {
    write(d, anchors_[i]);
  }
  write(d, scale_x_y_);
  write(d, score_thresh_);
  write(d, use_darknet_layer_);
}

void YoloLayerPlugin::destroy() noexcept { delete this; }

void YoloLayerPlugin::setPluginNamespace(const char * N) noexcept { (void)N; }

const char * YoloLayerPlugin::getPluginNamespace() const noexcept
{
  return YOLO_LAYER_PLUGIN_NAMESPACE;
}

// IPluginV2Ext Methods

DataType YoloLayerPlugin::getOutputDataType(
  int index, const DataType * inputTypes, int nbInputs) const noexcept
{
  (void)index;
  (void)inputTypes;
  (void)nbInputs;

  return DataType::kFLOAT;
}

// IPluginV2DynamicExt Methods

IPluginV2DynamicExt * YoloLayerPlugin::clone() const noexcept { return new YoloLayerPlugin(*this); }

DimsExprs YoloLayerPlugin::getOutputDimensions(
  int outputIndex, const DimsExprs * inputs, int nbInputs, IExprBuilder & exprBuilder) noexcept
{
  (void)nbInputs;

  DimsExprs ret = inputs[0];
  ret.nbDims = 3;
  const auto total_count =
    ret.d[2]->getConstantValue() * ret.d[3]->getConstantValue() * num_anchors_;
  ret.d[1] = exprBuilder.constant(total_count * (outputIndex == 1 ? 4 : 1));
  ret.d[2] = exprBuilder.constant(1);
  ret.d[3] = exprBuilder.constant(1);
  return ret;
}

bool YoloLayerPlugin::supportsFormatCombination(
  int pos, const PluginTensorDesc * inOut, int nbInputs, int nbOutputs) noexcept
{
  (void)nbInputs;
  (void)nbOutputs;

  assert(nbInputs == 1);
  assert(nbOutputs == 3);
  assert(pos < 4);
  return inOut[pos].type == DataType::kFLOAT && inOut[pos].format == PluginFormat::kLINEAR;
}

void YoloLayerPlugin::configurePlugin(
  const DynamicPluginTensorDesc * in, int nbInput, const DynamicPluginTensorDesc * out,
  int nbOutput) noexcept
{
  (void)in;
  (void)nbInput;
  (void)out;
  (void)nbOutput;

  assert(nbInput == 1);
  assert(nbOutput == 3);
}

size_t YoloLayerPlugin::getWorkspaceSize(
  const PluginTensorDesc * inputs, int nbInputs, const PluginTensorDesc * outputs,
  int nbOutputs) const noexcept
{
  (void)nbInputs;
  (void)outputs;
  (void)nbOutputs;

  if (size < 0) {
    const int batch_size = inputs[0].dims.d[0];
    const int grid_width = inputs[0].dims.d[2];
    const int grid_height = inputs[0].dims.d[3];
    const int num_classes = inputs[0].dims.d[1] / num_anchors_ - 5;
    size = yoloLayer(
      batch_size, nullptr, nullptr, grid_width, grid_height, num_classes, num_anchors_, anchors_,
      width_, height_, scale_x_y_, score_thresh_, use_darknet_layer_, nullptr, 0, nullptr);
  }
  return size;
}

int YoloLayerPlugin::enqueue(
  const PluginTensorDesc * inputDesc, const PluginTensorDesc * outputDesc,
  const void * const * inputs, void * const * outputs, void * workspace,
  cudaStream_t stream) noexcept
{
  const int batch_size = inputDesc[0].dims.d[0];
  const int grid_width = inputDesc[0].dims.d[2];
  const int grid_height = inputDesc[0].dims.d[3];
  const int num_classes = inputDesc[0].dims.d[1] / num_anchors_ - 5;

  int status = -1;
  status = yoloLayer(
    batch_size, inputs, outputs, grid_width, grid_height, num_classes, num_anchors_, anchors_,
    width_, height_, scale_x_y_, score_thresh_, use_darknet_layer_, workspace,
    getWorkspaceSize(inputDesc, 1, outputDesc, 3), stream);
  return status;
}

YoloLayerPluginCreator::YoloLayerPluginCreator() {}

const char * YoloLayerPluginCreator::getPluginName() const noexcept
{
  return YOLO_LAYER_PLUGIN_NAME;
}

const char * YoloLayerPluginCreator::getPluginVersion() const noexcept
{
  return YOLO_LAYER_PLUGIN_VERSION;
}

const char * YoloLayerPluginCreator::getPluginNamespace() const noexcept
{
  return YOLO_LAYER_PLUGIN_NAMESPACE;
}

void YoloLayerPluginCreator::setPluginNamespace(const char * N) noexcept { (void)N; }
const PluginFieldCollection * YoloLayerPluginCreator::getFieldNames() noexcept { return nullptr; }

IPluginV2DynamicExt * YoloLayerPluginCreator::createPlugin(
  const char * name, const PluginFieldCollection * fc) noexcept
{
  (void)name;
  (void)fc;

  return nullptr;
}

IPluginV2DynamicExt * YoloLayerPluginCreator::deserializePlugin(
  const char * name, const void * serialData, size_t serialLength) noexcept
{
  (void)name;

  return new YoloLayerPlugin(serialData, serialLength);
}
}  // namespace yolo
