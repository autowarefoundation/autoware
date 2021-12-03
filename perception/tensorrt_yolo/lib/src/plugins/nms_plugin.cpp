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

#include <nms.hpp>
#include <nms_plugin.hpp>

#include <cuda_runtime_api.h>
#include <stdio.h>
#include <string.h>

#include <cassert>
#include <cmath>

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
const char * NMS_PLUGIN_VERSION{"1"};
const char * NMS_PLUGIN_NAME{"YOLO_NMS_TRT"};
const char * NMS_PLUGIN_NAMESPACE{""};

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
NMSPlugin::NMSPlugin(float nms_thresh, int detections_per_im)
: nms_thresh_(nms_thresh), detections_per_im_(detections_per_im)
{
  assert(nms_thresh > 0);
  assert(detections_per_im > 0);
}

NMSPlugin::NMSPlugin(float nms_thresh, int detections_per_im, size_t count)
: nms_thresh_(nms_thresh), detections_per_im_(detections_per_im), count_(count)
{
  assert(nms_thresh > 0);
  assert(detections_per_im > 0);
  assert(count > 0);
}

NMSPlugin::NMSPlugin(void const * data, size_t length)
{
  (void)length;

  const char * d = static_cast<const char *>(data);
  read(d, nms_thresh_);
  read(d, detections_per_im_);
  read(d, count_);
}

const char * NMSPlugin::getPluginType() const noexcept { return NMS_PLUGIN_NAME; }

const char * NMSPlugin::getPluginVersion() const noexcept { return NMS_PLUGIN_VERSION; }

int NMSPlugin::getNbOutputs() const noexcept { return 3; }

int NMSPlugin::initialize() noexcept { return 0; }

void NMSPlugin::terminate() noexcept {}

size_t NMSPlugin::getSerializationSize() const noexcept
{
  return sizeof(nms_thresh_) + sizeof(detections_per_im_) + sizeof(count_);
}

void NMSPlugin::serialize(void * buffer) const noexcept
{
  char * d = static_cast<char *>(buffer);
  write(d, nms_thresh_);
  write(d, detections_per_im_);
  write(d, count_);
}

void NMSPlugin::destroy() noexcept { delete this; }

void NMSPlugin::setPluginNamespace(const char * N) noexcept { (void)N; }

const char * NMSPlugin::getPluginNamespace() const noexcept { return NMS_PLUGIN_NAMESPACE; }

// IPluginV2Ext Methods

DataType NMSPlugin::getOutputDataType(
  int index, const DataType * inputTypes, int nbInputs) const noexcept
{
  (void)index;
  (void)inputTypes;
  (void)nbInputs;

  assert(index < 3);
  return DataType::kFLOAT;
}

// IPluginV2DynamicExt Methods

IPluginV2DynamicExt * NMSPlugin::clone() const noexcept
{
  return new NMSPlugin(nms_thresh_, detections_per_im_, count_);
}

DimsExprs NMSPlugin::getOutputDimensions(
  int outputIndex, const DimsExprs * inputs, int nbInputs, IExprBuilder & exprBuilder) noexcept
{
  (void)nbInputs;

  DimsExprs output(inputs[0]);
  output.d[1] = exprBuilder.constant(detections_per_im_ * (outputIndex == 1 ? 4 : 1));
  output.d[2] = exprBuilder.constant(1);
  output.d[3] = exprBuilder.constant(1);
  return output;
}

bool NMSPlugin::supportsFormatCombination(
  int pos, const PluginTensorDesc * inOut, int nbInputs, int nbOutputs) noexcept
{
  (void)nbInputs;
  (void)nbOutputs;

  assert(nbInputs == 3);
  assert(nbOutputs == 3);
  assert(pos < 6);
  return inOut[pos].type == DataType::kFLOAT && inOut[pos].format == PluginFormat::kLINEAR;
}

void NMSPlugin::configurePlugin(
  const DynamicPluginTensorDesc * in, int nbInputs, const DynamicPluginTensorDesc * out,
  int nbOutputs) noexcept
{
  (void)nbInputs;
  (void)nbOutputs;
  (void)out;
  (void)nbOutputs;

  assert(nbInputs == 3);
  assert(in[0].desc.dims.d[1] == in[2].desc.dims.d[1]);
  assert(in[1].desc.dims.d[1] == in[2].desc.dims.d[1] * 4);
  count_ = in[0].desc.dims.d[1];
}

size_t NMSPlugin::getWorkspaceSize(
  const PluginTensorDesc * inputs, int nbInputs, const PluginTensorDesc * outputs,
  int nbOutputs) const noexcept
{
  (void)nbInputs;
  (void)outputs;
  (void)nbOutputs;

  if (size < 0) {
    size = nms(
      inputs->dims.d[0], nullptr, nullptr, count_, detections_per_im_, nms_thresh_, nullptr, 0,
      nullptr);
  }
  return size;
}

int NMSPlugin::enqueue(
  const PluginTensorDesc * inputDesc, const PluginTensorDesc * outputDesc,
  const void * const * inputs, void * const * outputs, void * workspace,
  cudaStream_t stream) noexcept
{
  return nms(
    inputDesc->dims.d[0], inputs, outputs, count_, detections_per_im_, nms_thresh_, workspace,
    getWorkspaceSize(inputDesc, 3, outputDesc, 3), stream);
}

NMSPluginCreator::NMSPluginCreator() {}

const char * NMSPluginCreator::getPluginName() const noexcept { return NMS_PLUGIN_NAME; }

const char * NMSPluginCreator::getPluginVersion() const noexcept { return NMS_PLUGIN_VERSION; }

const char * NMSPluginCreator::getPluginNamespace() const noexcept { return NMS_PLUGIN_NAMESPACE; }

void NMSPluginCreator::setPluginNamespace(const char * N) noexcept { (void)N; }
const PluginFieldCollection * NMSPluginCreator::getFieldNames() noexcept { return nullptr; }
IPluginV2DynamicExt * NMSPluginCreator::createPlugin(
  const char * name, const PluginFieldCollection * fc) noexcept
{
  (void)name;
  (void)fc;
  return nullptr;
}

IPluginV2DynamicExt * NMSPluginCreator::deserializePlugin(
  const char * name, const void * serialData, size_t serialLength) noexcept
{
  (void)name;

  return new NMSPlugin(serialData, serialLength);
}

}  // namespace yolo
