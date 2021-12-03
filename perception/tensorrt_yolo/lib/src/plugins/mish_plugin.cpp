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

#include <mish.hpp>
#include <mish_plugin.hpp>

#include <stdio.h>

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

using nvinfer1::DataType;
using nvinfer1::Dims;
using nvinfer1::DimsExprs;
using nvinfer1::DynamicPluginTensorDesc;
using nvinfer1::IExprBuilder;
using nvinfer1::IPluginV2DynamicExt;
using nvinfer1::PluginField;
using nvinfer1::PluginFieldCollection;
using nvinfer1::PluginFormat;
using nvinfer1::PluginTensorDesc;

namespace
{
const char * MISH_PLUGIN_VERSION{"1"};
const char * MISH_PLUGIN_NAME{"Mish_TRT"};

inline int64_t volume(const Dims & d)
{
  int64_t v = 1;
  for (int64_t i = 0; i < d.nbDims; i++) {
    v *= d.d[i];
  }
  return v;
}
}  // namespace

namespace yolo
{
MishPlugin::MishPlugin() {}

// create the plugin at runtime from a byte stream
MishPlugin::MishPlugin(const void * data, size_t length)
{
  (void)data;
  (void)length;
}

// IPluginV2 Methods

const char * MishPlugin::getPluginType() const noexcept { return MISH_PLUGIN_NAME; }

const char * MishPlugin::getPluginVersion() const noexcept { return MISH_PLUGIN_VERSION; }

int MishPlugin::getNbOutputs() const noexcept { return 1; }

int MishPlugin::initialize() noexcept { return 0; }

void MishPlugin::terminate() noexcept {}

size_t MishPlugin::getSerializationSize() const noexcept { return 0; }

void MishPlugin::serialize(void * buffer) const noexcept { (void)buffer; }

void MishPlugin::destroy() noexcept { delete this; }

void MishPlugin::setPluginNamespace(const char * pluginNamespace) noexcept
{
  mPluginNamespace = pluginNamespace;
}

const char * MishPlugin::getPluginNamespace() const noexcept { return mPluginNamespace; }

// IPluginV2Ext Methods

DataType MishPlugin::getOutputDataType(
  int index, const DataType * inputTypes, int nbInputs) const noexcept
{
  (void)index;
  (void)inputTypes;
  (void)nbInputs;

  assert(inputTypes[0] == DataType::kFLOAT);
  return inputTypes[0];
}

// IPluginV2DynamicExt Methods

IPluginV2DynamicExt * MishPlugin::clone() const noexcept
{
  auto plugin = new MishPlugin(*this);
  plugin->setPluginNamespace(mPluginNamespace);
  return plugin;
}

DimsExprs MishPlugin::getOutputDimensions(
  int outputIndex, const DimsExprs * inputs, int nbInputs, IExprBuilder & exprBuilder) noexcept
{
  (void)outputIndex;
  (void)nbInputs;
  (void)exprBuilder;

  return inputs[0];
}

bool MishPlugin::supportsFormatCombination(
  int pos, const PluginTensorDesc * inOut, int nbInputs, int nbOutputs) noexcept
{
  (void)nbInputs;
  (void)nbOutputs;

  return inOut[pos].type == DataType::kFLOAT && inOut[pos].format == PluginFormat::kLINEAR;
}

void MishPlugin::configurePlugin(
  const DynamicPluginTensorDesc * in, int nbInput, const DynamicPluginTensorDesc * out,
  int nbOutput) noexcept
{
  (void)in;
  (void)nbInput;
  (void)out;
  (void)nbOutput;

  assert(nbInput == 1);
  assert(nbOutput == 1);
}

size_t MishPlugin::getWorkspaceSize(
  const PluginTensorDesc * inputs, int nbInputs, const PluginTensorDesc * outputs,
  int nbOutputs) const noexcept
{
  (void)inputs;
  (void)nbInputs;
  (void)outputs;
  (void)nbOutputs;

  return 0;
}

int MishPlugin::enqueue(
  const PluginTensorDesc * inputDesc, const PluginTensorDesc * outputDesc,
  const void * const * inputs, void * const * outputs, void * workspace,
  cudaStream_t stream) noexcept
{
  (void)inputDesc;
  (void)outputDesc;
  (void)workspace;

  const int input_volume = volume(inputDesc[0].dims);

  int status = -1;

  const float * input = static_cast<const float *>(inputs[0]);
  float * output = static_cast<float *>(outputs[0]);
  status = mish(stream, input, output, input_volume);
  return status;
}

PluginFieldCollection MishPluginCreator::mFC{};
std::vector<PluginField> MishPluginCreator::mPluginAttributes;

MishPluginCreator::MishPluginCreator()
{
  mPluginAttributes.clear();

  mFC.nbFields = mPluginAttributes.size();
  mFC.fields = mPluginAttributes.data();
}

const char * MishPluginCreator::getPluginName() const noexcept { return MISH_PLUGIN_NAME; }

const char * MishPluginCreator::getPluginVersion() const noexcept { return MISH_PLUGIN_VERSION; }

const PluginFieldCollection * MishPluginCreator::getFieldNames() noexcept { return &mFC; }

IPluginV2DynamicExt * MishPluginCreator::createPlugin(
  const char * name, const PluginFieldCollection * fc) noexcept
{
  (void)name;
  (void)fc;

  MishPlugin * obj = new MishPlugin();
  obj->setPluginNamespace(mNamespace.c_str());
  return obj;
}

IPluginV2DynamicExt * MishPluginCreator::deserializePlugin(
  const char * name, const void * serialData, size_t serialLength) noexcept
{
  (void)name;

  // This object will be deleted when the network is destroyed, which will
  // call MishPlugin::destroy()
  MishPlugin * obj = new MishPlugin(serialData, serialLength);
  obj->setPluginNamespace(mNamespace.c_str());
  return obj;
}
}  // namespace yolo
