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

#include "grid_priors.hpp"

#include "grid_priors_kernel.hpp"
#include "trt_serialize.hpp"

#include <assert.h>

namespace ssd
{
namespace
{
static const char * PLUGIN_VERSION{"1"};
static const char * PLUGIN_NAME{"GridPriorsTRT"};
}  // namespace

GridPriors::GridPriors(const std::string & name, const nvinfer1::Dims & stride)
: mLayerName(name), mStride(stride)
{
}

GridPriors::GridPriors(const std::string & name, const void * data, size_t length)
: mLayerName(name)
{
  deserialize_value(&data, &length, &mStride);
}

// IPluginV2 methods
const char * GridPriors::getPluginVersion() const noexcept
{
  return PLUGIN_VERSION;
}

const char * GridPriors::getPluginType() const noexcept
{
  return PLUGIN_NAME;
}

int GridPriors::initialize() noexcept
{
  return STATUS_SUCCESS;
}

void GridPriors::terminate() noexcept
{
}

void GridPriors::destroy() noexcept
{
  delete this;
}

void GridPriors::setPluginNamespace(const char * pluginNamespace) noexcept
{
  mNamespace = pluginNamespace;
}

const char * GridPriors::getPluginNamespace() const noexcept
{
  return mNamespace.c_str();
}

int GridPriors::getNbOutputs() const noexcept
{
  return 1;
}

size_t GridPriors::getSerializationSize() const noexcept
{
  return serialized_size(mStride);
}

void GridPriors::serialize(void * buffer) const noexcept
{
  serialize_value(&buffer, mStride);
}

// IPluginV2Ext methods
nvinfer1::DataType GridPriors::getOutputDataType(
  int, const nvinfer1::DataType * inputTypes, int) const noexcept
{
  return inputTypes[0];
}

// IPluginV2DynamicExt methods
nvinfer1::IPluginV2DynamicExt * GridPriors::clone() const noexcept
{
  GridPriors * plugin = new GridPriors(mLayerName, mStride);
  plugin->setPluginNamespace(getPluginNamespace());
  return plugin;
}

nvinfer1::DimsExprs GridPriors::getOutputDimensions(
  int, const nvinfer1::DimsExprs * inputs, int, nvinfer1::IExprBuilder & exprBuilder) noexcept
{
  nvinfer1::DimsExprs ret;
  ret.nbDims = 2;
  auto area =
    exprBuilder.operation(nvinfer1::DimensionOperation::kPROD, *inputs[2].d[0], *inputs[1].d[0]);
  ret.d[0] = exprBuilder.operation(nvinfer1::DimensionOperation::kPROD, *area, *(inputs[0].d[0]));
  ret.d[1] = exprBuilder.constant(4);

  return ret;
}

bool GridPriors::supportsFormatCombination(
  int pos, const nvinfer1::PluginTensorDesc * ioDesc, int nbInputs, int) noexcept
{
  if (pos == 0) {
    return (
      ioDesc[pos].type == nvinfer1::DataType::kFLOAT &&
      ioDesc[pos].format == nvinfer1::TensorFormat::kLINEAR);
  } else if (pos == nbInputs) {
    return ioDesc[pos].type == ioDesc[0].type && ioDesc[pos].format == ioDesc[0].format;
  } else {
    return true;
  }
}

void GridPriors::configurePlugin(
  const nvinfer1::DynamicPluginTensorDesc *, int, const nvinfer1::DynamicPluginTensorDesc *,
  int) noexcept
{
}

size_t GridPriors::getWorkspaceSize(
  const nvinfer1::PluginTensorDesc *, int, const nvinfer1::PluginTensorDesc *, int) const noexcept
{
  return 0;
}

int GridPriors::enqueue(
  const nvinfer1::PluginTensorDesc * inputDesc, const nvinfer1::PluginTensorDesc *,
  const void * const * inputs, void * const * outputs, void *, cudaStream_t stream) noexcept
{
  int num_base_anchors = inputDesc[0].dims.d[0];
  int feat_h = inputDesc[1].dims.d[0];
  int feat_w = inputDesc[2].dims.d[0];

  const void * base_anchor = inputs[0];
  void * output = outputs[0];

  auto data_type = inputDesc[0].type;
  switch (data_type) {
    case nvinfer1::DataType::kFLOAT:
      grid_priors_impl<float>(
        reinterpret_cast<const float *>(base_anchor), reinterpret_cast<float *>(output),
        num_base_anchors, feat_w, feat_h, mStride.d[0], mStride.d[1], stream);
      break;
    default:
      return 1;
  }
  return 0;
}

// PluginCreator
GridPriorsCreator::GridPriorsCreator()
{
  mPluginAttributes.clear();
  mPluginAttributes.emplace_back(nvinfer1::PluginField("stride_h"));
  mPluginAttributes.emplace_back(nvinfer1::PluginField("stride_w"));
  mFC.nbFields = mPluginAttributes.size();
  mFC.fields = mPluginAttributes.data();
}

const char * GridPriorsCreator::getPluginName() const noexcept
{
  return PLUGIN_NAME;
}

const char * GridPriorsCreator::getPluginVersion() const noexcept
{
  return PLUGIN_VERSION;
}

const nvinfer1::PluginFieldCollection * GridPriorsCreator::getFieldNames() noexcept
{
  return &mFC;
}

void GridPriorsCreator::setPluginNamespace(const char * pluginNamespace) noexcept
{
  mNamespace = pluginNamespace;
}

const char * GridPriorsCreator::getPluginNamespace() const noexcept
{
  return mNamespace.c_str();
}

nvinfer1::IPluginV2 * GridPriorsCreator::createPlugin(
  const char * name, const nvinfer1::PluginFieldCollection * fc) noexcept
{
  int stride_h = 1;
  int stride_w = 1;

  for (int i = 0; i < fc->nbFields; ++i) {
    if (fc->fields[i].data == nullptr) {
      continue;
    }
    std::string field_name(fc->fields[i].name);

    if (field_name.compare("stride_w") == 0) {
      stride_w = static_cast<const int *>(fc->fields[i].data)[0];
    }
    if (field_name.compare("stride_h") == 0) {
      stride_h = static_cast<const int *>(fc->fields[i].data)[0];
    }
  }
  nvinfer1::Dims stride{2, {stride_w, stride_h}};

  GridPriors * plugin = new GridPriors(name, stride);
  plugin->setPluginNamespace(getPluginNamespace());
  return plugin;
}

nvinfer1::IPluginV2 * GridPriorsCreator::deserializePlugin(
  const char * name, const void * serialData, size_t serialLength) noexcept
{
  auto plugin = new GridPriors(name, serialData, serialLength);
  plugin->setPluginNamespace(getPluginNamespace());
  return plugin;
}

}  // namespace ssd
