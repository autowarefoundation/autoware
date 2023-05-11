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

#ifndef GATHER_TOPK_HPP_
#define GATHER_TOPK_HPP_

#include "trt_plugin_helper.hpp"

#include <NvInferRuntime.h>
#include <NvInferVersion.h>
#include <cublas_v2.h>

#include <memory>
#include <string>
#include <vector>

namespace ssd
{
class GatherTopk : public nvinfer1::IPluginV2DynamicExt
{
private:
  const std::string mLayerName;
  std::string mNamespace;

public:
  explicit GatherTopk(const std::string & name);
  GatherTopk(const std::string & name, const void * data, size_t length);
  GatherTopk() = delete;

  // IPluginV2 methods
  const char * getPluginVersion() const noexcept override;
  const char * getPluginType() const noexcept override;
  int initialize() noexcept override;
  void terminate() noexcept override;
  void destroy() noexcept override;
  void setPluginNamespace(const char * pluginNamespace) noexcept override;
  const char * getPluginNamespace() const noexcept override;
  int getNbOutputs() const noexcept override;
  size_t getSerializationSize() const noexcept override;
  void serialize(void * buffer) const noexcept override;

  // IPluginV2Ext methods
  nvinfer1::DataType getOutputDataType(
    int index, const nvinfer1::DataType * inputTypes, int nbInputs) const noexcept override;

  // IPluginV2DynamicExt methods
  nvinfer1::IPluginV2DynamicExt * clone() const noexcept override;
  nvinfer1::DimsExprs getOutputDimensions(
    int outputIndex, const nvinfer1::DimsExprs * inputs, int nbInputs,
    nvinfer1::IExprBuilder & exprBuilder) noexcept override;
  bool supportsFormatCombination(
    int pos, const nvinfer1::PluginTensorDesc * ioDesc, int nbInputs,
    int nbOutputs) noexcept override;
  void configurePlugin(
    const nvinfer1::DynamicPluginTensorDesc * inputs, int nbInputs,
    const nvinfer1::DynamicPluginTensorDesc * outputs, int nbOutputs) noexcept override;
  size_t getWorkspaceSize(
    const nvinfer1::PluginTensorDesc *, int, const nvinfer1::PluginTensorDesc *,
    int) const noexcept override;
  int enqueue(
    const nvinfer1::PluginTensorDesc *, const nvinfer1::PluginTensorDesc *, const void * const *,
    void * const *, void *, cudaStream_t) noexcept override;
};  // class GatherTopk

class GatherTopkCreator : public nvinfer1::IPluginCreator
{
private:
  nvinfer1::PluginFieldCollection mFC;
  std::vector<nvinfer1::PluginField> mPluginAttributes;
  std::string mNamespace;

public:
  GatherTopkCreator();
  const char * getPluginVersion() const noexcept override;
  const nvinfer1::PluginFieldCollection * getFieldNames() noexcept override;
  void setPluginNamespace(const char * pluginNamespace) noexcept override;
  const char * getPluginNamespace() const noexcept override;
  const char * getPluginName() const noexcept override;
  nvinfer1::IPluginV2 * createPlugin(
    const char * name, const nvinfer1::PluginFieldCollection * fc) noexcept override;
  nvinfer1::IPluginV2 * deserializePlugin(
    const char * name, const void * serialData, size_t serialLength) noexcept override;
};  // class GatherTopkCreator

REGISTER_TENSORRT_PLUGIN(GatherTopkCreator);

}  // namespace ssd

#endif  // GATHER_TOPK_HPP_
