// Copyright 2020 TierIV
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

#include "lidar_apollo_instance_segmentation/detector.hpp"

#include "lidar_apollo_instance_segmentation/feature_map.hpp"

#include <NvCaffeParser.h>
#include <NvInfer.h>
#include <pcl_conversions/pcl_conversions.h>

LidarApolloInstanceSegmentation::LidarApolloInstanceSegmentation(rclcpp::Node * node)
: node_(node), tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_)
{
  int range, width, height;
  bool use_intensity_feature, use_constant_feature;
  std::string engine_file;
  std::string prototxt_file;
  std::string caffemodel_file;
  score_threshold_ = node_->declare_parameter("score_threshold", 0.8);
  range = node_->declare_parameter("range", 60);
  width = node_->declare_parameter("width", 640);
  height = node_->declare_parameter("height", 640);
  engine_file = node_->declare_parameter("engine_file", "vls-128.engine");
  prototxt_file = node_->declare_parameter("prototxt_file", "vls-128.prototxt");
  caffemodel_file = node_->declare_parameter("caffemodel_file", "vls-128.caffemodel");
  use_intensity_feature = node_->declare_parameter("use_intensity_feature", true);
  use_constant_feature = node_->declare_parameter("use_constant_feature", true);
  target_frame_ = node_->declare_parameter("target_frame", "base_link");
  z_offset_ = node_->declare_parameter<float>("z_offset", -2.0);

  // load weight file
  std::ifstream fs(engine_file);
  if (!fs.is_open()) {
    RCLCPP_INFO(
      node_->get_logger(),
      "Could not find %s. try making TensorRT engine from caffemodel and prototxt",
      engine_file.c_str());
    Tn::Logger logger;
    nvinfer1::IBuilder * builder = nvinfer1::createInferBuilder(logger);
    nvinfer1::INetworkDefinition * network = builder->createNetworkV2(0U);
    nvcaffeparser1::ICaffeParser * parser = nvcaffeparser1::createCaffeParser();
    nvinfer1::IBuilderConfig * config = builder->createBuilderConfig();
    const nvcaffeparser1::IBlobNameToTensor * blob_name2tensor = parser->parse(
      prototxt_file.c_str(), caffemodel_file.c_str(), *network, nvinfer1::DataType::kFLOAT);
    std::string output_node = "deconv0";
    auto output = blob_name2tensor->find(output_node.c_str());
    if (output == nullptr) {
      RCLCPP_ERROR(node_->get_logger(), "can not find output named %s", output_node.c_str());
    }
    network->markOutput(*output);
#if (NV_TENSORRT_MAJOR * 1000) + (NV_TENSORRT_MINOR * 100) + NV_TENSOR_PATCH >= 8400
    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, 1 << 30);
#else
    config->setMaxWorkspaceSize(1 << 30);
#endif
    nvinfer1::IHostMemory * plan = builder->buildSerializedNetwork(*network, *config);
    assert(plan != nullptr);
    std::ofstream outfile(engine_file, std::ofstream::binary);
    assert(!outfile.fail());
    outfile.write(reinterpret_cast<char *>(plan->data()), plan->size());
    outfile.close();
    if (network) {
      delete network;
    }
    if (parser) {
      delete parser;
    }
    if (builder) {
      delete builder;
    }
    if (config) {
      delete config;
    }
    if (plan) {
      delete plan;
    }
  }
  net_ptr_.reset(new Tn::trtNet(engine_file));

  // feature map generator: pre process
  feature_generator_ = std::make_shared<FeatureGenerator>(
    width, height, range, use_intensity_feature, use_constant_feature);

  // cluster: post process
  cluster2d_ = std::make_shared<Cluster2D>(width, height, range);
}

bool LidarApolloInstanceSegmentation::transformCloud(
  const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & transformed_cloud,
  float z_offset)
{
  // TODO(mitsudome-r): remove conversion once pcl_ros transform are available.
  pcl::PointCloud<pcl::PointXYZI> pcl_input, pcl_transformed_cloud;
  pcl::fromROSMsg(input, pcl_input);

  // transform pointcloud to target_frame
  if (target_frame_ != input.header.frame_id) {
    try {
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_.lookupTransform(
        target_frame_, input.header.frame_id, input.header.stamp, std::chrono::milliseconds(500));
      Eigen::Matrix4f affine_matrix =
        tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
      pcl::transformPointCloud(pcl_input, pcl_transformed_cloud, affine_matrix);
      transformed_cloud.header.frame_id = target_frame_;
      pcl_transformed_cloud.header.frame_id = target_frame_;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "%s", ex.what());
      return false;
    }
  } else {
    pcl_transformed_cloud = pcl_input;
  }

  // move pointcloud z_offset in z axis
  pcl::PointCloud<pcl::PointXYZI> pointcloud_with_z_offset;
  Eigen::Affine3f z_up_translation(Eigen::Translation3f(0, 0, z_offset));
  Eigen::Matrix4f z_up_transform = z_up_translation.matrix();
  pcl::transformPointCloud(pcl_transformed_cloud, pcl_transformed_cloud, z_up_transform);

  pcl::toROSMsg(pcl_transformed_cloud, transformed_cloud);

  return true;
}

bool LidarApolloInstanceSegmentation::detectDynamicObjects(
  const sensor_msgs::msg::PointCloud2 & input,
  tier4_perception_msgs::msg::DetectedObjectsWithFeature & output)
{
  // move up pointcloud z_offset in z axis
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  transformCloud(input, transformed_cloud, z_offset_);

  // convert from ros to pcl
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud_raw_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(transformed_cloud, *pcl_pointcloud_raw_ptr);

  // generate feature map
  std::shared_ptr<FeatureMapInterface> feature_map_ptr =
    feature_generator_->generate(pcl_pointcloud_raw_ptr);

  // inference
  std::shared_ptr<float> inferred_data(new float[net_ptr_->getOutputSize() / sizeof(float)]);
  net_ptr_->doInference(feature_map_ptr->map_data.data(), inferred_data.get());

  // post process
  const float objectness_thresh = 0.5;
  pcl::PointIndices valid_idx;
  valid_idx.indices.resize(pcl_pointcloud_raw_ptr->size());
  std::iota(valid_idx.indices.begin(), valid_idx.indices.end(), 0);
  cluster2d_->cluster(
    inferred_data, pcl_pointcloud_raw_ptr, valid_idx, objectness_thresh,
    true /*use all grids for clustering*/);
  const float height_thresh = 0.5;
  const int min_pts_num = 3;
  cluster2d_->getObjects(
    score_threshold_, height_thresh, min_pts_num, output, transformed_cloud.header);

  // move down pointcloud z_offset in z axis
  for (std::size_t i = 0; i < output.feature_objects.size(); i++) {
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    transformCloud(output.feature_objects.at(i).feature.cluster, transformed_cloud, -z_offset_);
    output.feature_objects.at(i).feature.cluster = transformed_cloud;
  }

  output.header = transformed_cloud.header;
  return true;
}
