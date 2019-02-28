/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cnn_segmentation.h"

CNNSegmentation::CNNSegmentation() : nh_()
{
}

bool CNNSegmentation::init()
{
  std::string proto_file;
  std::string weight_file;

  ros::NodeHandle private_node_handle("~");//to receive args

  if (private_node_handle.getParam("network_definition_file", proto_file))
  {
    ROS_INFO("[%s] network_definition_file: %s", __APP_NAME__, proto_file.c_str());
  } else
  {
    ROS_INFO("[%s] No Network Definition File was received. Finishing execution.", __APP_NAME__);
    return false;
  }
  if (private_node_handle.getParam("pretrained_model_file", weight_file))
  {
    ROS_INFO("[%s] Pretrained Model File: %s", __APP_NAME__, weight_file.c_str());
  } else
  {
    ROS_INFO("[%s] No Pretrained Model File was received. Finishing execution.", __APP_NAME__);
    return false;
  }


  private_node_handle.param<std::string>("points_src", topic_src_, "points_raw");
  ROS_INFO("[%s] points_src: %s", __APP_NAME__, topic_src_.c_str());

  private_node_handle.param<double>("range", range_, 60.);
  ROS_INFO("[%s] Pretrained Model File: %.2f", __APP_NAME__, range_);

  private_node_handle.param<double>("score_threshold", score_threshold_, 0.6);
  ROS_INFO("[%s] score_threshold: %.2f", __APP_NAME__, score_threshold_);

  private_node_handle.param<int>("width", width_, 512);
  ROS_INFO("[%s] width: %d", __APP_NAME__, width_);

  private_node_handle.param<int>("height", height_, 512);
  ROS_INFO("[%s] height: %d", __APP_NAME__, height_);

  private_node_handle.param<bool>("use_gpu", use_gpu_, false);
  ROS_INFO("[%s] use_gpu: %d", __APP_NAME__, use_gpu_);

  private_node_handle.param<int>("gpu_device_id", gpu_device_id_, 0);
  ROS_INFO("[%s] gpu_device_id: %d", __APP_NAME__, gpu_device_id_);

  /// Instantiate Caffe net
  if (!use_gpu_)
  {
    caffe::Caffe::set_mode(caffe::Caffe::CPU);
  }
  else
  {
    caffe::Caffe::SetDevice(gpu_device_id_);
    caffe::Caffe::set_mode(caffe::Caffe::GPU);
    caffe::Caffe::DeviceQuery();
  }

  caffe_net_.reset(new caffe::Net<float>(proto_file, caffe::TEST));
  caffe_net_->CopyTrainedLayersFrom(weight_file);


  std::string instance_pt_blob_name = "instance_pt";
  instance_pt_blob_ = caffe_net_->blob_by_name(instance_pt_blob_name);
  CHECK(instance_pt_blob_ != nullptr) << "`" << instance_pt_blob_name
                                      << "` layer required";

  std::string category_pt_blob_name = "category_score";
  category_pt_blob_ = caffe_net_->blob_by_name(category_pt_blob_name);
  CHECK(category_pt_blob_ != nullptr) << "`" << category_pt_blob_name
                                      << "` layer required";

  std::string confidence_pt_blob_name = "confidence_score";
  confidence_pt_blob_ = caffe_net_->blob_by_name(confidence_pt_blob_name);
  CHECK(confidence_pt_blob_ != nullptr) << "`" << confidence_pt_blob_name
                                        << "` layer required";

  std::string height_pt_blob_name = "height_pt";
  height_pt_blob_ = caffe_net_->blob_by_name(height_pt_blob_name);
  CHECK(height_pt_blob_ != nullptr) << "`" << height_pt_blob_name
                                    << "` layer required";

  std::string feature_blob_name = "data";
  feature_blob_ = caffe_net_->blob_by_name(feature_blob_name);
  CHECK(feature_blob_ != nullptr) << "`" << feature_blob_name
                                  << "` layer required";

  std::string class_pt_blob_name = "class_score";
  class_pt_blob_ = caffe_net_->blob_by_name(class_pt_blob_name);
  CHECK(class_pt_blob_ != nullptr) << "`" << class_pt_blob_name
                                   << "` layer required";

  cluster2d_.reset(new Cluster2D());
  if (!cluster2d_->init(height_, width_, range_))
  {
    ROS_ERROR("[%s] Fail to Initialize cluster2d for CNNSegmentation", __APP_NAME__);
    return false;
  }

  feature_generator_.reset(new FeatureGenerator());
  if (!feature_generator_->init(feature_blob_.get()))
  {
    ROS_ERROR("[%s] Fail to Initialize feature generator for CNNSegmentation", __APP_NAME__);
    return false;
  }

  return true;
}

bool CNNSegmentation::segment(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_ptr,
                              const pcl::PointIndices &valid_idx,
                              autoware_msgs::DetectedObjectArray &objects)
{
  int num_pts = static_cast<int>(pc_ptr->points.size());
  if (num_pts == 0)
  {
    ROS_INFO("[%s] Empty point cloud.", __APP_NAME__);
    return true;
  }

  feature_generator_->generate(pc_ptr);

// network forward process
  caffe_net_->Forward();
#ifndef USE_CAFFE_GPU
//  caffe::Caffe::set_mode(caffe::Caffe::CPU);
#else
//  int gpu_id = 0;
//   caffe::Caffe::SetDevice(gpu_id);
//    caffe::Caffe::set_mode(caffe::Caffe::GPU);
//    caffe::Caffe::DeviceQuery();
#endif

  // clutser points and construct segments/objects
  float objectness_thresh = 0.5;
  bool use_all_grids_for_clustering = true;
  cluster2d_->cluster(*category_pt_blob_, *instance_pt_blob_, pc_ptr,
                      valid_idx, objectness_thresh,
                      use_all_grids_for_clustering);
  cluster2d_->filter(*confidence_pt_blob_, *height_pt_blob_);
  cluster2d_->classify(*class_pt_blob_);
  float confidence_thresh = score_threshold_;
  float height_thresh = 0.5;
  int min_pts_num = 3;
  cluster2d_->getObjects(confidence_thresh, height_thresh, min_pts_num,
                         objects, message_header_);
  return true;
}

void CNNSegmentation::test_run()
{
  std::string in_pcd_file = "uscar_12_1470770225_1470770492_1349.pcd";

  pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile(in_pcd_file, *in_pc_ptr);


  pcl::PointIndices valid_idx;
  auto &indices = valid_idx.indices;
  indices.resize(in_pc_ptr->size());
  std::iota(indices.begin(), indices.end(), 0);

  autoware_msgs::DetectedObjectArray objects;
  init();
  segment(in_pc_ptr, valid_idx, objects);


}

void CNNSegmentation::run()
{
  init();

  points_sub_ = nh_.subscribe(topic_src_, 1, &CNNSegmentation::pointsCallback, this);
  points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/detection/lidar_detector/points_cluster", 1);
  objects_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1);

  ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);
}

void CNNSegmentation::pointsCallback(const sensor_msgs::PointCloud2 &msg)
{
  std::chrono::system_clock::time_point start, end;
  start = std::chrono::system_clock::now();

  pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(msg, *in_pc_ptr);
  pcl::PointIndices valid_idx;
  auto &indices = valid_idx.indices;
  indices.resize(in_pc_ptr->size());
  std::iota(indices.begin(), indices.end(), 0);
  message_header_ = msg.header;

  autoware_msgs::DetectedObjectArray objects;
  objects.header = message_header_;
  segment(in_pc_ptr, valid_idx, objects);

  pubColoredPoints(objects);

  objects_pub_.publish(objects);

  end = std::chrono::system_clock::now();
  double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
}

void CNNSegmentation::pubColoredPoints(const autoware_msgs::DetectedObjectArray &objects_array)
{
  pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
  for (size_t object_i = 0; object_i < objects_array.objects.size(); object_i++)
  {
    // std::cout << "objct i" << object_i << std::endl;
    pcl::PointCloud<pcl::PointXYZI> object_cloud;
    pcl::fromROSMsg(objects_array.objects[object_i].pointcloud, object_cloud);
    int red = (object_i) % 256;
    int green = (object_i * 7) % 256;
    int blue = (object_i * 13) % 256;

    for (size_t i = 0; i < object_cloud.size(); i++)
    {
      // std::cout << "point i" << i << "/ size: "<<object_cloud.size()  << std::endl;
      pcl::PointXYZRGB colored_point;
      colored_point.x = object_cloud[i].x;
      colored_point.y = object_cloud[i].y;
      colored_point.z = object_cloud[i].z;
      colored_point.r = red;
      colored_point.g = green;
      colored_point.b = blue;
      colored_cloud.push_back(colored_point);
    }
  }
  sensor_msgs::PointCloud2 output_colored_cloud;
  pcl::toROSMsg(colored_cloud, output_colored_cloud);
  output_colored_cloud.header = message_header_;
  points_pub_.publish(output_colored_cloud);
}
