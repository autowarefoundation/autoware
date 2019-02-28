/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include "lidar_localizer/ndt/ndt_slam_pcl_anh_gpu.h"

#ifndef CUDA_FOUND

template <class PointSource, class PointTarget>
NdtSlamPCLANHGPU<PointSource, PointTarget>::NdtSlamPCLANHGPU() {
  std::cerr << "**************************************************************"
            << std::endl;
  std::cerr << "[ERROR]PCL_ANH_GPU is not built. Please use other method type."
            << std::endl;
  std::cerr << "**************************************************************"
            << std::endl;
  exit(1);
}

#else

template <class PointSource, class PointTarget>
NdtSlamPCLANHGPU<PointSource, PointTarget>::NdtSlamPCLANHGPU()
    : ndt_ptr_(new gpu::GNormalDistributionsTransform),
      swap_ndt_ptr_(ndt_ptr_) {}

template <class PointSource, class PointTarget>
void NdtSlamPCLANHGPU<PointSource, PointTarget>::setTransformationEpsilon(
    double trans_eps) {
  ndt_ptr_->setTransformationEpsilon(trans_eps);
}

template <class PointSource, class PointTarget>
void NdtSlamPCLANHGPU<PointSource, PointTarget>::setStepSize(double step_size) {
  ndt_ptr_->setStepSize(step_size);
}

template <class PointSource, class PointTarget>
void NdtSlamPCLANHGPU<PointSource, PointTarget>::setResolution(float res) {
  ndt_ptr_->setResolution(res);
}

template <class PointSource, class PointTarget>
void NdtSlamPCLANHGPU<PointSource, PointTarget>::setMaximumIterations(
    int max_iter) {
  ndt_ptr_->setMaximumIterations(max_iter);
}

template <class PointSource, class PointTarget>
double NdtSlamPCLANHGPU<PointSource, PointTarget>::getTransformationEpsilon() {
  return ndt_ptr_->getTransformationEpsilon();
}

template <class PointSource, class PointTarget>
double NdtSlamPCLANHGPU<PointSource, PointTarget>::getStepSize() const {
  return ndt_ptr_->getStepSize();
}

template <class PointSource, class PointTarget>
float NdtSlamPCLANHGPU<PointSource, PointTarget>::getResolution() const {
  return ndt_ptr_->getResolution();
}

template <class PointSource, class PointTarget>
int NdtSlamPCLANHGPU<PointSource, PointTarget>::getMaximumIterations() {
  return ndt_ptr_->getMaximumIterations();
}

template <class PointSource, class PointTarget>
double NdtSlamPCLANHGPU<PointSource,
                        PointTarget>::getTransformationProbability() const {
  return ndt_ptr_->getTransformationProbability();
}

template <class PointSource, class PointTarget>
void NdtSlamPCLANHGPU<PointSource, PointTarget>::align(
    const Pose &predict_pose) {
  const auto predict_matrix = convertToEigenMatrix4f(predict_pose);
  ndt_ptr_->align(predict_matrix);
}

template <class PointSource, class PointTarget>
void NdtSlamPCLANHGPU<PointSource, PointTarget>::setInputTarget(
    const boost::shared_ptr<pcl::PointCloud<PointTarget>> &map_ptr) {
  ndt_ptr_->setInputTarget(map_ptr);
}

template <class PointSource, class PointTarget>
void NdtSlamPCLANHGPU<PointSource, PointTarget>::setInputSource(
    const boost::shared_ptr<pcl::PointCloud<PointSource>> &scan_ptr) {
  ndt_ptr_->setInputSource(scan_ptr);
}

template <class PointSource, class PointTarget>
double NdtSlamPCLANHGPU<PointSource, PointTarget>::getFitnessScore() {
  return ndt_ptr_->getFitnessScore();
}

template <class PointSource, class PointTarget>
Pose NdtSlamPCLANHGPU<PointSource, PointTarget>::getFinalPose() {
  return convertToPose(ndt_ptr_->getFinalTransformation());
}

template <class PointSource, class PointTarget>
void NdtSlamPCLANHGPU<PointSource, PointTarget>::buildMap(
    const boost::shared_ptr<pcl::PointCloud<PointTarget>> &map_ptr) {
  const auto trans_estimation = getTransformationEpsilon();
  const auto step_size = getStepSize();
  const auto resolution = getResolution();
  const auto max_iter = getMaximumIterations();

  boost::shared_ptr<gpu::GNormalDistributionsTransform> tmp_ndt_ptr(
      new gpu::GNormalDistributionsTransform);
  tmp_ndt_ptr->setTransformationEpsilon(trans_estimation);
  tmp_ndt_ptr->setStepSize(step_size);
  tmp_ndt_ptr->setResolution(resolution);
  tmp_ndt_ptr->setMaximumIterations(max_iter);
  tmp_ndt_ptr->setInputTarget(map_ptr);

  boost::shared_ptr<pcl::PointCloud<PointSource>> dummy_scan_ptr(
      new pcl::PointCloud<PointSource>());
  PointSource dummy_point;
  dummy_scan_ptr->push_back(dummy_point);
  tmp_ndt_ptr->setInputSource(dummy_scan_ptr);

  const auto identity_matrix = Eigen::Matrix4f::Identity();
  tmp_ndt_ptr->align(identity_matrix);

  swap_ndt_ptr_ = tmp_ndt_ptr;
}

template <class PointSource, class PointTarget>
void NdtSlamPCLANHGPU<PointSource, PointTarget>::swapInstance() {
  ndt_ptr_ = swap_ndt_ptr_;
}

#endif

template class NdtSlamPCLANHGPU<pcl::PointXYZ, pcl::PointXYZ>;
template class NdtSlamPCLANHGPU<pcl::PointXYZI, pcl::PointXYZI>;
