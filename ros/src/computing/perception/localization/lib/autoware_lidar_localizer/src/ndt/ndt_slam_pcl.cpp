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

#include "autoware_lidar_localizer/ndt/ndt_slam_pcl.h"

template <class PointSource, class PointTarget>
NdtSlamPCL<PointSource, PointTarget>::NdtSlamPCL()
    : ndt_ptr_(new pcl::NormalDistributionsTransformModified<PointSource, PointTarget>),
      swap_ndt_ptr_(ndt_ptr_) {}

template <class PointSource, class PointTarget>
void NdtSlamPCL<PointSource, PointTarget>::setTransformationEpsilon(
    double trans_eps) {
  ndt_ptr_->setTransformationEpsilon(trans_eps);
}

template <class PointSource, class PointTarget>
void NdtSlamPCL<PointSource, PointTarget>::setStepSize(double step_size) {
  ndt_ptr_->setStepSize(step_size);
}

template <class PointSource, class PointTarget>
void NdtSlamPCL<PointSource, PointTarget>::setResolution(float res) {
  ndt_ptr_->setResolution(res);
}

template <class PointSource, class PointTarget>
void NdtSlamPCL<PointSource, PointTarget>::setMaximumIterations(int max_iter) {
  ndt_ptr_->setMaximumIterations(max_iter);
}

template <class PointSource, class PointTarget>
double NdtSlamPCL<PointSource, PointTarget>::getTransformationEpsilon() {
  return ndt_ptr_->getTransformationEpsilon();
}

template <class PointSource, class PointTarget>
double NdtSlamPCL<PointSource, PointTarget>::getStepSize() const {
  return ndt_ptr_->getStepSize();
}

template <class PointSource, class PointTarget>
float NdtSlamPCL<PointSource, PointTarget>::getResolution() const {
  return ndt_ptr_->getResolution();
}

template <class PointSource, class PointTarget>
int NdtSlamPCL<PointSource, PointTarget>::getMaximumIterations() {
  return ndt_ptr_->getMaximumIterations();
}

template <class PointSource, class PointTarget>
double
NdtSlamPCL<PointSource, PointTarget>::getTransformationProbability() const {
  return ndt_ptr_->getTransformationProbability();
}

template <class PointSource, class PointTarget>
void NdtSlamPCL<PointSource, PointTarget>::align(const Pose &predict_pose) {
  const auto predict_matrix = convertToEigenMatrix4f(predict_pose);
  pcl::PointCloud<PointSource> output_cloud;
  ndt_ptr_->align(output_cloud, predict_matrix);
}

template <class PointSource, class PointTarget>
void NdtSlamPCL<PointSource, PointTarget>::setInputTarget(
    const boost::shared_ptr<pcl::PointCloud<PointTarget>> &map_ptr) {
  ndt_ptr_->setInputTarget(map_ptr);
}

template <class PointSource, class PointTarget>
void NdtSlamPCL<PointSource, PointTarget>::setInputSource(
    const boost::shared_ptr<pcl::PointCloud<PointSource>> &scan_ptr) {
  ndt_ptr_->setInputSource(scan_ptr);
}

template <class PointSource, class PointTarget>
double NdtSlamPCL<PointSource, PointTarget>::getFitnessScore() {
  return ndt_ptr_->getFitnessScore();
}

template <class PointSource, class PointTarget>
Pose NdtSlamPCL<PointSource, PointTarget>::getFinalPose() {
  return convertToPose(ndt_ptr_->getFinalTransformation());
}

template <class PointSource, class PointTarget>
void NdtSlamPCL<PointSource, PointTarget>::buildMap(const boost::shared_ptr<pcl::PointCloud<PointTarget>> &map_ptr) {
  const auto trans_estimation = getTransformationEpsilon();
  const auto step_size = getStepSize();
  const auto resolution = getResolution();
  const auto max_iter = getMaximumIterations();

  boost::shared_ptr<pcl::NormalDistributionsTransformModified<PointSource, PointTarget>>
      tmp_ndt_ptr(new pcl::NormalDistributionsTransformModified<PointSource, PointTarget>);
  tmp_ndt_ptr->setTransformationEpsilon(trans_estimation);
  tmp_ndt_ptr->setStepSize(step_size);
  tmp_ndt_ptr->setResolution(resolution);
  tmp_ndt_ptr->setMaximumIterations(max_iter);
  tmp_ndt_ptr->setInputTarget(map_ptr);

  const auto identity_matrix = Eigen::Matrix4f::Identity();
  pcl::PointCloud<PointSource> output_cloud;
  tmp_ndt_ptr->align(output_cloud, identity_matrix);

  swap_ndt_ptr_ = tmp_ndt_ptr;
}

template <class PointSource, class PointTarget>
void NdtSlamPCL<PointSource, PointTarget>::swapInstance() {
  ndt_ptr_ = swap_ndt_ptr_; //TODO 2ms~3ms
}

template <class PointSource, class PointTarget>
Eigen::Matrix<double, 6, 6> NdtSlamPCL<PointSource, PointTarget>::getHessian() const {
    return ndt_ptr_->getHessian();
}

template <class PointSource, class PointTarget>
boost::shared_ptr<pcl::search::KdTree<PointTarget>> NdtSlamPCL<PointSource, PointTarget>::getSearchMethodTarget() const {
    return ndt_ptr_->getSearchMethodTarget();
}

template class NdtSlamPCL<pcl::PointXYZ, pcl::PointXYZ>;
template class NdtSlamPCL<pcl::PointXYZI, pcl::PointXYZI>;
