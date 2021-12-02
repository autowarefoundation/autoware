// Copyright 2015-2019 Autoware Foundation
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

#ifndef NORMAL_DISTRIBUTIONS_TRANSFORM_OMP_HPP
#define NORMAL_DISTRIBUTIONS_TRANSFORM_OMP_HPP

#include "ndt/omp.hpp"

#include <vector>

template <class PointSource, class PointTarget>
NormalDistributionsTransformOMP<PointSource, PointTarget>::NormalDistributionsTransformOMP()
: ndt_ptr_(new pclomp::NormalDistributionsTransform<PointSource, PointTarget>)
{
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMP<PointSource, PointTarget>::align(
  pcl::PointCloud<PointSource> & output, const Eigen::Matrix4f & guess)
{
  ndt_ptr_->align(output, guess);
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMP<PointSource, PointTarget>::setInputTarget(
  const boost::shared_ptr<pcl::PointCloud<PointTarget>> & map_ptr)
{
  ndt_ptr_->setInputTarget(map_ptr);
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMP<PointSource, PointTarget>::setInputSource(
  const boost::shared_ptr<pcl::PointCloud<PointSource>> & scan_ptr)
{
  ndt_ptr_->setInputSource(scan_ptr);
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMP<PointSource, PointTarget>::setMaximumIterations(int max_iter)
{
  ndt_ptr_->setMaximumIterations(max_iter);
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMP<PointSource, PointTarget>::setResolution(float res)
{
  ndt_ptr_->setResolution(res);
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMP<PointSource, PointTarget>::setStepSize(double step_size)
{
  ndt_ptr_->setStepSize(step_size);
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMP<PointSource, PointTarget>::setTransformationEpsilon(
  double trans_eps)
{
  ndt_ptr_->setTransformationEpsilon(trans_eps);
}

template <class PointSource, class PointTarget>
int NormalDistributionsTransformOMP<PointSource, PointTarget>::getMaximumIterations()
{
  return ndt_ptr_->getMaximumIterations();
}

template <class PointSource, class PointTarget>
int NormalDistributionsTransformOMP<PointSource, PointTarget>::getFinalNumIteration() const
{
  return ndt_ptr_->getFinalNumIteration();
}

template <class PointSource, class PointTarget>
float NormalDistributionsTransformOMP<PointSource, PointTarget>::getResolution() const
{
  return ndt_ptr_->getResolution();
}

template <class PointSource, class PointTarget>
double NormalDistributionsTransformOMP<PointSource, PointTarget>::getStepSize() const
{
  return ndt_ptr_->getStepSize();
}

template <class PointSource, class PointTarget>
double NormalDistributionsTransformOMP<PointSource, PointTarget>::getTransformationEpsilon()
{
  return ndt_ptr_->getTransformationEpsilon();
}

template <class PointSource, class PointTarget>
double NormalDistributionsTransformOMP<PointSource, PointTarget>::getTransformationProbability()
  const
{
  return ndt_ptr_->getTransformationProbability();
}

template <class PointSource, class PointTarget>
double NormalDistributionsTransformOMP<PointSource, PointTarget>::getFitnessScore()
{
  return ndt_ptr_->getFitnessScore();
}

template <class PointSource, class PointTarget>
boost::shared_ptr<const pcl::PointCloud<PointTarget>>
NormalDistributionsTransformOMP<PointSource, PointTarget>::getInputTarget() const
{
  return ndt_ptr_->getInputTarget();
}

template <class PointSource, class PointTarget>
boost::shared_ptr<const pcl::PointCloud<PointSource>>
NormalDistributionsTransformOMP<PointSource, PointTarget>::getInputSource() const
{
  return ndt_ptr_->getInputSource();
}

template <class PointSource, class PointTarget>
Eigen::Matrix4f NormalDistributionsTransformOMP<PointSource, PointTarget>::getFinalTransformation()
  const
{
  return ndt_ptr_->getFinalTransformation();
}

template <class PointSource, class PointTarget>
std::vector<Eigen::Matrix4f>
NormalDistributionsTransformOMP<PointSource, PointTarget>::getFinalTransformationArray() const
{
  return ndt_ptr_->getFinalTransformationArray();
}

template <class PointSource, class PointTarget>
Eigen::Matrix<double, 6, 6> NormalDistributionsTransformOMP<PointSource, PointTarget>::getHessian()
  const
{
  // return ndt_ptr_->getHessian();
  return Eigen::Matrix<double, 6, 6>();
}

template <class PointSource, class PointTarget>
boost::shared_ptr<pcl::search::KdTree<PointTarget>>
NormalDistributionsTransformOMP<PointSource, PointTarget>::getSearchMethodTarget() const
{
  return ndt_ptr_->getSearchMethodTarget();
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMP<PointSource, PointTarget>::setNumThreads(int n)
{
  ndt_ptr_->setNumThreads(n);
}

template <class PointSource, class PointTarget>
void NormalDistributionsTransformOMP<PointSource, PointTarget>::setNeighborhoodSearchMethod(
  pclomp::NeighborSearchMethod method)
{
  ndt_ptr_->setNeighborhoodSearchMethod(method);
}

template <class PointSource, class PointTarget>
int NormalDistributionsTransformOMP<PointSource, PointTarget>::getNumThreads() const
{
  return ndt_ptr_->getNumThreads();
}

template <class PointSource, class PointTarget>
pclomp::NeighborSearchMethod
NormalDistributionsTransformOMP<PointSource, PointTarget>::getNeighborhoodSearchMethod() const
{
  return ndt_ptr_->getNeighborhoodSearchMethod();
}

#endif  // NORMAL_DISTRIBUTIONS_TRANSFORM_OMP_HPP
