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

#ifndef NORMAL_DISTRIBUTIONS_TRANSFORM_PCL_GENERIC_H
#define NORMAL_DISTRIBUTIONS_TRANSFORM_PCL_GENERIC_H

#include "ndt/base.hpp"

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

#include <vector>

template <class PointSource, class PointTarget>
class NormalDistributionsTransformPCLGeneric
: public NormalDistributionsTransformBase<PointSource, PointTarget>
{
public:
  NormalDistributionsTransformPCLGeneric();
  ~NormalDistributionsTransformPCLGeneric() = default;

  void align(pcl::PointCloud<PointSource> & output, const Eigen::Matrix4f & guess) override;
  void setInputTarget(const boost::shared_ptr<pcl::PointCloud<PointTarget>> & map_ptr) override;
  void setInputSource(const boost::shared_ptr<pcl::PointCloud<PointSource>> & scan_ptr) override;

  void setMaximumIterations(int max_iter) override;
  void setResolution(float res) override;
  void setStepSize(double step_size) override;
  void setTransformationEpsilon(double trans_eps) override;

  int getMaximumIterations() override;
  int getFinalNumIteration() const override;
  float getResolution() const override;
  double getStepSize() const override;
  double getTransformationEpsilon() override;
  double getTransformationProbability() const override;
  double getFitnessScore() override;
  boost::shared_ptr<const pcl::PointCloud<PointTarget>> getInputTarget() const override;
  boost::shared_ptr<const pcl::PointCloud<PointSource>> getInputSource() const override;
  Eigen::Matrix4f getFinalTransformation() const override;
  std::vector<Eigen::Matrix4f> getFinalTransformationArray() const override;

  Eigen::Matrix<double, 6, 6> getHessian() const override;

  boost::shared_ptr<pcl::search::KdTree<PointTarget>> getSearchMethodTarget() const override;

private:
  boost::shared_ptr<pcl::NormalDistributionsTransform<PointSource, PointTarget>> ndt_ptr_;
};

#include "ndt/impl/pcl_generic.hpp"

#endif
