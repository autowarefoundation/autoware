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

#ifndef NDT_SLAM_BASE_H
#define NDT_SLAM_BASE_H

#include "autoware_lidar_localizer/lidar_localizer.h"

#include <pcl/search/kdtree.h>

template <class PointSource, class PointTarget>
class NdtSlamBase : public LidarLocalizer<PointSource, PointTarget> {
public:
  NdtSlamBase();
  virtual ~NdtSlamBase() = default;

  virtual void setTransformationEpsilon(double trans_eps) = 0;
  virtual void setStepSize(double step_size) = 0;
  virtual void setResolution(float res) = 0;
  virtual void setMaximumIterations(int max_iter) = 0;

  virtual double getTransformationEpsilon() = 0;
  virtual double getStepSize() const = 0;
  virtual float getResolution() const = 0;
  virtual int getMaximumIterations() = 0;

  virtual double getTransformationProbability() const = 0;

  virtual double getFitnessScore() = 0;

  //TODO
  virtual Eigen::Matrix<double, 6, 6> getHessian() const {
      return Eigen::Matrix<double, 6, 6>::Identity();
  };

  //TODO
  virtual boost::shared_ptr<pcl::search::KdTree<PointTarget>> getSearchMethodTarget() const {
      return boost::make_shared<pcl::search::KdTree<PointTarget>>();
  };

};

#endif
