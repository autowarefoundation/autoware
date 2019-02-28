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

#include "lidar_localizer/ndt/ndt_slam_dummy.h"

template <class PointSource, class PointTarget>
void NdtSlamDummy<PointSource, PointTarget>::setTransformationEpsilon(double trans_eps)
{
}

template <class PointSource, class PointTarget>
void NdtSlamDummy<PointSource, PointTarget>::setStepSize(double step_size)
{
}

template <class PointSource, class PointTarget>
void NdtSlamDummy<PointSource, PointTarget>::setResolution(float res)
{
}

template <class PointSource, class PointTarget>
void NdtSlamDummy<PointSource, PointTarget>::setMaximumIterations(int max_iter)
{
}

template <class PointSource, class PointTarget>
double NdtSlamDummy<PointSource, PointTarget>::getTransformationEpsilon()
{
  return 0;
}

template <class PointSource, class PointTarget>
double NdtSlamDummy<PointSource, PointTarget>::getStepSize() const
{
  return 0;
}

template <class PointSource, class PointTarget>
float NdtSlamDummy<PointSource, PointTarget>::getResolution() const
{
  return 0;
}

template <class PointSource, class PointTarget>
int NdtSlamDummy<PointSource, PointTarget>::getMaximumIterations()
{
  return 0;
}

template <class PointSource, class PointTarget>
double NdtSlamDummy<PointSource, PointTarget>::getTransformationProbability() const
{
  return 0;
}

template <class PointSource, class PointTarget>
void NdtSlamDummy<PointSource, PointTarget>::align(const Pose& predict_pose)
{
}

template <class PointSource, class PointTarget>
double NdtSlamDummy<PointSource, PointTarget>::getFitnessScore()
{
  return 0;
}

template <class PointSource, class PointTarget>
void NdtSlamDummy<PointSource, PointTarget>::setInputTarget(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& map_ptr)
{
}

template <class PointSource, class PointTarget>
void NdtSlamDummy<PointSource, PointTarget>::setInputSource(const boost::shared_ptr< pcl::PointCloud<PointSource> >& scan_ptr)
{
}

template <class PointSource, class PointTarget>
Pose NdtSlamDummy<PointSource, PointTarget>::getFinalPose()
{
  return Pose();
}

template class NdtSlamDummy<pcl::PointXYZ, pcl::PointXYZ>;
template class NdtSlamDummy<pcl::PointXYZI, pcl::PointXYZI>;
