/*
 *  Copyright (c) 2017, Tier IV, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
void NdtSlamDummy<PointSource, PointTarget>::setInputTarget(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_ptr)
{
}

template <class PointSource, class PointTarget>
void NdtSlamDummy<PointSource, PointTarget>::setInputSource(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& scan_ptr)
{
}

template <class PointSource, class PointTarget>
Pose NdtSlamDummy<PointSource, PointTarget>::getFinalPose()
{
  return Pose();
}

template class NdtSlamDummy<pcl::PointXYZ, pcl::PointXYZ>;
template class NdtSlamDummy<pcl::PointXYZI, pcl::PointXYZI>;
