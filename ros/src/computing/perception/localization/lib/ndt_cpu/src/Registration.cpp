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
/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "ndt_cpu/Registration.h"
#include "ndt_cpu/debug.h"
#include <iostream>

namespace cpu {

template <typename PointSourceType, typename PointTargetType>
Registration<PointSourceType, PointTargetType>::Registration()
{
	max_iterations_ = 0;

	converged_ = false;
	nr_iterations_ = 0;

	transformation_epsilon_ = 0;
	target_cloud_updated_ = true;

	trans_cloud_.points.clear();
}

template <typename PointSourceType, typename PointTargetType>
Registration<PointSourceType, PointTargetType>::~Registration()
{
	return;
}

template <typename PointSourceType, typename PointTargetType>
void Registration<PointSourceType, PointTargetType>::setTransformationEpsilon(double trans_eps)
{
	transformation_epsilon_ = trans_eps;
}

template <typename PointSourceType, typename PointTargetType>
double Registration<PointSourceType, PointTargetType>::getTransformationEpsilon() const
{
	return transformation_epsilon_;
}

template <typename PointSourceType, typename PointTargetType>
void Registration<PointSourceType, PointTargetType>::setMaximumIterations(int max_itr)
{
	max_iterations_ = max_itr;
}

template <typename PointSourceType, typename PointTargetType>
int Registration<PointSourceType, PointTargetType>::getMaximumIterations() const
{
	return max_iterations_;
}

template <typename PointSourceType, typename PointTargetType>
Eigen::Matrix<float, 4, 4> Registration<PointSourceType, PointTargetType>::getFinalTransformation() const
{
	return final_transformation_;
}

template <typename PointSourceType, typename PointTargetType>
int Registration<PointSourceType, PointTargetType>::getFinalNumIteration() const
{
	return nr_iterations_;
}

template <typename PointSourceType, typename PointTargetType>
bool Registration<PointSourceType, PointTargetType>::hasConverged() const
{
	return converged_;
}

template <typename PointSourceType, typename PointTargetType>
void Registration<PointSourceType, PointTargetType>::setInputSource(typename pcl::PointCloud<PointSourceType>::Ptr input)
{
	source_cloud_ = input;
}


//Set input MAP data
template <typename PointSourceType, typename PointTargetType>
void Registration<PointSourceType, PointTargetType>::setInputTarget(typename pcl::PointCloud<PointTargetType>::Ptr input)
{
	target_cloud_ = input;
}

template <typename PointSourceType, typename PointTargetType>
void Registration<PointSourceType, PointTargetType>::align(const Eigen::Matrix<float, 4, 4> &guess)
{
	converged_ = false;

	final_transformation_ = transformation_ = previous_transformation_ = Eigen::Matrix<float, 4, 4>::Identity();

	trans_cloud_.points.resize(source_cloud_->points.size());

	for (int i = 0; i < trans_cloud_.points.size(); i++) {
		trans_cloud_.points[i] = source_cloud_->points[i];
	}

	computeTransformation(guess);
}

template <typename PointSourceType, typename PointTargetType>
void Registration<PointSourceType, PointTargetType>::align(typename pcl::PointCloud<PointSourceType> &output, const Eigen::Matrix<float, 4, 4> &guess)
{
	align(guess);
}

template <typename PointSourceType, typename PointTargetType>
void Registration<PointSourceType, PointTargetType>::computeTransformation(const Eigen::Matrix<float, 4, 4> &guess) {
	printf("Unsupported by Registration\n");
}

template class Registration<pcl::PointXYZI, pcl::PointXYZI>;
template class Registration<pcl::PointXYZ, pcl::PointXYZ>;
}
