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

#ifndef CPU_REG_H_
#define CPU_REG_H_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace cpu {

template <typename PointSourceType, typename PointTargetType>
class Registration {
public:
	Registration();

	void align(const Eigen::Matrix<float, 4, 4> &guess);

	void align(typename pcl::PointCloud<PointSourceType> &output, const Eigen::Matrix<float, 4, 4> &guess);

	void setTransformationEpsilon(double trans_eps);

	double getTransformationEpsilon() const;

	void setMaximumIterations(int max_itr);

	int getMaximumIterations() const;

	Eigen::Matrix<float, 4, 4> getFinalTransformation() const;

	/* Set input Scanned point cloud.
	 * Simply set the point cloud input_ */
	void setInputSource(typename pcl::PointCloud<PointSourceType>::Ptr input);

	/* Set input reference map point cloud.
	 * Set the target point cloud ptr target_cloud_ */
	void setInputTarget(typename pcl::PointCloud<PointTargetType>::Ptr input);

	int getFinalNumIteration() const;

	bool hasConverged() const;

	void updateInputTarget(typename pcl::PointCloud<PointTargetType>::Ptr new_cloud);

	virtual ~Registration();

protected:

	virtual void computeTransformation(const Eigen::Matrix<float, 4, 4> &guess);

	double transformation_epsilon_;
	int max_iterations_;

	//Original scanned point clouds
	typename pcl::PointCloud<PointSourceType>::Ptr source_cloud_;

	//Transformed point clouds
	typename pcl::PointCloud<PointSourceType> trans_cloud_;

	bool converged_;
	int nr_iterations_;

	Eigen::Matrix<float, 4, 4> final_transformation_, transformation_, previous_transformation_;

	bool target_cloud_updated_;

	// Reference map point
	typename pcl::PointCloud<PointTargetType>::Ptr target_cloud_;
};
}

#endif
