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

#ifndef CPU_NDT_H_
#define CPU_NDT_H_

#include "Registration.h"
#include "VoxelGrid.h"
#include <eigen3/Eigen/Geometry>

namespace cpu {

template <typename PointSourceType, typename PointTargetType>
class NormalDistributionsTransform: public Registration<PointSourceType, PointTargetType> {
public:
	NormalDistributionsTransform();

	NormalDistributionsTransform(const NormalDistributionsTransform &other);

	void setStepSize(double step_size);

	void setResolution(float resolution);

	void setOutlierRatio(double olr);

	double getStepSize() const;

	float getResolution() const;

	double getOutlierRatio() const;

	double getTransformationProbability() const;

	int getRealIterations();

	/* Set the input map points */
	void setInputTarget(typename pcl::PointCloud<PointTargetType>::Ptr input);

	/* Compute and get fitness score */
	double getFitnessScore(double max_range = DBL_MAX);

	void updateVoxelGrid(typename pcl::PointCloud<PointTargetType>::Ptr new_cloud);

protected:
	void computeTransformation(const Eigen::Matrix<float, 4, 4> &guess);


	using Registration<PointSourceType, PointTargetType>::transformation_epsilon_;
	using Registration<PointSourceType, PointTargetType>::max_iterations_;
	using Registration<PointSourceType, PointTargetType>::source_cloud_;
	using Registration<PointSourceType, PointTargetType>::trans_cloud_;
	using Registration<PointSourceType, PointTargetType>::converged_;
	using Registration<PointSourceType, PointTargetType>::nr_iterations_;
	using Registration<PointSourceType, PointTargetType>::final_transformation_;
	using Registration<PointSourceType, PointTargetType>::transformation_;
	using Registration<PointSourceType, PointTargetType>::previous_transformation_;
	using Registration<PointSourceType, PointTargetType>::target_cloud_updated_;
	using Registration<PointSourceType, PointTargetType>::target_cloud_;

private:
	//Copied from ndt.h
    double auxilaryFunction_PsiMT (double a, double f_a, double f_0, double g_0, double mu = 1.e-4);

    //Copied from ndt.h
    double auxilaryFunction_dPsiMT (double g_a, double g_0, double mu = 1.e-4);

    double updateIntervalMT (double &a_l, double &f_l, double &g_l,
								double &a_u, double &f_u, double &g_u,
								double a_t, double f_t, double g_t);

    double trialValueSelectionMT (double a_l, double f_l, double g_l,
									double a_u, double f_u, double g_u,
									double a_t, double f_t, double g_t);

	void computeAngleDerivatives(Eigen::Matrix<double, 6, 1> pose, bool compute_hessian = true);

	double computeStepLengthMT(const Eigen::Matrix<double, 6, 1> &x, Eigen::Matrix<double, 6, 1> &step_dir,
								double step_init, double step_max, double step_min, double &score,
								Eigen::Matrix<double, 6, 1> &score_gradient, Eigen::Matrix<double, 6, 6> &hessian,
								typename pcl::PointCloud<PointSourceType> &trans_cloud);

	void computeHessian(Eigen::Matrix<double, 6, 6> &hessian, typename pcl::PointCloud<PointSourceType> &trans_cloud, Eigen::Matrix<double, 6, 1> &p);

	double computeDerivatives(Eigen::Matrix<double, 6, 1> &score_gradient, Eigen::Matrix<double, 6, 6> &hessian,
								typename pcl::PointCloud<PointSourceType> &trans_cloud,
								Eigen::Matrix<double, 6, 1> pose, bool compute_hessian = true);
	void computePointDerivatives(Eigen::Vector3d &x, Eigen::Matrix<double, 3, 6> &point_gradient, Eigen::Matrix<double, 18, 6> &point_hessian, bool computeHessian = true);
	double updateDerivatives(Eigen::Matrix<double, 6, 1> &score_gradient, Eigen::Matrix<double, 6, 6> &hessian,
								Eigen::Matrix<double, 3, 6> point_gradient, Eigen::Matrix<double, 18, 6> point_hessian,
								Eigen::Vector3d &x_trans, Eigen::Matrix3d &c_inv, bool compute_hessian = true);
	void updateHessian(Eigen::Matrix<double, 6, 6> &hessian,
						Eigen::Matrix<double, 3, 6> point_gradient, Eigen::Matrix<double, 18, 6> point_hessian,
						Eigen::Vector3d &x_trans, Eigen::Matrix3d &c_inv);

	double gauss_d1_, gauss_d2_;
	double outlier_ratio_;
	Eigen::Vector3d j_ang_a_, j_ang_b_, j_ang_c_, j_ang_d_, j_ang_e_, j_ang_f_, j_ang_g_, j_ang_h_;

	Eigen::Vector3d h_ang_a2_, h_ang_a3_,
					h_ang_b2_, h_ang_b3_,
					h_ang_c2_, h_ang_c3_,
					h_ang_d1_, h_ang_d2_, h_ang_d3_,
					h_ang_e1_, h_ang_e2_, h_ang_e3_,
					h_ang_f1_, h_ang_f2_, h_ang_f3_;

	double step_size_;
	float resolution_;
	double trans_probability_;

	int real_iterations_;


	VoxelGrid<PointSourceType> voxel_grid_;
};
}

#endif
