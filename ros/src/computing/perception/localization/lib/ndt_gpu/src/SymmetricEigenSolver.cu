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

#include "ndt_gpu/SymmetricEigenSolver.h"
#include "ndt_gpu/debug.h"

namespace gpu {

SymmetricEigensolver3x3::SymmetricEigensolver3x3(int offset)
{
	offset_ = offset;

	checkCudaErrors(cudaMalloc(&buffer_, sizeof(double) * 18 * offset_));
	checkCudaErrors(cudaMalloc(&maxAbsElement_, sizeof(double) * offset_));
	checkCudaErrors(cudaMalloc(&norm_, sizeof(double) * offset_));
	checkCudaErrors(cudaMalloc(&i02_, sizeof(int) * 2 * offset_));

	eigenvectors_ = NULL;
	eigenvalues_ = NULL;
	input_matrices_ = NULL;

	is_copied_ = false;
}

void SymmetricEigensolver3x3::setInputMatrices(double *input_matrices)
{
	input_matrices_ = input_matrices;
}

void SymmetricEigensolver3x3::setEigenvectors(double *eigenvectors)
{
	eigenvectors_ = eigenvectors;
}

void SymmetricEigensolver3x3::setEigenvalues(double *eigenvalues)
{
	eigenvalues_ = eigenvalues;
}

double* SymmetricEigensolver3x3::getBuffer() const
{
	return buffer_;
}

void SymmetricEigensolver3x3::memFree()
{
	if (!is_copied_) {
		if (buffer_ != NULL) {
			checkCudaErrors(cudaFree(buffer_));
			buffer_ = NULL;
		}

		if (maxAbsElement_ != NULL) {
			checkCudaErrors(cudaFree(maxAbsElement_));
			maxAbsElement_ = NULL;
		}

		if (norm_ != NULL) {
			checkCudaErrors(cudaFree(norm_));
			norm_ = NULL;
		}

		if (i02_ != NULL) {
			checkCudaErrors(cudaFree(i02_));
			i02_ = NULL;
		}
	}
}
}
