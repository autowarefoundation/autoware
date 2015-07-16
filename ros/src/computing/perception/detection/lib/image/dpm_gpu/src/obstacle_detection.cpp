/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <sys/time.h>
#include <sys/resource.h>

#include <dpm_gpu.hpp>

#include "for_use_GPU.h"
#include "load_model.hpp"
#include "detect.hpp"
#include "GPU_init.hpp"

struct timeval tv_memcpy_start, tv_memcpy_end;
float time_memcpy;
struct timeval tv_kernel_start, tv_kernel_end;
float time_kernel;

int device_num;

// Should be removed!!!!
static MODEL *MO;

void dpm_gpu_init_cuda(const std::string& cubin_path)
{
	init_cuda_with_cubin(cubin_path.c_str());
}

void dpm_gpu_load_models(const std::string& com_csv,
			 const std::string& root_csv,
			 const std::string& part_csv)
{
	constexpr double RATIO = 1;
	MO = load_model(RATIO, com_csv.c_str(), root_csv.c_str(), part_csv.c_str());
}

void dpm_gpu_cleanup_cuda()
{
	clean_cuda();
	//free_model(MO);
}

DPMGPUResult dpm_gpu_detect_objects(IplImage *image, double threshold,
				    double overlap, int lambda, int num_cells)
{
	MO->MI->interval = lambda;
	MO->MI->sbin     = num_cells;

	int detected_objects;
	FLOAT *ac_score = ini_ac_score(image);
	RESULT *cars = car_detection(image, MO, threshold,
				     &detected_objects, ac_score,
				     overlap);
	free(ac_score);

	DPMGPUResult result;
	result.num = cars->num;
	for (int i = 0; i < cars->num; ++i) {
		result.type.push_back(cars->type[i]);
	}

	for (int i = 0; i < cars->num; ++i) {
		int base = i * 4;
		int *data = &(cars->OR_point[base]);

		result.corner_points.push_back(data[0]);
		result.corner_points.push_back(data[1]);
		result.corner_points.push_back(data[2] - data[0]);
		result.corner_points.push_back(data[3] - data[1]);
		result.score.push_back(cars->score[i]);
	}

	free(cars->point);
	free(cars->type);
	free(cars->scale);
	free(cars->score);
	free(cars->IM);
	return result;
}

DPMGPUModel::DPMGPUModel(const char *com_csv, const char *root_csv, const char *part_csv)
{
	constexpr double RATIO = 1;
	model_ = load_model(RATIO, com_csv, root_csv, part_csv);
}

DPMGPUModel::~DPMGPUModel()
{
	free_model(model_);
}

DPMGPUResult DPMGPUModel::detect_objects(IplImage *image, const DPMGPUParam& param)
{
	model_->MI->interval = param.lambda;
	model_->MI->sbin     = param.num_cells;

	int detected_objects;
	FLOAT *ac_score = ini_ac_score(image);
	RESULT *objects = car_detection(image, model_, param.threshold,
					&detected_objects, ac_score,
					param.overlap);
	free(ac_score);

	DPMGPUResult result;
	result.num = objects->num;
	for (int i = 0; i < objects->num; ++i) {
		result.type.push_back(objects->type[i]);
	}

	for (int i = 0; i < objects->num; ++i) {
		int base = i * 4;
		int *data = &(objects->OR_point[base]);

		result.corner_points.push_back(data[0]);
		result.corner_points.push_back(data[1]);
		result.corner_points.push_back(data[2] - data[0]);
		result.corner_points.push_back(data[3] - data[1]);
		result.score.push_back(objects->score[i]);
	}

	free(objects->point);
	free(objects->type);
	free(objects->scale);
	free(objects->score);
	free(objects->IM);
	return result;
}
