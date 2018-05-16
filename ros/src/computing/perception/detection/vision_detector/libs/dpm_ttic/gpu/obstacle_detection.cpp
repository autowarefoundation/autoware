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

#include <libdpm_ttic/dpm_ttic.hpp>

#include "for_use_GPU.h"
#include "load_model.hpp"
#include "detect.hpp"
#include "GPU_init.hpp"

#include "common.hpp"

struct timeval tv_memcpy_start, tv_memcpy_end;
float time_memcpy;
struct timeval tv_kernel_start, tv_kernel_end;
float time_kernel;

int device_num;

void dpm_ttic_gpu_init_cuda(const std::string& cubin_path)
{
	dpm_ttic_gpu_init_cuda_with_cubin(cubin_path.c_str());
}

void dpm_ttic_gpu_cleanup_cuda()
{
	dpm_ttic_gpu_clean_cuda();
	//free_model(MO);
}

DPMTTICGPU::DPMTTICGPU(const char *com_csv, const char *root_csv, const char *part_csv)
{
	constexpr double RATIO = 1;
	model_ = dpm_ttic_gpu_load_model(RATIO, com_csv, root_csv, part_csv);
}

DPMTTICGPU::~DPMTTICGPU()
{
	dpm_ttic_gpu_free_model(model_);
}

static FLOAT *init_accumulated_score(IplImage *image, size_t& accumulated_size)
{
	size_t num = image->height * image->width;
	accumulated_size = num * sizeof(FLOAT);

	FLOAT *scores = (FLOAT *)calloc(num, sizeof(FLOAT));
	for(size_t i = 0; i < num; i++)
		scores[i] = -100.0;

	return scores;
}

DPMTTICResult DPMTTICGPU::detect_objects(IplImage *image, const DPMTTICParam& param)
{
	model_->MI->interval = param.lambda;
	model_->MI->sbin     = param.num_cells;

	int detected_objects;
	FLOAT *ac_score = init_accumulated_score(image, gpu_size_A_SCORE);
	RESULT *objects = dpm_ttic_gpu_car_detection(image, model_, param.threshold,
						     &detected_objects, ac_score,
						     param.overlap);
	free(ac_score);

	DPMTTICResult result;
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
