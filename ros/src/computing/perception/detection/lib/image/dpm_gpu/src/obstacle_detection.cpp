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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <dpm/ImageObjects.h>
#include <runtime_manager/ConfigCarDpm.h>
#include <runtime_manager/ConfigPedestrianDpm.h>

#include "Laser_func.h"
#include "car_det_func.h"
#include "Common.h"
#include "Depth_points_func.h"
#include "for_use_GPU.h"
#include <dpm_gpu.hpp>

MODEL *MO;

struct timeval tv_memcpy_start, tv_memcpy_end;
float time_memcpy;
struct timeval tv_kernel_start, tv_kernel_end;
float time_kernel;

int device_num;

std::string com_name;
std::string root_name;
std::string part_name;

void dpm_gpu_init_cuda(const std::string& cubin_path)
{
	init_cuda_with_cubin(cubin_path.c_str());
}

void dpm_gpu_load_models(const std::string& com_csv,
			 const std::string& root_csv,
			 const std::string& part_csv)
{
	constexpr double RATIO = 1; 

	com_name = com_csv;
	root_name = root_csv;
	part_name = part_csv;
	MO = load_model(RATIO);
}

void dpm_gpu_cleanup_cuda()
{
	clean_cuda();
	free_model(MO);
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
				     overlap);	//detect car
	s_free(ac_score);

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
	}

	s_free(cars->point);
	s_free(cars->type);
	s_free(cars->scale);
	s_free(cars->score);
	s_free(cars->IM);
	return result;
}
