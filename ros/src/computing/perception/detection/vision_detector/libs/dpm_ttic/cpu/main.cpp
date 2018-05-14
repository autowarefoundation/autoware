/// Car tracking project with laser_radar_data_fusion
/// Copyright 2009-10 Akihiro Takeuchi

///main.cpp   main function of car tracking

//OpenCV library

#include <cstdio>
#include <cstdlib>

#include <libdpm_ttic/dpm_ttic.hpp>

#include "MODEL_info.h"
#include "switch_float.h"
#include "detect.hpp"
#include "load_model.hpp"

DPMTTIC::DPMTTIC(const char *com_csv, const char *root_csv, const char *part_csv)
{
	constexpr double RATIO = 1; 
	model_ = dpm_ttic_cpu_load_model(RATIO, com_csv, root_csv, part_csv);
}

DPMTTIC::~DPMTTIC()
{
	dpm_ttic_cpu_free_model(model_);
}

static FLOAT *init_accumulated_score(IplImage *image)
{
	int size = image->height * image->width;
	FLOAT *score = (FLOAT *)calloc(size,sizeof(FLOAT));

	for(int i = 0; i < size; ++i)
		score[i] = -100.0;

	return score;
}

DPMTTICResult DPMTTIC::detect_objects(IplImage *image, const DPMTTICParam& param)
{
	// model_->MI->interval = param.lambda;
	// model_->MI->sbin     = param.num_cells;

	int detected_objects;
	FLOAT *ac_score = init_accumulated_score(image);
	RESULT *cars = dpm_ttic_cpu_car_detection(image, model_, param.threshold, &detected_objects, ac_score,
						  param.overlap);
	free(ac_score);

	DPMTTICResult result;
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
