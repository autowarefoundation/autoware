/// Car tracking project with laser_radar_data_fusion
/// Copyright 2009-10 Akihiro Takeuchi

///main.cpp   main function of car tracking

//OpenCV library

#include <cstdio>
#include <cstdlib>

#include <dpm_ttic.hpp>

#include "MODEL_info.h"
#include "switch_float.h"
#include "detect.hpp"
#include "load_model.hpp"

DPMTTICModel::DPMTTICModel(const char *com_csv, const char *root_csv, const char *part_csv)
{
	constexpr double RATIO = 1; 
	model_ = load_model(RATIO, com_csv, root_csv, part_csv);
}

DPMTTICModel::~DPMTTICModel()
{
	free_model(model_);
}

DPMTTICResult DPMTTICModel::detect_objects(IplImage *image, const DPMTTICParam& param)
{
	// model_->MI->interval = param.lambda;
	// model_->MI->sbin     = param.num_cells;

	int detected_objects;
	FLOAT *ac_score = ini_ac_score(image);
	RESULT *cars = car_detection(image, model_, param.threshold, &detected_objects, ac_score,
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
