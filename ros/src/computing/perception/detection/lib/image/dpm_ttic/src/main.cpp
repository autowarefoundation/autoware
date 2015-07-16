/// Car tracking project with laser_radar_data_fusion
/// Copyright 2009-10 Akihiro Takeuchi

///main.cpp   main function of car tracking

//OpenCV library
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#include <cstdio>
#include <cstdlib>
#include <time.h>

#include <cmath>
#include <dpm_ttic.hpp>

#include "MODEL_info.h"
#include "Common.h"
#include "switch_float.h"
#include "detect.hpp"
#include "load_model.hpp"

static MODEL *MO;

std::string com_name;
std::string root_name;
std::string part_name;

void dpm_ttic_load_models(const std::string& com_csv,
			  const std::string& root_csv,
			  const std::string& part_csv)
{
	constexpr double RATIO = 1; 

	com_name = com_csv;
	root_name = root_csv;
	part_name = part_csv;
	MO = load_model(RATIO);
}

DPMTTICResult dpm_ttic_detect_objects(IplImage *image, double threshold,
				      double overlap, int lambda, int num_cells)
{
	//MO->MI->interval = lambda;
	//MO->MI->sbin     = num_cells;

	int detected_objects;
	FLOAT *ac_score = ini_ac_score(image);
	RESULT *cars = car_detection(image, MO, threshold,
				     &detected_objects, ac_score,
				     overlap);
	s_free(ac_score);

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

	s_free(cars->point);
	s_free(cars->type);
	s_free(cars->scale);
	s_free(cars->score);
	s_free(cars->IM);
	return result;
}
