#ifndef _DPM_TTIC_H_
#define _DPM_TTIC_H_

struct DPMTTICResult {
	int num;
	std::vector<int> corner_points;
	std::vector<int> type;
	std::vector<float> score;
};

extern DPMTTICResult dpm_ttic_detect_objects(IplImage *image, double threshold,
					     double overlap, int lambda, int num_cells);

#endif /* _DPM_TTIC_H_ */
