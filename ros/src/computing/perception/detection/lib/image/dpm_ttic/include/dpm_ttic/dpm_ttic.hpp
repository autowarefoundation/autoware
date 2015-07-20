#ifndef _DPM_TTIC_H_
#define _DPM_TTIC_H_

#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

extern void dpm_ttic_gpu_init_cuda(const std::string& cubin_path);
extern void dpm_ttic_gpu_cleanup_cuda();

struct DPMTTICResult {
	int num;
	std::vector<int> corner_points;
	std::vector<int> type;
	std::vector<float> score;
};

struct DPMTTICParam {
	double threshold;
	double overlap;
	double lambda;
	double num_cells;

	DPMTTICParam() = default;
};

struct MODEL;
class DPMTTICModel {
private:
	MODEL *model_;

public:
	DPMTTICModel(const char *com_csv, const char *root_csv, const char *part_csv);
	~DPMTTICModel();

	DPMTTICResult detect_objects(IplImage *image, const DPMTTICParam& param);
};

struct GPUModel;
class DPMGPUModel {
private:
	GPUModel *model_;
	double RATIO;

public:
	DPMGPUModel(const char *com_csv, const char *root_csv, const char *part_csv);
	~DPMGPUModel();

	DPMTTICResult detect_objects(IplImage *image, const DPMTTICParam& param);
};

#endif /* _DPM_TTIC_H_ */
