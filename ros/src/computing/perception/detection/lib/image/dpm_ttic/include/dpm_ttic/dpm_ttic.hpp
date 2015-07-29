#ifndef _DPM_TTIC_H_
#define _DPM_TTIC_H_

#include <string>
#include <vector>
#include <opencv/cv.h>

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
class DPMTTIC {
private:
	MODEL *model_;

public:
	DPMTTIC(const char *com_csv, const char *root_csv, const char *part_csv);
	~DPMTTIC();

	DPMTTICResult detect_objects(IplImage *image, const DPMTTICParam& param);
};

struct GPUModel;
class DPMTTICGPU {
private:
	GPUModel *model_;
	double RATIO;

public:
	DPMTTICGPU(const char *com_csv, const char *root_csv, const char *part_csv);
	~DPMTTICGPU();

	DPMTTICResult detect_objects(IplImage *image, const DPMTTICParam& param);
};

#endif /* _DPM_TTIC_H_ */
