#ifndef _DPM_GPU_H_
#define _DPM_GPU_H_

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>

extern void dpm_gpu_init_cuda(const std::string& cubin_path);
extern void dpm_gpu_cleanup_cuda();
extern void dpm_gpu_load_models(const std::string& com_csv,
				const std::string& root_csv,
				const std::string& part_csv);

struct DPMGPUResult {
	int num;
	std::vector<int> corner_points;
	std::vector<int> type;
	std::vector<float> score;
};

extern DPMGPUResult dpm_gpu_detect_objects(IplImage *image, double threshold,
					   double overlap, int lambda, int num_cells);

#endif /* _DPM_GPU_H_ */
