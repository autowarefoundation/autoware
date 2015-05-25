#ifndef _DPM_H_
#define _DPM_H_

#include <vector>
#include <opencv2/core/core.hpp>

struct DPMObject {
	int class_id;
	cv::Rect rect;
};

extern std::vector<DPMObject> dpm_detect_objects(const cv::Mat& image,
												  const std::vector<std::string>& model_files,
												  float overlap_threshold, 
												  int threads,
												  double score_threshold,
												  int lambda,
												  int num_cells,
												  int num_bins
												  );

#endif /* _DPM_H_ */
