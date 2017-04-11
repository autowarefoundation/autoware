#ifndef _DPM_OCV_H_
#define _DPM_OCV_H_

#include <vector>
#include <opencv2/core/core.hpp>

class DPMOCVCPULatentSvmDetector : public cv::LatentSvmDetector {
private:
    std::vector<CvLatentSvmDetector*> detectors; // this is private member in cv::LatentSvmDetector

public:
    DPMOCVCPULatentSvmDetector(const std::vector<std::string>& filenames);
    void detect( const cv::Mat& image, std::vector<ObjectDetection>& objectDetections,
                 float overlapThreshold, int numThreads, double score_threshold,
                 int lambda, int num_cells, int num_bins);

    bool empty() const;
};

class DPMOCVGPULatentSvmDetector : public cv::LatentSvmDetector {
private:
// this is private member in cv::LatentSvmDetector
    std::vector<CvLatentSvmDetector*> detectors;
    std::vector<std::string> classNames;

public:
    explicit DPMOCVGPULatentSvmDetector(const std::vector<std::string>& filenames,
        const float scoreThreshold);
    ~DPMOCVGPULatentSvmDetector();

    bool load( const std::vector<std::string>& filenames, const float scoreThreshold );
    void detect(const cv::Mat& image,
        std::vector<ObjectDetection>& objectDetections, float overlapThreshold,
        int numThreads, const unsigned int lambda,
        const unsigned int sideLength, const float valOfTruncate);

    bool empty() const;
};

#endif /* _DPM_OCV_H_ */
