#ifndef _DPM_OCV_GPU_H_
#define _DPM_OCV_GPU_H_

#include <string>
#include <vector>
#include <opencv2/objdetect/objdetect.hpp>

//class DPMOCVGPULatentSvmDetector : public cv::LatentSvmDetector {
//private:
//// this is private member in cv::LatentSvmDetector
//    std::vector<CvLatentSvmDetector*> detectors;
//    std::vector<std::string> classNames;
//
//public:
//    explicit DPMOCVGPULatentSvmDetector(const std::vector<std::string>& filenames,
//        const float scoreThreshold, const std::vector<std::string>& classNames);
//    ~DPMOCVGPULatentSvmDetector();
//
//    bool load( const std::vector<std::string>& filenames, const float scoreThreshold );
//    void detect(const cv::Mat& image,
//        std::vector<ObjectDetection>& objectDetections, float overlapThreshold,
//        int numThreads, const unsigned int lambda,
//        const unsigned int sideLength, const float valOfTruncate);
//
//    bool empty() const;
//};

#endif /* _DPM_OCV_GPU_H_ */
