/*
 * cnn_lidar_detector.hpp
 *
 *  Created on: Jun 20, 2017
 *      Author: ne0
 */

#ifndef CNN_LIDAR_DETECTOR_HPP_
#define CNN_LIDAR_DETECTOR_HPP_

#include <caffe/caffe.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>


class CnnLidarDetector
{
public:
	CnnLidarDetector(const std::string& in_network_definition_file, const std::string& in_pre_trained_model_file,  bool in_use_gpu, unsigned int in_gpu_id);

	void Detect();



private:
	boost::shared_ptr<caffe::Net<float> > net_;
};



#endif /* CNN_LIDAR_DETECTOR_HPP_ */
