/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <iostream>
#include <sstream>
#include "objdetect.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <cstdio>
#include <vector>

#include <ros/ros.h>
#include <dpm.hpp>
#include "objdetect.hpp"

std::vector<DPMObject> dpm_detect_objects(const cv::Mat& image,							//opencv image
										  const std::vector<std::string>& model_files,	//dpm model xml files
										  float overlap_threshold,						//overlap threshold
										  int threads,									//threads to execute
										  double score_threshold,						//detection score threshold
										  int lambda,									//used to calculate the pyramid levels
										  int num_cells,								//hog num cells to divide the image
										  int num_bins									//num of bins for hog
										  )
{
	cv::LatentSvmDetector detector(model_files);
	if (detector.empty())
	{
		std::cerr << "Model files can't be loaded" << std::endl;
		for (const auto& file : model_files)
		{
			std::cerr << "\t File: " << file << std::endl;
		}
		std::exit(1);
	}

	const std::vector<std::string> classes = detector.getClassNames();
	size_t class_num = classes.size();

	std::cout << "Load: " << class_num << " models" << std::endl;
	for (int i = 0; i < static_cast<int>(class_num); ++i)
	{
		std::cout << "(" << i << ") " << classes[i] << std::endl;
	}

	std::vector<cv::Scalar> colors;
	cv::generateColors(colors, class_num);

	std::vector<cv::LatentSvmDetector::ObjectDetection> detections;
	detector.detect(image, detections, overlap_threshold, threads, score_threshold, lambda, num_cells, num_bins);

	std::vector<DPMObject> results(detections.size());
	for (int i = 0; i < static_cast<int>(detections.size()); ++i)
	{
		results[i].rect = detections[i].rect;
		results[i].class_id = detections[i].classID;
	}

	return results;
}
