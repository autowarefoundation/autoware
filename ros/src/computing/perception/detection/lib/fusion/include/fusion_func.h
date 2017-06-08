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

#ifndef INCLUDED_MFunctions_
#define INCLUDED_MFunctions_

#ifndef _DEBUG
#define _DEBUG 0
#endif

#include <vector>

#include <ros/ros.h>
#include "autoware_msgs/image_obj.h"
#include "autoware_msgs/image_rect_ranged.h"
#include "autoware_msgs/ScanImage.h"
#include "autoware_msgs/PointsImage.h"
#include "autoware_msgs/image_obj_tracked.h"

#include <opencv2/opencv.hpp>

#define NO_DATA 0

#if _DEBUG
#define IMAGE_TOPIC "/image_raw"
#define IMAGE_CALLBACK imageCallback
#endif

struct Scan_image{
	std::vector<std::vector<float>> distance;
	std::vector<std::vector<float>> intensity;
	int max_y;
	int min_y;
};

struct Point5
{
	int x;
	int y;
	double distance;
	float min_h;
	float max_h;
};

extern void fuse();
extern void fuseFilterDetections(std::vector<Point5>& vScanPoints);
extern void getVScanPoints(std::vector<Point5> &vScanPoints);
extern bool dispersed(std::vector<Point5> &vScanPoints, std::vector<int> &indices);
extern float getStdDev(std::vector<Point5> &vScanPoints, std::vector<int> &indices, float avg);
extern float getMinAverage(std::vector<Point5> &vScanPoints, std::vector<int> &indices);
extern bool rectangleContainsPoints(cv::Rect rect, std::vector<Point5> &vScanPoints, float object_distance, std::vector<int> &outIndices);
extern std::vector<float> getMinHeights();
extern std::vector<float> getMaxHeights();
extern void setParams(float minLowHeight, float maxLowHeight, float maxHeight, int minPoints, float disp);

extern void calcDistance();
extern void setDetectedObjects(const autoware_msgs::image_obj& image_objects);
extern void setScanImage(const autoware_msgs::ScanImage& scan_image);
extern void setPointsImage(const autoware_msgs::PointsImage& points_image);
extern std::vector<autoware_msgs::image_rect_ranged> getObjectsRectRanged();
extern std::string getObjectsType();
extern void init();
extern void destroy();
#if _DEBUG
extern void imageCallback(const sensor_msgs::Image& image_source);
#endif

#endif
