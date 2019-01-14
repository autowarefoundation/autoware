/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef INCLUDED_MFunctions_
#define INCLUDED_MFunctions_

#ifndef _DEBUG
#define _DEBUG 0
#endif

#include <vector>

#include <ros/ros.h>
#include "autoware_msgs/ImageObj.h"
#include "autoware_msgs/ImageRectRanged.h"
#include "autoware_msgs/ScanImage.h"
#include "autoware_msgs/PointsImage.h"
#include "autoware_msgs/ImageObjTracked.h"

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
extern void setDetectedObjects(const autoware_msgs::ImageObj& image_objects);
extern void setScanImage(const autoware_msgs::ScanImage& scan_image);
extern void setPointsImage(const autoware_msgs::PointsImage& points_image);
extern std::vector<autoware_msgs::ImageRectRanged> getObjectsRectRanged();
extern std::string getObjectsType();
extern void init();
extern void destroy();
#if _DEBUG
extern void imageCallback(const sensor_msgs::Image& image_source);
#endif

#endif
