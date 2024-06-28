// Copyright 2024 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc_c.h>

#include <functional>
#include <map>
#include <string>

struct DrawFunctionParams
{
  cv::Mat & image;
  cv::Point position;
  cv::Scalar color;
  int size;
  float probability;
};

using DrawFunction = std::function<void(const DrawFunctionParams & params)>;

void drawShape(
  const DrawFunctionParams & params, const std::string & filename, bool flipHorizontally,
  bool flipVertically, int x_offset, int y_offset, double scale_factor = 0.3);
void drawCircle(const DrawFunctionParams & params);
void drawLeftArrow(const DrawFunctionParams & params);
void drawRightArrow(const DrawFunctionParams & params);
void drawStraightArrow(const DrawFunctionParams & params);
void drawDownArrow(const DrawFunctionParams & params);
void drawDownLeftArrow(const DrawFunctionParams & params);
void drawDownRightArrow(const DrawFunctionParams & params);
void drawCross(const DrawFunctionParams & params);
void drawUnknown(const DrawFunctionParams & params);
void drawTrafficLightShape(
  cv::Mat & image, const std::string & shape, const cv::Point & position, const cv::Scalar & color,
  int size, float probability);
