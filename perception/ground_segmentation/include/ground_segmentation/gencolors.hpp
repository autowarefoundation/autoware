// Copyright 2020 Tier IV, Inc.
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
/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#ifndef GROUND_SEGMENTATION__GENCOLORS_HPP_
#define GROUND_SEGMENTATION__GENCOLORS_HPP_

#include <opencv2/core/core.hpp>

#include <opencv2/core/core_c.h>

#include <iostream>
#include <vector>
//  #include <precomp.hpp>
#include <opencv2/opencv.hpp>

namespace ray_ground_filter
{
using namespace cv;  // NOLINT

inline static void downsamplePoints(const Mat & src, Mat & dst, size_t count)
{
  CV_Assert(count >= 2);
  CV_Assert(src.cols == 1 || src.rows == 1);
  CV_Assert(src.total() >= count);
  CV_Assert(src.type() == CV_8UC3);

  dst.create(1, static_cast<int>(count), CV_8UC3);
  // TODO(YamatoAndo): optimize by exploiting symmetry in the distance matrix
  Mat dists(static_cast<int>(src.total()), static_cast<int>(src.total()), CV_32FC1, Scalar(0));
  if (dists.empty()) {
    std::cerr << "Such big matrix can't be created." << std::endl;
  }

  for (int i = 0; i < dists.rows; i++) {
    for (int j = i; j < dists.cols; j++) {
      float dist = static_cast<float>(norm(src.at<Point3_<uchar>>(i) - src.at<Point3_<uchar>>(j)));
      dists.at<float>(j, i) = dists.at<float>(i, j) = dist;
    }
  }

  double maxVal;
  Point maxLoc;
  minMaxLoc(dists, 0, &maxVal, 0, &maxLoc);

  dst.at<Point3_<uchar>>(0) = src.at<Point3_<uchar>>(maxLoc.x);
  dst.at<Point3_<uchar>>(1) = src.at<Point3_<uchar>>(maxLoc.y);

  // ignore cspell error due to the source from OpenCV
  // cspell: ignore actived Dists randu
  Mat activedDists(0, dists.cols, dists.type());
  Mat candidatePointsMask(1, dists.cols, CV_8UC1, Scalar(255));
  activedDists.push_back(dists.row(maxLoc.y));
  candidatePointsMask.at<uchar>(0, maxLoc.y) = 0;

  for (size_t i = 2; i < count; i++) {
    activedDists.push_back(dists.row(maxLoc.x));
    candidatePointsMask.at<uchar>(0, maxLoc.x) = 0;

    Mat minDists;
    reduce(activedDists, minDists, 0, CV_REDUCE_MIN);
    minMaxLoc(minDists, 0, &maxVal, 0, &maxLoc, candidatePointsMask);
    dst.at<Point3_<uchar>>(static_cast<int>(i)) = src.at<Point3_<uchar>>(maxLoc.x);
  }
}

inline void generateColors(std::vector<Scalar> & colors, size_t count, size_t factor = 100)
{
  if (count < 1) {
    return;
  }

  colors.resize(count);

  if (count == 1) {
    colors[0] = Scalar(0, 0, 255);  // red
    return;
  }
  if (count == 2) {
    colors[0] = Scalar(0, 0, 255);  // red
    colors[1] = Scalar(0, 255, 0);  // green
    return;
  }

  // Generate a set of colors in RGB space. A size of the set is several times (=factor) larger then
  // the needed count of colors.
  Mat bgr(1, static_cast<int>(count * factor), CV_8UC3);
  randu(bgr, 0, 256);

  // Convert the colors set to Lab space.
  // Distances between colors in this space correspond a human perception.
  Mat lab;
  cvtColor(bgr, lab, cv::COLOR_BGR2Lab);

  // Subsample colors from the generated set so that
  // to maximize the minimum distances between each other.
  // Douglas-Peucker algorithm is used for this.
  Mat lab_subset;
  downsamplePoints(lab, lab_subset, count);

  // Convert subsampled colors back to RGB
  Mat bgr_subset;
  cvtColor(lab_subset, bgr_subset, cv::COLOR_BGR2Lab);

  CV_Assert(bgr_subset.total() == count);
  for (size_t i = 0; i < count; i++) {
    Point3_<uchar> c = bgr_subset.at<Point3_<uchar>>(static_cast<int>(i));
    colors[i] = Scalar(c.x, c.y, c.z);
  }
}
}  // namespace ray_ground_filter
#endif  // GROUND_SEGMENTATION__GENCOLORS_HPP_
