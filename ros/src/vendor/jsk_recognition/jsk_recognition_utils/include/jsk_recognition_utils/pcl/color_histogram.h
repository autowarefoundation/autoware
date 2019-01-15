// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/*
 * color_histogram.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef JSK_RECOGNITION_UTILS_COLOR_HISTOGRAM_H__
#define JSK_RECOGNITION_UTILS_COLOR_HISTOGRAM_H__

#include <algorithm>
#include <iterator>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace jsk_recognition_utils {
  enum HistogramPolicy {
    HUE = 0,
    HUE_AND_SATURATION
  };

  enum ComparePolicy {
    CORRELATION = 0,
    BHATTACHARYYA,
    INTERSECTION,
    CHISQUARE,
    KL_DIVERGENCE
  };

  struct PointXYZHLS {
    double x, y, z;
    double h, l, s;
  };

  inline void
  HSV2HLS(const pcl::PointXYZHSV& hsv, PointXYZHLS& hls) {
    hls.x = hsv.x; hls.y = hsv.y; hls.z = hsv.z;
    hls.h = hsv.h;
    hls.l = (2.0 - hsv.s) * hsv.v;
    hls.s = hsv.s * hsv.v;
    hls.s /= (hls.l <= 1.0) ? hls.l : 2.0 - hls.l;
    hls.l /= 2.0;
  }

  inline int
  getHistogramBin(const double& val, const int& step,
         const double& min, const double& max)
  {
    int idx = std::floor((val - min) / (max - min) * step);
    if (idx >= step) return step - 1;
    else if (idx <= 0) return 0;
    else return idx;
  }

  inline void
  normalizeHistogram(std::vector<float>& histogram)
  {
    float sum = 0.0f;
    for (size_t i = 0; i < histogram.size(); ++i)
      sum += histogram[i];
    if (sum != 0.0) {
      for (size_t i = 0; i < histogram.size(); ++i)
        histogram[i] /= sum;
    }
  }

  inline void
  computeColorHistogram1d(const pcl::PointCloud<pcl::PointXYZHSV>& cloud,
                          std::vector<float>& histogram,
                          const int bin_size,
                          const double white_threshold=0.1,
                          const double black_threshold=0.1)
  {
    histogram.resize(bin_size + 2, 0.0f); // h + w + b
    int white_index = bin_size;
    int black_index = bin_size + 1;

    for (size_t i = 0; i < cloud.points.size(); ++i) {
      PointXYZHLS p;
      HSV2HLS(cloud.points[i], p);
      if (p.l > 1.0 - white_threshold)
        histogram[white_index] += 1.0f;
      else if (p.l < black_threshold)
        histogram[black_index] += 1.0f;
      else {
        int h_bin = getHistogramBin(p.h, bin_size, 0.0, 360.0);
        histogram[h_bin] += 1.0f;
      }
    }

    normalizeHistogram(histogram);
  }

  inline void
  computeColorHistogram2d(const pcl::PointCloud<pcl::PointXYZHSV>& cloud,
                          std::vector<float>& histogram,
                          const int bin_size_per_channel,
                          const double white_threshold=0.1,
                          const double black_threshold=0.1)
  {
    histogram.resize(bin_size_per_channel  * bin_size_per_channel + 2, 0.0f); // h * s + w + b
    int white_index = bin_size_per_channel * bin_size_per_channel;
    int black_index = bin_size_per_channel * bin_size_per_channel + 1;

    // vote
    for (size_t i = 0; i < cloud.points.size(); ++i) {
      PointXYZHLS p;
      HSV2HLS(cloud.points[i], p);
      if (p.l < white_threshold)
        histogram[white_index] += 1.0f;
      else if (p.l > 1.0 - black_threshold)
        histogram[black_index] += 1.0f;
      else {
        int h_bin = getHistogramBin(p.h, bin_size_per_channel, 0.0, 360.0);
        int s_bin = getHistogramBin(p.s, bin_size_per_channel, 0.0, 1.0);
        histogram[s_bin * bin_size_per_channel + h_bin] += 1.0f;
      }
    }

    normalizeHistogram(histogram);
  }

  inline void
  rotateHistogram1d(const std::vector<float>& input,
                    std::vector<float>& output,
                    const double degree)
  {
    size_t bin_size = input.size() - 2;
    int offset = std::floor(degree / 360.0 * bin_size);
    output.resize(input.size());
    for (size_t h = 0; h < bin_size; ++h) {
      int hout = h + offset % bin_size;
      output[hout] = input[h];
    }

    // copy white + black
    int index = bin_size;
    output[index] = input[index];
    output[index+1] = input[index+1];
  }

  inline void
  rotateHistogram2d(const std::vector<float>& input,
                    std::vector<float>& output,
                    const double degree)
  {
    size_t bin_size = (size_t)(std::sqrt(input.size() - 2));
    int offset = std::floor(degree / 360.0 * bin_size);
    output.resize(input.size());
    for (size_t s = 0; s < bin_size; ++s) {
      for (size_t h = 0; h < bin_size; ++h) {
        int hout = h + offset % bin_size;
        output[s * bin_size + hout] = input[s * bin_size + h];
      }
    }

    // copy white + black
    int index = bin_size * bin_size;
    output[index] = input[index];
    output[index+1] = input[index+1];
  }

  inline bool
  compareHistogram(const std::vector<float>& input,
                   const std::vector<float>& reference,
                   const ComparePolicy policy,
                   double& distance)
  {
    if (input.size() != reference.size()) {
      ROS_ERROR("Mismatch histogram bin size");
      return false;
    }

    distance = 0.0;
    size_t len = input.size();
    if (policy == CHISQUARE) {
      for (size_t i = 0; i < len; ++i) {
        double a = input[i] - reference[i], b = input[i];
        if (std::fabs(b) > DBL_EPSILON) distance += a * a / b;
      }
    } else if (policy == CORRELATION) {
      double s1 = 0.0, s2 = 0.0, s11 = 0.0, s12 = 0.0, s22 = 0.0;
      for (size_t i = 0; i < len; ++i) {
        double a = input[i], b = reference[i];
        s11 += a*a; s12 += a*b; s22 += b*b;
        s1 += a; s2 += b;
      }
      double num = s12 - s1*s2/len;
      double denom2 = (s11 - s1 * s1 / len) * (s22 - s2 * s2 / len);
      distance = std::fabs(denom2) > DBL_EPSILON ? num / std::sqrt(denom2) : 1.0;
    } else if (policy == INTERSECTION) {
      for (size_t i = 0; i < len; ++i) {
        distance += std::min(input[i], reference[i]);
      }
    } else if (policy == BHATTACHARYYA) {
      double s1 = 0.0, s2 = 0.0;
      for (size_t i = 0; i < len; ++i) {
        distance += std::sqrt(input[i] * reference[i]);
        s1 += input[i]; s2 += reference[i];
      }
      s1 *= s2;
      s1 = std::fabs(s1) > DBL_EPSILON ? 1.0 / std::sqrt(s1) : 1.0;
      distance = std::sqrt(std::max(1.0 - distance * s1, 0.0));
    } else if (policy == KL_DIVERGENCE) {
      for (size_t i = 0; i < len; ++i) {
        double p = input[i], q = reference[i];
        if (std::fabs(p) <= DBL_EPSILON) continue;
        if (std::fabs(q) <= DBL_EPSILON) q = 1e-10;
        distance += p * std::log(p / q);
      }
    } else {
      ROS_ERROR("Invalid compare policy");
      return false;
    }

    return true;
  }
} // namespace

#endif // JSK_RECOGNITION_UTILS_COLOR_HISTOGRAM_H__
