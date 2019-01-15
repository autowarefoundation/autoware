// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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

#include "jsk_recognition_utils/cv_utils.h"
#include "jsk_recognition_utils/rgb_colors.h"
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;

namespace jsk_recognition_utils
{
  cv::MatND computeHistogram(const cv::Mat& input_image, int bin_size,
                             float min_value, float max_value,
                             const cv::Mat& mask_image)
  {
    int channels[] = {0};
    cv::MatND hist;
    int hist_size[] = {bin_size};
    float range[] = {min_value, max_value};
    const float* ranges[] = {range};
    cv::calcHist(&input_image, 1, channels, mask_image, 
                 hist, 1, hist_size,
                 ranges, true, false);
    return hist;
  }
  
  std::vector<jsk_recognition_msgs::HistogramWithRangeBin>
  cvMatNDToHistogramWithRangeBinArray(const cv::MatND& cv_hist, float min_value, float max_value)
  {
    std::vector<jsk_recognition_msgs::HistogramWithRangeBin> bins(cv_hist.total());
    const float bin_width = (max_value - min_value) / cv_hist.total();
    for (size_t i = 0; i < cv_hist.total(); i++) {
      const float left = i * bin_width + min_value;
      const float right = (i + 1) * bin_width + min_value;
      jsk_recognition_msgs::HistogramWithRangeBin bin;
      bin.min_value = left;
      bin.max_value = right;
      bin.count = cv_hist.at<float>(0, i);
      bins[i] = bin;
    }
    return bins;
  }

  cv::MatND
  HistogramWithRangeBinArrayTocvMatND(
    const std::vector<jsk_recognition_msgs::HistogramWithRangeBin>& histogram)
  {
    cv::MatND ret(1, histogram.size(), CV_32F);
    for (size_t i = 0; i < histogram.size(); i++) {
      jsk_recognition_msgs::HistogramWithRangeBin bin = histogram[i];
      ret.at<float>(0, i) = bin.count;
    }
    return ret;
  }

  bool compareHistogramWithRangeBin(const jsk_recognition_msgs::HistogramWithRangeBin& left,
                                    const jsk_recognition_msgs::HistogramWithRangeBin& right)
  {
    return left.count > right.count;
  }

  void sortHistogramWithRangeBinArray(std::vector<jsk_recognition_msgs::HistogramWithRangeBin>& bins)
  {
    std::sort(bins.begin(), bins.end(), compareHistogramWithRangeBin);
  }

  std::vector<jsk_recognition_msgs::HistogramWithRangeBin>
  topNHistogramWithRangeBins(const std::vector<jsk_recognition_msgs::HistogramWithRangeBin>& bins,
                             double top_n_rate)
  {
    int sum = 0;
    for (size_t i = 0; i < bins.size(); i++) {
      sum += bins[i].count;
    }
    const int target_sum = sum * top_n_rate;
    std::vector<jsk_recognition_msgs::HistogramWithRangeBin> top_n_bins;
    top_n_bins.reserve(bins.size());
    
    int current_sum = 0;
    for (size_t i = 0; i < bins.size(); i++) {
      jsk_recognition_msgs::HistogramWithRangeBin bin = bins[i];
      if (current_sum >= target_sum) {
        return top_n_bins;
      }
      top_n_bins.push_back(bin);
      current_sum += bins[i].count;
    }
    return top_n_bins;
  }

  void
  drawHistogramWithRangeBin(cv::Mat& image,
                            const jsk_recognition_msgs::HistogramWithRangeBin& bin,
                            float min_width_value,
                            float max_width_value,
                            float max_height_value,
                            cv::Scalar color)
  {
    if (max_height_value == 0.0) {
      return;
    }
    const int height = image.rows;
    const int width = image.cols;
    const int left = (bin.min_value - min_width_value) / (max_width_value - min_width_value) * width;
    const int right = (bin.max_value - min_width_value) / (max_width_value - min_width_value) * width;
    const int top = bin.count / max_height_value * height;
    if (bin.count == 0 || top == 0 || left == right || left < 0 || right >= width || top > height) {
      return;
    }
    
    cv::rectangle(image, cv::Point(left, height), cv::Point(right, height - top),
                  color, CV_FILLED);
  }

  void labelToRGB(const cv::Mat src, cv::Mat& dst)
  {
    dst = cv::Mat::zeros(src.rows, src.cols, CV_8UC3);
    for (size_t j = 0; j < src.rows; ++j) {
      for (size_t i = 0; i < src.cols; ++i) {
        int label = src.at<int>(j, i);
        if (label == 0) {  // background label
          dst.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
        }
        else {
          cv::Vec3d rgb = jsk_recognition_utils::getRGBColor(label);
          dst.at<cv::Vec3b>(j, i) = cv::Vec3b(int(rgb[0] * 255), int(rgb[1] * 255), int(rgb[2] * 255));
        }
      }
    }
  }

  cv::Rect boundingRectOfMaskImage(const cv::Mat& image)
  {
    int min_x = image.cols;
    int min_y = image.rows;
    int max_x = 0;
    int max_y = 0;
    for (int j = 0; j < image.rows; j++) {
      for (int i = 0; i < image.cols; i++) {
        if (image.at<uchar>(j, i) != 0) {
          min_x = std::min(min_x, i);
          min_y = std::min(min_y, j);
          max_x = std::max(max_x, i);
          max_y = std::max(max_y, j);
        }
      }
    }
    return cv::Rect(min_x, min_y, std::max(max_x - min_x, 0), std::max(max_y - min_y, 0));
  }

  // Utility functions for inspecting an encoding string
  bool isBGR(const std::string& encoding)
  {
    return encoding == enc::BGR8 || encoding == enc::BGR16;
  }

  bool isRGB(const std::string& encoding)
  {
    return encoding == enc::RGB8 || encoding == enc::RGB16;
  }

  bool isBGRA(const std::string& encoding)
  {
    return encoding == enc::BGRA8 || encoding == enc::BGRA16;
  }

  bool isRGBA(const std::string& encoding)
  {
    return encoding == enc::RGBA8 || encoding == enc::RGBA16;
  }

}
