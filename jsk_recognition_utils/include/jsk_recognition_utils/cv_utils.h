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

#ifndef JSK_RECOGNITION_UTILS_CV_UTILS_H_
#define JSK_RECOGNITION_UTILS_CV_UTILS_H_

#include <opencv2/opencv.hpp>
#include <jsk_recognition_msgs/HistogramWithRangeBin.h>

namespace jsk_recognition_utils
{
  /**
   * @brief
   * simple wrapper for cv::calcHist.
   *
   * cv::MatND is a class to represent histogram in OpenCV 2.x.
   * computeHistogram create 1-dimension cv::MatND.
   */
  cv::MatND computeHistogram(const cv::Mat& input_image, int bin_size,
                             float min_value, float max_value,
                             const cv::Mat& mask_image);

  /**
   * @brief
   * convert cv::MatND to jsk_recognition_msgs::HistogramimageWithRangeBin array
   */
  std::vector<jsk_recognition_msgs::HistogramWithRangeBin>
  cvMatNDToHistogramWithRangeBinArray(const cv::MatND& cv_hist, float min_value, float max_value);

  /**
   * @brief
   * return true if left.count is larger than right.count.
   */
  bool compareHistogramWithRangeBin(const jsk_recognition_msgs::HistogramWithRangeBin& left,
                                    const jsk_recognition_msgs::HistogramWithRangeBin& right);
  
  /**
   * @brief
   * sort std::vector<jsk_recognition_msgs::HistogramWithRangeBin>.
   * largest value will be at the first element.
   */
  void sortHistogramWithRangeBinArray(std::vector<jsk_recognition_msgs::HistogramWithRangeBin>& bins);

  /**
   * @brief
   * extract top-N histograms. bins should be sorted.
   * top_n_rate should be 0-1.
   */
  std::vector<jsk_recognition_msgs::HistogramWithRangeBin>
  topNHistogramWithRangeBins(const std::vector<jsk_recognition_msgs::HistogramWithRangeBin>& bins,
                             double top_n_rate);

  /**
   * @brief
   * draw bin to cv::Mat
   */
  void
  drawHistogramWithRangeBin(cv::Mat& image,
                            const jsk_recognition_msgs::HistogramWithRangeBin& bin,
                            float min_width_value,
                            float max_width_value,
                            float max_height_value,
                            cv::Scalar color);
}

#endif
