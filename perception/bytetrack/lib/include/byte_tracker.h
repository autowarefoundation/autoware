/*
 * MIT License
 *
 * Copyright (c) 2021 Yifu Zhang
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// Copyright 2023 TIER IV, Inc.
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

#include "strack.h"

#include <vector>

struct ByteTrackObject
{
  cv::Rect_<float> rect;
  int label;
  float prob;
};

class ByteTracker
{
public:
  ByteTracker(
    int track_buffer = 30, float track_thresh = 0.5, float high_thresh = 0.6,
    float match_thresh = 0.8);
  ~ByteTracker();

  std::vector<STrack> update(const std::vector<ByteTrackObject> & objects);
  cv::Scalar get_color(int idx);

private:
  std::vector<STrack *> joint_stracks(std::vector<STrack *> & tlista, std::vector<STrack> & tlistb);
  std::vector<STrack> joint_stracks(std::vector<STrack> & tlista, std::vector<STrack> & tlistb);

  std::vector<STrack> sub_stracks(std::vector<STrack> & tlista, std::vector<STrack> & tlistb);
  void remove_duplicate_stracks(
    std::vector<STrack> & resa, std::vector<STrack> & resb, std::vector<STrack> & stracksa,
    std::vector<STrack> & stracksb);

  void linear_assignment(
    std::vector<std::vector<float>> & cost_matrix, int cost_matrix_size, int cost_matrix_size_size,
    float thresh, std::vector<std::vector<int>> & matches, std::vector<int> & unmatched_a,
    std::vector<int> & unmatched_b);
  std::vector<std::vector<float>> iou_distance(
    std::vector<STrack *> & atracks, std::vector<STrack> & btracks, int & dist_size,
    int & dist_size_size);
  std::vector<std::vector<float>> iou_distance(
    std::vector<STrack> & atracks, std::vector<STrack> & btracks);
  std::vector<std::vector<float>> ious(
    std::vector<std::vector<float>> & atlbrs, std::vector<std::vector<float>> & btlbrs);

  double lapjv(
    const std::vector<std::vector<float>> & cost, std::vector<int> & rowsol,
    std::vector<int> & colsol, bool extend_cost = false, float cost_limit = LONG_MAX,
    bool return_cost = true);

private:
  float track_thresh;
  float high_thresh;
  float match_thresh;
  int frame_id;
  int max_time_lost;

  std::vector<STrack> tracked_stracks;
  std::vector<STrack> lost_stracks;
  std::vector<STrack> removed_stracks;
  KalmanFilter kalman_filter;
};
