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

#include "kalman_filter.h"

#include <opencv2/opencv.hpp>

#include <boost/uuid/uuid.hpp>

#include <vector>

enum TrackState { New = 0, Tracked, Lost, Removed };

class STrack
{
public:
  STrack(std::vector<float> tlwh_, float score, int label);
  ~STrack();

  std::vector<float> static tlbr_to_tlwh(std::vector<float> & tlbr);
  static void multi_predict(
    std::vector<STrack *> & stracks, byte_kalman::KalmanFilter & kalman_filter);
  void static_tlwh();
  void static_tlbr();
  std::vector<float> tlwh_to_xyah(std::vector<float> tlwh_tmp);
  std::vector<float> to_xyah();
  void mark_lost();
  void mark_removed();
  int next_id();
  int end_frame();

  void activate(byte_kalman::KalmanFilter & kalman_filter, int frame_id);
  void re_activate(STrack & new_track, int frame_id, bool new_id = false);
  void update(STrack & new_track, int frame_id);

public:
  bool is_activated;
  int track_id;
  boost::uuids::uuid unique_id;
  int state;

  std::vector<float> _tlwh;
  std::vector<float> tlwh;
  std::vector<float> tlbr;
  int frame_id;
  int tracklet_len;
  int start_frame;

  KAL_MEAN mean;
  KAL_COVA covariance;
  float score;

  int label;

private:
  byte_kalman::KalmanFilter kalman_filter;
};
