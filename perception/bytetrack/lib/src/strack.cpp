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

#include "strack.h"

#include <boost/uuid/uuid_generators.hpp>

STrack::STrack(std::vector<float> tlwh_, float score, int label)
{
  _tlwh.resize(4);
  _tlwh.assign(tlwh_.begin(), tlwh_.end());

  is_activated = false;
  track_id = 0;
  state = TrackState::New;

  tlwh.resize(4);
  tlbr.resize(4);

  static_tlwh();
  static_tlbr();
  frame_id = 0;
  tracklet_len = 0;
  this->score = score;
  start_frame = 0;
  this->label = label;
}

STrack::~STrack()
{
}

void STrack::activate(byte_kalman::KalmanFilter & kalman_filter, int frame_id)
{
  this->kalman_filter = kalman_filter;
  this->track_id = this->next_id();
  this->unique_id = boost::uuids::random_generator()();

  std::vector<float> _tlwh_tmp(4);
  _tlwh_tmp[0] = this->_tlwh[0];
  _tlwh_tmp[1] = this->_tlwh[1];
  _tlwh_tmp[2] = this->_tlwh[2];
  _tlwh_tmp[3] = this->_tlwh[3];
  std::vector<float> xyah = tlwh_to_xyah(_tlwh_tmp);
  DETECTBOX xyah_box;
  xyah_box[0] = xyah[0];
  xyah_box[1] = xyah[1];
  xyah_box[2] = xyah[2];
  xyah_box[3] = xyah[3];
  auto mc = this->kalman_filter.initiate(xyah_box);
  this->mean = mc.first;
  this->covariance = mc.second;

  static_tlwh();
  static_tlbr();

  this->tracklet_len = 0;
  this->state = TrackState::Tracked;
  // if (frame_id == 1)
  // {
  //   this->is_activated = true;
  // }
  this->is_activated = true;
  this->frame_id = frame_id;
  this->start_frame = frame_id;
}

void STrack::re_activate(STrack & new_track, int frame_id, bool new_id)
{
  std::vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
  DETECTBOX xyah_box;
  xyah_box[0] = xyah[0];
  xyah_box[1] = xyah[1];
  xyah_box[2] = xyah[2];
  xyah_box[3] = xyah[3];
  auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
  this->mean = mc.first;
  this->covariance = mc.second;

  static_tlwh();
  static_tlbr();

  this->tracklet_len = 0;
  this->state = TrackState::Tracked;
  this->is_activated = true;
  this->frame_id = frame_id;
  this->score = new_track.score;
  if (new_id) {
    this->track_id = next_id();
    this->unique_id = boost::uuids::random_generator()();
  }
}

void STrack::update(STrack & new_track, int frame_id)
{
  this->frame_id = frame_id;
  this->tracklet_len++;

  std::vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
  DETECTBOX xyah_box;
  xyah_box[0] = xyah[0];
  xyah_box[1] = xyah[1];
  xyah_box[2] = xyah[2];
  xyah_box[3] = xyah[3];

  auto mc = this->kalman_filter.update(this->mean, this->covariance, xyah_box);
  this->mean = mc.first;
  this->covariance = mc.second;

  static_tlwh();
  static_tlbr();

  this->state = TrackState::Tracked;
  this->is_activated = true;

  this->score = new_track.score;
}

void STrack::static_tlwh()
{
  if (this->state == TrackState::New) {
    tlwh[0] = _tlwh[0];
    tlwh[1] = _tlwh[1];
    tlwh[2] = _tlwh[2];
    tlwh[3] = _tlwh[3];
    return;
  }

  tlwh[0] = mean[0];
  tlwh[1] = mean[1];
  tlwh[2] = mean[2];
  tlwh[3] = mean[3];

  tlwh[2] *= tlwh[3];
  tlwh[0] -= tlwh[2] / 2;
  tlwh[1] -= tlwh[3] / 2;
}

void STrack::static_tlbr()
{
  tlbr.clear();
  tlbr.assign(tlwh.begin(), tlwh.end());
  tlbr[2] += tlbr[0];
  tlbr[3] += tlbr[1];
}

std::vector<float> STrack::tlwh_to_xyah(std::vector<float> tlwh_tmp)
{
  std::vector<float> tlwh_output = tlwh_tmp;
  tlwh_output[0] += tlwh_output[2] / 2;
  tlwh_output[1] += tlwh_output[3] / 2;
  tlwh_output[2] /= tlwh_output[3];
  return tlwh_output;
}

std::vector<float> STrack::to_xyah()
{
  return tlwh_to_xyah(tlwh);
}

std::vector<float> STrack::tlbr_to_tlwh(std::vector<float> & tlbr)
{
  tlbr[2] -= tlbr[0];
  tlbr[3] -= tlbr[1];
  return tlbr;
}

void STrack::mark_lost()
{
  state = TrackState::Lost;
}

void STrack::mark_removed()
{
  state = TrackState::Removed;
}

int STrack::next_id()
{
  static int _count = 0;
  _count++;
  return _count;
}

int STrack::end_frame()
{
  return this->frame_id;
}

void STrack::multi_predict(
  std::vector<STrack *> & stracks, byte_kalman::KalmanFilter & kalman_filter)
{
  for (size_t i = 0; i < stracks.size(); i++) {
    if (stracks[i]->state != TrackState::Tracked) {
      stracks[i]->mean[7] = 0;
    }
    kalman_filter.predict(stracks[i]->mean, stracks[i]->covariance);
    stracks[i]->static_tlwh();
    stracks[i]->static_tlbr();
  }
}
