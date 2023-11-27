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

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <boost/uuid/uuid_generators.hpp>

#include <yaml-cpp/yaml.h>

// init static variable
bool STrack::_parameters_loaded = false;
STrack::KfParams STrack::_kf_parameters;

STrack::STrack(std::vector<float> input_tlwh, float score, int label)
{
  original_tlwh.resize(4);
  original_tlwh.assign(input_tlwh.begin(), input_tlwh.end());

  is_activated = false;
  track_id = 0;
  state = TrackState::New;

  tlwh.resize(4);
  tlbr.resize(4);

  static_tlwh();  // update object size
  static_tlbr();  // update object size
  frame_id = 0;
  tracklet_len = 0;
  this->score = score;
  start_frame = 0;
  this->label = label;

  // load static kf parameters: initialized once in program
  const std::string package_share_directory =
    ament_index_cpp::get_package_share_directory("bytetrack");
  const std::string default_config_path =
    package_share_directory + "/config/bytetracker.param.yaml";
  if (!_parameters_loaded) {
    load_parameters(default_config_path);
    _parameters_loaded = true;
  }
}

STrack::~STrack()
{
}

void STrack::init_kalman_filter()
{
  // assert parameter is loaded
  assert(_parameters_loaded);

  // init kalman filter state
  Eigen::MatrixXd X0 = Eigen::MatrixXd::Zero(_kf_parameters.dim_x, 1);
  Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(_kf_parameters.dim_x, _kf_parameters.dim_x);
  X0(IDX::X1) = this->original_tlwh[0];
  X0(IDX::Y1) = this->original_tlwh[1];
  X0(IDX::X2) = this->original_tlwh[2];
  X0(IDX::Y2) = this->original_tlwh[3];
  P0(IDX::X1, IDX::X1) = _kf_parameters.p0_cov_x;
  P0(IDX::Y1, IDX::Y1) = _kf_parameters.p0_cov_y;
  P0(IDX::X2, IDX::X2) = _kf_parameters.p0_cov_x;
  P0(IDX::Y2, IDX::Y2) = _kf_parameters.p0_cov_y;
  P0(IDX::VX1, IDX::VX1) = _kf_parameters.p0_cov_vx;
  P0(IDX::VY1, IDX::VY1) = _kf_parameters.p0_cov_vy;
  P0(IDX::VX2, IDX::VX2) = _kf_parameters.p0_cov_vx;
  P0(IDX::VY2, IDX::VY2) = _kf_parameters.p0_cov_vy;
  this->kalman_filter_.init(X0, P0);
}

/** init a tracklet */
void STrack::activate(int frame_id)
{
  this->track_id = this->next_id();
  this->unique_id = boost::uuids::random_generator()();

  std::vector<float> _tlwh_tmp(4);
  _tlwh_tmp[0] = this->original_tlwh[0];
  _tlwh_tmp[1] = this->original_tlwh[1];
  _tlwh_tmp[2] = this->original_tlwh[2];
  _tlwh_tmp[3] = this->original_tlwh[3];
  std::vector<float> xyah = tlwh_to_xyah(_tlwh_tmp);
  // init kf
  init_kalman_filter();
  // reflect state
  reflect_state();

  this->tracklet_len = 0;
  this->state = TrackState::Tracked;
  this->is_activated = true;
  this->frame_id = frame_id;
  this->start_frame = frame_id;
}

void STrack::re_activate(STrack & new_track, int frame_id, bool new_id)
{
  std::vector<float> xyah = tlwh_to_xyah(new_track.tlwh);
  // TODO(me): write kf update
  Eigen::MatrixXd measurement = Eigen::MatrixXd::Zero(_kf_parameters.dim_z, 1);
  measurement << new_track.tlwh[0], new_track.tlwh[1], new_track.tlwh[2], new_track.tlwh[3];
  update_kalman_filter(measurement);

  reflect_state();

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

  // update
  // TODO(me): write update
  Eigen::MatrixXd measurement = Eigen::MatrixXd::Zero(_kf_parameters.dim_z, 1);
  measurement << new_track.tlwh[0], new_track.tlwh[1], new_track.tlwh[2], new_track.tlwh[3];
  update_kalman_filter(measurement);

  reflect_state();

  this->state = TrackState::Tracked;
  this->is_activated = true;

  this->score = new_track.score;
}

/** reflect kalman filter state to current object variables*/
void STrack::reflect_state()
{
  // update tlwh
  static_tlwh();
  // update tlbr
  static_tlbr();
}

void STrack::static_tlwh()
{
  if (this->state == TrackState::New) {
    tlwh[0] = original_tlwh[0];
    tlwh[1] = original_tlwh[1];
    tlwh[2] = original_tlwh[2];
    tlwh[3] = original_tlwh[3];
    return;
  }
  // put kf state to tlwh
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(_kf_parameters.dim_x, 1);
  this->kalman_filter_.getX(X);
  tlwh[0] = X(IDX::X1);
  tlwh[1] = X(IDX::Y1);
  tlwh[2] = X(IDX::X2);
  tlwh[3] = X(IDX::Y2);
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

void STrack::multi_predict(std::vector<STrack *> & stracks)
{
  for (size_t i = 0; i < stracks.size(); i++) {
    if (stracks[i]->state != TrackState::Tracked) {
      // not tracked
    }
    // prediction
    stracks[i]->predict(stracks[i]->frame_id + 1);
    // TODO(me): write prediction
    stracks[i]->static_tlwh();
    stracks[i]->static_tlbr();
  }
}

void STrack::update_kalman_filter(const Eigen::MatrixXd & measurement)
{
  // assert parameter is loaded
  assert(_parameters_loaded);

  // get C matrix
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(_kf_parameters.dim_z, _kf_parameters.dim_x);
  C(IDX::X1, IDX::X1) = 1;
  C(IDX::Y1, IDX::Y1) = 1;
  C(IDX::X2, IDX::X2) = 1;
  C(IDX::Y2, IDX::Y2) = 1;

  // get R matrix
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(_kf_parameters.dim_z, _kf_parameters.dim_z);
  R(IDX::X1, IDX::X1) = _kf_parameters.r_cov_x;
  R(IDX::Y1, IDX::Y1) = _kf_parameters.r_cov_y;
  R(IDX::X2, IDX::X2) = _kf_parameters.r_cov_x;
  R(IDX::Y2, IDX::Y2) = _kf_parameters.r_cov_y;

  // update
  if (!this->kalman_filter_.update(measurement, C, R)) {
    std::cerr << "Cannot update" << std::endl;
  }
}

void STrack::predict(const int frame_id)
{
  // check state is Tracked
  if (this->state != TrackState::Tracked) {
    // not tracked
    return;
  }

  // else do prediction
  float time_elapsed = _kf_parameters.dt * (frame_id - this->frame_id);
  // A matrix
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(_kf_parameters.dim_x, _kf_parameters.dim_x);
  A(IDX::X1, IDX::VX1) = time_elapsed;
  A(IDX::Y1, IDX::VY1) = time_elapsed;
  A(IDX::X2, IDX::VX2) = time_elapsed;
  A(IDX::Y2, IDX::VY2) = time_elapsed;

  // u and B matrix
  Eigen::MatrixXd u = Eigen::MatrixXd::Zero(_kf_parameters.dim_x, 1);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(_kf_parameters.dim_x, _kf_parameters.dim_x);

  // get P_t
  Eigen::MatrixXd P_t = Eigen::MatrixXd::Zero(_kf_parameters.dim_x, _kf_parameters.dim_x);
  this->kalman_filter_.getP(P_t);

  // Q matrix
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(_kf_parameters.dim_x, _kf_parameters.dim_x);
  Q(IDX::X1, IDX::X1) = _kf_parameters.q_cov_x;
  Q(IDX::Y1, IDX::Y1) = _kf_parameters.q_cov_y;
  Q(IDX::VX1, IDX::VX1) = _kf_parameters.q_cov_vx;
  Q(IDX::VY1, IDX::VY1) = _kf_parameters.q_cov_vy;
  Q(IDX::X2, IDX::X2) = _kf_parameters.q_cov_x;
  Q(IDX::Y2, IDX::Y2) = _kf_parameters.q_cov_y;
  Q(IDX::VX2, IDX::VX2) = _kf_parameters.q_cov_vx;
  Q(IDX::VY2, IDX::VY2) = _kf_parameters.q_cov_vy;

  // prediction
  if (!this->kalman_filter_.predict(u, A, B, Q)) {
    std::cerr << "Cannot predict" << std::endl;
  }
}

void STrack::load_parameters(const std::string & path)
{
  YAML::Node config = YAML::LoadFile(path);
  // initialize ekf params
  _kf_parameters.dim_x = config["dim_x"].as<int>();
  _kf_parameters.dim_z = config["dim_z"].as<int>();
  _kf_parameters.q_cov_x = config["q_cov_x"].as<float>();
  _kf_parameters.q_cov_y = config["q_cov_y"].as<float>();
  _kf_parameters.q_cov_vx = config["q_cov_vx"].as<float>();
  _kf_parameters.q_cov_vy = config["q_cov_vy"].as<float>();
  _kf_parameters.r_cov_x = config["r_cov_x"].as<float>();
  _kf_parameters.r_cov_y = config["r_cov_y"].as<float>();
  _kf_parameters.p0_cov_x = config["p0_cov_x"].as<float>();
  _kf_parameters.p0_cov_y = config["p0_cov_y"].as<float>();
  _kf_parameters.p0_cov_vx = config["p0_cov_vx"].as<float>();
  _kf_parameters.p0_cov_vy = config["p0_cov_vy"].as<float>();
  _kf_parameters.dt = config["dt"].as<float>();
}
