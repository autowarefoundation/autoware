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

#include "byte_tracker.h"
#include "lapjv.h"

#include <cstddef>

std::vector<STrack *> ByteTracker::joint_stracks(
  std::vector<STrack *> & tlista, std::vector<STrack> & tlistb)
{
  std::map<int, int> exists;
  std::vector<STrack *> res;
  for (size_t i = 0; i < tlista.size(); i++) {
    exists.insert(std::pair<int, int>(tlista[i]->track_id, 1));
    res.push_back(tlista[i]);
  }
  for (size_t i = 0; i < tlistb.size(); i++) {
    int tid = tlistb[i].track_id;
    if (!exists[tid] || exists.count(tid) == 0) {
      exists[tid] = 1;
      res.push_back(&tlistb[i]);
    }
  }
  return res;
}

std::vector<STrack> ByteTracker::joint_stracks(
  std::vector<STrack> & tlista, std::vector<STrack> & tlistb)
{
  std::map<int, int> exists;
  std::vector<STrack> res;
  for (size_t i = 0; i < tlista.size(); i++) {
    exists.insert(std::pair<int, int>(tlista[i].track_id, 1));
    res.push_back(tlista[i]);
  }
  for (size_t i = 0; i < tlistb.size(); i++) {
    int tid = tlistb[i].track_id;
    if (!exists[tid] || exists.count(tid) == 0) {
      exists[tid] = 1;
      res.push_back(tlistb[i]);
    }
  }
  return res;
}

std::vector<STrack> ByteTracker::sub_stracks(
  std::vector<STrack> & tlista, std::vector<STrack> & tlistb)
{
  std::map<int, STrack> stracks;
  for (size_t i = 0; i < tlista.size(); i++) {
    stracks.insert(std::pair<int, STrack>(tlista[i].track_id, tlista[i]));
  }
  for (size_t i = 0; i < tlistb.size(); i++) {
    int tid = tlistb[i].track_id;
    if (stracks.count(tid) != 0) {
      stracks.erase(tid);
    }
  }

  std::vector<STrack> res;
  std::map<int, STrack>::iterator it;
  for (it = stracks.begin(); it != stracks.end(); ++it) {
    res.push_back(it->second);
  }

  return res;
}

void ByteTracker::remove_duplicate_stracks(
  std::vector<STrack> & resa, std::vector<STrack> & resb, std::vector<STrack> & stracksa,
  std::vector<STrack> & stracksb)
{
  std::vector<std::vector<float>> pdist = iou_distance(stracksa, stracksb);
  std::vector<std::pair<int, int>> pairs;
  for (size_t i = 0; i < pdist.size(); i++) {
    for (size_t j = 0; j < pdist[i].size(); j++) {
      if (pdist[i][j] < 0.15) {
        pairs.push_back(std::pair<int, int>(i, j));
      }
    }
  }

  std::vector<int> dupa, dupb;
  for (size_t i = 0; i < pairs.size(); i++) {
    int timep = stracksa[pairs[i].first].frame_id - stracksa[pairs[i].first].start_frame;
    int timeq = stracksb[pairs[i].second].frame_id - stracksb[pairs[i].second].start_frame;
    if (timep > timeq)
      dupb.push_back(pairs[i].second);
    else
      dupa.push_back(pairs[i].first);
  }

  for (size_t i = 0; i < stracksa.size(); i++) {
    std::vector<int>::iterator iter = find(dupa.begin(), dupa.end(), i);
    if (iter == dupa.end()) {
      resa.push_back(stracksa[i]);
    }
  }

  for (size_t i = 0; i < stracksb.size(); i++) {
    std::vector<int>::iterator iter = find(dupb.begin(), dupb.end(), i);
    if (iter == dupb.end()) {
      resb.push_back(stracksb[i]);
    }
  }
}

void ByteTracker::linear_assignment(
  std::vector<std::vector<float>> & cost_matrix, int cost_matrix_size, int cost_matrix_size_size,
  float thresh, std::vector<std::vector<int>> & matches, std::vector<int> & unmatched_a,
  std::vector<int> & unmatched_b)
{
  if (cost_matrix.size() == 0) {
    for (int i = 0; i < cost_matrix_size; i++) {
      unmatched_a.push_back(i);
    }
    for (int i = 0; i < cost_matrix_size_size; i++) {
      unmatched_b.push_back(i);
    }
    return;
  }

  std::vector<int> rowsol;
  std::vector<int> colsol;
  [[maybe_unused]] float c = lapjv(cost_matrix, rowsol, colsol, true, thresh);
  for (size_t i = 0; i < rowsol.size(); i++) {
    if (rowsol[i] >= 0) {
      std::vector<int> match;
      match.push_back(i);
      match.push_back(rowsol[i]);
      matches.push_back(match);
    } else {
      unmatched_a.push_back(i);
    }
  }

  for (size_t i = 0; i < colsol.size(); i++) {
    if (colsol[i] < 0) {
      unmatched_b.push_back(i);
    }
  }
}

std::vector<std::vector<float>> ByteTracker::ious(
  std::vector<std::vector<float>> & atlbrs, std::vector<std::vector<float>> & btlbrs)
{
  std::vector<std::vector<float>> ious;
  if (atlbrs.size() * btlbrs.size() == 0) return ious;

  ious.resize(atlbrs.size());
  for (size_t i = 0; i < ious.size(); i++) {
    ious[i].resize(btlbrs.size());
  }

  // bbox_ious
  for (size_t k = 0; k < btlbrs.size(); k++) {
    std::vector<float> ious_tmp;
    float box_area = (btlbrs[k][2] - btlbrs[k][0] + 1) * (btlbrs[k][3] - btlbrs[k][1] + 1);
    for (size_t n = 0; n < atlbrs.size(); n++) {
      float iw = std::min(atlbrs[n][2], btlbrs[k][2]) - std::max(atlbrs[n][0], btlbrs[k][0]) + 1;
      if (iw > 0) {
        float ih = std::min(atlbrs[n][3], btlbrs[k][3]) - std::max(atlbrs[n][1], btlbrs[k][1]) + 1;
        if (ih > 0) {
          float ua = (atlbrs[n][2] - atlbrs[n][0] + 1) * (atlbrs[n][3] - atlbrs[n][1] + 1) +
                     box_area - iw * ih;
          ious[n][k] = iw * ih / ua;
        } else {
          ious[n][k] = 0.0;
        }
      } else {
        ious[n][k] = 0.0;
      }
    }
  }

  return ious;
}

std::vector<std::vector<float>> ByteTracker::iou_distance(
  std::vector<STrack *> & atracks, std::vector<STrack> & btracks, int & dist_size,
  int & dist_size_size)
{
  std::vector<std::vector<float>> cost_matrix;
  if (atracks.size() * btracks.size() == 0) {
    dist_size = atracks.size();
    dist_size_size = btracks.size();
    return cost_matrix;
  }
  std::vector<std::vector<float>> atlbrs, btlbrs;
  for (size_t i = 0; i < atracks.size(); i++) {
    atlbrs.push_back(atracks[i]->tlbr);
  }
  for (size_t i = 0; i < btracks.size(); i++) {
    btlbrs.push_back(btracks[i].tlbr);
  }

  dist_size = atracks.size();
  dist_size_size = btracks.size();

  std::vector<std::vector<float>> _ious = ious(atlbrs, btlbrs);

  for (size_t i = 0; i < _ious.size(); i++) {
    std::vector<float> _iou;
    for (size_t j = 0; j < _ious[i].size(); j++) {
      _iou.push_back(1 - _ious[i][j]);
    }
    cost_matrix.push_back(_iou);
  }

  return cost_matrix;
}

std::vector<std::vector<float>> ByteTracker::iou_distance(
  std::vector<STrack> & atracks, std::vector<STrack> & btracks)
{
  std::vector<std::vector<float>> atlbrs, btlbrs;
  for (size_t i = 0; i < atracks.size(); i++) {
    atlbrs.push_back(atracks[i].tlbr);
  }
  for (size_t i = 0; i < btracks.size(); i++) {
    btlbrs.push_back(btracks[i].tlbr);
  }

  std::vector<std::vector<float>> _ious = ious(atlbrs, btlbrs);
  std::vector<std::vector<float>> cost_matrix;
  for (size_t i = 0; i < _ious.size(); i++) {
    std::vector<float> _iou;
    for (size_t j = 0; j < _ious[i].size(); j++) {
      _iou.push_back(1 - _ious[i][j]);
    }
    cost_matrix.push_back(_iou);
  }

  return cost_matrix;
}

double ByteTracker::lapjv(
  const std::vector<std::vector<float>> & cost, std::vector<int> & rowsol,
  std::vector<int> & colsol, bool extend_cost, float cost_limit, bool return_cost)
{
  std::vector<std::vector<float>> cost_c;
  cost_c.assign(cost.begin(), cost.end());

  std::vector<std::vector<float>> cost_c_extended;

  int n_rows = cost.size();
  int n_cols = cost[0].size();
  rowsol.resize(n_rows);
  colsol.resize(n_cols);

  int n = 0;
  if (n_rows == n_cols) {
    n = n_rows;
  } else {
    if (!extend_cost) {
      std::cout << "set extend_cost=True" << std::endl;
      // system("pause");
      exit(0);
    }
  }

  if (extend_cost || cost_limit < LONG_MAX) {
    n = n_rows + n_cols;
    cost_c_extended.resize(n);
    for (size_t i = 0; i < cost_c_extended.size(); i++) cost_c_extended[i].resize(n);

    if (cost_limit < LONG_MAX) {
      for (size_t i = 0; i < cost_c_extended.size(); i++) {
        for (size_t j = 0; j < cost_c_extended[i].size(); j++) {
          cost_c_extended[i][j] = cost_limit / 2.0;
        }
      }
    } else {
      float cost_max = -1;
      for (size_t i = 0; i < cost_c.size(); i++) {
        for (size_t j = 0; j < cost_c[i].size(); j++) {
          if (cost_c[i][j] > cost_max) cost_max = cost_c[i][j];
        }
      }
      for (size_t i = 0; i < cost_c_extended.size(); i++) {
        for (size_t j = 0; j < cost_c_extended[i].size(); j++) {
          cost_c_extended[i][j] = cost_max + 1;
        }
      }
    }

    for (size_t i = n_rows; i < cost_c_extended.size(); i++) {
      for (size_t j = n_cols; j < cost_c_extended[i].size(); j++) {
        cost_c_extended[i][j] = 0;
      }
    }
    for (int i = 0; i < n_rows; i++) {
      for (int j = 0; j < n_cols; j++) {
        cost_c_extended[i][j] = cost_c[i][j];
      }
    }

    cost_c.clear();
    cost_c.assign(cost_c_extended.begin(), cost_c_extended.end());
  }

  double ** cost_ptr;
  cost_ptr = new double *[sizeof(double *) * n];
  for (int i = 0; i < n; i++) cost_ptr[i] = new double[sizeof(double) * n];

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      cost_ptr[i][j] = cost_c[i][j];
    }
  }

  int * x_c = new int[sizeof(int) * n];
  int * y_c = new int[sizeof(int) * n];

  int ret = lapjv_internal(n, cost_ptr, x_c, y_c);
  if (ret != 0) {
    std::cout << "Calculate Wrong!" << std::endl;
    // system("pause");
    exit(0);
  }

  double opt = 0.0;

  if (n != n_rows) {
    for (int i = 0; i < n; i++) {
      if (x_c[i] >= n_cols) x_c[i] = -1;
      if (y_c[i] >= n_rows) y_c[i] = -1;
    }
    for (int i = 0; i < n_rows; i++) {
      rowsol[i] = x_c[i];
    }
    for (int i = 0; i < n_cols; i++) {
      colsol[i] = y_c[i];
    }

    if (return_cost) {
      for (size_t i = 0; i < rowsol.size(); i++) {
        if (rowsol[i] != -1) {
          opt += cost_ptr[i][rowsol[i]];
        }
      }
    }
  } else if (return_cost) {
    for (size_t i = 0; i < rowsol.size(); i++) {
      opt += cost_ptr[i][rowsol[i]];
    }
  }

  for (int i = 0; i < n; i++) {
    delete[] cost_ptr[i];
  }
  delete[] cost_ptr;
  delete[] x_c;
  delete[] y_c;

  return opt;
}

cv::Scalar ByteTracker::get_color(int idx)
{
  idx += 3;
  return cv::Scalar(37 * idx % 255, 17 * idx % 255, 29 * idx % 255);
}
