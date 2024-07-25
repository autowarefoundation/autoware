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

#include <cstddef>
#include <fstream>

ByteTracker::ByteTracker(
  int track_buffer, float track_thresh, float high_thresh, float match_thresh)
: track_thresh(track_thresh), high_thresh(high_thresh), match_thresh(match_thresh)

{
  frame_id = 0;
  max_time_lost = track_buffer;
  std::cout << "Init ByteTrack!" << std::endl;
}

ByteTracker::~ByteTracker()
{
}

std::vector<STrack> ByteTracker::update(const std::vector<ByteTrackObject> & objects)
{
  ////////////////// Step 1: Get detections //////////////////
  this->frame_id++;
  std::vector<STrack> activated_stracks;
  std::vector<STrack> refind_stracks;
  std::vector<STrack> removed_stracks;
  std::vector<STrack> lost_stracks;
  std::vector<STrack> detections;
  std::vector<STrack> detections_low;

  std::vector<STrack> detections_cp;
  std::vector<STrack> tracked_stracks_swap;
  std::vector<STrack> resa, resb;
  std::vector<STrack> output_stracks;

  std::vector<STrack *> unconfirmed;
  std::vector<STrack *> tracked_stracks;
  std::vector<STrack *> strack_pool;
  std::vector<STrack *> r_tracked_stracks;

  if (objects.size() > 0) {
    for (size_t i = 0; i < objects.size(); i++) {
      std::vector<float> tlbr_;
      tlbr_.resize(4);
      tlbr_[0] = objects[i].rect.x;
      tlbr_[1] = objects[i].rect.y;
      tlbr_[2] = objects[i].rect.x + objects[i].rect.width;
      tlbr_[3] = objects[i].rect.y + objects[i].rect.height;

      float score = objects[i].prob;

      STrack strack(STrack::tlbr_to_tlwh(tlbr_), score, objects[i].label);
      if (score >= track_thresh) {
        detections.push_back(strack);
      } else {
        detections_low.push_back(strack);
      }
    }
  }

  // Add newly detected tracklets to tracked_stracks
  for (size_t i = 0; i < this->tracked_stracks.size(); i++) {
    if (!this->tracked_stracks[i].is_activated)
      unconfirmed.push_back(&this->tracked_stracks[i]);
    else
      tracked_stracks.push_back(&this->tracked_stracks[i]);
  }

  ////////////////// Step 2: First association, with IoU //////////////////
  strack_pool = joint_stracks(tracked_stracks, this->lost_stracks);
  // do prediction for each stracks
  for (size_t i = 0; i < strack_pool.size(); i++) {
    strack_pool[i]->predict(this->frame_id);
  }

  std::vector<std::vector<float> > dists;
  int dist_size = 0, dist_size_size = 0;
  dists = iou_distance(strack_pool, detections, dist_size, dist_size_size);

  std::vector<std::vector<int> > matches;
  std::vector<int> u_track, u_detection;
  linear_assignment(dists, dist_size, dist_size_size, match_thresh, matches, u_track, u_detection);

  for (size_t i = 0; i < matches.size(); i++) {
    STrack * track = strack_pool[matches[i][0]];
    STrack * det = &detections[matches[i][1]];
    if (track->state == TrackState::Tracked) {
      track->update(*det, this->frame_id);
      activated_stracks.push_back(*track);
    } else {
      track->re_activate(*det, this->frame_id, false);
      refind_stracks.push_back(*track);
    }
  }

  ////////////////// Step 3: Second association, using low score dets //////////////////
  for (size_t i = 0; i < u_detection.size(); i++) {
    detections_cp.push_back(detections[u_detection[i]]);
  }
  detections.clear();
  detections.assign(detections_low.begin(), detections_low.end());

  for (size_t i = 0; i < u_track.size(); i++) {
    if (strack_pool[u_track[i]]->state == TrackState::Tracked) {
      r_tracked_stracks.push_back(strack_pool[u_track[i]]);
    }
  }

  dists.clear();
  dists = iou_distance(r_tracked_stracks, detections, dist_size, dist_size_size);

  matches.clear();
  u_track.clear();
  u_detection.clear();
  linear_assignment(dists, dist_size, dist_size_size, 0.5, matches, u_track, u_detection);

  for (size_t i = 0; i < matches.size(); i++) {
    STrack * track = r_tracked_stracks[matches[i][0]];
    STrack * det = &detections[matches[i][1]];
    if (track->state == TrackState::Tracked) {
      track->update(*det, this->frame_id);
      activated_stracks.push_back(*track);
    } else {
      track->re_activate(*det, this->frame_id, false);
      refind_stracks.push_back(*track);
    }
  }

  for (size_t i = 0; i < u_track.size(); i++) {
    STrack * track = r_tracked_stracks[u_track[i]];
    if (track->state != TrackState::Lost) {
      track->mark_lost();
      lost_stracks.push_back(*track);
    }
  }

  // Deal with unconfirmed tracks, usually tracks with only one beginning frame
  detections.clear();
  detections.assign(detections_cp.begin(), detections_cp.end());

  dists.clear();
  dists = iou_distance(unconfirmed, detections, dist_size, dist_size_size);

  matches.clear();
  std::vector<int> u_unconfirmed;
  u_detection.clear();
  linear_assignment(dists, dist_size, dist_size_size, 0.7, matches, u_unconfirmed, u_detection);

  for (size_t i = 0; i < matches.size(); i++) {
    unconfirmed[matches[i][0]]->update(detections[matches[i][1]], this->frame_id);
    activated_stracks.push_back(*unconfirmed[matches[i][0]]);
  }

  for (size_t i = 0; i < u_unconfirmed.size(); i++) {
    STrack * track = unconfirmed[u_unconfirmed[i]];
    track->mark_removed();
    removed_stracks.push_back(*track);
  }

  ////////////////// Step 4: Init new stracks //////////////////
  for (size_t i = 0; i < u_detection.size(); i++) {
    STrack * track = &detections[u_detection[i]];
    if (track->score < this->high_thresh) continue;
    track->activate(this->frame_id);
    activated_stracks.push_back(*track);
  }

  ////////////////// Step 5: Update state //////////////////
  for (size_t i = 0; i < this->lost_stracks.size(); i++) {
    if (this->frame_id - this->lost_stracks[i].end_frame() > this->max_time_lost) {
      this->lost_stracks[i].mark_removed();
      removed_stracks.push_back(this->lost_stracks[i]);
    }
  }

  for (size_t i = 0; i < this->tracked_stracks.size(); i++) {
    if (this->tracked_stracks[i].state == TrackState::Tracked) {
      tracked_stracks_swap.push_back(this->tracked_stracks[i]);
    }
  }
  this->tracked_stracks.clear();
  this->tracked_stracks.assign(tracked_stracks_swap.begin(), tracked_stracks_swap.end());

  this->tracked_stracks = joint_stracks(this->tracked_stracks, activated_stracks);
  this->tracked_stracks = joint_stracks(this->tracked_stracks, refind_stracks);

  this->lost_stracks = sub_stracks(this->lost_stracks, this->tracked_stracks);
  for (size_t i = 0; i < lost_stracks.size(); i++) {
    this->lost_stracks.push_back(lost_stracks[i]);
  }

  this->lost_stracks = sub_stracks(this->lost_stracks, this->removed_stracks);
  for (size_t i = 0; i < removed_stracks.size(); i++) {
    this->removed_stracks.push_back(removed_stracks[i]);
  }

  remove_duplicate_stracks(resa, resb, this->tracked_stracks, this->lost_stracks);

  this->tracked_stracks.clear();
  this->tracked_stracks.assign(resa.begin(), resa.end());
  this->lost_stracks.clear();
  this->lost_stracks.assign(resb.begin(), resb.end());

  for (size_t i = 0; i < this->tracked_stracks.size(); i++) {
    if (this->tracked_stracks[i].is_activated) {
      output_stracks.push_back(this->tracked_stracks[i]);
    }
  }
  return output_stracks;
}
