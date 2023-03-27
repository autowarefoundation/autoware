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

#include <bytetrack/bytetrack.hpp>

#include <algorithm>
#include <fstream>
#include <functional>

namespace bytetrack
{
ByteTrack::ByteTrack(const int track_buffer_length)
{
  // Tracker initialization
  tracker_ = std::make_unique<ByteTracker>(track_buffer_length);
}

bool ByteTrack::do_inference(ObjectArray & objects)
{
  // Re-format data
  std::vector<ByteTrackObject> bytetrack_objects;
  for (auto & obj : objects) {
    ByteTrackObject bytetrack_obj;
    bytetrack_obj.rect = cv::Rect(obj.x_offset, obj.y_offset, obj.width, obj.height);
    bytetrack_obj.prob = obj.score;
    bytetrack_obj.label = obj.type;
    bytetrack_objects.emplace_back(bytetrack_obj);
  }

  // Update tracker
  std::vector<STrack> output_stracks = tracker_->update(bytetrack_objects);

  // Pack results
  latest_objects_.clear();
  for (const auto & tracking_result : output_stracks) {
    Object object{};
    std::vector<float> tlwh = tracking_result.tlwh;
    object.x_offset = tlwh[0];
    object.y_offset = tlwh[1];
    object.width = tlwh[2];
    object.height = tlwh[3];
    object.score = tracking_result.score;
    object.type = tracking_result.label;
    object.track_id = tracking_result.track_id;
    object.unique_id = tracking_result.unique_id;
    latest_objects_.emplace_back(object);
  }

  return true;
}

ObjectArray ByteTrack::update_tracker(ObjectArray & input_objects)
{
  do_inference(input_objects);
  return latest_objects_;
}
}  // namespace bytetrack
