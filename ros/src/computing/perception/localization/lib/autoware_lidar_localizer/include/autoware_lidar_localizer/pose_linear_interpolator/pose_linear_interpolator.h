/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef POSE_LINEAR_INTERPOLATOR_H
#define POSE_LINEAR_INTERPOLATOR_H

#include "autoware_lidar_localizer/util/data_structs.h"

PoseStamped interpolatePose(const PoseStamped &pose_a,
                            const PoseStamped &pose_b, const double time_stamp);

class PoseLinearInterpolator {
public:
  PoseLinearInterpolator();
  ~PoseLinearInterpolator() = default;
  void clearPoseStamped();
  bool isNotSetPoseStamped() const;
  void pushbackPoseStamped(const PoseStamped &pose);
  PoseStamped getInterpolatePoseStamped(const double time_stamp) const;
  PoseStamped getCurrentPoseStamped() const;
  PoseStamped getPrevPoseStamped() const;
  Velocity getVelocity() const;

private:
  PoseStamped current_pose_;
  PoseStamped prev_pose_;
};

#endif
