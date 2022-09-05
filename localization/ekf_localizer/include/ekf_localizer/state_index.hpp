// Copyright 2022 Autoware Foundation
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

#ifndef EKF_LOCALIZER__STATE_INDEX_HPP_
#define EKF_LOCALIZER__STATE_INDEX_HPP_

enum IDX {
  X = 0,
  Y = 1,
  YAW = 2,
  YAWB = 3,
  VX = 4,
  WZ = 5,
};

#endif  // EKF_LOCALIZER__STATE_INDEX_HPP_
