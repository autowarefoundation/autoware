
// Copyright 2024 TIER IV, Inc.
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

#ifndef TEST_PARAMETER_HPP_
#define TEST_PARAMETER_HPP_

constexpr double deg2rad = 3.14159265 / 180.0;
constexpr double kmph2mps = 1.0 / 3.6;

constexpr double WHEELBASE = 3.5;

constexpr double THRESHOLD_INTERVAL = 1.0;
constexpr double THRESHOLD_RELATIVE_ANGLE = 115.0 * deg2rad;
constexpr double THRESHOLD_CURVATURE = 0.3;
constexpr double THRESHOLD_LATERAL_ACC = 5.0;
constexpr double THRESHOLD_LONGITUDINAL_MAX_ACC = 3.0;
constexpr double THRESHOLD_LONGITUDINAL_MIN_ACC = -6.0;
constexpr double THRESHOLD_STEERING = 35.0 * deg2rad;
constexpr double THRESHOLD_STEERING_RATE = 7.0 * deg2rad;
constexpr double THRESHOLD_VELOCITY_DEVIATION = 15.0 * kmph2mps;
constexpr double THRESHOLD_DISTANCE_DEVIATION = 3.0;
constexpr double THRESHOLD_LONGITUDINAL_DISTANCE_DEVIATION = 2.0;
constexpr double PARAMETER_FORWARD_TRAJECTORY_LENGTH_ACCELERATION = -5.0;
constexpr double PARAMETER_FORWARD_TRAJECTORY_LENGTH_MARGIN = 2.0;

#endif  // TEST_PARAMETER_HPP_
