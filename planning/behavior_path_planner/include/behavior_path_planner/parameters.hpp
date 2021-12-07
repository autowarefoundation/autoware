// Copyright 2021 Tier IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__PARAMETERS_HPP_
#define BEHAVIOR_PATH_PLANNER__PARAMETERS_HPP_

struct BehaviorPathPlannerParameters
{
  double backward_path_length;
  double forward_path_length;
  double backward_length_buffer_for_end_of_lane;
  double backward_length_buffer_for_end_of_pull_over;
  double backward_length_buffer_for_end_of_pull_out;
  double minimum_lane_change_length;
  double minimum_pull_over_length;
  double minimum_pull_out_length;
  double drivable_area_resolution;
  double drivable_area_width;
  double drivable_area_height;
  double refine_goal_search_radius_range;
  double turn_light_on_threshold_dis_lat;
  double turn_light_on_threshold_dis_long;
  double turn_light_on_threshold_time;

  // vehicle info
  double wheel_base;
  double front_overhang;
  double rear_overhang;
  double vehicle_width;
  double vehicle_length;
  double wheel_tread;
  double left_over_hang;
  double right_over_hang;
  double base_link2front;
  double base_link2rear;
};

#endif  // BEHAVIOR_PATH_PLANNER__PARAMETERS_HPP_
