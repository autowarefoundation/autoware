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

#include "gtest/gtest.h"
#include "occlusion_spot_utils.hpp"
#include "utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using Point = geometry_msgs::msg::Point;
using Vector3 = geometry_msgs::msg::Vector3;
using DynamicObjects = autoware_perception_msgs::msg::PredictedObjects;
using DynamicObject = autoware_perception_msgs::msg::PredictedObject;
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::PathPoint;
using tier4_planning_msgs::msg::PathWithLaneId;

TEST(calcSlowDownPointsForPossibleCollision, TooManyPossibleCollisions)
{
  using autoware::behavior_velocity_planner::occlusion_spot_utils::
    calcSlowDownPointsForPossibleCollision;
  using autoware::behavior_velocity_planner::occlusion_spot_utils::PossibleCollisionInfo;
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::microseconds;
  std::vector<PossibleCollisionInfo> possible_collisions;
  size_t num = 2000;
  // make a path with 2000 points from x=0 to x=4
  tier4_planning_msgs::msg::PathWithLaneId path = test::generatePath(0.0, 3.0, 4.0, 3.0, num);
  // make 2000 possible collision from x=0 to x=10
  test::generatePossibleCollisions(possible_collisions, 0.0, 3.0, 4.0, 3.0, num);

  /**
   * @brief too many possible collisions on path
   *
   * Ego -col-col-col-col-col-col-col--col-col-col-col-col-col-col-col-col-> path
   *
   */

  auto start_naive = high_resolution_clock::now();
  calcSlowDownPointsForPossibleCollision(0, path, 0, possible_collisions);

  auto end_naive = high_resolution_clock::now();
  // 2000 path * 2000 possible collisions
  EXPECT_EQ(possible_collisions.size(), size_t{num});
  EXPECT_EQ(path.points.size(), size_t{num});
  std::cout << " runtime (microsec) "
            << duration_cast<microseconds>(end_naive - start_naive).count() << std::endl;
}

TEST(calcSlowDownPointsForPossibleCollision, ConsiderSignedOffset)
{
  using autoware::behavior_velocity_planner::occlusion_spot_utils::
    calcSlowDownPointsForPossibleCollision;
  using autoware::behavior_velocity_planner::occlusion_spot_utils::PossibleCollisionInfo;
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::microseconds;
  std::vector<PossibleCollisionInfo> pcs;
  // for public
  {
    const double offset_from_start_to_ego = 0;
    PathWithLaneId path = test::generatePath(0.0, 3.0, 6.0, 3.0, 7);
    for (size_t i = 0; i < path.points.size(); i++) {
      path.points[i].point.longitudinal_velocity_mps = static_cast<double>(i);
    }
    test::generatePossibleCollisions(pcs, 3.0, 3.0, 6.0, 3.0, 3);
    /**
     * @brief generated path and possible collisions : path start from 2 to 6
     *    0 1 2 3 4 5 6
     * x: e-p-p-p-p-p-p-> path
     * v: 0-1-2-3-4-5-6-> velocity
     * c: N-N-N-c-NcN-c-> collision
     *    --->| longitudinal offset
     *    e : ego
     *    p : path
     *    c : collision
     */
    calcSlowDownPointsForPossibleCollision(0, path, -offset_from_start_to_ego, pcs);
    if (pcs[0].collision_with_margin.longitudinal_velocity_mps != 3.0) {
      for (size_t i = 0; i < path.points.size(); i++) {
        std::cout << "v : " << path.points[i].point.longitudinal_velocity_mps << "\t";
      }
      std::cout << std::endl;
      for (const auto & pc : pcs) {
        std::cout << "len : " << pc.arc_lane_dist_at_collision.length << "\t";
      }
      std::cout << std::endl;
    }
    EXPECT_DOUBLE_EQ(pcs[0].collision_with_margin.longitudinal_velocity_mps, 3);
    EXPECT_DOUBLE_EQ(pcs[1].collision_with_margin.longitudinal_velocity_mps, 4);
    EXPECT_DOUBLE_EQ(pcs[2].collision_with_margin.longitudinal_velocity_mps, 6);
  }

  pcs.clear();
}
