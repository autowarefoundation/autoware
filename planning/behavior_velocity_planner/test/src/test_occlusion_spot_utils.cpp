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

#include "utils.hpp"

#include <scene_module/occlusion_spot/occlusion_spot_utils.hpp>
#include <utilization/path_utilization.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

autoware_auto_planning_msgs::msg::Path toPath(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path_with_id)
{
  autoware_auto_planning_msgs::msg::Path path;
  for (const auto & p : path_with_id.points) {
    path.points.push_back(p.point);
  }
  return path;
}

TEST(spline, splineInterpolate)
{
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::microseconds;
  autoware_auto_planning_msgs::msg::PathWithLaneId path = test::generatePath(0, 0.0, 6.0, 0.0, 7);
  const auto path_interp = behavior_velocity_planner::interpolatePath(toPath(path), 100, 0.5);
  for (const auto & p : path_interp.points) {
    std::cout << "interp" << p.pose.position.x << std::endl;
  }
  ASSERT_EQ(path_interp.points.size(), path.points.size() * 2 - +1);
}

TEST(buildPathLanelet, nominal)
{
  using behavior_velocity_planner::occlusion_spot_utils::buildPathLanelet;
  lanelet::ConstLanelet path_lanelet;
  /* straight diagonal path
      0 1 2 3 4
    0 x
    1   x
    2     x
    3       x
    4         x
  */
  autoware_auto_planning_msgs::msg::PathWithLaneId path = test::generatePath(0, 0, 4, 4, 5);
  path_lanelet = buildPathLanelet(path);
  ASSERT_EQ(path_lanelet.centerline2d().front().x(), 0.0);
  ASSERT_EQ(path_lanelet.centerline2d().front().y(), 0.0);
  ASSERT_NE(path_lanelet.centerline2d().back().x(), 4.0);
  ASSERT_NE(path_lanelet.centerline2d().back().y(), 4.0);
  std::cout << "path lanelet size: " << path_lanelet.centerline2d().size() << std::endl;
}

TEST(calcSlowDownPointsForPossibleCollision, TooManyPossibleCollisions)
{
  using behavior_velocity_planner::occlusion_spot_utils::calcSlowDownPointsForPossibleCollision;
  using behavior_velocity_planner::occlusion_spot_utils::PossibleCollisionInfo;
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::microseconds;
  std::vector<PossibleCollisionInfo> possible_collisions;
  // make a path with 2000 points from x=0 to x=4
  autoware_auto_planning_msgs::msg::PathWithLaneId path =
    test::generatePath(0.0, 3.0, 4.0, 3.0, 2000);
  // make 2000 possible collision from x=0 to x=10
  test::generatePossibleCollisions(possible_collisions, 0.0, 3.0, 4.0, 3.0, 2000);

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
  EXPECT_EQ(possible_collisions.size(), size_t{2000});
  EXPECT_EQ(path.points.size(), size_t{2000});
  EXPECT_TRUE(duration_cast<microseconds>(end_naive - start_naive).count() < 2000);
  std::cout << " runtime (microsec) "
            << duration_cast<microseconds>(end_naive - start_naive).count() << std::endl;
}

TEST(calcSlowDownPointsForPossibleCollision, ConsiderSignedOffset)
{
  using behavior_velocity_planner::occlusion_spot_utils::calcSlowDownPointsForPossibleCollision;
  using behavior_velocity_planner::occlusion_spot_utils::PossibleCollisionInfo;
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::microseconds;
  std::vector<PossibleCollisionInfo> possible_collisions;
  const double offset = 2;
  autoware_auto_planning_msgs::msg::PathWithLaneId path =
    test::generatePath(offset, 3.0, 6.0, 3.0, 5);
  test::generatePossibleCollisions(possible_collisions, 3.0, 3.0, 6.0, 3.0, 3);
  for (size_t i = 0; i < path.points.size(); i++) {
    path.points[i].point.longitudinal_velocity_mps = static_cast<double>(i + offset);
  }

  /**
   * @brief generated path and possible collisions : path start from 2 to 6
   *    0 1 2 3 4 5 6
   * x: e-n-n-p-p-p-p-> path
   * v: N-N-2-3-4-5-6-> velocity
   * c: N-N-N-c-NcN-c-> collision
   *    --->| longitudinal offset
   *    e : ego
   *    p : path
   *    c : collision
   */

  calcSlowDownPointsForPossibleCollision(0, path, offset, possible_collisions);
  EXPECT_EQ(possible_collisions[0].collision_path_point.longitudinal_velocity_mps, 3);
  EXPECT_EQ(possible_collisions[1].collision_path_point.longitudinal_velocity_mps, 4.5);
  EXPECT_EQ(possible_collisions[2].collision_path_point.longitudinal_velocity_mps, 6);
}

TEST(createPossibleCollisionBehindParkedVehicle, TooManyPathPointsAndObstacles)
{
  using behavior_velocity_planner::occlusion_spot_utils::createPossibleCollisionBehindParkedVehicle;
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::microseconds;

  // make a path with 200 points from x=0 to x=200
  autoware_auto_planning_msgs::msg::PathWithLaneId path =
    test::generatePath(0.0, 3.0, 200.0, 3.0, 100);
  // There is a parked bus,car,truck along with ego path.
  // Ignore vehicle dimensions to simplify test
  std::cout << " 6 -   |TRU|   |   |     -> truck is ignored because of lateral distance   \n"
            << " 5 -   |   |   |   |                    \n"
            << " 4 -   |CAR|   |   |     -> considered  \n"
            << " 3Ego--|---|--path-->   (2000 points)   \n"
            << " ＝＝median strip====                   \n"
            << " 2 -   |   |   |SUB|     -> bus is ignored because of opposite direction \n"
            << " 1 -   |   |   |   |                    \n"
            << " 0 | 1 | 2 | 3 | 4 | \n";

  autoware_auto_perception_msgs::msg::PredictedObjects obj_arr;
  autoware_auto_perception_msgs::msg::PredictedObject obj;
  obj.shape.dimensions.x = 0.0;
  obj.shape.dimensions.y = 0.0;
  obj.kinematics.initial_pose_with_covariance.pose.orientation =
    autoware_utils::createQuaternionFromYaw(0.0);
  obj.kinematics.initial_twist_with_covariance.twist.linear.x = 0;
  obj.classification.push_back(autoware_auto_perception_msgs::msg::ObjectClassification{});

  // car
  obj.kinematics.initial_pose_with_covariance.pose.position.x = 2.5;
  obj.kinematics.initial_pose_with_covariance.pose.position.y = 4.0;
  obj.classification.at(0).label = autoware_auto_perception_msgs::msg::ObjectClassification::CAR;
  const size_t num_car = 30;
  for (size_t i = 0; i < num_car; i++) {
    obj_arr.objects.emplace_back(obj);
  }

  // truck
  obj.kinematics.initial_pose_with_covariance.pose.position.x = 2.5;
  obj.kinematics.initial_pose_with_covariance.pose.position.y = 6.0;
  obj.classification.at(0).label = autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK;
  obj_arr.objects.emplace_back(obj);

  // bus
  obj.kinematics.initial_pose_with_covariance.pose.position.x = 4.5;
  obj.kinematics.initial_pose_with_covariance.pose.position.y = 2.0;
  obj.kinematics.initial_pose_with_covariance.pose.orientation =
    autoware_utils::createQuaternionFromYaw(M_PI);
  obj.classification.at(0).label = autoware_auto_perception_msgs::msg::ObjectClassification::BUS;
  obj_arr.objects.emplace_back(obj);

  // Set parameters: enable sidewalk obstacles
  behavior_velocity_planner::occlusion_spot_utils::PlannerParam parameters;
  parameters.vehicle_info.baselink_to_front = 0.0;
  parameters.vehicle_info.vehicle_width = 0.0;
  parameters.detection_area_length = 100;
  parameters.pedestrian_vel = 1.6;
  parameters.lateral_distance_thr = 2.5;
  parameters.angle_thr = 2.5;

  auto obj_arr_ptr =
    std::make_shared<autoware_auto_perception_msgs::msg::PredictedObjects>(obj_arr);
  auto start_naive = high_resolution_clock::now();
  std::vector<behavior_velocity_planner::occlusion_spot_utils::PossibleCollisionInfo>
    possible_collisions;
  createPossibleCollisionBehindParkedVehicle(possible_collisions, path, parameters, 0, obj_arr_ptr);
  auto end_naive = high_resolution_clock::now();
  // the possible collision is inserted and
  // it's ego distance and obstacle distance is the same as (x,|y|)
  ASSERT_EQ(possible_collisions.size(), static_cast<size_t>(num_car));
  EXPECT_EQ(possible_collisions[0].arc_lane_dist_at_collision.length, 2.5);
  EXPECT_EQ(possible_collisions[0].arc_lane_dist_at_collision.distance, 1);
  std::cout << " runtime (microsec) "
            << duration_cast<microseconds>(end_naive - start_naive).count() << std::endl;
}
