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

#include "autoware/planning_validator/debug_marker.hpp"
#include "autoware/planning_validator/planning_validator.hpp"
#include "test_parameter.hpp"
#include "test_planning_validator_helper.hpp"

#include <gtest/gtest.h>

#include <string>

using autoware::planning_validator::PlanningValidator;
using autoware_planning_msgs::msg::Trajectory;

TEST(PlanningValidatorTestSuite, checkValidFiniteValueFunction)
{
  auto validator = std::make_shared<PlanningValidator>(getNodeOptionsWithDefaultParams());

  // Valid Trajectory
  {
    Trajectory valid_traj = generateTrajectory(THRESHOLD_INTERVAL * 0.9);
    ASSERT_TRUE(validator->checkValidFiniteValue(valid_traj));
  }

  // Nan Trajectory
  {
    Trajectory nan_traj = generateNanTrajectory();
    ASSERT_FALSE(validator->checkValidFiniteValue(nan_traj));
  }

  // Inf Trajectory
  {
    Trajectory inf_traj = generateInfTrajectory();
    ASSERT_FALSE(validator->checkValidFiniteValue(inf_traj));
  }
}

TEST(PlanningValidatorTestSuite, checkValidIntervalFunction)
{
  auto validator = std::make_shared<PlanningValidator>(getNodeOptionsWithDefaultParams());

  // Normal Trajectory
  {
    Trajectory valid_traj = generateTrajectory(THRESHOLD_INTERVAL * 0.9);
    ASSERT_TRUE(validator->checkValidInterval(valid_traj));
  }

  // Boundary Trajectory
  {
    // Note: too small value is not supported like numerical_limits::epsilon
    const auto ep = 1.0e-5;

    Trajectory ok_bound_traj = generateTrajectory(THRESHOLD_INTERVAL - ep);
    ASSERT_TRUE(validator->checkValidInterval(ok_bound_traj));

    Trajectory ng_bound_traj = generateTrajectory(THRESHOLD_INTERVAL + ep);
    ASSERT_FALSE(validator->checkValidInterval(ng_bound_traj));
  }

  // Long Interval Trajectory
  {
    Trajectory long_interval_traj = generateTrajectory(THRESHOLD_INTERVAL * 2.0);
    ASSERT_FALSE(validator->checkValidInterval(long_interval_traj));
  }
}

TEST(PlanningValidatorTestSuite, checkValidCurvatureFunction)
{
  auto validator = std::make_shared<PlanningValidator>(getNodeOptionsWithDefaultParams());

  // Normal Trajectory
  {
    Trajectory valid_traj = generateTrajectory(THRESHOLD_INTERVAL * 2.0);
    ASSERT_TRUE(validator->checkValidCurvature(valid_traj));
  }

  // Invalid curvature trajectory
  {
    // TODO(Horibe): write me
  }
}

TEST(PlanningValidatorTestSuite, checkValidRelativeAngleFunction)
{
  auto validator = std::make_shared<PlanningValidator>(getNodeOptionsWithDefaultParams());

  // valid case
  {
    /**
     * x: 0   1 2    3  4   5 6 7 8 9 10
     * y: 0 0.1 0 -0.1  0 0.2 0 0 0 0  0
     * max relative angle is about 0.197 radian (= 11 degree)
     **/
    constexpr auto interval = 1.0;
    Trajectory valid_traj = generateTrajectory(interval);
    valid_traj.points[1].pose.position.y = 0.1;
    valid_traj.points[3].pose.position.y = -0.1;
    valid_traj.points[5].pose.position.y = 0.2;
    ASSERT_TRUE(validator->checkValidRelativeAngle(valid_traj));
  }

  // invalid case
  {
    /**
     * x: 0 1 2 3  4 5 6 7 8 9 10
     * y: 0 0 0 0 10 0 0 0 0 0 0
     * the relative angle around index [4] is about 1.4 radian (= 84 degree)
     **/
    constexpr auto interval = 1.0;
    Trajectory invalid_traj = generateTrajectory(interval);
    invalid_traj.points[4].pose.position.x = 3;
    invalid_traj.points[4].pose.position.y = 10;
    // for (auto t : invalid_traj.points) {
    //   std::cout << "p: (x , y) = " << "( "<<t.pose.position.x <<
    // " , " << t.pose.position.y <<" )"<< std::endl;
    // }
    ASSERT_FALSE(validator->checkValidRelativeAngle(invalid_traj));
  }

  {
    /** <---inverted pattern-----
     * x: 0 -1 -2 -3  -4 -5 -6 -7 -8 -9 -10
     * y: 0  0  0  0  10  0  0  0  0  0   0
     **/
    constexpr auto interval = 1.0;
    Trajectory invalid_traj = generateTrajectory(interval);
    invalid_traj.points[4].pose.position.y = 10;
    for (auto t : invalid_traj.points) {
      t.pose.position.x *= -1;
    }
    ASSERT_FALSE(validator->checkValidRelativeAngle(invalid_traj));
  }

  {
    /** vertical pattern
     * x: 0 0 0 0 10 0 0 0 0 0  0
     * y: 0 1 2 3  4 5 6 7 8 9 10
     **/
    constexpr auto interval = 1.0;
    Trajectory invalid_traj = generateTrajectory(interval);
    for (size_t i = 0; i < invalid_traj.points.size(); i++) {
      auto & p = invalid_traj.points[i].pose.position;
      p.x = 0;
      p.y = i;
    }
    invalid_traj.points[4].pose.position.x = 10;
    std::string valid_error_msg;
    ASSERT_FALSE(validator->checkValidRelativeAngle(invalid_traj));
  }
}
