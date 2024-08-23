# Copyright 2024 TIER IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import autoware_freespace_planning_algorithms.astar_search as fp
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from pyquaternion import Quaternion

# -- Vehicle Shape --
vehicle_shape = fp.VehicleShape()
vehicle_shape.length = 2.0
vehicle_shape.width = 1.0
vehicle_shape.base2back = 1.0


# -- Planner Common Parameter --
planner_param = fp.PlannerCommonParam()
# base configs
planner_param.time_limit = 30000.0
planner_param.max_turning_ratio = 0.5
planner_param.turning_steps = 1
# search configs
planner_param.theta_size = 144
planner_param.angle_goal_range = 6.0
planner_param.curve_weight = 1.2
planner_param.reverse_weight = 1.0
planner_param.direction_change_weight = 2.0
planner_param.lateral_goal_range = 0.5
planner_param.longitudinal_goal_range = 2.0
# costmap configs
planner_param.obstacle_threshold = 100


# -- A* search Configurations --
astar_param = fp.AstarParam()
astar_param.search_method = "forward"
astar_param.only_behind_solutions = False
astar_param.use_back = True
astar_param.adapt_expansion_distance = True
astar_param.expansion_distance = 0.4
astar_param.near_goal_distance = 3.0
astar_param.distance_heuristic_weight = 1.0
astar_param.smoothness_weight = 1.0
astar_param.obstacle_distance_weight = 1.0
astar_param.goal_lat_distance_weight = 1.0

astar = fp.AstarSearch(planner_param, vehicle_shape, astar_param)


# -- Costmap Definition
size = 350
resolution = 0.2

costmap = OccupancyGrid()
costmap.info.resolution = resolution
costmap.info.height = size
costmap.info.width = size
costmap.info.origin.position.x = -size * resolution / 2
costmap.info.origin.position.y = -size * resolution / 2
costmap.data = [0 for i in range(size**2)]

astar.setMap(costmap)


# -- Start and Goal Pose
start_pose = Pose()
goal_pose = Pose()

start_pose.position.x = 0.0
start_pose.position.y = 0.0

goal_pose.position.x = 10.0
goal_pose.position.y = 0.0

yaw = 0
quaternion = Quaternion(axis=[0, 0, 1], angle=yaw)

goal_pose.orientation.w = quaternion.w
goal_pose.orientation.x = quaternion.x
goal_pose.orientation.y = quaternion.y
goal_pose.orientation.z = quaternion.z


# -- Search --
if astar.makePlan(start_pose, goal_pose):
    print("Success to find path.")
    print("  Path length:", astar.getWaypoints().length)
else:
    print("Fail to find path.")
