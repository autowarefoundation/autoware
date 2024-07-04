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

import autoware_freespace_planning_algorithms.autoware_freespace_planning_algorithms_pybind as _fp
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.serialization import serialize_message

PlannerCommonParam = _fp.PlannerCommonParam
VehicleShape = _fp.VehicleShape
AstarParam = _fp.AstarParam


class PlannerWaypoint:
    def __init__(self, pose, is_back):
        self.pose = pose
        self.is_back = is_back


class PlannerWaypoints:
    def __init__(self):
        self.waypoints = []
        self.length = 0.0

    def compute_length(self):
        return self.length


class AstarSearch:
    def __init__(
        self,
        planner_param: PlannerCommonParam,
        vehicle_shape: VehicleShape,
        astar_param: AstarParam,
    ):
        self.astar_search = _fp.AstarSearch(planner_param, vehicle_shape, astar_param)

    def setMap(self, costmap: OccupancyGrid):
        costmap_byte = serialize_message(costmap)
        self.astar_search.setMap(costmap_byte)

    def makePlan(self, start_pose: Pose, goal_pose: Pose):
        start_pose_byte = serialize_message(start_pose)
        goal_pose_byte = serialize_message(goal_pose)
        return self.astar_search.makePlan(start_pose_byte, goal_pose_byte)

    def getWaypoints(self):
        waypoints_vector = self.astar_search.getWaypoints()
        waypoints = PlannerWaypoints()

        waypoints.length = waypoints_vector.length
        for waypoint in waypoints_vector.waypoints:
            pos = Point(x=waypoint[0], y=waypoint[1], z=waypoint[2])
            quat = Quaternion(x=waypoint[3], y=waypoint[4], z=waypoint[5], w=waypoint[6])
            is_bask = bool(waypoint[7])
            waypoints.waypoints.append(
                PlannerWaypoint(Pose(position=pos, orientation=quat), is_bask)
            )

        return waypoints
