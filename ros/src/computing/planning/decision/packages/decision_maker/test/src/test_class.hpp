/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#include "decision_maker_node.hpp"

namespace decision_maker {

class TestClass {
public:
	TestClass(){}

	DecisionMakerNode *dmn;

	void createFinalWaypoints() {
		autoware_msgs::Lane final_lane;
		for (int idx = 0; idx < 100; idx++) {
			static autoware_msgs::Waypoint wp;
			wp.gid = idx;
			wp.lid = idx;
			wp.pose.pose.position.x = 0.0 + (double)idx;
			wp.pose.pose.position.y = 0.0;
			wp.pose.pose.position.z = 0.0;
			wp.twist.twist.linear.x = 5.0;
			wp.twist.twist.angular.z = 0.0;

			wp.wpstate.steering_state = autoware_msgs::WaypointState::STR_STRAIGHT;
			wp.wpstate.stop_state = autoware_msgs::WaypointState::NULLSTATE;
			wp.wpstate.event_state = autoware_msgs::WaypointState::TYPE_EVENT_NULL;

			final_lane.waypoints.push_back(wp);
		}

		dmn->current_status_.finalwaypoints = final_lane;
	}

	void setSteeringState(int index, uint8_t state) {
		dmn->current_status_.finalwaypoints.waypoints.at(index)
	        		.wpstate.steering_state = state;
	}

	void setEventState(int index, uint8_t state) {
		dmn->current_status_.finalwaypoints.waypoints.at(index)
	        		.wpstate.event_state = state;
	}

	void setStopState(int index, uint8_t state) {
		dmn->current_status_.finalwaypoints.waypoints.at(index).wpstate.stop_state =
				state;
	}

	void setCurrentPose(double x, double y, double yaw) {
		geometry_msgs::Pose current_pose;
		current_pose.position.x = x;
		current_pose.position.y = y;
		tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, yaw);
		quaternionTFToMsg(quaternion, current_pose.orientation);

		dmn->current_status_.pose = current_pose;
	}

	void setCurrentVelocity(double vel) { dmn->current_status_.velocity = vel; }

	bool isLocalizationConvergence(geometry_msgs::Point _current_point) { return dmn->isLocalizationConvergence(_current_point); }

	bool isArrivedGoal() { return dmn->isArrivedGoal(); }

	uint8_t getSteeringStateFromWaypoint() {
		return dmn->getSteeringStateFromWaypoint();
	}
	uint8_t getEventStateFromWaypoint() {
		return dmn->getEventStateFromWaypoint();
	}
	std::pair<uint8_t, int> getStopSignStateFromWaypoint() {
		return dmn->getStopSignStateFromWaypoint();
	}
};

}
