/*
 *  Copyright (c) 2018, TierIV,Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "waypoint_generator.h"

namespace waypoint_maker
{

WaypointGenerator::WaypointGenerator() : private_nh_("~"), pose_ok_(false)
{
	private_nh_.param<int>("waypoint_max", waypoint_max_, 100);
	larray_pub_ = nh_.advertise<autoware_msgs::LaneArray>("/lane_waypoints_array", 10, true);
	pose_sub_ = nh_.subscribe("/current_pose", 1, &WaypointGenerator::poseCallback, this);
	vmap_point_sub_ = nh_.subscribe("/vector_map_info/point", 1, &WaypointGenerator::cachePoint, this);
	vmap_lane_sub_ = nh_.subscribe("/vector_map_info/lane", 1, &WaypointGenerator::cacheLane, this);
	vmap_node_sub_ = nh_.subscribe("/vector_map_info/node", 1, &WaypointGenerator::cacheNode, this);
}

WaypointGenerator::~WaypointGenerator()
{
}

bool WaypointGenerator::checkEmpty(const VMap::VectorMap& vmap)
{
	return (vmap.points.empty() || vmap.lanes.empty() || vmap.nodes.empty());
}

void WaypointGenerator::cachePoint(const vector_map::PointArray& msg)
{
	all_vmap_.points = msg.data;
	autoware_msgs::LaneArray larray;
	if(calcLaneArray(&larray))
	{
		larray_pub_.publish(larray);
	}
}

void WaypointGenerator::cacheLane(const vector_map::LaneArray& msg)
{
	all_vmap_.lanes = msg.data;
	autoware_msgs::LaneArray larray;
	if(calcLaneArray(&larray))
	{
		larray_pub_.publish(larray);
	}
}

void WaypointGenerator::cacheNode(const vector_map::NodeArray& msg)
{
	all_vmap_.nodes = msg.data;
	autoware_msgs::LaneArray larray;
	if(calcLaneArray(&larray))
	{
		larray_pub_.publish(larray);
	}
}

void WaypointGenerator::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
	current_pose_ = pose->pose;
	pose_ok_ = true;
	autoware_msgs::LaneArray larray;
	if(calcLaneArray(&larray))
	{
		larray_pub_.publish(larray);
	}
}

bool WaypointGenerator::calcLaneArray(autoware_msgs::LaneArray *larray)
{
	if (checkEmpty(all_vmap_) || !pose_ok_)
	{
		return false;
	}

	lane_vmap_ = VMap::create_lane_vmap(all_vmap_, VMap::LNO_ALL);
	const vector_map::Point vehicle(VMap::create_vector_map_point(current_pose_.position));
	const vector_map::Point departure(VMap::find_nearest_point(lane_vmap_, vehicle));
	const VMap::VectorMap vmap(createVMapWithLane(lane_vmap_, departure, waypoint_max_));
	if (vmap.points.size() < 2)
	{
		return false;
	}
	convertVMapToLaneArray(vmap, larray);
	return true;
}

vector_map::DTLane search(const std::vector<vector_map::DTLane>& dtlanes, int target_id)
{
	vector_map::DTLane dtlane;
	dtlane.did = -1;
	for (const auto& el : dtlanes)
	{
		if (el.did == target_id)
		{
			dtlane = el;
			break;
		}
	}
	return dtlane;
}

vector_map::StopLine search(const std::vector<vector_map::StopLine>& stoplines, int target_id)
{
	vector_map::StopLine stopline;
	stopline.id = -1;
	for (const auto& el : stoplines)
	{
		if (el.linkid == target_id)
		{
			stopline = el;
			break;
		}
	}
	return stopline;
}

VMap::VectorMap WaypointGenerator::createVMapWithLane(const VMap::VectorMap& lane_vmap,
	const vector_map::Point& departure, int waypoint_max) const
{
	VMap::VectorMap vmap;
	vector_map::Point point = departure;
	vector_map::Lane lane = VMap::find_lane(lane_vmap, VMap::LNO_ALL, point);
	if (lane.lnid < 0)
	{
		return VMap::VectorMap();
	}

	bool finish = false;
	for (int i = 0;; i++)
	{
		vector_map::DTLane dtlane(search(lane_vmap.dtlanes, lane.did));
		vector_map::StopLine stopline(search(lane_vmap.stoplines, lane.lnid));

		vmap.points.push_back(point);
		vmap.dtlanes.push_back(dtlane);
		vmap.stoplines.push_back(stopline);
		if (finish)
		{
			break;
		}
		vmap.lanes.push_back(lane);
		point = VMap::find_end_point(lane_vmap, lane);
		if (point.pid < 0)
		{
			return VMap::VectorMap();
		}
		if (i >= waypoint_max - 2)
		{
			finish = true;
			continue;
		}
		lane = VMap::find_next_lane(lane_vmap, VMap::LNO_ALL, lane);
		if (lane.lnid < 0)
		{
			return VMap::VectorMap();
		}
	}
	return vmap;
}

void WaypointGenerator::initLaneArray(autoware_msgs::LaneArray *larray, unsigned int size)
{
	larray->lanes.clear();
	larray->lanes.resize(1);
	autoware_msgs::lane& lane = larray->lanes[0];
	lane.header.stamp = ros::Time(0);
	lane.header.frame_id = "/map";
	lane.waypoints.resize(size);
}

void WaypointGenerator::convertVMapToLaneArray(const VMap::VectorMap& vmap, autoware_msgs::LaneArray *larray)
{
	const unsigned int size = vmap.points.size();
	if (size < 2)
	{
		return;
	}
	initLaneArray(larray, size);
	autoware_msgs::lane& lane = larray->lanes[0];
	for (unsigned int i = 0; i < size; i++)
	{
		const bool is_last_idx = (i == size - 1);
		const int next_ptid = (is_last_idx) ? i - 1 : i + 1;
		const int velid = (is_last_idx) ? i - 1 : i;
		double yaw = (is_last_idx) ? -M_PI : 0.0;
		const vector_map::Point& p0 = vmap.points[i];
		const vector_map::Point& p1 = vmap.points[next_ptid];
		yaw += atan2(p1.bx - p0.bx, p1.ly - p0.ly);
		lane.waypoints[i].pose.pose.position = VMap::create_geometry_msgs_point(p0);
		lane.waypoints[i].pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
		lane.waypoints[i].twist.twist.linear.x = vmap.lanes[velid].limitvel / 3.6;
	}
}


} // namespace

int main(int argc, char **argv)
{
	ros::init(argc, argv, "waypoint_generator");
	waypoint_maker::WaypointGenerator wg;
	ros::Rate r(1);
	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
