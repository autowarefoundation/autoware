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

WaypointGenerator::WaypointGenerator() : private_nh_("~"), pose_ok_(false), vmap_ok_(false)
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
	createLane();
	if(calcLaneArray(&larray))
	{
		larray_pub_.publish(larray);
	}
}

void WaypointGenerator::cacheLane(const vector_map::LaneArray& msg)
{
	all_vmap_.lanes = msg.data;
	autoware_msgs::LaneArray larray;
	createLane();
	if(calcLaneArray(&larray))
	{
		larray_pub_.publish(larray);
	}
}

void WaypointGenerator::cacheNode(const vector_map::NodeArray& msg)
{
	all_vmap_.nodes = msg.data;
	autoware_msgs::LaneArray larray;
	createLane();
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

void WaypointGenerator::createLane()
{
	if (checkEmpty(all_vmap_))
	{
		return;
	}
	lane_vmap_ = VMap::create_lane_vmap(all_vmap_, VMap::LNO_ALL);
	vmap_ok_ = true;
}

bool WaypointGenerator::calcLaneArray(autoware_msgs::LaneArray *larray)
{
	if (!vmap_ok_ || !pose_ok_)
	{
		return false;
	}
	const vector_map::Point vehicle(VMap::create_vector_map_point(current_pose_.position));
	const vector_map::Point departure(VMap::find_nearest_point(lane_vmap_, vehicle));
	const std::vector<VMap::VectorMap> vmap(createVMapArray(lane_vmap_, departure, waypoint_max_));
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

vector_map::Lane findNextLane(const VMap::VectorMap& vmap, const vector_map::Lane& lane, int id)
{
	vector_map::Lane error;
	error.lnid = -1;
	const int flid = (id == 0) ? lane.flid :
									(id == 1) ? lane.flid2 :
									(id == 2) ? lane.flid3 :
									(id == 3) ? lane.flid4 :
									-1;
	if (flid < 0)
	{
		return error;
	}

	for (const vector_map::Lane& l : vmap.lanes)
	{
		if (l.lnid == flid)
		{
			return l;
		}
	}
	return error;
}

std::vector<VMap::VectorMap>
	WaypointGenerator::createVMapArray(const VMap::VectorMap& lane_vmap,
		const vector_map::Point& departure, int waypoint_max) const
{
	std::vector<VMap::VectorMap> vmap(1);
	std::vector<vector_map::Point> point(1, departure);
	std::vector<vector_map::Lane> lane(1, VMap::find_lane(lane_vmap, VMap::LNO_ALL, point[0]));
	std::vector<int> active(1, true);
	if (lane[0].lnid < 0)
	{
		return std::vector<VMap::VectorMap>();
	}

	for (int k = 0; k < waypoint_max - 1; k++)
	{
		bool active_any = false;
		for (const auto& el : active)
		{
			active_any |= el;
		}
		if (!active_any)
		{
			break;
		}
		const int vmap_size = vmap.size();
		for(int i = 0; i < vmap_size; i++)
		{
			if (!active[i])
			{
				continue;
			}
			vector_map::Point end(VMap::find_end_point(lane_vmap, lane[i]));
			if (point[i].pid < 0)
			{
				active[i] = false;
				continue;
			}
			vector_map::DTLane dtlane(search(lane_vmap.dtlanes, lane[i].did));
			vector_map::StopLine stopline(search(lane_vmap.stoplines, lane[i].lnid));
			vmap[i].points.push_back(point[i]);
			vmap[i].dtlanes.push_back(dtlane);
			vmap[i].stoplines.push_back(stopline);
			vmap[i].lanes.push_back(lane[i]);
			point[i] = end;
			bool find_next = false;
			for (int j = 0; j < 4; j++)
			{
				const vector_map::Lane next_lane(findNextLane(lane_vmap, lane[i], j));
				if (next_lane.lnid < 0)
				{
					continue;
				}
				if (find_next)
				{
					vmap.push_back(vmap[i]);
					point.push_back(point[i]);
					lane.push_back(next_lane);
				}
				else
				{
					lane[i] = next_lane;
					find_next = true;
				}
			}
			if (!find_next)
			{
				active[i] = false;
				continue;
			}
		}
	}

	const int vmap_size = vmap.size();
	for (int i = vmap_size - 1; i >= 0; --i)
	{
		if (vmap[i].points.empty())
		{
			vmap.erase(vmap.begin() + i);
			continue;
		}
		vector_map::DTLane dtlane(search(lane_vmap.dtlanes, lane[i].did));
		vector_map::StopLine stopline(search(lane_vmap.stoplines, lane[i].lnid));

		vmap[i].points.push_back(point[i]);
		vmap[i].dtlanes.push_back(dtlane);
		vmap[i].stoplines.push_back(stopline);
	}

	return vmap;
}

void WaypointGenerator::initLane(autoware_msgs::lane *lane, unsigned int size)
{
	lane->header.stamp = ros::Time(0);
	lane->header.frame_id = "/map";
	lane->waypoints.resize(size);
}

void WaypointGenerator::convertVMapToLaneArray(const std::vector<VMap::VectorMap>& vmap_vec, autoware_msgs::LaneArray *larray)
{
	larray->lanes.clear();
	larray->lanes.resize(vmap_vec.size());
	for (const auto& vmap : vmap_vec)
	{
		const unsigned int size = vmap.points.size();
		if (size < 2)
		{
			continue;
		}
		autoware_msgs::lane& lane = larray->lanes[&vmap - &vmap_vec[0]];
		initLane(&lane, size);
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
			double vel = vmap.lanes[velid].limitvel;
			vel = (vel == 0.0) ? 5.0 : vel;
			lane.waypoints[i].twist.twist.linear.x = vel / 3.6;
		}
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
