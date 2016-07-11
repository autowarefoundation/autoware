/*
 *  Copyright (c) 2015, Nagoya University
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
#include "planner_x_core.h"
#include "RosHelpers.h"
#include "waypoint_follower/libwaypoint_follower.h"
#include "waypoint_follower/LaneArray.h"
#include <visualization_msgs/MarkerArray.h>
#include "geo_pos_conv.hh"


// Constructor
PlannerX::PlannerX(ros::NodeHandle* pnh)
{
	clock_gettime(0, &m_Timer);
	m_counter = 0;
	m_frequency = 0;
	pNodeHandle = pnh;
	m_PositionPublisher = pNodeHandle->advertise<geometry_msgs::PoseStamped>("sim_pose", 10);
	m_PathPublisherRviz = pNodeHandle->advertise<visualization_msgs::MarkerArray>("global_waypoints_mark", 10, true);
	m_PathPublisher = pNodeHandle->advertise<waypoint_follower::LaneArray>("lane_waypoints_array", 10, true);
}

// Destructor
PlannerX::~PlannerX()
{
}

void PlannerX::GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform)
{
  static tf::TransformListener listener;

  while (1)
  {
    try
    {
      listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
      break;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }
}

void PlannerX::callbackSimuGoalPose(const geometry_msgs::PoseStamped &msg)
{
	PlannerHNS::WayPoint p;
	ROS_INFO("Target Pose Data: x=%f, y=%f, z=%f, freq=%d", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, m_frequency);
	tf::StampedTransform transform;
	GetTransformFromTF("map", "world", transform);

	m_GoalPos.position.x  = msg.pose.position.x + transform.getOrigin().x();
	m_GoalPos.position.y  = msg.pose.position.y + transform.getOrigin().y();
	m_GoalPos.position.z  = msg.pose.position.z + transform.getOrigin().z();
	m_GoalPos.orientation = msg.pose.orientation;




	PlannerHNS::PlanningInternalParams params;
	PlannerHNS::PlannerH pathPlanner(params);
	PlannerHNS::WayPoint start(m_InitPos.position.x, m_InitPos.position.y, m_InitPos.position.z, tf::getYaw(m_InitPos.orientation));
	PlannerHNS::WayPoint goal(m_GoalPos.position.x, m_GoalPos.position.y, m_GoalPos.position.z, tf::getYaw(m_GoalPos.orientation));

	std::vector<PlannerHNS::WayPoint> path;

	if(m_lanes_data.size() > 0 && m_points_data.size() > 0 && m_dt_data.size()>0)
	{
		PlannerHNS::MappingHelpers::ConstructRoadNetworkFromRosMessage(m_lanes_data, m_points_data, m_dt_data, m_Map);
		m_origin = PlannerHNS::MappingHelpers::GetFirstWaypoint(m_Map);

		ROS_INFO("Target Pose Data: x=%f, y=%f, z=%f", m_origin.pos.x, m_origin.pos.y, m_origin.pos.z);

		ROS_INFO("Transformation  : x=%f, y=%f, z=%f", transform.getOrigin().x(),transform.getOrigin().y(), transform.getOrigin().z());

		geo_pos_conv geo;
		geo.set_plane(6);
		geo.llh_to_xyz(m_origin.pos.lon, m_origin.pos.lat, 0);

		m_origin.pos.x = geo.x();
		m_origin.pos.y = geo.y();

		ROS_INFO("Cartisian      : x=%f, y=%f, z=%f", m_origin.pos.x, m_origin.pos.y, m_origin.pos.z);
	}

	//Testing Free Planner
	//pathPlanner.PlanUsingReedShepp(start, goal,path );

	std::cout << "WayPoints in the generated Path = " << path.size() << std::endl;


	ConvertAndPulishDrivingTrajectory(path);
}

void PlannerX::ConvertAndPulishDrivingTrajectory(const std::vector<PlannerHNS::WayPoint>& path)
{
	visualization_msgs::MarkerArray marker_array;
	waypoint_follower::LaneArray lane_array;
	waypoint_follower::lane l;
	lane_array.lanes.push_back(l);
	for(unsigned int i=0; i < path.size(); i++)
	{
		waypoint_follower::waypoint wp;
		wp.pose.pose.position.x = path.at(i).pos.x;
		wp.pose.pose.position.y = path.at(i).pos.y;
		wp.pose.pose.position.z = path.at(i).pos.z;
		lane_array.lanes.at(0).waypoints.push_back(wp);
	}

	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	lane_waypoint_marker.ns = "global_lane_array_marker";
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
	lane_waypoint_marker.scale.x = 1.0;
	std_msgs::ColorRGBA color;
	color.r = 0;
	color.g = 1;
	color.b = 0;
	color.a = 0.5;

	lane_waypoint_marker.color = color;
	lane_waypoint_marker.frame_locked = true;

	int count = 0;
	for (unsigned int i = 0; i < lane_array.lanes.size(); i++)
	{
	lane_waypoint_marker.points.clear();
	lane_waypoint_marker.id = count;

	for (unsigned int j=0; j < lane_array.lanes.at(i).waypoints.size(); j++)
	{
	  geometry_msgs::Point point;
	  point = lane_array.lanes.at(i).waypoints.at(j).pose.pose.position;
	  lane_waypoint_marker.points.push_back(point);
	}
	marker_array.markers.push_back(lane_waypoint_marker);
	count++;
	}

	m_PathPublisher.publish(lane_array);
	m_PathPublisherRviz.publish(marker_array);
}

void PlannerX::callbackSimuInitPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
	PlannerHNS::WayPoint p;
	ROS_INFO("init Simulation Rviz Pose Data: x=%f, y=%f, z=%f, freq=%d", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, m_frequency);
	tf::StampedTransform transform;
	GetTransformFromTF("map", "world", transform);

	m_InitPos.position.x  = msg->pose.pose.position.x + transform.getOrigin().x();
	m_InitPos.position.y  = msg->pose.pose.position.y + transform.getOrigin().y();
	m_InitPos.position.z  = msg->pose.pose.position.z + transform.getOrigin().z();
	m_InitPos.orientation = msg->pose.pose.orientation;

	ROS_INFO("Origin : x=%f, y=%f, z=%f", transform.getOrigin().x(),transform.getOrigin().y(), transform.getOrigin().z());

	std_msgs::Header h;
	//h.stamp = current_time;
	h.frame_id = "StartPosition";

	geometry_msgs::PoseStamped ps;
	ps.header = h;
	ps.pose = msg->pose.pose;
	m_PositionPublisher.publish(ps);
}

void PlannerX::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
  // write procedure for current pose

	PlannerHNS::WayPoint p;
	ROS_INFO("Pose Data: x=%f, y=%f, z=%f, freq=%d", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, m_frequency);
	m_counter++;
	if(UtilityHNS::UtilityH::GetTimeDiffNow(m_Timer) >= 1.0)
	{
		m_frequency = m_counter;
		m_counter = 0;
		clock_gettime(0, &m_Timer);
	}

}

void PlannerX::callbackFromLightColor(const runtime_manager::traffic_light& msg)
{
  // write procedure for traffic light
}

void PlannerX::callbackFromObjCar(const cv_tracker::obj_label& msg)
{
  // write procedure for car obstacle
}

void PlannerX::callbackGetVMPoints(const map_file::PointClassArray& msg)
{
	ROS_INFO("Received Map Points");

	for(unsigned int i=0; i < msg.point_classes.size();i++)
	{
		UtilityHNS::AisanPointsFileReader::AisanPoints p;
		double integ_part = msg.point_classes.at(i).l;
		double deg = trunc(msg.point_classes.at(i).l);
		double min = trunc((msg.point_classes.at(i).l - deg) * 100.0) / 60.0;
		double sec = modf((msg.point_classes.at(i).l - deg) * 100.0, &integ_part)/36.0;
		double L =  deg + min + sec;

		deg = trunc(msg.point_classes.at(i).b);
		min = trunc((msg.point_classes.at(i).b - deg) * 100.0) / 60.0;
		sec = modf((msg.point_classes.at(i).b - deg) * 100.0, &integ_part)/36.0;
		double B =  deg + min + sec;


		p.B 		= B;
		p.Bx 		= msg.point_classes.at(i).bx;
		p.H 		= msg.point_classes.at(i).h;
		p.L 		= L;
		p.Ly 		= msg.point_classes.at(i).ly;
		p.MCODE1 	= msg.point_classes.at(i).mcode1;
		p.MCODE2 	= msg.point_classes.at(i).mcode2;
		p.MCODE3 	= msg.point_classes.at(i).mcode3;
		p.PID 		= msg.point_classes.at(i).pid;
		p.Ref 		= msg.point_classes.at(i).ref;

		m_points_data.push_back(p);
	}
}

void PlannerX::callbackGetVMLanes(const map_file::LaneArray& msg)
{
	ROS_INFO("Received Map Lane Array");
	for(unsigned int i=0; i < msg.lanes.size();i++)
	{
		UtilityHNS::AisanLanesFileReader::AisanLane l;
		l.BLID 		=  msg.lanes.at(i).blid;
		l.BLID2 	=  msg.lanes.at(i).blid2;
		l.BLID3 	=  msg.lanes.at(i).blid3;
		l.BLID4 	=  msg.lanes.at(i).blid4;
		l.BNID 		=  msg.lanes.at(i).bnid;
		l.ClossID 	=  msg.lanes.at(i).clossid;
		l.DID 		=  msg.lanes.at(i).did;
		l.FLID 		=  msg.lanes.at(i).flid;
		l.FLID2 	=  msg.lanes.at(i).flid2;
		l.FLID3 	=  msg.lanes.at(i).flid3;
		l.FLID4 	=  msg.lanes.at(i).flid4;
		l.FNID 		=  msg.lanes.at(i).fnid;
		l.JCT 		=  msg.lanes.at(i).jct;
		l.LCnt 		=  msg.lanes.at(i).lcnt;
		//l.LaneChgFG =  msg.lanes.at(i).;
		//l.LaneType 	=  msg.lanes.at(i).blid;
		//l.LimitVel 	=  msg.lanes.at(i).;
		//l.LinkWAID 	=  msg.lanes.at(i).blid;
		l.LnID 		=  msg.lanes.at(i).lnid;
		l.Lno 		=  msg.lanes.at(i).lno;
		//l.RefVel 	=  msg.lanes.at(i).blid;
		//l.RoadSecID =  msg.lanes.at(i).;
		l.Span 		=  msg.lanes.at(i).span;


		m_lanes_data.push_back(l);
	}
}

void PlannerX::callbackGetVMNodes(const map_file::NodeArray& msg)
{
	ROS_INFO("Received Map Nodes");


}

void PlannerX::callbackGetVMStopLines(const map_file::StopLineArray& msg)
{
	ROS_INFO("Received Map Stop Lines");
}

void PlannerX::callbackGetVMCenterLines(const map_file::DTLaneArray& msg)
{
	ROS_INFO("Received Map Center Lines");

	for(unsigned int i=0; i < msg.dtlanes.size();i++)
	{
		UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine dt;

		dt.Apara 	= msg.dtlanes.at(i).apara;
		dt.DID 		= msg.dtlanes.at(i).did;
		dt.Dir 		= msg.dtlanes.at(i).dir;
		dt.Dist 	= msg.dtlanes.at(i).dist;
		dt.LW 		= msg.dtlanes.at(i).lw;
		dt.PID 		= msg.dtlanes.at(i).pid;
		dt.RW 		= msg.dtlanes.at(i).rw;
		dt.cant 	= msg.dtlanes.at(i).cant;
		dt.r 		= msg.dtlanes.at(i).r;
		dt.slope 	= msg.dtlanes.at(i).slope;

		m_dt_data.push_back(dt);

	}
}
