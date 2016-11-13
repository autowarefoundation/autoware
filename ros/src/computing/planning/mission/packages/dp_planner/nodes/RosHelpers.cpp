/*
 * RosHelpers.cpp
 *
 *  Created on: Jun 30, 2016
 *      Author: ai-driver
 */

#include "RosHelpers.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include "PolygonGenerator.h"

namespace PlannerXNS
{

RosHelpers::RosHelpers() {

}

RosHelpers::~RosHelpers() {
}

void RosHelpers::GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform)
{
	static tf::TransformListener listener;

	while (1)
	{
		try
		{
			listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
			break;
		}
		catch (tf::TransformException& ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
	}
}

void RosHelpers::ConvertFromPlannerHToAutowarePathFormat(const std::vector<PlannerHNS::WayPoint>& path,
		waypoint_follower::LaneArray& laneArray)
{
	waypoint_follower::lane l;

	for(unsigned int i=0; i < path.size(); i++)
	{
		waypoint_follower::waypoint wp;
		wp.pose.pose.position.x = path.at(i).pos.x;
		wp.pose.pose.position.y = path.at(i).pos.y;
		wp.pose.pose.position.z = path.at(i).pos.z;
		wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(path.at(i).pos.a));
		wp.twist.twist.linear.x = path.at(i).v;
		if(path.at(i).bDir == FORWARD_DIR)
			wp.dtlane.dir = 0;
		else if(path.at(i).bDir == FORWARD_LEFT_DIR)
			wp.dtlane.dir = 1;
		else if(path.at(i).bDir == FORWARD_RIGHT_DIR)
			wp.dtlane.dir = 2;
		//PlannerHNS::GPSPoint p = path.at(i).pos;
		//std::cout << p.ToString() << std::endl;

		l.waypoints.push_back(wp);
	}

	if(l.waypoints.size()>0)
		laneArray.lanes.push_back(l);
}

void RosHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(const PlannerHNS::RoadNetwork& map,	visualization_msgs::MarkerArray& markerArray)
{
	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	lane_waypoint_marker.ns = "road_network_vector_map";
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
	lane_waypoint_marker.scale.x = 0.25;
	std_msgs::ColorRGBA roll_color, total_color, curr_color;
	roll_color.r = 1;
	roll_color.g = 1;
	roll_color.b = 1;
	roll_color.a = 0.5;

	lane_waypoint_marker.color = roll_color;
	lane_waypoint_marker.frame_locked = true;

	markerArray.markers.clear();

	for(unsigned int i = 0; i< map.roadSegments.size(); i++)
	{
		for(unsigned int j = 0; j < map.roadSegments.at(i).Lanes.size(); j++)
		{
			lane_waypoint_marker.points.clear();

			lane_waypoint_marker.id = map.roadSegments.at(i).Lanes.at(j).id;
			for(unsigned int p = 0; p < map.roadSegments.at(i).Lanes.at(j).points.size(); p++)
			{
				geometry_msgs::Point point;

				  point.x = map.roadSegments.at(i).Lanes.at(j).points.at(p).pos.x;
				  point.y = map.roadSegments.at(i).Lanes.at(j).points.at(p).pos.y;
				  point.z = map.roadSegments.at(i).Lanes.at(j).points.at(p).pos.z;

				  lane_waypoint_marker.points.push_back(point);
			}

			markerArray.markers.push_back(lane_waypoint_marker);
		}
	}
}

void RosHelpers::ConvertFromPlannerHToAutowareVisualizePathFormat(const std::vector<PlannerHNS::WayPoint>& curr_path,
		const std::vector<std::vector<PlannerHNS::WayPoint> >& paths,
			visualization_msgs::MarkerArray& markerArray)
{
	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	lane_waypoint_marker.ns = "global_lane_array_marker";
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
	lane_waypoint_marker.scale.x = 0.1;
	lane_waypoint_marker.scale.y = 0.1;
	std_msgs::ColorRGBA roll_color, total_color, curr_color;
	roll_color.r = 0;
	roll_color.g = 1;
	roll_color.b = 0;
	roll_color.a = 0.5;

	lane_waypoint_marker.color = roll_color;
	lane_waypoint_marker.frame_locked = true;

	int count = 0;
	for (unsigned int i = 0; i < paths.size(); i++)
	{
		lane_waypoint_marker.points.clear();
		lane_waypoint_marker.id = count;

		for (unsigned int j=0; j < paths.at(i).size(); j++)
		{
		  geometry_msgs::Point point;

		  point.x = paths.at(i).at(j).pos.x;
		  point.y = paths.at(i).at(j).pos.y;
		  //point.z = paths.at(i).at(j).pos.z;

		  lane_waypoint_marker.points.push_back(point);
		}

		markerArray.markers.push_back(lane_waypoint_marker);
		count++;
	}

	lane_waypoint_marker.points.clear();
	lane_waypoint_marker.id = count;
	lane_waypoint_marker.scale.x = 0.1;
	lane_waypoint_marker.scale.y = 0.1;
	curr_color.r = 1;
	curr_color.g = 0;
	curr_color.b = 1;
	curr_color.a = 0.9;
	lane_waypoint_marker.color = curr_color;

	for (unsigned int j=0; j < curr_path.size(); j++)
	{
	  geometry_msgs::Point point;

	  point.x = curr_path.at(j).pos.x;
	  point.y = curr_path.at(j).pos.y;
	  //point.z = curr_path.at(j).pos.z;

	  lane_waypoint_marker.points.push_back(point);
	}

	markerArray.markers.push_back(lane_waypoint_marker);
	count++;
}

void RosHelpers::ConvertFromPlannerHToAutowareVisualizePathFormat(const std::vector<std::vector<PlannerHNS::WayPoint> >& globalPaths,
			visualization_msgs::MarkerArray& markerArray)
{
	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	lane_waypoint_marker.ns = "global_lane_array_marker";
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;


	std_msgs::ColorRGBA roll_color, total_color, curr_color;
	lane_waypoint_marker.points.clear();
	lane_waypoint_marker.id = 1;
	lane_waypoint_marker.scale.x = 0.35;
	lane_waypoint_marker.scale.y = 0.35;
	total_color.r = 1;
	total_color.g = 0;
	total_color.b = 0;
	total_color.a = 0.5;
	lane_waypoint_marker.color = total_color;
	lane_waypoint_marker.frame_locked = true;

	int count = 0;
	for (unsigned int i = 0; i < globalPaths.size(); i++)
	{
		lane_waypoint_marker.points.clear();
		lane_waypoint_marker.id = count;

		for (unsigned int j=0; j < globalPaths.at(i).size(); j++)
		{
		  geometry_msgs::Point point;

		  point.x = globalPaths.at(i).at(j).pos.x;
		  point.y = globalPaths.at(i).at(j).pos.y;
		  point.z = globalPaths.at(i).at(j).pos.z;

		  lane_waypoint_marker.points.push_back(point);
		}

		markerArray.markers.push_back(lane_waypoint_marker);
		count++;
	}
}

void RosHelpers::ConvertFromPlannerObstaclesToAutoware(const std::vector<PlannerHNS::DetectedObject>& trackedObstacles,
		jsk_recognition_msgs::BoundingBoxArray& obstaclesBoxes,
		visualization_msgs::MarkerArray& detectedPolygons)
{
	obstaclesBoxes.boxes.clear();

//	obstaclesBoxes.header.frame_id = "map";
//	obstaclesBoxes.header.stamp = ros::Time();
//
//	for(unsigned int i =0; i < trackedObstacles.size(); i++)
//	{
//		jsk_recognition_msgs::BoundingBox box;
//		box.header.frame_id = "map";
//		box.header.stamp = ros::Time();
//		box.header.seq = i;
//
//		box.dimensions.x = trackedObstacles.at(i).l;
//		box.dimensions.y = trackedObstacles.at(i).w;
//		box.dimensions.z = trackedObstacles.at(i).h;
//
//		box.pose.position.x = trackedObstacles.at(i).center.pos.x;
//		box.pose.position.y = trackedObstacles.at(i).center.pos.y;
//		box.pose.position.z = trackedObstacles.at(i).center.pos.z;
//		box.pose.orientation = tf::createQuaternionMsgFromYaw(trackedObstacles.at(i).center.pos.a);
//
//		box.label = trackedObstacles.at(i).t;
//		box.value = trackedObstacles.at(i).center.v;
//
//
//		obstaclesBoxes.boxes.push_back(box);
//	}

	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	lane_waypoint_marker.ns = "detected_polygons";
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
	lane_waypoint_marker.scale.x = .05;
	lane_waypoint_marker.scale.y = .05;
	lane_waypoint_marker.scale.z = .05;

	lane_waypoint_marker.frame_locked = true;

	detectedPolygons.markers.clear();



	for(unsigned int i =0; i < trackedObstacles.size(); i++)
	{
		std_msgs::ColorRGBA roll_color, total_color, curr_color;
		roll_color.r = 0;
		roll_color.g = 0;//(double)((int)(i*10.0)%256)/256.0;
		roll_color.b = 1;//(double)((int)(256-i*10.0)%256)/256.0;
		roll_color.a = 0.5;

		lane_waypoint_marker.color = roll_color;
		lane_waypoint_marker.points.clear();
		lane_waypoint_marker.id = i;
		for(unsigned int p = 0; p < trackedObstacles.at(i).contour.size(); p++)
		{

			geometry_msgs::Point point;

			  point.x = trackedObstacles.at(i).contour.at(p).x;
			  point.y = trackedObstacles.at(i).contour.at(p).y;
			  //point.z = trackedObstacles.at(i).contour.at(p).z;

			  lane_waypoint_marker.points.push_back(point);
		}

		if(trackedObstacles.at(i).contour.size()>0)
		{
		geometry_msgs::Point point;

		  point.x = trackedObstacles.at(i).contour.at(0).x;
		  point.y = trackedObstacles.at(i).contour.at(0).y;
		  //point.z = trackedObstacles.at(i).contour.at(p).z;

		  lane_waypoint_marker.points.push_back(point);
		}

		detectedPolygons.markers.push_back(lane_waypoint_marker);
	}

}

void RosHelpers::ConvertFromAutowareBoundingBoxObstaclesToPlannerH(const jsk_recognition_msgs::BoundingBoxArray& detectedObstacles,
		std::vector<PlannerHNS::DetectedObject>& obstacles_list)
{
	for(unsigned int i =0; i < detectedObstacles.boxes.size(); i++)
	{
		PlannerHNS::DetectedObject obj;
		obj.center = PlannerHNS::WayPoint(detectedObstacles.boxes.at(i).pose.position.x,
				detectedObstacles.boxes.at(i).pose.position.y,
				0,
				0);
		obj.w = detectedObstacles.boxes.at(i).dimensions.y;
		obj.l = detectedObstacles.boxes.at(i).dimensions.x;
		obj.h = detectedObstacles.boxes.at(i).dimensions.z;
		double objSize = obj.w*obj.l;
		//double d = hypot(m_State.state.pos.y - obj.center.pos.y, m_State.state.pos.x - obj.center.pos.x);
		//std::cout << ", Distance of  : " << d;
		//if(d < 7)
		{

			double l2 = obj.l/2.0;
			double w2 = obj.w/2.0;

			obj.contour.push_back(PlannerHNS::GPSPoint(-w2, -l2, 0,0));
			obj.contour.push_back(PlannerHNS::GPSPoint(w2, -l2, 0,0));
			obj.contour.push_back(PlannerHNS::GPSPoint(w2, l2, 0,0));
			obj.contour.push_back(PlannerHNS::GPSPoint(-w2, l2, 0,0));
			obstacles_list.push_back(obj);
		}
	}
}

void RosHelpers::ConvertFromAutowareCloudClusterObstaclesToPlannerH(const lidar_tracker::CloudClusterArray& clusters,
		std::vector<PlannerHNS::DetectedObject>& obstacles_list)
{
	for(unsigned int i =0; i < clusters.clusters.size(); i++)
	{
		PolygonGenerator polyGen;
		PlannerHNS::DetectedObject obj;
		obj.center.pos = GPSPoint(clusters.clusters.at(i).centroid_point.point.x,
				clusters.clusters.at(i).centroid_point.point.y,
				clusters.clusters.at(i).centroid_point.point.z,0);
				//tf::getYaw(clusters.clusters.at(i).bounding_box.pose.orientation));

		pcl::PointCloud<pcl::PointXYZ> point_cloud;
		pcl::fromROSMsg(clusters.clusters.at(i).cloud, point_cloud);
		obj.contour = polyGen.EstimateClusterPolygon(point_cloud ,obj.center.pos);
		obj.w = clusters.clusters.at(i).dimensions.y;
		obj.l = clusters.clusters.at(i).dimensions.x;
		obj.h = clusters.clusters.at(i).dimensions.z;

		//double d = hypot(m_State.state.pos.y - obj.center.pos.y, m_State.state.pos.x - obj.center.pos.x);
		//std::cout << " Distance of  : " << d << ", ";

		obstacles_list.push_back(obj);
	}
}

PlannerHNS::SHIFT_POS RosHelpers::ConvertShiftFromAutowareToPlannerH(const PlannerXNS::AUTOWARE_SHIFT_POS& shift)
{
	if(shift == PlannerXNS::AW_SHIFT_POS_DD)
		return PlannerHNS::SHIFT_POS_DD;
	else if(shift == PlannerXNS::AW_SHIFT_POS_RR)
		return PlannerHNS::SHIFT_POS_RR;
	else if(shift == PlannerXNS::AW_SHIFT_POS_NN)
		return PlannerHNS::SHIFT_POS_NN;
	else if(shift == PlannerXNS::AW_SHIFT_POS_PP)
		return PlannerHNS::SHIFT_POS_PP;
	else if(shift == PlannerXNS::AW_SHIFT_POS_BB)
		return PlannerHNS::SHIFT_POS_BB;
	else if(shift == PlannerXNS::AW_SHIFT_POS_SS)
		return PlannerHNS::SHIFT_POS_SS;
	else
		return PlannerHNS::SHIFT_POS_UU;
}

PlannerXNS::AUTOWARE_SHIFT_POS RosHelpers::ConvertShiftFromPlannerHToAutoware(const PlannerHNS::SHIFT_POS& shift)
{
	if(shift == PlannerHNS::SHIFT_POS_DD)
		return PlannerXNS::AW_SHIFT_POS_DD;
	else if(shift == PlannerHNS::SHIFT_POS_RR)
		return PlannerXNS::AW_SHIFT_POS_RR;
	else if(shift == PlannerHNS::SHIFT_POS_NN)
		return PlannerXNS::AW_SHIFT_POS_NN;
	else if(shift == PlannerHNS::SHIFT_POS_PP)
		return PlannerXNS::AW_SHIFT_POS_PP;
	else if(shift == PlannerHNS::SHIFT_POS_BB)
		return PlannerXNS::AW_SHIFT_POS_BB;
	else if(shift == PlannerHNS::SHIFT_POS_SS)
		return PlannerXNS::AW_SHIFT_POS_SS;
	else
		return PlannerXNS::AW_SHIFT_POS_UU;
}

PlannerXNS::AutowareBehaviorState RosHelpers::ConvertBehaviorStateFromPlannerHToAutoware(const PlannerHNS::BehaviorState& beh)
{
	PlannerXNS::AutowareBehaviorState arw_state;
	arw_state.followDistance = beh.followDistance;
	arw_state.followVelocity = beh.followVelocity;
	arw_state.maxVelocity = beh.maxVelocity;
	arw_state.minVelocity = beh.minVelocity;
	arw_state.stopDistance = beh.stopDistance;

	if(beh.indicator == PlannerHNS::LIGHT_INDICATOR::INDICATOR_LEFT)
		arw_state.indicator = PlannerXNS::AW_INDICATOR_LEFT;
	else if(beh.indicator == PlannerHNS::LIGHT_INDICATOR::INDICATOR_RIGHT)
		arw_state.indicator = PlannerXNS::AW_INDICATOR_RIGHT;
	else if(beh.indicator == PlannerHNS::LIGHT_INDICATOR::INDICATOR_BOTH)
		arw_state.indicator = PlannerXNS::AW_INDICATOR_BOTH;
	else if(beh.indicator == PlannerHNS::LIGHT_INDICATOR::INDICATOR_NONE)
		arw_state.indicator = PlannerXNS::AW_INDICATOR_NONE;

	if(beh.state == PlannerHNS::INITIAL_STATE)
		arw_state.state = PlannerXNS::AW_INITIAL_STATE;
	else if(beh.state == PlannerHNS::WAITING_STATE)
		arw_state.state = PlannerXNS::AW_WAITING_STATE;
	else if(beh.state == PlannerHNS::FORWARD_STATE)
		arw_state.state = PlannerXNS::AW_FORWARD_STATE;
	else if(beh.state == PlannerHNS::STOPPING_STATE)
		arw_state.state = PlannerXNS::AW_STOPPING_STATE;
	else if(beh.state == PlannerHNS::EMERGENCY_STATE)
		arw_state.state = PlannerXNS::AW_EMERGENCY_STATE;
	else if(beh.state == PlannerHNS::TRAFFIC_LIGHT_STOP_STATE)
		arw_state.state = PlannerXNS::AW_TRAFFIC_LIGHT_STOP_STATE;
	else if(beh.state == PlannerHNS::STOP_SIGN_STOP_STATE)
		arw_state.state = PlannerXNS::AW_STOP_SIGN_STOP_STATE;
	else if(beh.state == PlannerHNS::FOLLOW_STATE)
		arw_state.state = PlannerXNS::AW_FOLLOW_STATE;
	else if(beh.state == PlannerHNS::LANE_CHANGE_STATE)
		arw_state.state = PlannerXNS::AW_LANE_CHANGE_STATE;
	else if(beh.state == PlannerHNS::OBSTACLE_AVOIDANCE_STATE)
		arw_state.state = PlannerXNS::AW_OBSTACLE_AVOIDANCE_STATE;
	else if(beh.state == PlannerHNS::FINISH_STATE)
		arw_state.state = PlannerXNS::AW_FINISH_STATE;


	return arw_state;

}

}
