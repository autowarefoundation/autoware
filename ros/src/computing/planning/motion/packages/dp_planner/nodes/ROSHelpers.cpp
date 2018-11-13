/*
 * ROSHelpers.cpp
 *
 *  Created on: Jun 30, 2016
 *      Author: ai-driver
 */

#include "ROSHelpers.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include "PolygonGenerator.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/MatrixOperations.h"


namespace PlannerXNS
{

ROSHelpers::ROSHelpers() {

}

ROSHelpers::~ROSHelpers() {
}

void ROSHelpers::GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform)
{
	static tf::TransformListener listener;

	int nFailedCounter = 0;
	while (1)
	{
		try
		{
			listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
			break;
		}
		catch (tf::TransformException& ex)
		{
			if(nFailedCounter > 2)
			{
				ROS_ERROR("%s", ex.what());
			}
			ros::Duration(1.0).sleep();
			nFailedCounter ++;
		}
	}
}

void ROSHelpers::ConvertFromPlannerHToAutowarePathFormat(const std::vector<PlannerHNS::WayPoint>& path, const int& iStart,
		autoware_msgs::Lane& trajectory)
{
	trajectory.waypoints.clear();
	for(unsigned int i=iStart; i < path.size(); i++)
	{
		autoware_msgs::Waypoint wp;
		wp.pose.pose.position.x = path.at(i).pos.x;
		wp.pose.pose.position.y = path.at(i).pos.y;
		wp.pose.pose.position.z = path.at(i).pos.z;
		wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(path.at(i).pos.a));
		wp.twist.twist.linear.x = path.at(i).v;
		if(path.at(i).bDir == PlannerHNS::FORWARD_DIR)
			wp.dtlane.dir = 0;
		else if(path.at(i).bDir == PlannerHNS::FORWARD_LEFT_DIR)
			wp.dtlane.dir = 1;
		else if(path.at(i).bDir == PlannerHNS::FORWARD_RIGHT_DIR)
			wp.dtlane.dir = 2;
		//PlannerHNS::GPSPoint p = path.at(i).pos;
		//std::cout << p.ToString() << std::endl;
		trajectory.waypoints.push_back(wp);
	}
}

void ROSHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(const PlannerHNS::RoadNetwork& map,	visualization_msgs::MarkerArray& markerArray)
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
	lane_waypoint_marker.frame_locked = false;

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

void ROSHelpers::ConvertFromPlannerHRectangleToAutowareRviz(const std::vector<PlannerHNS::GPSPoint>& safety_rect,
		visualization_msgs::Marker& marker)
{
	//if(safety_rect.size() != 4) return;

	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	lane_waypoint_marker.ns = "global_lane_array_marker";
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
	lane_waypoint_marker.scale.x = 0.2;
	lane_waypoint_marker.scale.y = 0.2;
	//lane_waypoint_marker.scale.z = 0.1;
	lane_waypoint_marker.frame_locked = false;
	lane_waypoint_marker.color.r = 0.0;
	lane_waypoint_marker.color.g = 1.0;
	lane_waypoint_marker.color.b = 0.0;
	lane_waypoint_marker.color.a = 0.6;

	int id = 0;
	for(unsigned int i = 0; i < safety_rect.size(); i++)
	{
		geometry_msgs::Point p;
		p.x = safety_rect.at(i).x;
		p.y = safety_rect.at(i).y;
		p.z = safety_rect.at(i).z;

		lane_waypoint_marker.points.push_back(p);
	}
	if(safety_rect.size() > 0)
	{
		geometry_msgs::Point p;
		p.x = safety_rect.at(0).x;
		p.y = safety_rect.at(0).y;
		p.z = safety_rect.at(0).z;
		lane_waypoint_marker.points.push_back(p);
	}

//	geometry_msgs::Point p1, p2,p3,p4;
//	p1.x = safety_rect.at(0).x;
//	p1.y = safety_rect.at(0).y;
//	p1.z = safety_rect.at(0).z;
//
//	p2.x = safety_rect.at(1).x;
//	p2.y = safety_rect.at(1).y;
//	p2.z = safety_rect.at(1).z;
//
//	p3.x = safety_rect.at(2).x;
//	p3.y = safety_rect.at(2).y;
//	p3.z = safety_rect.at(2).z;
//
//	p4.x = safety_rect.at(3).x;
//	p4.y = safety_rect.at(3).y;
//	p4.z = safety_rect.at(3).z;
//
//	lane_waypoint_marker.points.push_back(p1);
//	lane_waypoint_marker.points.push_back(p2);
//	lane_waypoint_marker.points.push_back(p3);
//	lane_waypoint_marker.points.push_back(p4);
//	lane_waypoint_marker.points.push_back(p1);

	 marker = lane_waypoint_marker;

}

void ROSHelpers::ConvertFromPlannerHToAutowareVisualizePathFormat(const std::vector<PlannerHNS::WayPoint>& curr_path,
		const std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > >& paths, const PlannerHNS::LocalPlannerH& localPlanner,
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
	//lane_waypoint_marker.scale.z = 0.1;
	lane_waypoint_marker.frame_locked = false;
	std_msgs::ColorRGBA  total_color, curr_color;


	int count = 0;
	for (unsigned int il = 0; il < paths.size(); il++)
	{
		for (unsigned int i = 0; i < paths.at(il).size(); i++)
		{
			lane_waypoint_marker.points.clear();
			lane_waypoint_marker.id = count;

			for (unsigned int j=0; j < paths.at(il).at(i).size(); j++)
			{
			  geometry_msgs::Point point;

			  point.x = paths.at(il).at(i).at(j).pos.x;
			  point.y = paths.at(il).at(i).at(j).pos.y;
			  point.z = paths.at(il).at(i).at(j).pos.z;

			  lane_waypoint_marker.points.push_back(point);
			}

			lane_waypoint_marker.color.a = 0.9;
			if(localPlanner.m_TrajectoryCostsCalculatotor.m_TrajectoryCosts.size() == paths.size())
			{
				float norm_cost = localPlanner.m_TrajectoryCostsCalculatotor.m_TrajectoryCosts.at(i).cost * paths.size();
				if(norm_cost <= 1.0)
				{
					lane_waypoint_marker.color.r = norm_cost;
					lane_waypoint_marker.color.g = 1.0;
				}
				else if(norm_cost > 1.0)
				{
					lane_waypoint_marker.color.r = 1.0;
					lane_waypoint_marker.color.g = 2.0 - norm_cost;
				}
			}
			else
			{
				lane_waypoint_marker.color.r = 0.0;
				lane_waypoint_marker.color.g = 1.0;
			}

			if(i == localPlanner.m_iSafeTrajectory && il == localPlanner.m_iCurrentTotalPathId)
			{
				lane_waypoint_marker.color.r = 1.0;
				lane_waypoint_marker.color.g = 0.0;
				lane_waypoint_marker.color.b = 1.0;
			}
			else
			{
				lane_waypoint_marker.color.b = 0;
			}

			markerArray.markers.push_back(lane_waypoint_marker);
			count++;
		}
	}
}

void ROSHelpers::ConvertFromPlannerHToAutowareVisualizePathFormat(const std::vector<std::vector<PlannerHNS::WayPoint> >& globalPaths,
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
	lane_waypoint_marker.scale.x = 0.1;
	lane_waypoint_marker.scale.y = 0.1;
	total_color.r = 1;
	total_color.g = 0;
	total_color.b = 0;
	total_color.a = 0.5;
	lane_waypoint_marker.color = total_color;
	lane_waypoint_marker.frame_locked = false;

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

void ROSHelpers::ConvertFromPlannerObstaclesToAutoware(const PlannerHNS::WayPoint& currState, const std::vector<PlannerHNS::DetectedObject>& trackedObstacles,
		visualization_msgs::MarkerArray& detectedPolygons)
{
	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	lane_waypoint_marker.ns = "detected_polygons";
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
	lane_waypoint_marker.scale.x = .1;
	lane_waypoint_marker.scale.y = .1;
	//lane_waypoint_marker.scale.z = .05;
	lane_waypoint_marker.color.a = 0.8;
	lane_waypoint_marker.frame_locked = false;

	visualization_msgs::Marker corner_marker;
	corner_marker.header.frame_id = "map";
	corner_marker.header.stamp = ros::Time();
	corner_marker.ns = "Polygon_Corners";
	corner_marker.type = visualization_msgs::Marker::SPHERE;
	corner_marker.action = visualization_msgs::Marker::ADD;
	corner_marker.scale.x = .1;
	corner_marker.scale.y = .1;
	corner_marker.scale.z = .1;
	corner_marker.color.a = 0.8;
	corner_marker.frame_locked = false;


	visualization_msgs::Marker quarters_marker;
	quarters_marker.header.frame_id = "map";
	quarters_marker.header.stamp = ros::Time();
	quarters_marker.ns = "Quarters_Lines";
	quarters_marker.type = visualization_msgs::Marker::LINE_STRIP;
	quarters_marker.action = visualization_msgs::Marker::ADD;
	quarters_marker.scale.x = .03;
	quarters_marker.scale.y = .03;
	quarters_marker.scale.z = .03;
	quarters_marker.color.a = 0.8;
	quarters_marker.color.r = 0.6;
	quarters_marker.color.g = 0.5;
	quarters_marker.color.b = 0;
	quarters_marker.frame_locked = false;

	visualization_msgs::Marker direction_marker;
	direction_marker.header.frame_id = "map";
	direction_marker.header.stamp = ros::Time();
	direction_marker.ns = "Object_Direction";
	direction_marker.type = visualization_msgs::Marker::ARROW;
	direction_marker.action = visualization_msgs::Marker::ADD;
	direction_marker.scale.x = .9;
	direction_marker.scale.y = .4;
	direction_marker.scale.z = .4;
	direction_marker.color.a = 0.8;
	direction_marker.color.r = 0;
	direction_marker.color.g = 1;
	direction_marker.color.b = 0;
	direction_marker.frame_locked = false;


	visualization_msgs::Marker velocity_marker;
	velocity_marker.header.frame_id = "map";
	velocity_marker.header.stamp = ros::Time();
	velocity_marker.ns = "detected_polygons_velocity";
	velocity_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	//velocity_marker.action = visualization_msgs::Marker::ADD;
	velocity_marker.scale.z = 0.9;
	velocity_marker.scale.x = 0.9;
	velocity_marker.scale.y = 0.9;
	velocity_marker.color.a = 0.5;

	velocity_marker.frame_locked = false;
	detectedPolygons.markers.clear();

	int pointID = 0;
	int quartersIds = 0;
	for(unsigned int i =0; i < trackedObstacles.size(); i++)
	{
		//double distance = hypot(currState.pos.y-trackedObstacles.at(i).center.pos.y, currState.pos.x-trackedObstacles.at(i).center.pos.x);

		lane_waypoint_marker.color.g = 0;
		lane_waypoint_marker.color.r = 0;
		lane_waypoint_marker.color.b = 1;

		velocity_marker.color.r = 1;//trackedObstacles.at(i).center.v/16.0;
		velocity_marker.color.g = 1;// - trackedObstacles.at(i).center.v/16.0;
		velocity_marker.color.b = 1;

		lane_waypoint_marker.points.clear();
		lane_waypoint_marker.id = i;
		velocity_marker.id = i;

		//std::cout << " Distance : " << distance << ", Of Object" << trackedObstacles.at(i).id << std::endl;

		for(unsigned int p = 0; p < trackedObstacles.at(i).contour.size(); p++)
		{

			geometry_msgs::Point point;

			  point.x = trackedObstacles.at(i).contour.at(p).x;
			  point.y = trackedObstacles.at(i).contour.at(p).y;
			  point.z = trackedObstacles.at(i).contour.at(p).z;

			  lane_waypoint_marker.points.push_back(point);

			  corner_marker.pose.position = point;
			  corner_marker.color.r = 0.8;
			  corner_marker.color.g = 0;
			  corner_marker.color.b = 0.7;
			  corner_marker.color.a = 0.5;
			  corner_marker.id = pointID;
			  pointID++;

			  detectedPolygons.markers.push_back(corner_marker);
		}

		if(trackedObstacles.at(i).contour.size()>0)
		{
		geometry_msgs::Point point;

		  point.x = trackedObstacles.at(i).contour.at(0).x;
		  point.y = trackedObstacles.at(i).contour.at(0).y;
		  point.z = trackedObstacles.at(i).contour.at(0).z+1;

		  lane_waypoint_marker.points.push_back(point);

		}


		geometry_msgs::Point point;

		point.x = trackedObstacles.at(i).center.pos.x;
		point.y = trackedObstacles.at(i).center.pos.y;
		point.z = trackedObstacles.at(i).center.pos.z+1;

//		geometry_msgs::Point relative_p;
		//relative_p.y = 0.5;
//		velocity_marker.pose.position = calcAbsoluteCoordinate(relative_p, point);
		velocity_marker.pose.position = point;
	    velocity_marker.pose.position.z += 0.5;

	    direction_marker.id = i;
	    direction_marker.pose.position = point;
	    direction_marker.pose.position.z += 0.5;
	    direction_marker.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(trackedObstacles.at(i).center.pos.a));


		for(unsigned int iq = 0; iq < 8; iq++)
		{
			quarters_marker.points.clear();
			quarters_marker.id = quartersIds;
			quarters_marker.points.push_back(point);
			geometry_msgs::Point point2 = point;
			double a_q = UtilityHNS::UtilityH::SplitPositiveAngle(trackedObstacles.at(i).center.pos.a+(iq*M_PI_4));
			point2.x += 2.0*cos(a_q);
			point2.y += 1.5*sin(a_q);
			quarters_marker.points.push_back(point2);

			quartersIds++;
			detectedPolygons.markers.push_back(quarters_marker);
		}

		int speed = (trackedObstacles.at(i).center.v*3.6);

	  // double to string
	  std::ostringstream str_out;
	//  if(trackedObstacles.at(i).center.v > 0.75)
	  str_out << "(" << trackedObstacles.at(i).id << " , " << speed << ")";
//	  else
//		  str_out << trackedObstacles.at(i).id;
	  //std::string vel = str_out.str();
	  velocity_marker.text = str_out.str();//vel.erase(vel.find_first_of(".") + 2);
	  //if(speed > 0.5)

	  detectedPolygons.markers.push_back(velocity_marker);
	  detectedPolygons.markers.push_back(lane_waypoint_marker);
	  detectedPolygons.markers.push_back(direction_marker);

	}
}

std::string ROSHelpers::GetBehaviorNameFromCode(const PlannerHNS::STATE_TYPE& behState)
{
	std::string str = "Unknown";
	switch(behState)
	{
	case PlannerHNS::INITIAL_STATE:
		str = "Init";
		break;
	case PlannerHNS::WAITING_STATE:
		str = "Waiting";
		break;
	case PlannerHNS::FORWARD_STATE:
		str = "Forward";
		break;
	case PlannerHNS::STOPPING_STATE:
		str = "Stop";
		break;
	case PlannerHNS::FINISH_STATE:
		str = "End";
		break;
	case PlannerHNS::FOLLOW_STATE:
		str = "Follow";
		break;
	case PlannerHNS::OBSTACLE_AVOIDANCE_STATE:
		str = "Swerving";
		break;
	case PlannerHNS::TRAFFIC_LIGHT_STOP_STATE:
		str = "Light Stop";
		break;
	case PlannerHNS::TRAFFIC_LIGHT_WAIT_STATE:
		str = "Light Wait";
		break;
	case PlannerHNS::STOP_SIGN_STOP_STATE:
		str = "Sign Stop";
		break;
	case PlannerHNS::STOP_SIGN_WAIT_STATE:
		str = "Sign Wait";
		break;
	default:
		str = "Unknown";
		break;
	}

	return str;
}

void ROSHelpers::VisualizeBehaviorState(const PlannerHNS::WayPoint& currState, const PlannerHNS::BehaviorState& beh, const bool& bGreenLight, const int& avoidDirection, visualization_msgs::Marker& behaviorMarker)
{
	behaviorMarker.header.frame_id = "map";
	behaviorMarker.header.stamp = ros::Time();
	behaviorMarker.ns = "detected_polygons_velocity";
	behaviorMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	behaviorMarker.scale.z = 1.0;
	behaviorMarker.scale.x = 1.0;
	behaviorMarker.scale.y = 1.0;
	behaviorMarker.color.a = 0.9;
	behaviorMarker.frame_locked = false;
	if(bGreenLight)
	{
		behaviorMarker.color.r = 0.1;//trackedObstacles.at(i).center.v/16.0;
		behaviorMarker.color.g = 1;// - trackedObstacles.at(i).center.v/16.0;
		behaviorMarker.color.b = 0.1;
	}
	else
	{
		behaviorMarker.color.r = 1;//trackedObstacles.at(i).center.v/16.0;
		behaviorMarker.color.g = 0.1;// - trackedObstacles.at(i).center.v/16.0;
		behaviorMarker.color.b = 0.1;
		behaviorMarker.id = 0;
	}
	geometry_msgs::Point point;

	point.x = currState.pos.x;
	point.y = currState.pos.y;
	point.z = currState.pos.z+2.0;

	behaviorMarker.pose.position = point;

	std::ostringstream str_out;

	//str_out << "(" << (int)(beh.followDistance * 100) / 100 <<")";
	str_out << "(" << (int)(beh.stopDistance * 100.0) / 100.0 <<")";

	if(avoidDirection == -1)
		str_out << "<< ";

	str_out << GetBehaviorNameFromCode(beh.state);
	if(avoidDirection == 1)
		str_out << " >>";
	behaviorMarker.text = str_out.str();
}

void ROSHelpers::ConvertFromAutowareBoundingBoxObstaclesToPlannerH(const jsk_recognition_msgs::BoundingBoxArray& detectedObstacles,
		std::vector<PlannerHNS::DetectedObject>& obstacles_list)
{
	obstacles_list.clear();
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
		//double objSize = obj.w*obj.l;
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

void ROSHelpers::ConvertFromAutowareCloudClusterObstaclesToPlannerH(const PlannerHNS::WayPoint& currState, const PlannerHNS::CAR_BASIC_INFO& car_info,
		const autoware_msgs::CloudClusterArray& clusters, std::vector<PlannerHNS::DetectedObject>& obstacles_list,
		int& nOriginalPoints, int& nContourPoints)
{
	PlannerHNS::Mat3 rotationMat(-currState.pos.a);
	PlannerHNS::Mat3 translationMat(-currState.pos.x, -currState.pos.y);

	int nPoints = 0;
	int nOrPoints = 0;
	for(unsigned int i =0; i < clusters.clusters.size(); i++)
	{
		PolygonGenerator polyGen;
		PlannerHNS::DetectedObject obj;
		obj.center.pos = PlannerHNS::GPSPoint(clusters.clusters.at(i).centroid_point.point.x,
				clusters.clusters.at(i).centroid_point.point.y,
				clusters.clusters.at(i).centroid_point.point.z,0);
				//tf::getYaw(clusters.clusters.at(i).bounding_box.pose.orientation));

		pcl::PointCloud<pcl::PointXYZ> point_cloud;
		pcl::fromROSMsg(clusters.clusters.at(i).cloud, point_cloud);
		obj.contour = polyGen.EstimateClusterPolygon(point_cloud ,obj.center.pos);
		obj.w = clusters.clusters.at(i).dimensions.y;
		obj.l = clusters.clusters.at(i).dimensions.x;
		obj.h = clusters.clusters.at(i).dimensions.z;
		obj.id = 0;
		obj.distance_to_center = hypot(obj.center.pos.y-currState.pos.y, obj.center.pos.x-currState.pos.x);


		PlannerHNS::GPSPoint relative_point;
		relative_point = translationMat*obj.center.pos;
		relative_point = rotationMat*relative_point;

		double distance_x = fabs(relative_point.x - car_info.wheel_base/2.0);
		double distance_y = fabs(relative_point.y);

		double size = (obj.w+obj.l)/2.0;
//		if(size <= 0.25 || size >= 5 || distance_y > 20.0 || distance_x > 20.0)
//			continue;

//		if(distance_y > 10.0 || distance_x > 10.0)
//			continue;

		if(distance_x  <= car_info.length/1.5 && distance_y <= car_info.width/1.5) // don't detect yourself
			continue;


		nOrPoints += point_cloud.points.size();
		nPoints += obj.contour.size();
		//std::cout << " Distance_X: " << distance_x << ", " << " Distance_Y: " << distance_y << ", " << " Size: " << size << std::endl;

		obstacles_list.push_back(obj);
	}

	nOriginalPoints = nOrPoints;
	nContourPoints =  nPoints;
}

PlannerHNS::SHIFT_POS ROSHelpers::ConvertShiftFromAutowareToPlannerH(const PlannerXNS::AUTOWARE_SHIFT_POS& shift)
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

PlannerXNS::AUTOWARE_SHIFT_POS ROSHelpers::ConvertShiftFromPlannerHToAutoware(const PlannerHNS::SHIFT_POS& shift)
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

PlannerXNS::AutowareBehaviorState ROSHelpers::ConvertBehaviorStateFromPlannerHToAutoware(const PlannerHNS::BehaviorState& beh)
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

void ROSHelpers::UpdateRoadMap(const AutowareRoadNetwork& src_map, PlannerHNS::RoadNetwork& out_map)
{
	std::vector<UtilityHNS::AisanLanesFileReader::AisanLane> lanes;
	for(unsigned int i=0; i < src_map.lanes.data.size();i++)
	{
		UtilityHNS::AisanLanesFileReader::AisanLane l;
		l.BLID 		=  src_map.lanes.data.at(i).blid;
		l.BLID2 	=  src_map.lanes.data.at(i).blid2;
		l.BLID3 	=  src_map.lanes.data.at(i).blid3;
		l.BLID4 	=  src_map.lanes.data.at(i).blid4;
		l.BNID 		=  src_map.lanes.data.at(i).bnid;
		l.ClossID 	=  src_map.lanes.data.at(i).clossid;
		l.DID 		=  src_map.lanes.data.at(i).did;
		l.FLID 		=  src_map.lanes.data.at(i).flid;
		l.FLID2 	=  src_map.lanes.data.at(i).flid2;
		l.FLID3 	=  src_map.lanes.data.at(i).flid3;
		l.FLID4 	=  src_map.lanes.data.at(i).flid4;
		l.FNID 		=  src_map.lanes.data.at(i).fnid;
		l.JCT 		=  src_map.lanes.data.at(i).jct;
		l.LCnt 		=  src_map.lanes.data.at(i).lcnt;
		l.LnID 		=  src_map.lanes.data.at(i).lnid;
		l.Lno 		=  src_map.lanes.data.at(i).lno;
		l.Span 		=  src_map.lanes.data.at(i).span;
		l.RefVel	=  src_map.lanes.data.at(i).refvel;
		l.LimitVel	=  src_map.lanes.data.at(i).limitvel;

//		l.LaneChgFG =  src_map.lanes.at(i).;
//		l.LaneType 	=  src_map.lanes.at(i).blid;
//		l.LimitVel 	=  src_map.lanes.at(i).;
//		l.LinkWAID 	=  src_map.lanes.at(i).blid;
//		l.RefVel 	=  src_map.lanes.at(i).blid;
//		l.RoadSecID =  src_map.lanes.at(i).;

		lanes.push_back(l);
	}

	std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints> points;

	for(unsigned int i=0; i < src_map.points.data.size();i++)
	{
		UtilityHNS::AisanPointsFileReader::AisanPoints p;
		double integ_part = src_map.points.data.at(i).l;
		double deg = trunc(src_map.points.data.at(i).l);
		double min = trunc((src_map.points.data.at(i).l - deg) * 100.0) / 60.0;
		double sec = modf((src_map.points.data.at(i).l - deg) * 100.0, &integ_part)/36.0;
		double L =  deg + min + sec;

		deg = trunc(src_map.points.data.at(i).b);
		min = trunc((src_map.points.data.at(i).b - deg) * 100.0) / 60.0;
		sec = modf((src_map.points.data.at(i).b - deg) * 100.0, &integ_part)/36.0;
		double B =  deg + min + sec;

		p.B 		= B;
		p.Bx 		= src_map.points.data.at(i).bx;
		p.H 		= src_map.points.data.at(i).h;
		p.L 		= L;
		p.Ly 		= src_map.points.data.at(i).ly;
		p.MCODE1 	= src_map.points.data.at(i).mcode1;
		p.MCODE2 	= src_map.points.data.at(i).mcode2;
		p.MCODE3 	= src_map.points.data.at(i).mcode3;
		p.PID 		= src_map.points.data.at(i).pid;
		p.Ref 		= src_map.points.data.at(i).ref;

		points.push_back(p);
	}


	std::vector<UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine> dts;
	for(unsigned int i=0; i < src_map.dtlanes.data.size();i++)
	{
		UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine dt;

		dt.Apara 	= src_map.dtlanes.data.at(i).apara;
		dt.DID 		= src_map.dtlanes.data.at(i).did;
		dt.Dir 		= src_map.dtlanes.data.at(i).dir;
		dt.Dist 	= src_map.dtlanes.data.at(i).dist;
		dt.LW 		= src_map.dtlanes.data.at(i).lw;
		dt.PID 		= src_map.dtlanes.data.at(i).pid;
		dt.RW 		= src_map.dtlanes.data.at(i).rw;
		dt.cant 	= src_map.dtlanes.data.at(i).cant;
		dt.r 		= src_map.dtlanes.data.at(i).r;
		dt.slope 	= src_map.dtlanes.data.at(i).slope;

		dts.push_back(dt);
	}

	std::vector<UtilityHNS::AisanAreasFileReader::AisanArea> areas;
	std::vector<UtilityHNS::AisanIntersectionFileReader::AisanIntersection> inters;
	std::vector<UtilityHNS::AisanLinesFileReader::AisanLine> line_data;
	std::vector<UtilityHNS::AisanStopLineFileReader::AisanStopLine> stop_line_data;
	std::vector<UtilityHNS::AisanSignalFileReader::AisanSignal> signal_data;
	std::vector<UtilityHNS::AisanVectorFileReader::AisanVector> vector_data;
	std::vector<UtilityHNS::AisanCurbFileReader::AisanCurb> curb_data;
	std::vector<UtilityHNS::AisanRoadEdgeFileReader::AisanRoadEdge> roadedge_data;
	std::vector<UtilityHNS::AisanWayareaFileReader::AisanWayarea> way_area;
	std::vector<UtilityHNS::AisanCrossWalkFileReader::AisanCrossWalk> crossing;
	std::vector<UtilityHNS::AisanNodesFileReader::AisanNode > nodes_data;
	std::vector<UtilityHNS::AisanDataConnFileReader::DataConn> conn_data;

	PlannerHNS::GPSPoint origin;//(m_OriginPos.position.x, m_OriginPos.position.y, m_OriginPos.position.z, 0);
	PlannerHNS::MappingHelpers::ConstructRoadNetworkFromROSMessage(lanes, points, dts, inters, areas, line_data, stop_line_data, signal_data, vector_data,curb_data, roadedge_data, way_area, crossing, nodes_data, conn_data, origin, out_map);
}

}
