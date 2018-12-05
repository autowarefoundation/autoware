/// \file  ROSHelpers.cpp
/// \brief Helper functions for rviz visualization
/// \author Hatem Darweesh
/// \date Jun 30, 2016

#include "op_ros_helpers/op_ROSHelpers.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include "op_ros_helpers/PolygonGenerator.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/MatrixOperations.h"


namespace PlannerHNS
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

visualization_msgs::Marker ROSHelpers::CreateGenMarker(const double& x, const double& y, const double& z,const double& a,
		const double& r, const double& g, const double& b, const double& scale, const int& id, const std::string& ns, const int& type)
{
	visualization_msgs::Marker mkr;
	mkr.header.frame_id = "map";
	mkr.header.stamp = ros::Time();
	mkr.ns = ns;
	mkr.type = type;
	mkr.action = visualization_msgs::Marker::ADD;
	mkr.scale.x = scale;
	mkr.scale.y = scale;
	mkr.scale.z = scale;
	mkr.color.a = 0.8;
	mkr.color.r = r;
	mkr.color.g = g;
	mkr.color.b = b;
	mkr.pose.position.x = x;
	mkr.pose.position.y = y;
	mkr.pose.position.z = z;
	mkr.pose.orientation = tf::createQuaternionMsgFromYaw(a);
	mkr.id = id;
	return mkr;
}

void ROSHelpers::InitMarkers(const int& nMarkers,
		visualization_msgs::MarkerArray& centers,
		visualization_msgs::MarkerArray& dirs,
		visualization_msgs::MarkerArray& text_info,
		visualization_msgs::MarkerArray& polygons,
		visualization_msgs::MarkerArray& trajectories)
{
	centers.markers.clear();
	dirs.markers.clear();
	text_info.markers.clear();
	polygons.markers.clear();
	trajectories.markers.clear();

	for(int i=0; i<nMarkers; i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,1,i,"CenterMarker", visualization_msgs::Marker::SPHERE);
		centers.markers.push_back(mkr);
	}

	for(int i=nMarkers; i<nMarkers*2; i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,1,i,"Directions", visualization_msgs::Marker::ARROW);
		dirs.markers.push_back(mkr);
	}

	for(int i=nMarkers*2; i<nMarkers*3; i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,1,i,"InfoText", visualization_msgs::Marker::TEXT_VIEW_FACING);
		text_info.markers.push_back(mkr);
	}

	for(int i=nMarkers*3; i<nMarkers*4; i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,1,i,"detected_polygons", visualization_msgs::Marker::LINE_STRIP);
		polygons.markers.push_back(mkr);
	}

	for(int i=nMarkers*4; i<nMarkers*5; i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,1,i,"tracked_trajectories", visualization_msgs::Marker::LINE_STRIP);
		trajectories.markers.push_back(mkr);
	}
}

void ROSHelpers::InitMatchingMarkers(const int& nMarkers, visualization_msgs::MarkerArray& connections)
{
	connections.markers.clear();
	for(int i=0; i<nMarkers; i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,1,i,"matching_connections", visualization_msgs::Marker::LINE_STRIP);
		connections.markers.push_back(mkr);
	}
}

void ROSHelpers::ConvertMatchingMarkers(const std::vector<std::pair<PlannerHNS::WayPoint, PlannerHNS::WayPoint> >& match_list,
		visualization_msgs::MarkerArray& tracked_traj_d, visualization_msgs::MarkerArray& tracked_traj, int start_id)
{

	tracked_traj = tracked_traj_d;

	for(unsigned int i = 0; i < match_list.size(); i++)
	{
		visualization_msgs::Marker match_mkr = CreateGenMarker(0,0,0,0,1,0,0,0.2, start_id+i,"matching_connections", visualization_msgs::Marker::LINE_STRIP);
		geometry_msgs::Point point;
		point.x = match_list.at(i).first.pos.x;
		point.y = match_list.at(i).first.pos.y;
		point.z = match_list.at(i).first.pos.z;
		match_mkr.points.push_back(point);

		point.x = match_list.at(i).second.pos.x;
		point.y = match_list.at(i).second.pos.y;
		point.z = match_list.at(i).second.pos.z;
		match_mkr.points.push_back(point);

		if(i < tracked_traj.markers.size())
			tracked_traj.markers.at(i) = match_mkr;
		else
			tracked_traj.markers.push_back(match_mkr);
	}
}

int ROSHelpers::ConvertTrackedObjectsMarkers(const PlannerHNS::WayPoint& currState, const std::vector<PlannerHNS::DetectedObject>& trackedObstacles,
		visualization_msgs::MarkerArray& centers_d,
		visualization_msgs::MarkerArray& dirs_d,
		visualization_msgs::MarkerArray& text_info_d,
		visualization_msgs::MarkerArray& polygons_d,
		visualization_msgs::MarkerArray& tracked_traj_d,
		visualization_msgs::MarkerArray& centers,
		visualization_msgs::MarkerArray& dirs,
		visualization_msgs::MarkerArray& text_info,
		visualization_msgs::MarkerArray& polygons,
		visualization_msgs::MarkerArray& tracked_traj)
{

	int i_next_id = 0;
	centers = centers_d;
	dirs = dirs_d;
	text_info = text_info_d;
	polygons = polygons_d;
	tracked_traj = tracked_traj_d;

	for(unsigned int i =0; i < trackedObstacles.size(); i++)
	{
		int speed = (trackedObstacles.at(i).center.v*3.6);

		//Update Stage
		visualization_msgs::Marker center_mkr = CreateGenMarker(trackedObstacles.at(i).center.pos.x,trackedObstacles.at(i).center.pos.y,trackedObstacles.at(i).center.pos.z,
				trackedObstacles.at(i).center.pos.a,1,0,0,0.5,i,"CenterMarker", visualization_msgs::Marker::SPHERE);
		if(i < centers.markers.size())
		{
			center_mkr.id = centers.markers.at(i).id;
			centers.markers.at(i) = center_mkr;
		}
		else
			centers.markers.push_back(center_mkr);

		//Directions
		if(trackedObstacles.at(i).bDirection)
		{
			visualization_msgs::Marker dir_mkr = CreateGenMarker(trackedObstacles.at(i).center.pos.x,trackedObstacles.at(i).center.pos.y,trackedObstacles.at(i).center.pos.z+0.5,
					trackedObstacles.at(i).center.pos.a,0,1,0,0.3,centers.markers.size()+i,"Directions", visualization_msgs::Marker::ARROW);
			dir_mkr.scale.x = 0.4;
			if(i < dirs.markers.size())
			{
				dir_mkr.id = dirs.markers.at(i).id;
				dirs.markers.at(i) = dir_mkr;
			}
			else
				dirs.markers.push_back(dir_mkr);
		}


		//Text
		visualization_msgs::Marker text_mkr;
//		if(speed > 3.0)
//			text_mkr = CreateGenMarker(trackedObstacles.at(i).center.pos.x+0.5,trackedObstacles.at(i).center.pos.y+0.5,trackedObstacles.at(i).center.pos.z+1,
//					trackedObstacles.at(i).center.pos.a,1,0,0,0.75,centers.markers.size()*2+i,"InfoText", visualization_msgs::Marker::TEXT_VIEW_FACING);
//		else
		text_mkr = CreateGenMarker(trackedObstacles.at(i).center.pos.x+0.5,trackedObstacles.at(i).center.pos.y+0.5,trackedObstacles.at(i).center.pos.z+1,
							trackedObstacles.at(i).center.pos.a,1,1,1,1.2,centers.markers.size()*2+i,"InfoText", visualization_msgs::Marker::TEXT_VIEW_FACING);

		std::ostringstream str_out;
		//str_out << trackedObstacles.at(i).id << " ( " << speed << " )" << " (" << trackedObstacles.at(i).distance_to_center << ")";
		str_out << trackedObstacles.at(i).id << " (" << speed << ")";
		text_mkr.text = str_out.str();

		if(i < text_info.markers.size())
		{
			text_mkr.id = text_info.markers.at(i).id;
			text_info.markers.at(i) = text_mkr;
		}
		else
			text_info.markers.push_back(text_mkr);


		//Polygons
		visualization_msgs::Marker poly_mkr = CreateGenMarker(0,0,0,0, 1,0.25,0.25,0.1,centers.markers.size()*3+i,"detected_polygons", visualization_msgs::Marker::LINE_STRIP);

		for(unsigned int p = 0; p < trackedObstacles.at(i).contour.size(); p++)
		{
			geometry_msgs::Point point;
			point.x = trackedObstacles.at(i).contour.at(p).x;
			point.y = trackedObstacles.at(i).contour.at(p).y;
			point.z = trackedObstacles.at(i).contour.at(p).z;
			poly_mkr.points.push_back(point);
		}

		if(trackedObstacles.at(i).contour.size()>0)
		{
			geometry_msgs::Point point;
			point.x = trackedObstacles.at(i).contour.at(0).x;
			point.y = trackedObstacles.at(i).contour.at(0).y;
			point.z = trackedObstacles.at(i).contour.at(0).z;
			poly_mkr.points.push_back(point);
		}

		if(i < polygons.markers.size())
		{
			poly_mkr.id =  polygons.markers.at(i).id;
			polygons.markers.at(i) = poly_mkr;
		}
		else
			polygons.markers.push_back(poly_mkr);


		//Trajectories
		visualization_msgs::Marker traj_mkr = CreateGenMarker(0,0,0,0,1,1,0,0.1,centers.markers.size()*4+i,"tracked_trajectories", visualization_msgs::Marker::LINE_STRIP);

		for(unsigned int p = 0; p < trackedObstacles.at(i).centers_list.size(); p++)
		{
			geometry_msgs::Point point;
			point.x = trackedObstacles.at(i).centers_list.at(p).pos.x;
			point.y = trackedObstacles.at(i).centers_list.at(p).pos.y;
			point.z = trackedObstacles.at(i).centers_list.at(p).pos.z;
			traj_mkr.points.push_back(point);
		}


		if(i < tracked_traj.markers.size())
		{
			traj_mkr.id = tracked_traj.markers.at(i).id ;
			tracked_traj.markers.at(i) = traj_mkr;
		}
		else
			tracked_traj.markers.push_back(traj_mkr);

		i_next_id = traj_mkr.id;

	}

	return i_next_id +1;
}

void ROSHelpers::CreateCircleMarker(const PlannerHNS::WayPoint& _center, const double& radius, const int& start_id, visualization_msgs::Marker& circle_points)
{
	circle_points = CreateGenMarker(0,0,0,0,1,1,1,0.2,start_id,"Detection_Circles", visualization_msgs::Marker::LINE_STRIP);
	for (float i = 0; i < M_PI*2.0+0.05; i+=0.05)
	{
		geometry_msgs::Point point;
		point.x = _center.pos.x + (radius * cos(i));
		point.y = _center.pos.y + (radius * sin(i));
		point.z = _center.pos.z;
		circle_points.points.push_back(point);
	}
}

void ROSHelpers::InitPredMarkers(const int& nMarkers, visualization_msgs::MarkerArray& paths)
{
	paths.markers.clear();
	for(int i=0; i<nMarkers; i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,1,i,"Predicted_Trajectories", visualization_msgs::Marker::LINE_STRIP);
		paths.markers.push_back(mkr);
	}
}

void ROSHelpers::InitCurbsMarkers(const int& nMarkers, visualization_msgs::MarkerArray& curbs)
{
	curbs.markers.clear();
	for(int i=0; i<nMarkers; i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,1,i,"map_detected_curbs", visualization_msgs::Marker::SPHERE);
		curbs.markers.push_back(mkr);
	}
}

void ROSHelpers::ConvertPredictedTrqajectoryMarkers(std::vector<std::vector<PlannerHNS::WayPoint> >& paths,visualization_msgs::MarkerArray& path_markers, visualization_msgs::MarkerArray& path_markers_d)
{

	path_markers = path_markers_d;
	for(unsigned int i = 0; i < paths.size(); i++)
	{
		double additional_z = 0;
		double basic_color = 0.5;
		double prop = 1.0;
		bool bCurrent = false;
//		if(paths.at(i).size()>0)
//		{
//
//			prop = paths.at(i).at(0).collisionCost;
//			if(prop < 0.5)
//				continue;
//
//			if(prop > 0.5)
//			{
//				additional_z = prop;
//				bCurrent = true;
//			}
//		}


//		double r = 0, g = 0, b = 0;
//		if(bCurrent == true)
//		{
//			r = basic_color+additional_z;
//			g = basic_color+additional_z;
//			b = basic_color+additional_z;
//		}
//		else if(i == 0)
//		{
//			r = basic_color+additional_z;
//		}
//		else if(i == 1)
//		{
//			g = basic_color+additional_z;
//		}
//		else if(i == 2)
//		{
//			b = basic_color+additional_z;
//		}
//		else if(i == 3)
//		{
//			r = basic_color+additional_z;
//			b = basic_color+additional_z;
//		}
//		else
//		{
//			g = basic_color+additional_z;
//			b = basic_color+additional_z;
//		}
//
//		visualization_msgs::Marker path_mkr = CreateGenMarker(0,0,0,0,r,g,b,0.1,i,"Predicted_Trajectories", visualization_msgs::Marker::LINE_STRIP);

		visualization_msgs::Marker path_mkr = CreateGenMarker(0,0,0,0,1.0*prop,0.1*prop,0.1*prop,0.1,i,"Predicted_Trajectories", visualization_msgs::Marker::LINE_STRIP);


		for(unsigned int p = 0; p < paths.at(i).size(); p++)
		{
			geometry_msgs::Point point;
			point.x = paths.at(i).at(p).pos.x;
			point.y = paths.at(i).at(p).pos.y;
			point.z = paths.at(i).at(p).pos.z + additional_z;
			path_mkr.points.push_back(point);
		}

		if(i < path_markers.markers.size())
			path_markers.markers.at(i) = path_mkr;
		else
			path_markers.markers.push_back(path_mkr);
	}
}

void ROSHelpers::ConvertCurbsMarkers(const std::vector<PlannerHNS::DetectedObject>& curbs, visualization_msgs::MarkerArray& curbs_markers, visualization_msgs::MarkerArray& curbs_markers_d)
{

	curbs_markers = curbs_markers_d;
	for(unsigned int i = 0; i < curbs.size(); i++)
	{
		if(curbs.at(i).contour.size() > 0)
		{
			visualization_msgs::Marker curb_mkr = CreateGenMarker(curbs.at(i).contour.at(0).x,curbs.at(i).contour.at(0).y,curbs.at(i).contour.at(0).z,0,1,0.54,0,0.2,i,"map_detected_curbs", visualization_msgs::Marker::SPHERE);

			if(i < curbs_markers.markers.size())
				curbs_markers.markers.at(i) = curb_mkr;
			else
				curbs_markers.markers.push_back(curb_mkr);
		}
	}
}

void ROSHelpers::InitCollisionPointsMarkers(const int& nMarkers, visualization_msgs::MarkerArray& col_points)
{
	col_points.markers.clear();
	for(int i=0; i<nMarkers; i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,1,i,"collision_points_rviz", visualization_msgs::Marker::SPHERE);
		col_points.markers.push_back(mkr);
	}
}

void ROSHelpers::ConvertCollisionPointsMarkers(const std::vector<PlannerHNS::WayPoint>& col_points, visualization_msgs::MarkerArray& collision_markers, visualization_msgs::MarkerArray& collision_markers_d)
{
	collision_markers = collision_markers_d;
	for(unsigned int i = 0; i < col_points.size(); i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(col_points.at(i).pos.x, col_points.at(i).pos.y, col_points.at(i).pos.z,0,1,0,0,0.5,i,"collision_points_rviz", visualization_msgs::Marker::SPHERE);

		if(i < collision_markers.markers.size())
			collision_markers.markers.at(i) = mkr;
		else
			collision_markers.markers.push_back(mkr);

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
		if(path.at(i).bDir == FORWARD_DIR)
			wp.dtlane.dir = 0;
		else if(path.at(i).bDir == FORWARD_LEFT_DIR)
			wp.dtlane.dir = 1;
		else if(path.at(i).bDir == FORWARD_RIGHT_DIR)
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

void ROSHelpers::InitPredParticlesMarkers(const int& nMarkers, visualization_msgs::MarkerArray& paths)
{
	paths.markers.clear();
	for(int i=0; i<nMarkers; i++)
	{
		visualization_msgs::Marker mkr = CreateGenMarker(0,0,0,0,1,1,1,0.05,i,"Particles", visualization_msgs::Marker::ARROW);
		mkr.scale.x = 0.3;
		paths.markers.push_back(mkr);
	}
}

void ROSHelpers::ConvertParticles(std::vector<PlannerHNS::WayPoint>& points, visualization_msgs::MarkerArray& part_mkrs, visualization_msgs::MarkerArray& part_markers_d)
{
	part_mkrs = part_markers_d;
	for(unsigned int i = 0; i < points.size(); i++)
	{
		visualization_msgs::Marker mkr;
		if(points.at(i).bDir == PlannerHNS::STANDSTILL_DIR)
			mkr = CreateGenMarker(points.at(i).pos.x, points.at(i).pos.y,points.at(i).pos.z,points.at(i).pos.a,1,0,0,0.05,i,"Particles", visualization_msgs::Marker::ARROW);
		else if(points.at(i).bDir == PlannerHNS::FORWARD_DIR)
			mkr = CreateGenMarker(points.at(i).pos.x, points.at(i).pos.y,points.at(i).pos.z,points.at(i).pos.a,1,1,1,0.05,i,"Particles", visualization_msgs::Marker::ARROW);
		else if(points.at(i).bDir == PlannerHNS::FORWARD_RIGHT_DIR)
			mkr = CreateGenMarker(points.at(i).pos.x, points.at(i).pos.y,points.at(i).pos.z,points.at(i).pos.a,0,1,0,0.05,i,"Particles", visualization_msgs::Marker::ARROW);
		else if(points.at(i).bDir == PlannerHNS::FORWARD_LEFT_DIR)
			mkr = CreateGenMarker(points.at(i).pos.x, points.at(i).pos.y,points.at(i).pos.z,points.at(i).pos.a,0,0,1,0.05,i,"Particles", visualization_msgs::Marker::ARROW);
		else if(points.at(i).bDir == PlannerHNS::BACKWARD_DIR)
			mkr = CreateGenMarker(points.at(i).pos.x, points.at(i).pos.y,points.at(i).pos.z,points.at(i).pos.a,1,0,1,0.05,i,"Particles", visualization_msgs::Marker::ARROW);
		else
			mkr = CreateGenMarker(points.at(i).pos.x, points.at(i).pos.y,points.at(i).pos.z,points.at(i).pos.a,1,1,0,0.05,i,"Particles", visualization_msgs::Marker::ARROW);

		mkr.scale.x = 0.3;
		if(i < part_mkrs.markers.size())
			part_mkrs.markers.at(i) = mkr;
		else
			part_mkrs.markers.push_back(mkr);
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

void ROSHelpers::TrajectoriesToMarkers(const std::vector<std::vector<std::vector<PlannerHNS::WayPoint> > >& paths, visualization_msgs::MarkerArray& markerArray)
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

			lane_waypoint_marker.color.r = 0.0;
			lane_waypoint_marker.color.g = 1.0;
			lane_waypoint_marker.color.b = 0.0;

			markerArray.markers.push_back(lane_waypoint_marker);
			count++;
		}
	}
}

void ROSHelpers::TrajectoriesToColoredMarkers(const std::vector<std::vector<PlannerHNS::WayPoint> >& paths, const std::vector<PlannerHNS::TrajectoryCost>& traj_costs,const int& iClosest, visualization_msgs::MarkerArray& markerArray)
{
	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	lane_waypoint_marker.ns = "local_lane_array_marker_colored";
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
	lane_waypoint_marker.scale.x = 0.1;
	lane_waypoint_marker.scale.y = 0.1;
	//lane_waypoint_marker.scale.z = 0.1;
	lane_waypoint_marker.color.a = 0.9;
	lane_waypoint_marker.color.r = 1.0;
	lane_waypoint_marker.color.g = 1.0;
	lane_waypoint_marker.color.b = 1.0;
	lane_waypoint_marker.frame_locked = false;

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
			point.z = paths.at(i).at(j).pos.z;

			lane_waypoint_marker.points.push_back(point);
		}

		lane_waypoint_marker.color.b = 0;

		if(traj_costs.size() == paths.size())
		{
			float norm_cost = traj_costs.at(i).cost * paths.size();
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
			lane_waypoint_marker.color.r = 1.0;
			lane_waypoint_marker.color.g = 0.0;
		}

		if(traj_costs.at(i).bBlocked)
		{
			lane_waypoint_marker.color.r = 1.0;
			lane_waypoint_marker.color.g = 0.0;
			lane_waypoint_marker.color.b = 0.0;
		}

		if(i == iClosest)
		{
			lane_waypoint_marker.color.r = 1.0;
			lane_waypoint_marker.color.g = 0.0;
			lane_waypoint_marker.color.b = 1.0;
		}

		markerArray.markers.push_back(lane_waypoint_marker);
		count++;
	}
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

			if((int)i == localPlanner.m_iSafeTrajectory && (int)il == localPlanner.m_iCurrentTotalPathId)
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

void ROSHelpers::ConvertFromPlannerHToAutowareVisualizePathFormat(const std::vector<std::vector<PlannerHNS::WayPoint> >& globalPaths, visualization_msgs::MarkerArray& markerArray)
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

void ROSHelpers::VisualizeBehaviorState(const PlannerHNS::WayPoint& currState, const PlannerHNS::BehaviorState& beh, const bool& bGreenLight, const int& avoidDirection, visualization_msgs::Marker& behaviorMarker, std::string ns,double size_factor)
{
	behaviorMarker.header.frame_id = "map";
	behaviorMarker.header.stamp = ros::Time();
	behaviorMarker.ns = ns;
	behaviorMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	behaviorMarker.scale.z = 1.0*size_factor;
	behaviorMarker.scale.x = 1.0*size_factor;
	behaviorMarker.scale.y = 1.0*size_factor;
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
	}

	behaviorMarker.id = 0;

	geometry_msgs::Point point;

	point.x = currState.pos.x;
	point.y = currState.pos.y;
	point.z = currState.pos.z+3.0;

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

void ROSHelpers::ConvertFromAutowareCloudClusterObstaclesToPlannerH(const PlannerHNS::WayPoint& currState, const double& car_width,
		const double& car_length, const autoware_msgs::CloudClusterArray& clusters, vector<PlannerHNS::DetectedObject>& obstacles_list,
		const double max_obj_size, const double& min_obj_size, const double& detection_radius,
		const int& n_poly_quarters,const double& poly_resolution, int& nOriginalPoints, int& nContourPoints)
{
	PlannerHNS::Mat3 rotationMat(-currState.pos.a);
	PlannerHNS::Mat3 translationMat(-currState.pos.x, -currState.pos.y);

	int nPoints = 0;
	int nOrPoints = 0;
	double object_size = 0;
	PlannerHNS::GPSPoint relative_point;
	PlannerHNS::GPSPoint avg_center;
	PolygonGenerator polyGen(n_poly_quarters);
	PlannerHNS::DetectedObject obj;

	for(unsigned int i =0; i < clusters.clusters.size(); i++)
	{
		obj.id = clusters.clusters.at(i).id;
		obj.label = clusters.clusters.at(i).label;

		obj.center.pos.x = clusters.clusters.at(i).centroid_point.point.x;
		obj.center.pos.y = clusters.clusters.at(i).centroid_point.point.y;
		obj.center.pos.z = clusters.clusters.at(i).centroid_point.point.z;
		obj.center.pos.a = 0;
		obj.center.v = 0;
		obj.actual_yaw = clusters.clusters.at(i).estimated_angle;

		obj.w = clusters.clusters.at(i).dimensions.x;
		obj.l = clusters.clusters.at(i).dimensions.y;
		obj.h = clusters.clusters.at(i).dimensions.z;

		pcl::PointCloud<pcl::PointXYZ> point_cloud;
		pcl::fromROSMsg(clusters.clusters.at(i).cloud, point_cloud);


		obj.contour = polyGen.EstimateClusterPolygon(point_cloud ,obj.center.pos,avg_center, poly_resolution);

		obj.distance_to_center = hypot(obj.center.pos.y-currState.pos.y, obj.center.pos.x-currState.pos.x);

		object_size = hypot(obj.w, obj.l);

		if(obj.distance_to_center > detection_radius || object_size < min_obj_size || object_size > max_obj_size)
			continue;

		relative_point = translationMat*obj.center.pos;
		relative_point = rotationMat*relative_point;

		double distance_x = fabs(relative_point.x - car_length/3.0);
		double distance_y = fabs(relative_point.y);

		if(distance_x  <= car_length*0.5 && distance_y <= car_width*0.5) // don't detect yourself
			continue;

		//obj.center.pos = avg_center;
		nOrPoints += point_cloud.points.size();
		nPoints += obj.contour.size();
		//std::cout << " Distance_X: " << distance_x << ", " << " Distance_Y: " << distance_y << ", " << " Size: " << object_size << std::endl;
		obstacles_list.push_back(obj);
	}

	nOriginalPoints = nOrPoints;
	nContourPoints =  nPoints;
}

PlannerHNS::SHIFT_POS ROSHelpers::ConvertShiftFromAutowareToPlannerH(const PlannerHNS::AUTOWARE_SHIFT_POS& shift)
{
	if(shift == PlannerHNS::AW_SHIFT_POS_DD)
		return PlannerHNS::SHIFT_POS_DD;
	else if(shift == PlannerHNS::AW_SHIFT_POS_RR)
		return PlannerHNS::SHIFT_POS_RR;
	else if(shift == PlannerHNS::AW_SHIFT_POS_NN)
		return PlannerHNS::SHIFT_POS_NN;
	else if(shift == PlannerHNS::AW_SHIFT_POS_PP)
		return PlannerHNS::SHIFT_POS_PP;
	else if(shift == PlannerHNS::AW_SHIFT_POS_BB)
		return PlannerHNS::SHIFT_POS_BB;
	else if(shift == PlannerHNS::AW_SHIFT_POS_SS)
		return PlannerHNS::SHIFT_POS_SS;
	else
		return PlannerHNS::SHIFT_POS_UU;
}

PlannerHNS::AUTOWARE_SHIFT_POS ROSHelpers::ConvertShiftFromPlannerHToAutoware(const PlannerHNS::SHIFT_POS& shift)
{
	if(shift == PlannerHNS::SHIFT_POS_DD)
		return PlannerHNS::AW_SHIFT_POS_DD;
	else if(shift == PlannerHNS::SHIFT_POS_RR)
		return PlannerHNS::AW_SHIFT_POS_RR;
	else if(shift == PlannerHNS::SHIFT_POS_NN)
		return PlannerHNS::AW_SHIFT_POS_NN;
	else if(shift == PlannerHNS::SHIFT_POS_PP)
		return PlannerHNS::AW_SHIFT_POS_PP;
	else if(shift == PlannerHNS::SHIFT_POS_BB)
		return PlannerHNS::AW_SHIFT_POS_BB;
	else if(shift == PlannerHNS::SHIFT_POS_SS)
		return PlannerHNS::AW_SHIFT_POS_SS;
	else
		return PlannerHNS::AW_SHIFT_POS_UU;
}

PlannerHNS::AutowareBehaviorState ROSHelpers::ConvertBehaviorStateFromPlannerHToAutoware(const PlannerHNS::BehaviorState& beh)
{
	PlannerHNS::AutowareBehaviorState arw_state;
	arw_state.followDistance = beh.followDistance;
	arw_state.followVelocity = beh.followVelocity;
	arw_state.maxVelocity = beh.maxVelocity;
	arw_state.minVelocity = beh.minVelocity;
	arw_state.stopDistance = beh.stopDistance;

	if(beh.indicator == PlannerHNS::LIGHT_INDICATOR::INDICATOR_LEFT)
		arw_state.indicator = PlannerHNS::AW_INDICATOR_LEFT;
	else if(beh.indicator == PlannerHNS::LIGHT_INDICATOR::INDICATOR_RIGHT)
		arw_state.indicator = PlannerHNS::AW_INDICATOR_RIGHT;
	else if(beh.indicator == PlannerHNS::LIGHT_INDICATOR::INDICATOR_BOTH)
		arw_state.indicator = PlannerHNS::AW_INDICATOR_BOTH;
	else if(beh.indicator == PlannerHNS::LIGHT_INDICATOR::INDICATOR_NONE)
		arw_state.indicator = PlannerHNS::AW_INDICATOR_NONE;

	if(beh.state == PlannerHNS::INITIAL_STATE)
		arw_state.state = PlannerHNS::AW_INITIAL_STATE;
	else if(beh.state == PlannerHNS::WAITING_STATE)
		arw_state.state = PlannerHNS::AW_WAITING_STATE;
	else if(beh.state == PlannerHNS::FORWARD_STATE)
		arw_state.state = PlannerHNS::AW_FORWARD_STATE;
	else if(beh.state == PlannerHNS::STOPPING_STATE)
		arw_state.state = PlannerHNS::AW_STOPPING_STATE;
	else if(beh.state == PlannerHNS::EMERGENCY_STATE)
		arw_state.state = PlannerHNS::AW_EMERGENCY_STATE;
	else if(beh.state == PlannerHNS::TRAFFIC_LIGHT_STOP_STATE)
		arw_state.state = PlannerHNS::AW_TRAFFIC_LIGHT_STOP_STATE;
	else if(beh.state == PlannerHNS::STOP_SIGN_STOP_STATE)
		arw_state.state = PlannerHNS::AW_STOP_SIGN_STOP_STATE;
	else if(beh.state == PlannerHNS::FOLLOW_STATE)
		arw_state.state = PlannerHNS::AW_FOLLOW_STATE;
	else if(beh.state == PlannerHNS::LANE_CHANGE_STATE)
		arw_state.state = PlannerHNS::AW_LANE_CHANGE_STATE;
	else if(beh.state == PlannerHNS::OBSTACLE_AVOIDANCE_STATE)
		arw_state.state = PlannerHNS::AW_OBSTACLE_AVOIDANCE_STATE;
	else if(beh.state == PlannerHNS::FINISH_STATE)
		arw_state.state = PlannerHNS::AW_FINISH_STATE;


	return arw_state;

}

void ROSHelpers::ConvertFromLocalLaneToAutowareLane(const std::vector<PlannerHNS::WayPoint>& path, autoware_msgs::Lane& trajectory , const unsigned int& iStart)
{
	trajectory.waypoints.clear();

	for(unsigned int i = iStart; i < path.size(); i++)
	{
		autoware_msgs::Waypoint wp;
		wp.pose.pose.position.x = path.at(i).pos.x;
		wp.pose.pose.position.y = path.at(i).pos.y;
		wp.pose.pose.position.z = path.at(i).pos.z;
		wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(path.at(i).pos.a));

		wp.twist.twist.linear.x = path.at(i).v;
		wp.lane_id = path.at(i).laneId;
		wp.stop_line_id = path.at(i).stopLineID;
		wp.left_lane_id = path.at(i).LeftPointId;
		wp.right_lane_id = path.at(i).RightPointId;
		wp.time_cost = path.at(i).timeCost;

		wp.gid = path.at(i).gid;

		//wp.cost = path.at(i).cost;
		wp.cost = 0;

		if(path.at(i).actionCost.size()>0)
		{
			wp.direction = path.at(i).actionCost.at(0).first;
			wp.cost += path.at(i).actionCost.at(0).second;
		}

		trajectory.waypoints.push_back(wp);
	}
}

void ROSHelpers::ConvertFromLocalLaneToAutowareLane(const std::vector<PlannerHNS::GPSPoint>& path, autoware_msgs::Lane& trajectory)
{
	trajectory.waypoints.clear();

	for(unsigned int i=0; i < path.size(); i++)
	{
		autoware_msgs::Waypoint wp;
		wp.pose.pose.position.x = path.at(i).x;
		wp.pose.pose.position.y = path.at(i).y;
		wp.pose.pose.position.z = path.at(i).z;
		wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(path.at(i).a));

		trajectory.waypoints.push_back(wp);
	}
}

void ROSHelpers::ConvertFromAutowareLaneToLocalLane(const autoware_msgs::Lane& trajectory, std::vector<PlannerHNS::WayPoint>& path)
{
	path.clear();

	for(unsigned int i=0; i < trajectory.waypoints.size(); i++)
	{
		PlannerHNS::WayPoint wp;
		wp.pos.x = trajectory.waypoints.at(i).pose.pose.position.x;
		wp.pos.y = trajectory.waypoints.at(i).pose.pose.position.y;
		wp.pos.z = trajectory.waypoints.at(i).pose.pose.position.z;
		wp.pos.a = tf::getYaw(trajectory.waypoints.at(i).pose.pose.orientation);

		wp.v = trajectory.waypoints.at(i).twist.twist.linear.x;

		wp.gid = trajectory.waypoints.at(i).gid;
		wp.laneId = trajectory.waypoints.at(i).lane_id;
		wp.stopLineID = trajectory.waypoints.at(i).stop_line_id;
		wp.LeftPointId = trajectory.waypoints.at(i).left_lane_id;
		wp.RightPointId = trajectory.waypoints.at(i).right_lane_id;
		wp.timeCost = trajectory.waypoints.at(i).time_cost;

		if(trajectory.waypoints.at(i).direction == 0)
			wp.bDir = PlannerHNS::FORWARD_DIR;
		else if(trajectory.waypoints.at(i).direction == 1)
			wp.bDir = PlannerHNS::FORWARD_LEFT_DIR;
		else if(trajectory.waypoints.at(i).direction == 2)
			wp.bDir = PlannerHNS::FORWARD_RIGHT_DIR;
		else if(trajectory.waypoints.at(i).direction == 3)
			wp.bDir = PlannerHNS::BACKWARD_DIR;
		else if(trajectory.waypoints.at(i).direction == 4)
			wp.bDir = PlannerHNS::BACKWARD_LEFT_DIR;
		else if(trajectory.waypoints.at(i).direction == 5)
			wp.bDir = PlannerHNS::BACKWARD_RIGHT_DIR;
		else if(trajectory.waypoints.at(i).direction == 6)
			wp.bDir = PlannerHNS::STANDSTILL_DIR;

		wp.cost = trajectory.waypoints.at(i).cost;

		path.push_back(wp);
	}
}

void ROSHelpers::createGlobalLaneArrayMarker(std_msgs::ColorRGBA color,
		const autoware_msgs::LaneArray &lane_waypoints_array, visualization_msgs::MarkerArray& markerArray)
{
  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time();
  lane_waypoint_marker.ns = "global_lane_array_marker";
  lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.75;
  lane_waypoint_marker.scale.y = 0.75;
  lane_waypoint_marker.color = color;
  lane_waypoint_marker.frame_locked = false;

  int count = 0;
  for (unsigned int i=0; i<  lane_waypoints_array.lanes.size(); i++)
  {
    lane_waypoint_marker.points.clear();
    lane_waypoint_marker.id = count;

    for (unsigned int j=0; j < lane_waypoints_array.lanes.at(i).waypoints.size(); j++)
    {
      geometry_msgs::Point point;
      point = lane_waypoints_array.lanes.at(i).waypoints.at(j).pose.pose.position;
      lane_waypoint_marker.points.push_back(point);
    }
    markerArray.markers.push_back(lane_waypoint_marker);
    count++;
  }

}

void ROSHelpers::createGlobalLaneArrayVelocityMarker(const autoware_msgs::LaneArray &lane_waypoints_array
		, visualization_msgs::MarkerArray& markerArray)
{
  visualization_msgs::MarkerArray tmp_marker_array;
  // display by markers the velocity of each waypoint.
  visualization_msgs::Marker velocity_marker;
  velocity_marker.header.frame_id = "map";
  velocity_marker.header.stamp = ros::Time();
  velocity_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  velocity_marker.action = visualization_msgs::Marker::ADD;
  //velocity_marker.scale.z = 0.4;
  velocity_marker.color.a = 0.9;
  velocity_marker.color.r = 1;
  velocity_marker.color.g = 1;
  velocity_marker.color.b = 1;
  velocity_marker.frame_locked = false;

  int count = 1;
  for (unsigned int i=0; i<  lane_waypoints_array.lanes.size(); i++)
  {

	  std::ostringstream str_count;
	  str_count << count;
    velocity_marker.ns = "global_velocity_lane_" + str_count.str();
    for (unsigned int j=0; j < lane_waypoints_array.lanes.at(i).waypoints.size(); j++)
    {
      //std::cout << _waypoints[i].GetX() << " " << _waypoints[i].GetY() << " " << _waypoints[i].GetZ() << " " << _waypoints[i].GetVelocity_kmh() << std::endl;
      velocity_marker.id = j;
      geometry_msgs::Point relative_p;
      relative_p.y = 0.5;
      velocity_marker.pose.position = calcAbsoluteCoordinate(relative_p, lane_waypoints_array.lanes.at(i).waypoints.at(j).pose.pose);
      velocity_marker.pose.position.z += 0.2;

      // double to string
      std::ostringstream str_out;
      str_out << lane_waypoints_array.lanes.at(i).waypoints.at(j).twist.twist.linear.x;
      //std::string vel = str_out.str();
      velocity_marker.text = str_out.str();//vel.erase(vel.find_first_of(".") + 2);

      tmp_marker_array.markers.push_back(velocity_marker);
    }
    count++;
  }

  markerArray.markers.insert(markerArray.markers.end(), tmp_marker_array.markers.begin(),
                                       tmp_marker_array.markers.end());
}

void ROSHelpers::createGlobalLaneArrayOrientationMarker(const autoware_msgs::LaneArray &lane_waypoints_array
		, visualization_msgs::MarkerArray& markerArray)
{
  visualization_msgs::MarkerArray tmp_marker_array;
  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time();
  lane_waypoint_marker.type = visualization_msgs::Marker::ARROW;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.6;
  lane_waypoint_marker.scale.y = 0.2;
  lane_waypoint_marker.scale.z = 0.1;
  lane_waypoint_marker.color.r = 1.0;
  lane_waypoint_marker.color.a = 1.0;
  //lane_waypoint_marker.frame_locked = false;

  lane_waypoint_marker.ns = "global_lane_waypoint_orientation_marker";

  int count = 1;
  for (unsigned int i=0; i<  lane_waypoints_array.lanes.size(); i++)
  {
//	  std::ostringstream str_ns;
//	  str_ns << "global_lane_waypoint_orientation_marker_";
//	  str_ns << i;
//	 lane_waypoint_marker.ns = str_ns.str();

    for (unsigned int j=0; j < lane_waypoints_array.lanes.at(i).waypoints.size(); j++)
    {
    	lane_waypoint_marker.id = count;
    	lane_waypoint_marker.pose = lane_waypoints_array.lanes.at(i).waypoints.at(j).pose.pose;

    	if(lane_waypoints_array.lanes.at(i).waypoints.at(j).dtlane.dir == 1)
    	{
    		lane_waypoint_marker.color.r = 0.0;
    		lane_waypoint_marker.color.g = 1.0;
    		lane_waypoint_marker.color.b = 0.0;
    		tmp_marker_array.markers.push_back(lane_waypoint_marker);
    	}
    	else if(lane_waypoints_array.lanes.at(i).waypoints.at(j).dtlane.dir == 2)
    	{
    		lane_waypoint_marker.color.r = 0.0;
			lane_waypoint_marker.color.g = 0.0;
			lane_waypoint_marker.color.b = 1.0;
			tmp_marker_array.markers.push_back(lane_waypoint_marker);
    	}
    	else
    	{

    		if(lane_waypoints_array.lanes.at(i).waypoints.at(j).cost >= 100)
    		{
    			lane_waypoint_marker.color.r = 1.0;
				lane_waypoint_marker.color.g = 0.0;
				lane_waypoint_marker.color.b = 0.0;
				tmp_marker_array.markers.push_back(lane_waypoint_marker);
    		}
    		else
    		{
				lane_waypoint_marker.color.r = 0.0;
				lane_waypoint_marker.color.g = 0.8;
				lane_waypoint_marker.color.b = 0.0;
				tmp_marker_array.markers.push_back(lane_waypoint_marker);
    		}
    	}


      count++;
    }
  }

  markerArray.markers.insert(markerArray.markers.end(), tmp_marker_array.markers.begin(),
										   tmp_marker_array.markers.end());
}

void ROSHelpers::GetTrafficLightForVisualization(std::vector<PlannerHNS::TrafficLight>& lights, visualization_msgs::MarkerArray& markerArray)
{
	markerArray.markers.clear();
	for(unsigned int i=0; i<lights.size(); i++)
	{
		if(lights.at(i).lightState == RED_LIGHT)
		{
			visualization_msgs::Marker mkr = CreateGenMarker(lights.at(i).pos.x,lights.at(i).pos.y,lights.at(i).pos.z,0,1,0,0,3,i,"traffic_light_visualize", visualization_msgs::Marker::SPHERE);
			markerArray.markers.push_back(mkr);
		}
		else if(lights.at(i).lightState == GREEN_LIGHT)
		{
			visualization_msgs::Marker mkr = CreateGenMarker(lights.at(i).pos.x,lights.at(i).pos.y,lights.at(i).pos.z,0,0,1,0,3,i,"traffic_light_visualize", visualization_msgs::Marker::SPHERE);
			markerArray.markers.push_back(mkr);
		}
	}
}

void ROSHelpers::ConvertFromAutowareDetectedObjectToOpenPlannerDetectedObject(const autoware_msgs::DetectedObject& det_obj, PlannerHNS::DetectedObject& obj)
{
	obj.id = det_obj.id;
	obj.label = det_obj.label;
	obj.l = det_obj.dimensions.x;
	obj.w = det_obj.dimensions.y;
	obj.h = det_obj.dimensions.z;

	obj.center.pos.x = det_obj.pose.position.x;
	obj.center.pos.y = det_obj.pose.position.y;
	obj.center.pos.z = det_obj.pose.position.z;
	obj.center.pos.a = tf::getYaw(det_obj.pose.orientation);

	obj.center.v = det_obj.velocity.linear.x;
	obj.acceleration_raw = det_obj.velocity.linear.y;
	obj.acceleration_desc = det_obj.velocity.linear.z;
	obj.bVelocity = det_obj.velocity_reliable;
	obj.bDirection = det_obj.pose_reliable;

	if(det_obj.indicator_state == 0)
		obj.indicator_state = PlannerHNS::INDICATOR_LEFT;
	else if(det_obj.indicator_state == 1)
		obj.indicator_state = PlannerHNS::INDICATOR_RIGHT;
	else if(det_obj.indicator_state == 2)
		obj.indicator_state = PlannerHNS::INDICATOR_BOTH;
	else if(det_obj.indicator_state == 3)
		obj.indicator_state = PlannerHNS::INDICATOR_NONE;

	PlannerHNS::GPSPoint p;
	obj.contour.clear();

	for(unsigned int j=0; j < det_obj.convex_hull.polygon.points.size(); j++)
	{

		p.x = det_obj.convex_hull.polygon.points.at(j).x;
		p.y = det_obj.convex_hull.polygon.points.at(j).y;
		p.z = det_obj.convex_hull.polygon.points.at(j).z;
		obj.contour.push_back(p);
	}

	obj.predTrajectories.clear();

	for(unsigned int j = 0 ; j < det_obj.candidate_trajectories.lanes.size(); j++)
	{
		std::vector<PlannerHNS::WayPoint> _traj;
		PlannerHNS::ROSHelpers::ConvertFromAutowareLaneToLocalLane(det_obj.candidate_trajectories.lanes.at(j), _traj);
		for(unsigned int k=0; k < _traj.size(); k++)
			_traj.at(k).collisionCost = det_obj.candidate_trajectories.lanes.at(j).cost;

		obj.predTrajectories.push_back(_traj);
	}
}

void ROSHelpers::ConvertFromOpenPlannerDetectedObjectToAutowareDetectedObject(const PlannerHNS::DetectedObject& det_obj, const bool& bSimulationMode, autoware_msgs::DetectedObject& obj)
{
	if(bSimulationMode)
		obj.id = det_obj.originalID;
	else
		obj.id = det_obj.id;

	obj.label = det_obj.label;
	obj.indicator_state = det_obj.indicator_state;
	obj.dimensions.x = det_obj.l;
	obj.dimensions.y = det_obj.w;
	obj.dimensions.z = det_obj.h;

	obj.pose.position.x = det_obj.center.pos.x;
	obj.pose.position.y = det_obj.center.pos.y;
	obj.pose.position.z = det_obj.center.pos.z;
	obj.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, UtilityHNS::UtilityH::SplitPositiveAngle(det_obj.center.pos.a));

	obj.velocity.linear.x = det_obj.center.v;
	obj.velocity.linear.y = det_obj.acceleration_raw;
	obj.velocity.linear.z = det_obj.acceleration_desc;
	obj.velocity_reliable = det_obj.bVelocity;
	obj.pose_reliable = det_obj.bDirection;

	geometry_msgs::Point32 p;
	obj.convex_hull.polygon.points.clear();

	for(unsigned int j=0; j < det_obj.contour.size(); j++)
	{
		p.x = det_obj.contour.at(j).x;
		p.y = det_obj.contour.at(j).y;
		p.z = det_obj.contour.at(j).z;
		obj.convex_hull.polygon.points.push_back(p);
	}


	obj.candidate_trajectories.lanes.clear();
	for(unsigned int j = 0 ; j < det_obj.predTrajectories.size(); j++)
	{
		autoware_msgs::Lane pred_traj;
		PlannerHNS::ROSHelpers::ConvertFromLocalLaneToAutowareLane(det_obj.predTrajectories.at(j), pred_traj);
		if(det_obj.predTrajectories.at(j).size() > 0)
		{
			pred_traj.cost = det_obj.predTrajectories.at(j).at(0).collisionCost;
		}
		pred_traj.lane_index = 0;
		obj.candidate_trajectories.lanes.push_back(pred_traj);
	}
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
	PlannerHNS::MappingHelpers::ConstructRoadNetworkFromROSMessage(lanes, points, dts, inters, areas, line_data, stop_line_data, signal_data, vector_data, curb_data, roadedge_data,way_area, crossing, nodes_data,  conn_data, origin, out_map);
}

void ROSHelpers::GetIndicatorArrows(const PlannerHNS::WayPoint& center, const double& width,const double& length, const PlannerHNS::LIGHT_INDICATOR& indicator, const int& id, visualization_msgs::MarkerArray& markerArray)
{
	double critical_lateral_distance =  width/2.0 + 0.2;
	//double critical_long_front_distance =  carInfo.length/2.0 ;
	PlannerHNS::GPSPoint top_right(critical_lateral_distance, length, center.pos.z, 0);
	PlannerHNS::GPSPoint top_left(-critical_lateral_distance, length, center.pos.z, 0);

	PlannerHNS::Mat3 invRotationMat(center.pos.a-M_PI_2);
	PlannerHNS::Mat3 invTranslationMat(center.pos.x, center.pos.y);

	top_right = invRotationMat*top_right;
	top_right = invTranslationMat*top_right;
	top_left = invRotationMat*top_left;
	top_left = invTranslationMat*top_left;

	top_right.a = center.pos.a - M_PI_2;
	top_left.a = center.pos.a + M_PI_2;

	std_msgs::ColorRGBA color_l, color_r;
	color_l.r = 1; color_l.g = 1;color_l.b = 1;
	color_r.r = 1; color_r.g = 1;color_r.b = 1;

	if(indicator == PlannerHNS::INDICATOR_LEFT)
	{
		color_l.b = 0;
	}
	else if(indicator == PlannerHNS::INDICATOR_RIGHT )
	{
		color_r.b = 0;
	}
	else if(indicator == PlannerHNS::INDICATOR_BOTH)
	{
		color_l.b = 0;
		color_r.b = 0;
	}

	visualization_msgs::Marker mkr_l = PlannerHNS::ROSHelpers::CreateGenMarker(top_left.x,top_left.y,top_left.z,top_left.a,color_l.r,color_l.g,color_l.b,1.0, id,"simu_car_indicator_left", visualization_msgs::Marker::ARROW);
	mkr_l.scale.y = 0.4;
	mkr_l.scale.z = 0.4;
	visualization_msgs::Marker mkr_r = PlannerHNS::ROSHelpers::CreateGenMarker(top_right.x,top_right.y,top_right.z,top_right.a,color_r.r,color_r.g,color_r.b,1.0, id,"simu_car_indicator_right", visualization_msgs::Marker::ARROW);
	mkr_r.scale.y = 0.4;
	mkr_r.scale.z = 0.4;
	markerArray.markers.push_back(mkr_l);
	markerArray.markers.push_back(mkr_r);
}

void ROSHelpers::TTC_PathRviz(const std::vector<PlannerHNS::WayPoint>& path, visualization_msgs::MarkerArray& markerArray)
{
	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	lane_waypoint_marker.ns = "ttc_path";
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
	lane_waypoint_marker.scale.x = 1;
	lane_waypoint_marker.scale.y = 1;
	lane_waypoint_marker.scale.z = 1;
	lane_waypoint_marker.frame_locked = false;
	std_msgs::ColorRGBA  total_color, curr_color;

	lane_waypoint_marker.color.a = 0.9;
	lane_waypoint_marker.color.r = 0.5;
	lane_waypoint_marker.color.g = 1.0;
	lane_waypoint_marker.color.b = 0.0;

	lane_waypoint_marker.id = 1;
	for (unsigned int i = 0; i < path.size(); i++)
	{
		geometry_msgs::Point point;
		point.x = path.at(i).pos.x;
		point.y = path.at(i).pos.y;
		point.z = path.at(i).pos.z;

		lane_waypoint_marker.points.push_back(point);

		markerArray.markers.push_back(lane_waypoint_marker);
	}
}

}
