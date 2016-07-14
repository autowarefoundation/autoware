/*
 * PlannerHHandler.cpp
 *
 *  Created on: Jul 12, 2016
 *      Author: ai-driver
 */

#include "PlannerHHandler.h"

namespace PlannerXNS
{

PlannerH_Handler::PlannerH_Handler()
{
	//TODO Make sure these number are correct
	SimulationNS::CAR_BASIC_INFO carInfo;
	m_State.Init(1.9, 4.2,carInfo);

	m_pPlannerH = new PlannerHNS::PlannerH(m_PlanningParams);
}

PlannerH_Handler::~PlannerH_Handler()
{
}

void PlannerH_Handler::UpdateRoadMap(const AutowareRoadNetwork& map)
{
	if(bMap) return;

	std::vector<UtilityHNS::AisanLanesFileReader::AisanLane> lanes;
	for(unsigned int i=0; i < map.lanes.size();i++)
	{
		UtilityHNS::AisanLanesFileReader::AisanLane l;
		l.BLID 		=  map.lanes.at(i).blid;
		l.BLID2 	=  map.lanes.at(i).blid2;
		l.BLID3 	=  map.lanes.at(i).blid3;
		l.BLID4 	=  map.lanes.at(i).blid4;
		l.BNID 		=  map.lanes.at(i).bnid;
		l.ClossID 	=  map.lanes.at(i).clossid;
		l.DID 		=  map.lanes.at(i).did;
		l.FLID 		=  map.lanes.at(i).flid;
		l.FLID2 	=  map.lanes.at(i).flid2;
		l.FLID3 	=  map.lanes.at(i).flid3;
		l.FLID4 	=  map.lanes.at(i).flid4;
		l.FNID 		=  map.lanes.at(i).fnid;
		l.JCT 		=  map.lanes.at(i).jct;
		l.LCnt 		=  map.lanes.at(i).lcnt;
		l.LnID 		=  map.lanes.at(i).lnid;
		l.Lno 		=  map.lanes.at(i).lno;
		l.Span 		=  map.lanes.at(i).span;

//		l.LaneChgFG =  map.lanes.at(i).;
//		l.LaneType 	=  map.lanes.at(i).blid;
//		l.LimitVel 	=  map.lanes.at(i).;
//		l.LinkWAID 	=  map.lanes.at(i).blid;
//		l.RefVel 	=  map.lanes.at(i).blid;
//		l.RoadSecID =  map.lanes.at(i).;

		lanes.push_back(l);
	}

	std::vector<UtilityHNS::AisanPointsFileReader::AisanPoints> points;

	for(unsigned int i=0; i < map.points.point_classes.size();i++)
	{
		UtilityHNS::AisanPointsFileReader::AisanPoints p;
		double integ_part = map.points.point_classes.at(i).l;
		double deg = trunc(map.points.point_classes.at(i).l);
		double min = trunc((map.points.point_classes.at(i).l - deg) * 100.0) / 60.0;
		double sec = modf((map.points.point_classes.at(i).l - deg) * 100.0, &integ_part)/36.0;
		double L =  deg + min + sec;

		deg = trunc(map.points.point_classes.at(i).b);
		min = trunc((map.points.point_classes.at(i).b - deg) * 100.0) / 60.0;
		sec = modf((map.points.point_classes.at(i).b - deg) * 100.0, &integ_part)/36.0;
		double B =  deg + min + sec;

		p.B 		= B;
		p.Bx 		= map.points.point_classes.at(i).bx;
		p.H 		= map.points.point_classes.at(i).h;
		p.L 		= L;
		p.Ly 		= map.points.point_classes.at(i).ly;
		p.MCODE1 	= map.points.point_classes.at(i).mcode1;
		p.MCODE2 	= map.points.point_classes.at(i).mcode2;
		p.MCODE3 	= map.points.point_classes.at(i).mcode3;
		p.PID 		= map.points.point_classes.at(i).pid;
		p.Ref 		= map.points.point_classes.at(i).ref;

		points.push_back(p);
	}


	std::vector<UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine> dts;
	for(unsigned int i=0; i < map.dtlanes.size();i++)
	{
		UtilityHNS::AisanCenterLinesFileReader::AisanCenterLine dt;

		dt.Apara 	= map.dtlanes.at(i).apara;
		dt.DID 		= map.dtlanes.at(i).did;
		dt.Dir 		= map.dtlanes.at(i).dir;
		dt.Dist 	= map.dtlanes.at(i).dist;
		dt.LW 		= map.dtlanes.at(i).lw;
		dt.PID 		= map.dtlanes.at(i).pid;
		dt.RW 		= map.dtlanes.at(i).rw;
		dt.cant 	= map.dtlanes.at(i).cant;
		dt.r 		= map.dtlanes.at(i).r;
		dt.slope 	= map.dtlanes.at(i).slope;

		dts.push_back(dt);

	}

	PlannerHNS::GPSPoint origin;// =  m_OriginPoint.pos;
//	origin.x += -1;
//	origin.y += -1;
//	origin.z += -1;

	PlannerHNS::MappingHelpers::ConstructRoadNetworkFromRosMessage(lanes,
			points, dts, origin, m_Map);

	ROS_INFO("Map Data is Updated Successfully.");
	bMap = true;
}

void PlannerH_Handler::UpdateOriginTransformationPoint(const geometry_msgs::Pose& originPoint)
{
	m_OriginPoint = PlannerHNS::WayPoint(originPoint.position.x, originPoint.position.y,
			originPoint.position.z, 0);
	bOriginPoint = true;
}

void PlannerH_Handler::UpdateGlobalGoalPosition(const geometry_msgs::Pose& goalPose)
{
	m_Goal = PlannerHNS::WayPoint(goalPose.position.x+m_OriginPoint.pos.x,
			goalPose.position.y + m_OriginPoint.pos.y, goalPose.position.z + m_OriginPoint.pos.z, tf::getYaw(goalPose.orientation));

	m_State.m_Path.clear();
	m_State.m_TotalPath.clear();
	m_State.m_RollOuts.clear();
	m_State.m_pCurrentBehaviorState->m_Behavior = PlannerHNS::INITIAL_STATE;

	bGoal = true;
}

void PlannerH_Handler::UpdatePredefinedPath(const std::vector<int>& predefinedPath)
{
	m_PredefinedPath = predefinedPath;
	bPredefinedPath = true;
}

bool PlannerH_Handler::GeneratePlan(const geometry_msgs::Pose& currentPose, const cv_tracker::obj_label& detectedObstacles,
		const runtime_manager::traffic_light& trafficLight, const AutowareVehicleState& carState,
		AutowareBehaviorState& behaviorState, visualization_msgs::MarkerArray& pathToVisualize,
		waypoint_follower::LaneArray& pathToFollow)
{

	bool bNewPath = false;
	m_State.state = PlannerHNS::WayPoint(currentPose.position.x+m_OriginPoint.pos.x,
			currentPose.position.y + m_OriginPoint.pos.y, currentPose.position.z + m_OriginPoint.pos.z, tf::getYaw(currentPose.orientation));
	std::vector<PlannerHNS::Obstacle> obst;
	ConvertFromAutowareObstaclesToPlannerH(detectedObstacles, obst);
	PlannerHNS::VehicleState vehState;
	vehState.speed = carState.speed;
	vehState.steer = carState.steer;
	vehState.shift = ConvertShiftFromAutowareToPlannerH(carState.shift);
	PlannerHNS::BehaviorState behavior;

	if(vehState.shift == PlannerHNS::SHIFT_POS_DD)
		std::cout << "Shift DDDDD" << std::endl;
	else 	if(vehState.shift == PlannerHNS::SHIFT_POS_NN)
		std::cout << "Shift NNNNN" << std::endl;

	if(vehState.shift == PlannerHNS::SHIFT_POS_DD)
		m_State.m_pCurrentBehaviorState->GetCalcParams()->bOutsideControl = 1;
	else
		m_State.m_pCurrentBehaviorState->GetCalcParams()->bOutsideControl = 0;

	m_State.CalculateImportantParameterForDecisionMaking(obst, vehState, m_Goal.pos, m_Map);\


	m_State.m_pCurrentBehaviorState = m_State.m_pCurrentBehaviorState->GetNextState();

	PlannerHNS::PreCalculatedConditions *preCalcPrams = m_State.m_pCurrentBehaviorState->GetCalcParams();
	behavior.state = m_State.m_pCurrentBehaviorState->m_Behavior;
	if(behavior.state == PlannerHNS::FOLLOW_STATE)
		behavior.followDistance = preCalcPrams->distanceToNext;
	else
		behavior.followDistance = 0;

	if(preCalcPrams->bUpcomingRight)
		behavior.indicator = PlannerHNS::INDICATOR_RIGHT;
	else if(preCalcPrams->bUpcomingLeft)
		behavior.indicator = PlannerHNS::INDICATOR_LEFT;
	else
		behavior.indicator = PlannerHNS::INDICATOR_NONE;

	//TODO fix this , make get lookahead velocity work
	double max_velocity = 2; //BehaviorsNS::MappingHelpers::GetLookAheadVelocity(m_CarState.m_Path, GetCarPos(), 25);

	behavior.maxVelocity   = max_velocity;
	behavior.minVelocity	= 0;
	behavior.stopDistance 	= preCalcPrams->distanceToStop();
	behavior.followVelocity = preCalcPrams->velocityOfNext;

	std::cout <<  preCalcPrams->ToString(behavior.state) << std::endl;

	if(behavior.state == PlannerHNS::INITIAL_STATE && m_State.m_Path.size() == 0)
	{
		std::vector<PlannerHNS::WayPoint> generatedPath;
		//planner.PlanUsingReedShepp(m_State.state, m_goal, generatedPath);
		ROS_INFO(m_OriginPoint.pos.ToString().c_str());
		ROS_INFO(m_State.state.pos.ToString().c_str());
		ROS_INFO(m_Goal.pos.ToString().c_str());
		m_pPlannerH->PlanUsingDP(m_State.pLane, m_State.state, m_Goal, m_State.state, 2550, m_PredefinedPath, generatedPath);

		m_State.m_TotalPath = generatedPath;
	}

	if(m_State.m_TotalPath.size()>0)
	{

		int currIndex = 1;

		if(m_State.m_Path.size()>0)
		{
			currIndex = PlannerHNS::PlanningHelpers::GetClosestPointIndex(m_State.m_Path, m_State.state);
			std::cout << "before Roll Outs .. " << currIndex << std::endl;
			std::cout << m_State.state.pos.ToString() << std::endl;
			std::cout << m_State.m_Path.at(0).pos.ToString() << std::endl;
		}

		if(m_State.m_RollOuts.size() == 0 || currIndex*2.0 > m_State.m_Path.size())
		{
			m_pPlannerH->GenerateRunoffTrajectory(m_State.m_TotalPath, m_State.state, false,  5, 35, 5, 0,
					m_State.m_pCurrentBehaviorState->m_PlanningParams.carTipMargin,
					m_State.m_pCurrentBehaviorState->m_PlanningParams.rollInMargin,
					m_State.m_pCurrentBehaviorState->m_PlanningParams.rollInSpeedFactor,
					m_State.m_pCurrentBehaviorState->m_PlanningParams.pathDensity,
					m_State.m_pCurrentBehaviorState->m_PlanningParams.rollOutDensity,
					m_State.m_pCurrentBehaviorState->m_PlanningParams.rollOutNumber,
					m_State.m_pCurrentBehaviorState->m_PlanningParams.smoothingDataWeight,
					m_State.m_pCurrentBehaviorState->m_PlanningParams.smoothingSmoothWeight,
					m_State.m_pCurrentBehaviorState->m_PlanningParams.smoothingToleranceError,
					m_State.m_pCurrentBehaviorState->m_PlanningParams.speedProfileFactor,
					false, m_State.m_RollOuts);

			std::cout << "Safe Trajectoy : " << preCalcPrams->iCurrSafeTrajectory << std::endl;


			m_State.m_Path = m_State.m_RollOuts.at(preCalcPrams->iCurrSafeTrajectory);

			if(m_State.m_Path.size() >  0 )
				bNewPath = true;

			if(m_State.m_Path.size()<5)
				preCalcPrams->bGoalReached = true;

			std::cout << "after get next .. " << std::endl;

		}

	}


//	ROS_INFO(preCalcPrams->ToString(behavior.state).c_str());


	//behaviorState = behavior;
	if(bNewPath)
	{
		ROS_INFO("Convert New Path To Visualization.");
		ConvertFromPlannerHToAutowarePathFormat(m_State.m_Path, pathToFollow);
		ConvertFromPlannerHToAutowareVisualizePathFormat(m_State.m_TotalPath, m_State.m_Path, m_State.m_RollOuts, pathToVisualize);
	}

	return bNewPath;
}

void PlannerH_Handler::ConvertFromPlannerHToAutowarePathFormat(const std::vector<PlannerHNS::WayPoint>& path,
		waypoint_follower::LaneArray& laneArray)
{
	waypoint_follower::lane l;
	laneArray.lanes.clear();
	laneArray.lanes.push_back(l);

	for(unsigned int i=0; i < path.size(); i++)
	{
		waypoint_follower::waypoint wp;
		wp.pose.pose.position.x = path.at(i).pos.x;
		wp.pose.pose.position.y = path.at(i).pos.y;
		wp.pose.pose.position.z = path.at(i).pos.z;

		//PlannerHNS::GPSPoint p = path.at(i).pos;
		//std::cout << p.ToString() << std::endl;

		laneArray.lanes.at(0).waypoints.push_back(wp);
	}
}

void PlannerH_Handler::ConvertFromPlannerHToAutowareVisualizePathFormat(const std::vector<PlannerHNS::WayPoint>& total_path,
		const std::vector<PlannerHNS::WayPoint>& curr_path, const std::vector<std::vector<PlannerHNS::WayPoint> >& paths,
			visualization_msgs::MarkerArray& markerArray)
{
	visualization_msgs::Marker lane_waypoint_marker;
	lane_waypoint_marker.header.frame_id = "map";
	lane_waypoint_marker.header.stamp = ros::Time();
	lane_waypoint_marker.ns = "global_lane_array_marker";
	lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
	lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
	lane_waypoint_marker.scale.x = 0.5;
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
		  point.z = paths.at(i).at(j).pos.z;

		  lane_waypoint_marker.points.push_back(point);
		}

		markerArray.markers.push_back(lane_waypoint_marker);
		count++;
	}

	lane_waypoint_marker.points.clear();
	lane_waypoint_marker.id = count;
	lane_waypoint_marker.scale.x = 0.75;
	total_color.r = 1;
	total_color.g = 0;
	total_color.b = 0;
	total_color.a = 0.5;
	lane_waypoint_marker.color = total_color;

	for (unsigned int j=0; j < total_path.size(); j++)
	{
	  geometry_msgs::Point point;

	  point.x = total_path.at(j).pos.x;
	  point.y = total_path.at(j).pos.y;
	  point.z = total_path.at(j).pos.z;

	  lane_waypoint_marker.points.push_back(point);
	}

	markerArray.markers.push_back(lane_waypoint_marker);
	count++;

	lane_waypoint_marker.points.clear();
	lane_waypoint_marker.id = count;
	lane_waypoint_marker.scale.x = 0.5;
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
	  point.z = curr_path.at(j).pos.z;

	  lane_waypoint_marker.points.push_back(point);
	}

	markerArray.markers.push_back(lane_waypoint_marker);
	count++;
}

void PlannerH_Handler::ConvertFromAutowareObstaclesToPlannerH(const cv_tracker::obj_label& detectedObstacles, std::vector<PlannerHNS::Obstacle>& bstacles)
{

}

PlannerHNS::SHIFT_POS PlannerH_Handler::ConvertShiftFromAutowareToPlannerH(const PlannerXNS::AUTOWARE_SHIFT_POS& shift)
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

PlannerXNS::AUTOWARE_SHIFT_POS PlannerH_Handler::ConvertShiftFromPlannerHToAutoware(const PlannerHNS::SHIFT_POS& shift)
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

} /* namespace PlannerXNS */
