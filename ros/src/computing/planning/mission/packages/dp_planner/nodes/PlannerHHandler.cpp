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


	m_pPlannerH = new PlannerHNS::PlannerH();

	m_bFirstCall = false;
	m_bMakeNewPlan = true;
	UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);
}

PlannerH_Handler::~PlannerH_Handler()
{
	if(m_pPlannerH)
		delete m_pPlannerH;
}

void PlannerH_Handler::UpdateVehicleInfo(const double& width, const double& length, const double& wheelBase, const double& maxSteerAngle, const double& turningRadius)
{

	m_VehicleInfo.wheel_base = wheelBase;
	m_VehicleInfo.turning_radius = turningRadius;
	m_VehicleInfo.max_steer_angle = maxSteerAngle;
	m_VehicleInfo.width = width;
	m_VehicleInfo.length = length;


	m_ControlParams.Steering_Gain = SimulationNS::PID_CONST(0.07, 0.02, 0.01);
//	m_ControlParams.SteeringDelay = 0.85;
//	m_ControlParams.Steering_Gain.kD = 0.5;
//	m_ControlParams.Steering_Gain.kP = 0.1;
//	m_ControlParams.Steering_Gain.kI = 0.03;

	m_ControlParams.Velocity_Gain = SimulationNS::PID_CONST(0.1, 0.005, 0.1);

	m_State.m_SimulationSteeringDelayFactor = m_ControlParams.SimulationSteeringDelay;
	m_State.Init(m_ControlParams, m_PlanningParams, m_VehicleInfo);
}

void PlannerH_Handler::UpdatePlanningParams(const AutowarePlanningParams& planningParams)
{
	m_PlanningParams.carTipMargin 				= planningParams.carTipMargin 				;
	m_PlanningParams.enableFollowing 			= planningParams.enableFollowing 			;
	m_PlanningParams.enableHeadingSmoothing 	= planningParams.enableHeadingSmoothing 	;
	m_PlanningParams.enableLaneChange 			= planningParams.enableLaneChange 			;
	m_PlanningParams.enableSwerving 			= planningParams.enableSwerving 			;
	m_PlanningParams.enableTrafficLightBehavior = planningParams.enableTrafficLightBehavior ;
	m_PlanningParams.horizonDistance 			= planningParams.horizonDistance 			;
	m_PlanningParams.maxFollowingDistance 		= planningParams.maxFollowingDistance 		;
	m_PlanningParams.maxSpeed 					= planningParams.maxSpeed 					;
	m_PlanningParams.microPlanDistance 			= planningParams.microPlanDistance 			;
	m_PlanningParams.minDistanceToAvoid 		= planningParams.minDistanceToAvoid 		;
	m_PlanningParams.minFollowingDistance 		= planningParams.minFollowingDistance 		;
	m_PlanningParams.minSpeed 					= planningParams.minSpeed 					;
	m_PlanningParams.pathDensity 				= planningParams.pathDensity 				;
	m_PlanningParams.planningDistance 			= planningParams.planningDistance 			;
	m_PlanningParams.rollInMargin 				= planningParams.rollInMargin 				;
	m_PlanningParams.rollInSpeedFactor 			= planningParams.rollInSpeedFactor 			;
	m_PlanningParams.rollOutDensity 			= planningParams.rollOutDensity 			;
	m_PlanningParams.rollOutNumber 				= planningParams.rollOutNumber 				;
	m_PlanningParams.speedProfileFactor 		= planningParams.speedProfileFactor 		;

	m_ControlParams.Steering_Gain = SimulationNS::PID_CONST(0.07, 0.02, 0.01);
//	m_ControlParams.SteeringDelay = 0.85;
//	m_ControlParams.Steering_Gain.kD = 0.5;
//	m_ControlParams.Steering_Gain.kP = 0.1;
//	m_ControlParams.Steering_Gain.kI = 0.03;


	m_ControlParams.Velocity_Gain = SimulationNS::PID_CONST(0.1, 0.005, 0.1);
	m_State.m_SimulationSteeringDelayFactor = m_ControlParams.SimulationSteeringDelay;
	m_State.Init(m_ControlParams, m_PlanningParams, m_VehicleInfo);
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

	//PlannerHNS::WayPoint wp =  PlannerHNS::MappingHelpers::GetFirstWaypoint(m_Map);
	//std::cout << "Map First Point: " << wp.pos.ToString() << std::endl;

	//ROS_INFO("Map Data is Updated Successfully.");
	bMap = true;
}

bool PlannerH_Handler::LoadRoadMap(const std::string& mapFilePath, const bool& bKML_Map, visualization_msgs::MarkerArray& mapToVisualize)
{
	if(bMap) return false;

	if(bKML_Map)
		PlannerHNS::MappingHelpers::LoadKML(mapFilePath, m_Map);
	else
		PlannerHNS::MappingHelpers::ConstructRoadNetworkFromDataFiles(mapFilePath, m_Map);

	ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, mapToVisualize);

	bMap = true;
	return true;
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

	PlannerHNS::WayPoint* pW = PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(m_Goal, m_Map);
	if(pW)
		m_Goal.pos = pW->pos;

	m_State.m_Path.clear();
	m_State.m_TotalPath.clear();
	m_State.m_RollOuts.clear();
	m_State.m_pCurrentBehaviorState->m_Behavior = PlannerHNS::INITIAL_STATE;
	m_bMakeNewPlan = true;

	bGoal = true;
}

void PlannerH_Handler::UpdatePredefinedPath(const std::vector<int>& predefinedPath)
{
	m_PredefinedPath = predefinedPath;
	bPredefinedPath = true;
}

bool PlannerH_Handler::GeneratePlan(const geometry_msgs::Pose& currentPose, const jsk_recognition_msgs::BoundingBoxArray& detectedObstacles,
		const runtime_manager::traffic_light& trafficLight, const AutowareVehicleState& carState,
		AutowareBehaviorState& behaviorState, visualization_msgs::MarkerArray& pathToVisualize,
		waypoint_follower::LaneArray& pathToFollow)
{


	m_State.state = PlannerHNS::WayPoint(currentPose.position.x+m_OriginPoint.pos.x,
			currentPose.position.y + m_OriginPoint.pos.y, currentPose.position.z + m_OriginPoint.pos.z, tf::getYaw(currentPose.orientation));

	//std::cout << "## Position After Receiving "<< m_State.state.pos.ToString() << std::endl;

	if(!m_bFirstCall)
	{
		m_Start = m_State.state;
		m_bFirstCall = true;
		m_ObstacleTracking.Initialize(m_State.state);
	}

	PlannerHNS::VehicleState vehState;
	vehState.speed = carState.speed;
	vehState.steer = carState.steer;
	vehState.shift = ConvertShiftFromAutowareToPlannerH(carState.shift);


	if(vehState.shift == PlannerHNS::SHIFT_POS_DD)
		std::cout << "Shift DDDDD" << std::endl;
	else 	if(vehState.shift == PlannerHNS::SHIFT_POS_NN)
		std::cout << "Shift NNNNN" << std::endl;

	//if(vehState.shift == PlannerHNS::SHIFT_POS_DD)
		m_State.m_pCurrentBehaviorState->GetCalcParams()->bOutsideControl = 1;
	//else
	//	m_State.m_pCurrentBehaviorState->GetCalcParams()->bOutsideControl = 0;

	std::vector<PlannerHNS::WayPoint> generatedTotalPath = m_State.m_TotalPath;
	std::vector<std::vector<PlannerHNS::WayPoint> > rollOuts;
	bool bNewPlan = false;
	/**
	 * Path Planning Step (Global Planning)
	 */
//	int currIndexToal = PlannerHNS::PlanningHelpers::GetClosestPointIndex(m_State.m_TotalPath, m_State.state);
//	int index_limit_total = m_State.m_TotalPath.size() - 6;
//	if(index_limit_total<=0)
//		index_limit_total =  m_State.m_TotalPath.size()/2.0;
//
//	if(currIndexToal > index_limit_total)
//	{
//		std::cout << "Switch Start and Goal Positions " << std::endl;
//		PlannerHNS::WayPoint g_p = m_Goal;
//		m_Goal = m_Start;
//		m_Start = g_p;
//		m_bMakeNewPlan = true;
//	}

	if((m_CurrentBehavior.state == PlannerHNS::INITIAL_STATE && m_State.m_Path.size() == 0 && m_bMakeNewPlan) || m_bMakeNewPlan)
	{
		//planner.PlanUsingReedShepp(pR->m_State.state, pR->m_goal, generatedPath);
		double ret = m_pPlannerH->PlanUsingDP(m_State.pLane, m_State.state, m_Goal,
				m_State.state, m_PlanningParams.planningDistance, m_PredefinedPath, generatedTotalPath);
		if(ret == 0) generatedTotalPath.clear();

		if(generatedTotalPath.size() > 0)
		{
			std::cout << "New DP Path -> " << generatedTotalPath.size() << std::endl;
			m_Goal = generatedTotalPath.at(generatedTotalPath.size()-1);
			bNewPlan = true;
			m_bMakeNewPlan = false;
		}
	}

	/**
	 * Behavior Generator , State Machine , Decision making Step
	 */

	if(bNewPlan)
		m_State.m_TotalPath = generatedTotalPath;


	std::vector<PlannerHNS::DetectedObject> obj_list;
	ConvertFromAutowareObstaclesToPlannerH(detectedObstacles, obj_list);
	m_ObstacleTracking.DoOneStep(m_State.state, obj_list);
	obj_list = m_ObstacleTracking.m_DetectedObjects;


	double dt  = UtilityHNS::UtilityH::GetTimeDiffNow(m_PlanningTimer);
	UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);
	m_CurrentBehavior = m_State.DoOneStep(dt, vehState, obj_list, m_Goal.pos, m_Map, true);

	std::cout << m_State.m_pCurrentBehaviorState->GetCalcParams()->ToString(m_CurrentBehavior.state) << ", Speed : " << m_CurrentBehavior.maxVelocity << std::endl;

	behaviorState = ConvertBehaviorStateFromPlannerHToAutoware(m_CurrentBehavior);

	if(m_CurrentBehavior.bNewPlan)
	{
		//ROS_INFO("Convert New Path To Visualization.");
		ConvertFromPlannerHToAutowarePathFormat(m_State.m_Path, pathToFollow);
		ConvertFromPlannerHToAutowareVisualizePathFormat(m_State.m_TotalPath, m_State.m_Path, m_State.m_RollOuts, pathToVisualize);
	}

	return m_CurrentBehavior.bNewPlan;
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
		wp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(UtilityHNS::UtilityH::SplitPositiveAngle(path.at(i).pos.a));
		wp.twist.twist.linear.x = path.at(i).v;
		//PlannerHNS::GPSPoint p = path.at(i).pos;
		//std::cout << p.ToString() << std::endl;

		laneArray.lanes.at(0).waypoints.push_back(wp);
	}
}

void PlannerH_Handler::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(const PlannerHNS::RoadNetwork& map,	visualization_msgs::MarkerArray& markerArray)
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
		  point.z = paths.at(i).at(j).pos.z;

		  lane_waypoint_marker.points.push_back(point);
		}

		markerArray.markers.push_back(lane_waypoint_marker);
		count++;
	}

	lane_waypoint_marker.points.clear();
	lane_waypoint_marker.id = count;
	lane_waypoint_marker.scale.x = 0.35;
	lane_waypoint_marker.scale.y = 0.35;
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
	  point.z = curr_path.at(j).pos.z;

	  lane_waypoint_marker.points.push_back(point);
	}

	markerArray.markers.push_back(lane_waypoint_marker);
	count++;
}

void PlannerH_Handler::ConvertFromAutowareObstaclesToPlannerH(const jsk_recognition_msgs::BoundingBoxArray& detectedObstacles, std::vector<PlannerHNS::DetectedObject>& obstacles_list)
{
	obstacles_list.clear();
	for(unsigned int i =0; i < detectedObstacles.boxes.size(); i++)
	{
		PlannerHNS::DetectedObject obj;
		obj.center = PlannerHNS::WayPoint(detectedObstacles.boxes.at(i).pose.position.x,
				detectedObstacles.boxes.at(i).pose.position.y,
				detectedObstacles.boxes.at(i).pose.position.z,
				detectedObstacles.boxes.at(i).pose.orientation.z);
		obj.w = detectedObstacles.boxes.at(i).dimensions.y;
		obj.l = detectedObstacles.boxes.at(i).dimensions.x;
		obj.h = detectedObstacles.boxes.at(i).dimensions.z;
		obstacles_list.push_back(obj);
	}
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

PlannerXNS::AutowareBehaviorState PlannerH_Handler::ConvertBehaviorStateFromPlannerHToAutoware(const PlannerHNS::BehaviorState& beh)
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

} /* namespace PlannerXNS */
