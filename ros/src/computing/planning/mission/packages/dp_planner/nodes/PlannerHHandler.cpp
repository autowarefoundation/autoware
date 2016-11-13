/*
 * PlannerHHandler.cpp
 *
 *  Created on: Jul 12, 2016
 *      Author: ai-driver
 */

#include "PlannerHHandler.h"
#include "PolygonGenerator.h"

namespace PlannerXNS
{

PlannerH_Handler::PlannerH_Handler()
{
	//TODO Make sure these number are correct

	//Initialize Static Traffic Light
	PlannerHNS::TrafficLight t1, t2;
	PlannerHNS::GPSPoint stopT1(555.84, 181.89,0,0);
	t1.id = 1;
	t1.pos = PlannerHNS::GPSPoint(555.72,193.23, 0, 91.65*DEG2RAD);
	t1.stoppingDistance = hypot(t1.pos.y-stopT1.y, t1.pos.x - stopT1.x);
	m_State.m_TrafficLights.push_back(t1);

	PlannerHNS::GPSPoint stopT2(553.85,193.14,0,0);
	t2.id = 2;
	t2.pos = PlannerHNS::GPSPoint(552.33, 181.42, 0, 270*DEG2RAD);
	t2.stoppingDistance = hypot(t2.pos.y-stopT2.y, t2.pos.x - stopT2.x);
	m_State.m_TrafficLights.push_back(t2);

	m_bSlowDown = false;
	m_SlowDownPoint.x =  287.5;
	m_SlowDownPoint.y = 112.37;

	m_GoNormalPoint.x = 257.39;
	m_GoNormalPoint.y = 146.99;

	m_pPlannerH = new PlannerHNS::PlannerH();
	m_iCurrentGoal = 0;
	m_bMakeNewPlan = true;
	UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);
}

PlannerH_Handler::~PlannerH_Handler()
{
	if(m_pPlannerH)
		delete m_pPlannerH;
}

void PlannerH_Handler::InitStaticGoals(PlannerHNS::RoadNetwork& map)
{
	PlannerHNS::WayPoint g1(557.1, 177.43, 0, 0);
	PlannerHNS::WayPoint g2(553.03, 195.59, 0, 0);
	PlannerHNS::WayPoint g3(-57.23, 60.67, 0, 0);

	PlannerHNS::WayPoint* pW = PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(g1, map);
	if(pW)
	{
		pW->bDir = PlannerHNS::FORWARD_DIR;
		m_goals.push_back(*pW);
	}
	else
		std::cout << "#Planning Error: Goal Position is too far from the road network map!" << std::endl;

	 pW = PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(g2, map);
	if(pW)
	{
		pW->bDir = PlannerHNS::FORWARD_DIR;
		m_goals.push_back(*pW);
	}
	else
		std::cout << "#Planning Error: Goal Position is too far from the road network map!" << std::endl;

	pW = PlannerHNS::MappingHelpers::GetLastWaypoint(map);
	if(pW)
	{
		pW->bDir = PlannerHNS::FORWARD_DIR;
		m_goals.push_back(*pW);
	}
	else
		std::cout << "#Planning Error: Goal Position is too far from the road network map!" << std::endl;
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
	//if(bMap) return;

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

	PlannerHNS::MappingHelpers::ConstructRoadNetworkFromRosMessage(lanes, points, dts, origin, m_Map);

	InitStaticGoals(m_Map);

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

	RosHelpers::ConvertFromRoadNetworkToAutowareVisualizeMapFormat(m_Map, mapToVisualize);
	InitStaticGoals(m_Map);

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
//	m_Goal = PlannerHNS::WayPoint(goalPose.position.x+m_OriginPoint.pos.x,
//			goalPose.position.y + m_OriginPoint.pos.y, goalPose.position.z + m_OriginPoint.pos.z, tf::getYaw(goalPose.orientation));
//
//	PlannerHNS::WayPoint* pW = PlannerHNS::MappingHelpers::GetClosestWaypointFromMap(m_Goal, m_Map);
//	if(pW)
//		m_Goal.pos = pW->pos;

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

lidar_tracker::CloudCluster PlannerH_Handler::GenerateSimulatedObstacleCluster(const double& x_rand, const double& y_rand, const double& z_rand, const int& nPoints, const geometry_msgs::PointStamped& centerPose)
{
	lidar_tracker::CloudCluster cluster;
	cluster.centroid_point.point = centerPose.point;
	pcl::PointCloud<pcl::PointXYZ> point_cloud;

	timespec t;
	for(int i=1; i < nPoints; i++)
	{
		UtilityHNS::UtilityH::GetTickCount(t);
		pcl::PointXYZ p;
		srand(t.tv_nsec/i);
		double x = (double)(rand()%100)/100.0 - 0.5;

		srand(t.tv_nsec/i*i);
		double y = (double)(rand()%100)/100.0 - 0.5;

		srand(t.tv_nsec);
		double z = (double)(rand()%100)/100.0 - 0.5;

		p.x = centerPose.point.x + x*x_rand;
		p.y = centerPose.point.y + y*y_rand;
		p.z = centerPose.point.z + z*z_rand;
		point_cloud.points.push_back(p);
	}

	pcl::toROSMsg(point_cloud, cluster.cloud);

	return cluster;
}

bool PlannerH_Handler::GeneratePlan(const geometry_msgs::Pose& currentPose,
		const jsk_recognition_msgs::BoundingBoxArray& detectedObstacles,
		const lidar_tracker::CloudClusterArray& clusters,
		const runtime_manager::traffic_light& trafficLight, const AutowareVehicleState& carState,
		AutowareBehaviorState& behaviorState, visualization_msgs::MarkerArray& pathToVisualize,
		waypoint_follower::LaneArray& pathToFollow, jsk_recognition_msgs::BoundingBoxArray& trackedObjects,
		visualization_msgs::MarkerArray& detectedPolygons,
		const bool& bEmergencyStop, const bool& bGreenLight, const bool& bOutsideControl,
		const waypoint_follower::lane& aStarPath, geometry_msgs::PoseStamped& startPoint,
		geometry_msgs::PoseStamped& goalPoint, bool& bExternalPlanning)
{


	PlannerHNS::WayPoint current_pose(currentPose.position.x+m_OriginPoint.pos.x,
			currentPose.position.y + m_OriginPoint.pos.y, currentPose.position.z + m_OriginPoint.pos.z, tf::getYaw(currentPose.orientation));
	double drift = hypot(m_State.state.pos.y-current_pose.pos.y, m_State.state .pos.x-current_pose.pos.x);
	if(drift > 10)
		m_bMakeNewPlan = true;

	m_State.state = current_pose;

	//std::cout << "## Position After Receiving "<< m_State.state.pos.ToString() << std::endl;

	PlannerHNS::VehicleState vehState;
	vehState.speed = carState.speed;
	vehState.steer = carState.steer;
	vehState.shift = RosHelpers::ConvertShiftFromAutowareToPlannerH(carState.shift);

	m_State.state = PlannerHNS::MappingHelpers::GetFirstWaypoint(m_Map);
	geometry_msgs::PointStamped objCenter;
	objCenter.point.x = m_State.state.pos.x + 3;
	objCenter.point.y = m_State.state.pos.y + 1;
	objCenter.point.z =  1;
	lidar_tracker::CloudClusterArray clusters2;
	clusters2.clusters.push_back(GenerateSimulatedObstacleCluster(1,0.7,1,1000, objCenter));


//	if(vehState.shift == PlannerHNS::SHIFT_POS_DD)
//		std::cout << "Shift DDDDD" << std::endl;
//	else 	if(vehState.shift == PlannerHNS::SHIFT_POS_NN)
//		std::cout << "Shift NNNNN" << std::endl;

	//if(vehState.shift == PlannerHNS::SHIFT_POS_DD)
		m_State.m_pCurrentBehaviorState->GetCalcParams()->bOutsideControl = bOutsideControl;
	//else
	//	m_State.m_pCurrentBehaviorState->GetCalcParams()->bOutsideControl = 0;

	std::vector<std::vector<PlannerHNS::WayPoint> > generatedTotalPaths;// = m_State.m_TotalPath;
	std::vector<std::vector<PlannerHNS::WayPoint> > rollOuts;
	bool bNewPlan = false;
	/**
	 * Path Planning Step (Global Planning)
	 */
	int currIndexToal = PlannerHNS::PlanningHelpers::GetClosestPointIndex(m_State.m_TotalPath, m_State.state);
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


	if(m_bMakeNewPlan == false && m_CurrentBehavior.state == PlannerHNS::STOPPING_STATE && (m_iCurrentGoal+1) < m_goals.size())
	{
		if(m_State.m_TotalPath.size() > 0 && currIndexToal > m_State.m_TotalPath.size() - 8)
		{
			m_iCurrentGoal = m_iCurrentGoal + 1;
			m_bMakeNewPlan = true;
			m_State.m_pCurrentBehaviorState->GetCalcParams()->bRePlan = true;
		}
	}

	if((m_CurrentBehavior.state == PlannerHNS::INITIAL_STATE && m_State.m_Path.size() == 0 && m_bMakeNewPlan) || m_bMakeNewPlan)
	{
		//planner.PlanUsingReedShepp(pR->m_State.state, pR->m_goal, generatedPath);
		double ret = m_pPlannerH->PlanUsingDP(m_State.pLane, m_State.state, m_goals.at(m_iCurrentGoal),
				m_State.state, m_PlanningParams.planningDistance, m_PredefinedPath, generatedTotalPaths);
		if(ret == 0) generatedTotalPaths.clear();

		if(generatedTotalPaths.size() > 0 && generatedTotalPaths.at(0).size() > 0)
		{
			std::cout << "New DP Path -> " << generatedTotalPaths.size() << std::endl;
			m_goals.at(m_iCurrentGoal) = generatedTotalPaths.at(0).at(generatedTotalPaths.at(0).size()-1);
			bNewPlan = true;
			m_bMakeNewPlan = false;
		}
	}

	/**
	 * Behavior Generator , State Machine , Decision making Step
	 */

	if(bNewPlan)
		m_State.m_TotalPath = generatedTotalPaths.at(0);

	std::vector<PlannerHNS::DetectedObject> obj_list;

	RosHelpers::ConvertFromAutowareCloudClusterObstaclesToPlannerH(clusters2, obj_list);


	//std::cout << "Obstacles Before Tracking : " << obj_list.size() << std::endl;

	//m_ObstacleTracking.DoOneStep(m_State.state, obj_list);
	//obj_list = m_ObstacleTracking.m_DetectedObjects;

	//std::cout << "Obstacles After Tracking : " << obj_list.size() << std::endl;

	//ConvertFromPlannerObstaclesToAutoware(obj_list, trackedObjects);


	double dt  = UtilityHNS::UtilityH::GetTimeDiffNow(m_PlanningTimer);
	UtilityHNS::UtilityH::GetTickCount(m_PlanningTimer);

	PlannerHNS::WayPoint goal_wp;
	if(m_iCurrentGoal+1 < m_goals.size())
		goal_wp = m_goals.at(m_iCurrentGoal);



//	bool bEmergencyStopFlag = false;
//
//	if(pR->m_State.m_pCurrentBehaviorState->GetCalcParams()->bOut sideControl == 1 && pR->m_CurrentBehavior.state != PlannerHNS::INITIAL_STATE)
//		bEmergencyStop = true;

	//*************************************************/
	m_CurrentBehavior = m_State.DoOneStep(dt, vehState, obj_list, goal_wp.pos, m_Map, bEmergencyStop, bGreenLight, true);
	//*************************************************/

	double d_slow = hypot(m_SlowDownPoint.y - m_State.state.pos.y, m_SlowDownPoint.x - m_State.state.pos.x);
	double d_normal = hypot(m_GoNormalPoint.y - m_State.state.pos.y, m_GoNormalPoint.x - m_State.state.pos.x);

	if(d_slow  < 3)
		m_bSlowDown = true;
	if(d_normal < 3)
		m_bSlowDown = false;

	if(m_bSlowDown)
		m_CurrentBehavior.maxVelocity = 0.3;

	//std::cout << "Obstacles After Tracking : " << obj_list.size() << std::endl;

	RosHelpers::ConvertFromPlannerObstaclesToAutoware(obj_list, trackedObjects, detectedPolygons);

	//std::cout << "Obstacles converted : " << trackedObjects.boxes.size() << std::endl;

	std::cout << m_State.m_pCurrentBehaviorState->GetCalcParams()->ToString(m_CurrentBehavior.state) << ", Speed : " << m_CurrentBehavior.maxVelocity << std::endl;

	behaviorState = RosHelpers::ConvertBehaviorStateFromPlannerHToAutoware(m_CurrentBehavior);

	if(m_CurrentBehavior.bNewPlan)
	{
		//ROS_INFO("Convert New Path To Visualization.");
		RosHelpers::ConvertFromPlannerHToAutowarePathFormat(m_State.m_Path, pathToFollow);
		RosHelpers::ConvertFromPlannerHToAutowareVisualizePathFormat(m_State.m_Path, m_State.m_RollOuts, pathToVisualize);
	}

	return m_CurrentBehavior.bNewPlan;
}

} /* namespace PlannerXNS */
