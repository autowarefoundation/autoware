/*
 *  Copyright (c) 2018, Nagoya University
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

#include "lidar_kf_contour_track_core.h"
#include "op_ros_helpers/op_RosHelpers.h"
#include "op_planner/MappingHelpers.h"
#include "op_planner/PlannerH.h"

namespace ContourTrackerNS
{

ContourTracker::ContourTracker()
{
	m_MapFilterDistance = 0;
	m_dt = 0;
	m_tracking_time = 0;
	m_nOriginalPoints = 0;
	m_FilteringTime = 0 ;
	m_FilteringTime = 0;
	m_FilteringTime = 0;
	m_nContourPoints = 0;
	m_PolyEstimationTime = 0;
	m_MapType = PlannerHNS::MAP_KML_FILE;
	bMap = false;
	bNewCurrentPos = false;
	ReadNodeParams();
	ReadCommonParams();

	m_ObstacleTracking.m_dt = 0.1;
	m_ObstacleTracking.m_bUseCenterOnly = true;
	m_ObstacleTracking.m_Horizon = m_Params.DetectionRadius;
	m_ObstacleTracking.m_bEnableStepByStep = m_Params.bEnableStepByStep;
	m_ObstacleTracking.InitSimpleTracker();

	sub_cloud_clusters 		= nh.subscribe("/cloud_clusters", 1, &ContourTracker::callbackGetCloudClusters, this);
	sub_current_pose 		= nh.subscribe("/current_pose",   1, &ContourTracker::callbackGetCurrentPose, 	this);

	pub_AllTrackedObjects 	= nh.advertise<autoware_msgs::DetectedObjectArray>("tracked_objects", 1);
	pub_DetectedPolygonsRviz = nh.advertise<visualization_msgs::MarkerArray>("detected_polygons", 1);
	pub_TrackedObstaclesRviz = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("op_planner_tracked_boxes", 1);
	pub_TTC_PathRviz		= nh.advertise<visualization_msgs::MarkerArray>("ttc_direct_path", 1);

	//Mapping Section
	sub_lanes = nh.subscribe("/vector_map_info/lane", 1, &ContourTracker::callbackGetVMLanes,  this);
	sub_points = nh.subscribe("/vector_map_info/point", 1, &ContourTracker::callbackGetVMPoints,  this);
	sub_dt_lanes = nh.subscribe("/vector_map_info/dtlane", 1, &ContourTracker::callbackGetVMdtLanes,  this);
	sub_intersect = nh.subscribe("/vector_map_info/cross_road", 1, &ContourTracker::callbackGetVMIntersections,  this);
	sup_area = nh.subscribe("/vector_map_info/area", 1, &ContourTracker::callbackGetVMAreas,  this);
	sub_lines = nh.subscribe("/vector_map_info/line", 1, &ContourTracker::callbackGetVMLines,  this);
	sub_stop_line = nh.subscribe("/vector_map_info/stop_line", 1, &ContourTracker::callbackGetVMStopLines,  this);
	sub_signals = nh.subscribe("/vector_map_info/signal", 1, &ContourTracker::callbackGetVMSignal,  this);
	sub_vectors = nh.subscribe("/vector_map_info/vector", 1, &ContourTracker::callbackGetVMVectors,  this);
	sub_curbs = nh.subscribe("/vector_map_info/curb", 1, &ContourTracker::callbackGetVMCurbs,  this);
	sub_edges = nh.subscribe("/vector_map_info/road_edge", 1, &ContourTracker::callbackGetVMRoadEdges,  this);
	sub_way_areas = nh.subscribe("/vector_map_info/way_area", 1, &ContourTracker::callbackGetVMWayAreas,  this);
	sub_cross_walk = nh.subscribe("/vector_map_info/cross_walk", 1, &ContourTracker::callbackGetVMCrossWalks,  this);
	sub_nodes = nh.subscribe("/vector_map_info/node", 1, &ContourTracker::callbackGetVMNodes,  this);

	m_nDummyObjPerRep = 150;
	m_nDetectedObjRepresentations = 5;
	m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
	m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
	m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
	m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
	m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
	m_DetectedPolygonsActual = m_DetectedPolygonsDummy;
	PlannerHNS::RosHelpers::InitMarkers(m_nDummyObjPerRep, m_DetectedPolygonsDummy.at(0), m_DetectedPolygonsDummy.at(1), m_DetectedPolygonsDummy.at(2), m_DetectedPolygonsDummy.at(3), m_DetectedPolygonsDummy.at(4));

	m_MatchingInfoDummy.push_back(visualization_msgs::MarkerArray());
	m_MatchingInfoActual = m_MatchingInfoDummy;
	PlannerHNS::RosHelpers::InitMatchingMarkers(m_nDummyObjPerRep, m_MatchingInfoDummy.at(0));
}

ContourTracker::~ContourTracker()
{
	if(m_Params.bEnableLogging == true)
	{
		UtilityHNS::DataRW::WriteLogData(UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName+UtilityHNS::DataRW::TrackingFolderName, "contour_tracker",
					"time,dt,num_Tracked_Objects,num_new_objects,num_matched_objects,num_Cluster_Points,num_Contour_Points,t_filtering,t_poly_calc,t_Tracking,t_total",m_LogData);
	}
}

void ContourTracker::ReadNodeParams()
{
	ros::NodeHandle _nh;
	_nh.getParam("/lidar_kf_contour_track/vehicle_width" 			, m_Params.VehicleWidth);
	_nh.getParam("/lidar_kf_contour_track/vehicle_length" 			, m_Params.VehicleLength);
	_nh.getParam("/lidar_kf_contour_track/min_object_size" 			, m_Params.MinObjSize);
	_nh.getParam("/lidar_kf_contour_track/max_object_size" 			, m_Params.MaxObjSize);
	_nh.getParam("/lidar_kf_contour_track/polygon_quarters" 		, m_Params.nQuarters);
	_nh.getParam("/lidar_kf_contour_track/polygon_resolution" 		, m_Params.PolygonRes);
	_nh.getParam("/lidar_kf_contour_track/enableSimulationMode" 	, m_Params.bEnableSimulation);
	_nh.getParam("/lidar_kf_contour_track/enableStepByStepMode" 	, m_Params.bEnableStepByStep);


	_nh.getParam("/lidar_kf_contour_track/max_association_distance" , m_ObstacleTracking.m_MAX_ASSOCIATION_DISTANCE);
	_nh.getParam("/lidar_kf_contour_track/max_association_size_diff" , m_ObstacleTracking.m_MAX_ASSOCIATION_SIZE_DIFF);
	_nh.getParam("/lidar_kf_contour_track/enableLogging" , m_Params.bEnableLogging);
	//_nh.getParam("/lidar_kf_contour_track/enableTTC" , m_Params.bEnableTTC);


	int tracking_type = 0;
	_nh.getParam("/lidar_kf_contour_track/tracking_type" 			, tracking_type);
	if(tracking_type==0)
		m_Params.trackingType = ASSOCIATE_ONLY;
	else if (tracking_type == 1)
		m_Params.trackingType = SIMPLE_TRACKER;
	else if(tracking_type == 2)
		m_Params.trackingType = CONTOUR_TRACKER;

	_nh.getParam("/lidar_kf_contour_track/max_remeber_time" 			, m_ObstacleTracking.m_MaxKeepTime);
	_nh.getParam("/lidar_kf_contour_track/trust_counter" 				, m_ObstacleTracking.m_nMinTrustAppearances);
	_nh.getParam("/lidar_kf_contour_track/vector_map_filter_distance" 	, m_MapFilterDistance);
}

void ContourTracker::ReadCommonParams()
{
	ros::NodeHandle _nh("~");
	if(!_nh.getParam("/op_common_params/horizonDistance" , m_Params.DetectionRadius))
		m_Params.DetectionRadius = 150;

	m_ObstacleTracking.m_CirclesResolution = m_Params.DetectionRadius*0.05;

	if(!_nh.getParam("/op_common_params/enableLaneChange" , m_Params.bEnableLaneChange))
		m_Params.bEnableLaneChange = false;

	int iSource = 0;
	if(iSource == 0)
		m_MapType = PlannerHNS::MAP_AUTOWARE;
	else if (iSource == 1)
		m_MapType = PlannerHNS::MAP_FOLDER;
	else if(iSource == 2)
		m_MapType = PlannerHNS::MAP_KML_FILE;

	_nh.getParam("/op_common_params/mapFileName" , m_MapPath);
}

void ContourTracker::callbackGetCloudClusters(const autoware_msgs::CloudClusterArrayConstPtr &msg)
{
	if(bNewCurrentPos || m_Params.bEnableSimulation)
	{
		ImportCloudClusters(msg, m_OriginalClusters);

		struct timespec  tracking_timer;
		UtilityHNS::UtilityH::GetTickCount(tracking_timer);

		//std::cout << "Filter the detected Obstacles: " << msg->clusters.size() << ", " << m_OriginalClusters.size() << std::endl;

		m_ObstacleTracking.DoOneStep(m_CurrentPos, m_OriginalClusters, m_Params.trackingType);

		m_tracking_time = UtilityHNS::UtilityH::GetTimeDiffNow(tracking_timer);
		m_dt  = UtilityHNS::UtilityH::GetTimeDiffNow(m_loop_timer);
		UtilityHNS::UtilityH::GetTickCount(m_loop_timer);

		LogAndSend();
		VisualizeLocalTracking();

		if(m_Params.bEnableTTC)
		{
			CalculateTTC(m_ObstacleTracking.m_DetectedObjects, m_CurrentPos, m_Map);
		}
	}
}

void ContourTracker::ImportCloudClusters(const autoware_msgs::CloudClusterArrayConstPtr& msg, std::vector<PlannerHNS::DetectedObject>& originalClusters)
{
	originalClusters.clear();
	m_nOriginalPoints = 0;
	m_nContourPoints = 0;
	m_FilteringTime = 0;
	m_PolyEstimationTime = 0;
	struct timespec filter_time, poly_est_time;

	PlannerHNS::DetectedObject obj;
	PlannerHNS::GPSPoint avg_center;
	PolygonGenerator polyGen(m_Params.nQuarters);
	pcl::PointCloud<pcl::PointXYZ> point_cloud;

	if(bMap)
		m_ClosestLanesList = PlannerHNS::MappingHelpers::GetClosestLanesFast(m_CurrentPos, m_Map, m_Params.DetectionRadius);

	//Filter the detected Obstacles:
	//std::cout << "Filter the detected Obstacles: " << std::endl;
	for(unsigned int i=0; i < msg->clusters.size(); i++)
	{
		obj.center.pos.x = msg->clusters.at(i).centroid_point.point.x;
		obj.center.pos.y = msg->clusters.at(i).centroid_point.point.y;
		obj.center.pos.z = msg->clusters.at(i).centroid_point.point.z;
		obj.center.pos.a = msg->clusters.at(i).estimated_angle;

		obj.distance_to_center = hypot(obj.center.pos.y-m_CurrentPos.pos.y, obj.center.pos.x-m_CurrentPos.pos.x);

		obj.actual_yaw = msg->clusters.at(i).estimated_angle;

		obj.w = msg->clusters.at(i).dimensions.x;
		obj.l = msg->clusters.at(i).dimensions.y;
		obj.h = msg->clusters.at(i).dimensions.z;

		UtilityHNS::UtilityH::GetTickCount(filter_time);
		if(!IsCar(obj, m_CurrentPos, m_Map)) continue;
		m_FilteringTime += UtilityHNS::UtilityH::GetTimeDiffNow(filter_time);

		obj.id = msg->clusters.at(i).id;
		obj.originalID = msg->clusters.at(i).id;
		obj.label = msg->clusters.at(i).label;

		if(msg->clusters.at(i).indicator_state == 0)
			obj.indicator_state = PlannerHNS::INDICATOR_LEFT;
		else if(msg->clusters.at(i).indicator_state == 1)
			obj.indicator_state = PlannerHNS::INDICATOR_RIGHT;
		else if(msg->clusters.at(i).indicator_state == 2)
			obj.indicator_state = PlannerHNS::INDICATOR_BOTH;
		else if(msg->clusters.at(i).indicator_state == 3)
			obj.indicator_state = PlannerHNS::INDICATOR_NONE;


		UtilityHNS::UtilityH::GetTickCount(poly_est_time);
		point_cloud.clear();
		pcl::fromROSMsg(msg->clusters.at(i).cloud, point_cloud);

		obj.contour = polyGen.EstimateClusterPolygon(point_cloud ,obj.center.pos,avg_center, m_Params.PolygonRes);

		m_PolyEstimationTime += UtilityHNS::UtilityH::GetTimeDiffNow(poly_est_time);
		m_nOriginalPoints += point_cloud.points.size();
		m_nContourPoints += obj.contour.size();
		originalClusters.push_back(obj);

	}
}

bool ContourTracker::IsCar(const PlannerHNS::DetectedObject& obj, const PlannerHNS::WayPoint& currState, PlannerHNS::RoadNetwork& map)
{

	if(bMap)
	{
		bool bOnLane = false;
	//	std::cout << "Debug Obj: " << obj.id << ", Closest Lane: " << m_ClosestLanesList.size() << std::endl;

		for(unsigned int i =0 ; i < m_ClosestLanesList.size(); i++)
		{

			PlannerHNS::RelativeInfo info;
			PlannerHNS::PlanningHelpers::GetRelativeInfoLimited(m_ClosestLanesList.at(i)->points, obj.center, info);
			PlannerHNS::WayPoint wp = m_ClosestLanesList.at(i)->points.at(info.iFront);

			double direct_d = hypot(wp.pos.y - obj.center.pos.y, wp.pos.x - obj.center.pos.x);

		//	std::cout << "- Distance To Car: " << obj.distance_to_center << ", PerpD: " << info.perp_distance << ", DirectD: " << direct_d << ", bAfter: " << info.bAfter << ", bBefore: " << info.bBefore << std::endl;

			if((info.bAfter || info.bBefore) && direct_d > m_MapFilterDistance*2.0)
				continue;

			if(fabs(info.perp_distance) <= m_MapFilterDistance)
			{
				bOnLane = true;
				break;
			}
		}

		if(bOnLane == false)
			return false;
	}

	if(!m_Params.bEnableSimulation)
	{
		double object_size = hypot(obj.w, obj.l);

		//std::cout << "Filter the detected Obstacles: (" <<  obj.distance_to_center  << ",>" <<  m_Params.DetectionRadius << " | "<< object_size << ",< " <<  m_Params.MinObjSize  << "| " <<  object_size << ", >" <<  m_Params.MaxObjSize << ")"<< std::endl;

		if(obj.distance_to_center > m_Params.DetectionRadius || object_size < m_Params.MinObjSize || object_size > m_Params.MaxObjSize)
			return false;
	}

	if(m_Params.bEnableSimulation)
	{
		PlannerHNS::Mat3 rotationMat(-currState.pos.a);
		PlannerHNS::Mat3 translationMat(-currState.pos.x, -currState.pos.y);

		PlannerHNS::GPSPoint relative_point = translationMat*obj.center.pos;
		relative_point = rotationMat*relative_point;

		double distance_x = fabs(relative_point.x - m_Params.VehicleLength/3.0);
		double distance_y = fabs(relative_point.y);

		if(distance_x  <= m_Params.VehicleLength*0.5 && distance_y <=  m_Params.VehicleWidth*0.5) // don't detect yourself
			return false;
	}

	return true;
}

void ContourTracker::callbackGetCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
  m_CurrentPos = PlannerHNS::WayPoint(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
      tf::getYaw(msg->pose.orientation));

  bNewCurrentPos = true;
}

void ContourTracker::VisualizeLocalTracking()
{
	PlannerHNS::RosHelpers::ConvertTrackedObjectsMarkers(m_CurrentPos, m_ObstacleTracking.m_DetectedObjects,
				m_DetectedPolygonsDummy.at(0),
				m_DetectedPolygonsDummy.at(1),
				m_DetectedPolygonsDummy.at(2),
				m_DetectedPolygonsDummy.at(3),
				m_DetectedPolygonsDummy.at(4),
				m_DetectedPolygonsActual.at(0),
				m_DetectedPolygonsActual.at(1),
				m_DetectedPolygonsActual.at(2),
				m_DetectedPolygonsActual.at(3),
				m_DetectedPolygonsActual.at(4));

	PlannerHNS::RosHelpers::ConvertMatchingMarkers(m_ObstacleTracking.m_MatchList, m_MatchingInfoDummy.at(0), m_MatchingInfoActual.at(0), 0);

	m_DetectedPolygonsAllMarkers.markers.clear();
	m_DetectedPolygonsAllMarkers.markers.insert(m_DetectedPolygonsAllMarkers.markers.end(), m_DetectedPolygonsActual.at(0).markers.begin(), m_DetectedPolygonsActual.at(0).markers.end());
	m_DetectedPolygonsAllMarkers.markers.insert(m_DetectedPolygonsAllMarkers.markers.end(), m_DetectedPolygonsActual.at(1).markers.begin(), m_DetectedPolygonsActual.at(1).markers.end());
	m_DetectedPolygonsAllMarkers.markers.insert(m_DetectedPolygonsAllMarkers.markers.end(), m_DetectedPolygonsActual.at(2).markers.begin(), m_DetectedPolygonsActual.at(2).markers.end());
	m_DetectedPolygonsAllMarkers.markers.insert(m_DetectedPolygonsAllMarkers.markers.end(), m_DetectedPolygonsActual.at(3).markers.begin(), m_DetectedPolygonsActual.at(3).markers.end());
	m_DetectedPolygonsAllMarkers.markers.insert(m_DetectedPolygonsAllMarkers.markers.end(), m_DetectedPolygonsActual.at(4).markers.begin(), m_DetectedPolygonsActual.at(4).markers.end());


	visualization_msgs::MarkerArray all_circles;
	for(unsigned int i = 0; i < m_ObstacleTracking.m_InterestRegions.size(); i++)
	{
		visualization_msgs::Marker circle_mkrs;
		PlannerHNS::RosHelpers::CreateCircleMarker(m_CurrentPos, m_ObstacleTracking.m_InterestRegions.at(i)->radius, i ,circle_mkrs );
		all_circles.markers.push_back(circle_mkrs);
	}

	m_DetectedPolygonsAllMarkers.markers.insert(m_DetectedPolygonsAllMarkers.markers.end(), all_circles.markers.begin(), all_circles.markers.end());

	m_DetectedPolygonsAllMarkers.markers.insert(m_DetectedPolygonsAllMarkers.markers.end(), m_MatchingInfoActual.at(0).markers.begin(), m_MatchingInfoActual.at(0).markers.end());

	pub_DetectedPolygonsRviz.publish(m_DetectedPolygonsAllMarkers);

	jsk_recognition_msgs::BoundingBoxArray boxes_array;
	boxes_array.header.frame_id = "map";
	boxes_array.header.stamp  = ros::Time();

	for(unsigned int i = 0 ; i < m_ObstacleTracking.m_DetectedObjects.size(); i++)
	{
		jsk_recognition_msgs::BoundingBox box;
		box.header.frame_id = "map";
		box.header.stamp = ros::Time().now();
		box.pose.position.x = m_ObstacleTracking.m_DetectedObjects.at(i).center.pos.x;
		box.pose.position.y = m_ObstacleTracking.m_DetectedObjects.at(i).center.pos.y;
		box.pose.position.z = m_ObstacleTracking.m_DetectedObjects.at(i).center.pos.z;

		box.value = 0.9;

		box.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, m_ObstacleTracking.m_DetectedObjects.at(i).center.pos.a);
		box.dimensions.x = m_ObstacleTracking.m_DetectedObjects.at(i).l;
		box.dimensions.y = m_ObstacleTracking.m_DetectedObjects.at(i).w;
		box.dimensions.z = m_ObstacleTracking.m_DetectedObjects.at(i).h;
		boxes_array.boxes.push_back(box);
	}

	pub_TrackedObstaclesRviz.publish(boxes_array);
}

void ContourTracker::LogAndSend()
{
	timespec log_t;
	UtilityHNS::UtilityH::GetTickCount(log_t);
	std::ostringstream dataLine;
	std::ostringstream dataLineToOut;
	dataLine << UtilityHNS::UtilityH::GetLongTime(log_t) <<"," << m_dt << "," <<
			m_ObstacleTracking.m_DetectedObjects.size() << "," <<
			m_OriginalClusters.size() << "," <<
			m_ObstacleTracking.m_DetectedObjects.size() - m_OriginalClusters.size() << "," <<
			m_nOriginalPoints << "," <<
			m_nContourPoints<< "," <<
			m_FilteringTime<< "," <<
			m_PolyEstimationTime<< "," <<
			m_tracking_time<< "," <<
			m_tracking_time+m_FilteringTime+m_PolyEstimationTime<< ",";
	m_LogData.push_back(dataLine.str());

	//For Debugging
//	cout << "dt: " << m_dt << endl;
//	cout << "num_Tracked_Objects: " << m_ObstacleTracking.m_DetectedObjects.size() << endl;
//	cout << "num_new_objects: " << m_OriginalClusters.size() << endl;
//	cout << "num_matched_objects: " << m_ObstacleTracking.m_DetectedObjects.size() - m_OriginalClusters.size() << endl;
//	cout << "num_Cluster_Points: " << m_nOriginalPoints << endl;
//	cout << "num_Contour_Points: " << m_nContourPoints << endl;
//	cout << "t_filtering : " << m_FilteringTime << endl;
//	cout << "t_poly_calc : " << m_PolyEstimationTime << endl;
//	cout << "t_Tracking : " << m_tracking_time << endl;
//	cout << "t_total : " << m_tracking_time+m_FilteringTime+m_PolyEstimationTime << endl;
//	cout << endl;

	m_OutPutResults.objects.clear();
	autoware_msgs::DetectedObject obj;
	for(unsigned int i = 0 ; i <m_ObstacleTracking.m_DetectedObjects.size(); i++)
	{
		PlannerHNS::RosHelpers::ConvertFromOpenPlannerDetectedObjectToAutowareDetectedObject(m_ObstacleTracking.m_DetectedObjects.at(i), m_Params.bEnableSimulation, obj);
		m_OutPutResults.objects.push_back(obj);
	}

	m_OutPutResults.header.frame_id = "map";
	m_OutPutResults.header.stamp  = ros::Time();

	pub_AllTrackedObjects.publish(m_OutPutResults);
}

void ContourTracker::GetFrontTrajectories(std::vector<PlannerHNS::Lane*>& lanes, const PlannerHNS::WayPoint& currState, const double& max_distance, std::vector<std::vector<PlannerHNS::WayPoint> >& trajectories)
{
	double min_d = DBL_MAX;
	PlannerHNS::WayPoint* pClosest = nullptr;
	for(unsigned int i =0 ; i < lanes.size(); i++)
	{
		PlannerHNS::RelativeInfo info;
		PlannerHNS::PlanningHelpers::GetRelativeInfoLimited(lanes.at(i)->points, currState, info);
		PlannerHNS::WayPoint wp = lanes.at(i)->points.at(info.iFront);

		if(!info.bAfter && !info.bBefore && fabs(info.perp_distance) < min_d)
		{
			min_d = fabs(info.perp_distance);
			pClosest = &lanes.at(i)->points.at(info.iBack);
		}
	}

	if(pClosest == nullptr) return;

	PlannerHNS::PlannerH planner;
	std::vector<PlannerHNS::WayPoint*> closest_pts;
	closest_pts.push_back(pClosest);
	planner.PredictTrajectoriesUsingDP(currState, closest_pts, max_distance, trajectories, false, false);

}

void ContourTracker::CalculateTTC(const std::vector<PlannerHNS::DetectedObject>& objs, const PlannerHNS::WayPoint& currState, PlannerHNS::RoadNetwork& map)
{
	std::vector<std::vector<PlannerHNS::WayPoint> > paths;
	GetFrontTrajectories(m_ClosestLanesList, currState, m_Params.DetectionRadius, paths);

	double min_d = DBL_MAX;
	int closest_obj_id = -1;
	int closest_path_id = -1;
	int i_start = -1;
	int i_end = -1;


	for(unsigned int i_obj = 0; i_obj < objs.size(); i_obj++)
	{
		for(unsigned int i =0 ; i < paths.size(); i++)
		{
			PlannerHNS::RelativeInfo obj_info, car_info;
			PlannerHNS::PlanningHelpers::GetRelativeInfoLimited(paths.at(i), objs.at(i_obj).center , obj_info);

			if(!obj_info.bAfter && !obj_info.bBefore && fabs(obj_info.perp_distance) < m_MapFilterDistance)
			{
				PlannerHNS::PlanningHelpers::GetRelativeInfoLimited(paths.at(i), currState , car_info);
				double longitudinalDist = PlannerHNS::PlanningHelpers::GetExactDistanceOnTrajectory(paths.at(i), car_info, obj_info);
				if(longitudinalDist  < min_d)
				{
					min_d = longitudinalDist;
					closest_obj_id = i_obj;
					closest_path_id = i;
					i_start = car_info.iFront;
					i_end = obj_info.iBack;
				}
			}
		}
	}

	std::vector<PlannerHNS::WayPoint> direct_paths;
	if(closest_path_id >= 0 && closest_obj_id >= 0)
	{
		for(unsigned int i=i_start; i<=i_end; i++)
		{
			direct_paths.push_back(paths.at(closest_path_id).at(i));
		}
	}

	//Visualize Direct Path
	m_TTC_Path.markers.clear();
	if(direct_paths.size() == 0)
		direct_paths.push_back(currState);
	PlannerHNS::RosHelpers::TTC_PathRviz(direct_paths, m_TTC_Path);
	pub_TTC_PathRviz.publish(m_TTC_Path);


	//calculate TTC
	if(direct_paths.size() > 2 && closest_obj_id >= 0)
	{
		double dd = min_d;
		double dv = objs.at(closest_obj_id).center.v - currState.v;
		if(fabs(dv) > 0.1)
		{
			double ttc = (dd - 4) / dv;
			cout << "TTC: " << ttc << ", dv: << " << dv <<", dd:" << dd << endl;
		}
		else
			cout << "TTC: Inf" << endl;
	}
}

void ContourTracker::MainLoop()
{
	ros::Rate loop_rate(50);

	while (ros::ok())
	{
		ReadCommonParams();

		if(m_MapFilterDistance > 0 && m_MapType == PlannerHNS::MAP_KML_FILE && !bMap)
		{
			PlannerHNS::MappingHelpers::LoadKML(m_MapPath, m_Map);
			if(m_Map.roadSegments.size() > 0)
			{
				bMap = true;
				std::cout << " ******* Map Is Loaded successfully from the tracker !! " << std::endl;
			}
		}
		else if (m_MapFilterDistance > 0 && m_MapType == PlannerHNS::MAP_FOLDER && !bMap)
		{
			PlannerHNS::MappingHelpers::ConstructRoadNetworkFromDataFiles(m_MapPath, m_Map, true);
			if(m_Map.roadSegments.size() > 0)
			{
				std::cout << " ******* Map Is Loaded successfully from the tracker !! " << std::endl;
				bMap = true;
			}
		}
		else if (m_MapFilterDistance > 0 && m_MapType == PlannerHNS::MAP_AUTOWARE && !bMap)
		{
			std::vector<UtilityHNS::AisanDataConnFileReader::DataConn> conn_data;;

			if(m_MapRaw.GetVersion()==2)
			{
				PlannerHNS::MappingHelpers::ConstructRoadNetworkFromRosMessageV2(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
						m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list,m_MapRaw.pAreas->m_data_list,
						m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list,	m_MapRaw.pSignals->m_data_list,
						m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
						m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,
						m_MapRaw.pLanes, m_MapRaw.pPoints, m_MapRaw.pNodes, m_MapRaw.pLines, PlannerHNS::GPSPoint(), m_Map, true, m_Params.bEnableLaneChange, true);

				if(m_Map.roadSegments.size() > 0)
				{
					bMap = true;
					std::cout << " ******* Map Is Loaded successfully from the tracker !! " << std::endl;
				}
			}
			else if(m_MapRaw.GetVersion()==1)
			{
				PlannerHNS::MappingHelpers::ConstructRoadNetworkFromRosMessage(m_MapRaw.pLanes->m_data_list, m_MapRaw.pPoints->m_data_list,
						m_MapRaw.pCenterLines->m_data_list, m_MapRaw.pIntersections->m_data_list,m_MapRaw.pAreas->m_data_list,
						m_MapRaw.pLines->m_data_list, m_MapRaw.pStopLines->m_data_list,	m_MapRaw.pSignals->m_data_list,
						m_MapRaw.pVectors->m_data_list, m_MapRaw.pCurbs->m_data_list, m_MapRaw.pRoadedges->m_data_list, m_MapRaw.pWayAreas->m_data_list,
						m_MapRaw.pCrossWalks->m_data_list, m_MapRaw.pNodes->m_data_list, conn_data,  PlannerHNS::GPSPoint(), m_Map, true, m_Params.bEnableLaneChange, true);

				if(m_Map.roadSegments.size() > 0)
				{
					bMap = true;
					std::cout << " ******* Map Is Loaded successfully from the tracker !! " << std::endl;
				}
			}
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

}

//Mapping Section

void ContourTracker::callbackGetVMLanes(const vector_map_msgs::LaneArray& msg)
{
	std::cout << "Received Lanes" << endl;
	if(m_MapRaw.pLanes == nullptr)
		m_MapRaw.pLanes = new UtilityHNS::AisanLanesFileReader(msg);
}

void ContourTracker::callbackGetVMPoints(const vector_map_msgs::PointArray& msg)
{
	std::cout << "Received Points" << endl;
	if(m_MapRaw.pPoints  == nullptr)
		m_MapRaw.pPoints = new UtilityHNS::AisanPointsFileReader(msg);
}

void ContourTracker::callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg)
{
	std::cout << "Received dtLanes" << endl;
	if(m_MapRaw.pCenterLines == nullptr)
		m_MapRaw.pCenterLines = new UtilityHNS::AisanCenterLinesFileReader(msg);
}

void ContourTracker::callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg)
{
	std::cout << "Received CrossRoads" << endl;
	if(m_MapRaw.pIntersections == nullptr)
		m_MapRaw.pIntersections = new UtilityHNS::AisanIntersectionFileReader(msg);
}

void ContourTracker::callbackGetVMAreas(const vector_map_msgs::AreaArray& msg)
{
	std::cout << "Received Areas" << endl;
	if(m_MapRaw.pAreas == nullptr)
		m_MapRaw.pAreas = new UtilityHNS::AisanAreasFileReader(msg);
}

void ContourTracker::callbackGetVMLines(const vector_map_msgs::LineArray& msg)
{
	std::cout << "Received Lines" << endl;
	if(m_MapRaw.pLines == nullptr)
		m_MapRaw.pLines = new UtilityHNS::AisanLinesFileReader(msg);
}

void ContourTracker::callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg)
{
	std::cout << "Received StopLines" << endl;
	if(m_MapRaw.pStopLines == nullptr)
		m_MapRaw.pStopLines = new UtilityHNS::AisanStopLineFileReader(msg);
}

void ContourTracker::callbackGetVMSignal(const vector_map_msgs::SignalArray& msg)
{
	std::cout << "Received Signals" << endl;
	if(m_MapRaw.pSignals  == nullptr)
		m_MapRaw.pSignals = new UtilityHNS::AisanSignalFileReader(msg);
}

void ContourTracker::callbackGetVMVectors(const vector_map_msgs::VectorArray& msg)
{
	std::cout << "Received Vectors" << endl;
	if(m_MapRaw.pVectors  == nullptr)
		m_MapRaw.pVectors = new UtilityHNS::AisanVectorFileReader(msg);
}

void ContourTracker::callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg)
{
	std::cout << "Received Curbs" << endl;
	if(m_MapRaw.pCurbs == nullptr)
		m_MapRaw.pCurbs = new UtilityHNS::AisanCurbFileReader(msg);
}

void ContourTracker::callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg)
{
	std::cout << "Received Edges" << endl;
	if(m_MapRaw.pRoadedges  == nullptr)
		m_MapRaw.pRoadedges = new UtilityHNS::AisanRoadEdgeFileReader(msg);
}

void ContourTracker::callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg)
{
	std::cout << "Received Wayareas" << endl;
	if(m_MapRaw.pWayAreas  == nullptr)
		m_MapRaw.pWayAreas = new UtilityHNS::AisanWayareaFileReader(msg);
}

void ContourTracker::callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg)
{
	std::cout << "Received CrossWalks" << endl;
	if(m_MapRaw.pCrossWalks == nullptr)
		m_MapRaw.pCrossWalks = new UtilityHNS::AisanCrossWalkFileReader(msg);
}

void ContourTracker::callbackGetVMNodes(const vector_map_msgs::NodeArray& msg)
{
	std::cout << "Received Nodes" << endl;
	if(m_MapRaw.pNodes == nullptr)
		m_MapRaw.pNodes = new UtilityHNS::AisanNodesFileReader(msg);
}

}
