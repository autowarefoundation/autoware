/*
// *  Copyright (c) 2018, Nagoya University
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

#ifndef OP_SIGNS_SIMULATOR
#define OP_SIGNS_SIMULATOR

#include <ros/ros.h>
//#include <runtime_manager/traffic_light.h>

#include "vector_map_msgs/PointArray.h"
#include "vector_map_msgs/LaneArray.h"
#include "vector_map_msgs/NodeArray.h"
#include "vector_map_msgs/StopLineArray.h"
#include "vector_map_msgs/DTLaneArray.h"
#include "vector_map_msgs/LineArray.h"
#include "vector_map_msgs/AreaArray.h"
#include "vector_map_msgs/SignalArray.h"
#include "vector_map_msgs/StopLine.h"
#include "vector_map_msgs/VectorArray.h"

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarkerPose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <autoware_msgs/Signals.h>
#include <autoware_msgs/ExtractedPosition.h>
#include <geometry_msgs/PoseArray.h>
#include <op_planner/RoadNetwork.h>
#include <op_planner/MappingHelpers.h>
#include "op_planner/PlannerCommonDef.h"
#include "op_utility/DataRW.h"


namespace SignsSimulatorNS
{

class SignsCommandParams
{
public:
	std::vector<int>  	firstSignsIds;
	std::vector<int>	secondSignsIds;
	double firstGreenTime;
	double secondGreenTime;
	double firstyellowTime;
	double secondyellowTime;

	void SetCommandParams(const std::string& firstSet, const std::string& secondSet)
	{
		std::vector<std::string> s_list = PlannerHNS::MappingHelpers::SplitString(firstSet, ",");
		for(unsigned int i=0; i < s_list.size(); i++)
		{
			if(s_list.at(i).size()>0)
			{
				firstSignsIds.push_back(strtol(s_list.at(i).c_str(), NULL, 10));
			}
		}

		s_list = PlannerHNS::MappingHelpers::SplitString(secondSet, ",");
		for(unsigned int i=0; i < s_list.size(); i++)
			if(s_list.at(i).size()>0)
			{

				secondSignsIds.push_back(strtol(s_list.at(i).c_str(), NULL, 10));
			}
	}

	SignsCommandParams()
	{
		firstGreenTime = 10;
		secondGreenTime = 10;
		firstyellowTime = 1;
		secondyellowTime = 1;
	}
};

class OpenPlannerSimulatorSigns
{
public:
	autoware_msgs::Signals m_FirstSignals;
	autoware_msgs::Signals m_SecondSignals;

protected:
	ros::NodeHandle nh;
	timespec m_Timer;
	PlannerHNS::TrafficLightState m_CurrLightState;
	SignsCommandParams m_Params;

	PlannerHNS::MAP_SOURCE_TYPE m_MapType;
	std::string m_MapPath;
	PlannerHNS::RoadNetwork m_Map;
	bool bMap;



	ros::Publisher pub_TrafficLightsRviz;
	ros::Publisher pub_trafficLights;

	void VisualizeTrafficLight(autoware_msgs::Signals& _signals);

public:
	OpenPlannerSimulatorSigns();
	virtual ~OpenPlannerSimulatorSigns();
	void MainLoop();

	//Mapping Section

	UtilityHNS::MapRaw m_MapRaw;

	ros::Subscriber sub_lanes;
	ros::Subscriber sub_points;
	ros::Subscriber sub_dt_lanes;
	ros::Subscriber sub_intersect;
	ros::Subscriber sup_area;
	ros::Subscriber sub_lines;
	ros::Subscriber sub_stop_line;
	ros::Subscriber sub_signals;
	ros::Subscriber sub_vectors;
	ros::Subscriber sub_curbs;
	ros::Subscriber sub_edges;
	ros::Subscriber sub_way_areas;
	ros::Subscriber sub_cross_walk;
	ros::Subscriber sub_nodes;


	void callbackGetVMLanes(const vector_map_msgs::LaneArray& msg);
	void callbackGetVMPoints(const vector_map_msgs::PointArray& msg);
	void callbackGetVMdtLanes(const vector_map_msgs::DTLaneArray& msg);
	void callbackGetVMIntersections(const vector_map_msgs::CrossRoadArray& msg);
	void callbackGetVMAreas(const vector_map_msgs::AreaArray& msg);
	void callbackGetVMLines(const vector_map_msgs::LineArray& msg);
	void callbackGetVMStopLines(const vector_map_msgs::StopLineArray& msg);
	void callbackGetVMSignal(const vector_map_msgs::SignalArray& msg);
	void callbackGetVMVectors(const vector_map_msgs::VectorArray& msg);
	void callbackGetVMCurbs(const vector_map_msgs::CurbArray& msg);
	void callbackGetVMRoadEdges(const vector_map_msgs::RoadEdgeArray& msg);
	void callbackGetVMWayAreas(const vector_map_msgs::WayAreaArray& msg);
	void callbackGetVMCrossWalks(const vector_map_msgs::CrossWalkArray& msg);
	void callbackGetVMNodes(const vector_map_msgs::NodeArray& msg);
};

}

#endif  // OP_SIGNS_SIMULATOR
