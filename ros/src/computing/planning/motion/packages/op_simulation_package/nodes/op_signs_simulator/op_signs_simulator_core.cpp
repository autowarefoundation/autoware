/*
 *  Copyright (c) 2017, Nagoya University
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
#include "op_signs_simulator_core.h"


#include "op_utility/UtilityH.h"
#include "math.h"
#include "op_planner/MatrixOperations.h"
#include "op_ros_helpers/op_RosHelpers.h"


namespace SignsSimulatorNS
{

OpenPlannerSimulatorSigns::OpenPlannerSimulatorSigns()
{
	bMap = false;

	std::string first_str, second_str;
	ros::NodeHandle _nh("~");
	_nh.getParam("first_signs_list_ids" , first_str);
	_nh.getParam("second_signs_list_ids" , second_str);

	_nh.getParam("first_green_time" , m_Params.firstGreenTime);
	_nh.getParam("second_green_time" , m_Params.secondGreenTime);
	_nh.getParam("first_yellow_time" , m_Params.firstyellowTime);
	_nh.getParam("second_yellow_time" , m_Params.secondyellowTime);

	int iSource = 0;
	_nh.getParam("mapSource" , iSource);
	if(iSource == 0)
		m_MapType = PlannerHNS::MAP_FOLDER;
	else if(iSource == 1)
		m_MapType = PlannerHNS::MAP_KML_FILE;

	_nh.getParam("mapFileName" , m_MapPath);

	std::cout << first_str  << " | " <<   second_str << std::endl;
	m_Params.SetCommandParams(first_str, second_str);

	for(unsigned int i = 0; i < m_Params.firstSignsIds.size(); i++)
	{
		autoware_msgs::ExtractedPosition s;
		s.signalId = m_Params.firstSignsIds.at(i);
		s.type = 1;
		m_FirstSignals.Signals.push_back(s);
	}

	for(unsigned int i=0; i < m_Params.secondSignsIds.size(); i++)
	{
		autoware_msgs::ExtractedPosition s;
		s.signalId = m_Params.secondSignsIds.at(i);
		s.type = 0;
		m_SecondSignals.Signals.push_back(s);
	}


	pub_TrafficLightsRviz = nh.advertise<visualization_msgs::MarkerArray>("op_traffic_lights_rviz", 1);

	pub_trafficLights 	= nh.advertise<autoware_msgs::Signals>("roi_signal",1);

	UtilityHNS::UtilityH::GetTickCount(m_Timer);
	m_CurrLightState = PlannerHNS::GREEN_LIGHT;

	std::cout << "OpenPlannerSimulatorSigns initialized successfully " << std::endl;

}

OpenPlannerSimulatorSigns::~OpenPlannerSimulatorSigns()
{
}

void OpenPlannerSimulatorSigns::VisualizeTrafficLight(autoware_msgs::Signals& _signals)
{
	std::vector<PlannerHNS::TrafficLight> simulatedLights;
	for(unsigned int i = 0 ; i < _signals.Signals.size() ; i++)
	{
		PlannerHNS::TrafficLight tl;
		tl.id = _signals.Signals.at(i).signalId;

		for(unsigned int k = 0; k < m_Map.trafficLights.size(); k++)
		{
			if(m_Map.trafficLights.at(k).id == tl.id)
			{
				tl.pos = m_Map.trafficLights.at(k).pos;
				break;
			}
		}

		if(_signals.Signals.at(i).type == 1)
		{
			tl.lightState = PlannerHNS::GREEN_LIGHT;
		}
		else
		{
			tl.lightState = PlannerHNS::RED_LIGHT;
		}

		simulatedLights.push_back(tl);
	}

	//visualize traffic light
	visualization_msgs::MarkerArray lights;
	PlannerHNS::RosHelpers::GetTrafficLightForVisualization(simulatedLights, lights);
	pub_TrafficLightsRviz.publish(lights);
}

void OpenPlannerSimulatorSigns::MainLoop()
{

	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		ros::spinOnce();

		if(m_MapType == PlannerHNS::MAP_KML_FILE && !bMap)
		{
			bMap = true;
			PlannerHNS::MappingHelpers::LoadKML(m_MapPath, m_Map);
		}
		else if (m_MapType == PlannerHNS::MAP_FOLDER && !bMap)
		{
			bMap = true;
			PlannerHNS::MappingHelpers::ConstructRoadNetworkFromDataFiles(m_MapPath, m_Map, true);
		}

		if(m_CurrLightState == PlannerHNS::GREEN_LIGHT)
		{
			//std::cout << "Greeeeeen" << std::endl;
			if(UtilityHNS::UtilityH::GetTimeDiffNow(m_Timer) < m_Params.firstGreenTime)
			{
				for(unsigned int i =0; i<m_FirstSignals.Signals.size(); i++)
					m_FirstSignals.Signals.at(i).type = 1;

				for(unsigned int i =0; i<m_SecondSignals.Signals.size(); i++)
					m_SecondSignals.Signals.at(i).type = 0;
			}
			else
			{
				m_CurrLightState = PlannerHNS::YELLOW_LIGHT;
				UtilityHNS::UtilityH::GetTickCount(m_Timer);
			}
		}
		else if(m_CurrLightState == PlannerHNS::YELLOW_LIGHT)
		{
			//std::cout << "Yelowwwwww" << std::endl;
			if(UtilityHNS::UtilityH::GetTimeDiffNow(m_Timer) < m_Params.firstyellowTime)
			{
				for(unsigned int i =0; i< m_FirstSignals.Signals.size(); i++)
					m_FirstSignals.Signals.at(i).type = 0;

				for(unsigned int i =0; i<m_SecondSignals.Signals.size(); i++)
					m_SecondSignals.Signals.at(i).type = 0;
			}
			else
			{
				m_CurrLightState = PlannerHNS::RED_LIGHT;
				UtilityHNS::UtilityH::GetTickCount(m_Timer);
			}
		}
		else if(m_CurrLightState == PlannerHNS::RED_LIGHT)
		{
			//std::cout << "Reeeeeeed" << std::endl;
			if(UtilityHNS::UtilityH::GetTimeDiffNow(m_Timer) < m_Params.secondGreenTime)
			{
				for(unsigned int i =0; i<m_FirstSignals.Signals.size(); i++)
					m_FirstSignals.Signals.at(i).type = 0;

				for(unsigned int i =0; i<m_SecondSignals.Signals.size(); i++)
					m_SecondSignals.Signals.at(i).type = 1;
			}
			else
			{
				m_CurrLightState = PlannerHNS::FLASH_YELLOW;
				UtilityHNS::UtilityH::GetTickCount(m_Timer);
			}
		}
		else if(m_CurrLightState == PlannerHNS::FLASH_YELLOW)
		{
			//std::cout << "Yelowwwwww" << std::endl;
			if(UtilityHNS::UtilityH::GetTimeDiffNow(m_Timer) < m_Params.secondyellowTime)
			{
				for(unsigned int i =0; i<m_FirstSignals.Signals.size(); i++)
					m_FirstSignals.Signals.at(i).type = 0;

				for(unsigned int i =0; i<m_SecondSignals.Signals.size(); i++)
					m_SecondSignals.Signals.at(i).type = 0;
			}
			else
			{
				m_CurrLightState = PlannerHNS::GREEN_LIGHT;
				UtilityHNS::UtilityH::GetTickCount(m_Timer);
			}
		}

		autoware_msgs::Signals all_signals;
		all_signals.Signals.insert(all_signals.Signals.end(), m_FirstSignals.Signals.begin(), m_FirstSignals.Signals.end());
		all_signals.Signals.insert(all_signals.Signals.end(), m_SecondSignals.Signals.begin(), m_SecondSignals.Signals.end());

		//std::cout << "Number of Signals before send: " << all_signals.Signals.size()  << std::endl;
		pub_trafficLights.publish(all_signals);

		if(bMap)
			VisualizeTrafficLight(all_signals);

		loop_rate.sleep();
	}
}

}
