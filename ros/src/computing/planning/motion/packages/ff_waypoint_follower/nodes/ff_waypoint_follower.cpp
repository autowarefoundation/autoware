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
#include "ff_waypoint_follower_core.h"

void printHelp()
{
//	cout << "-mode" << endl;
//	cout << "      torque" << endl;
//	cout << "      angle" << endl;
	cout << "-signal" << endl;
	cout << "      vehicle" << endl;
	cout << "      autoware" << endl;
	cout << "      segway" << endl;
	cout << "      simulation" << endl;
	cout << "-map" << endl;
	cout << "      record distance" << endl;
	cout << "-h" << endl;
}


int main(int argc, char **argv)
{
	FFSteerControlNS::ControlCommandParams params;

//	vector<string> cmdData;
//	for ( unsigned int i = 1 ;  i < (unsigned int)argc; i++)
//	{
//		cmdData.push_back(std::string(argv[i]));
//	}
//
//	for(unsigned int i=0; i < cmdData.size(); i++)
//	{
//		if(cmdData.at(i).compare("-mode") == 0)
//		{
//			params.bMode = true;
//			i++;
//			if(i < cmdData.size())
//			{
//				if(cmdData.at(i).compare("torque") == 0)
//				{
//					params.bTorqueMode = true;
//				}
//				else if(cmdData.at(i).compare("angle") == 0)
//				{
//					params.bTorqueMode = false;
//				}
//			}
//
//		}
//		else if(cmdData.at(i).compare("-signal") == 0)
//		{
//			params.bSignal = true;
//			i++;
//			if(i < cmdData.size())
//			{
//				if(cmdData.at(i).compare("vehicle") == 0)
//				{
//					params.statusSource = FFSteerControlNS::CONTROL_BOX_STATUS;
//				}
//				else if(cmdData.at(i).compare("autoware") == 0)
//				{
//					params.statusSource = FFSteerControlNS::AUTOWARE_STATUS;
//				}
//				else if(cmdData.at(i).compare("segway") == 0)
//				{
//					params.statusSource = FFSteerControlNS::SEGWAY_STATUS;
//					cout << "segway Option" << endl;
//				}
//				else if(cmdData.at(i).compare("simulation") == 0)
//				{
//					params.statusSource = FFSteerControlNS::SIMULATION_STATUS;
//				}
//			}
//			else
//			{
//				printHelp();
//				return 0;
//			}
//		}
//		else if(cmdData.at(i).compare("-map") == 0)
//		{
//			params.iMapping = 1;
//
//			i++;
//			if(i < cmdData.size())
//			{
//				params.recordDistance = atof(cmdData.at(i).c_str());
//			}
//		}
//		else if(cmdData.at(i).compare("-h") == 0)
//		{
//			printHelp();
//			return 0;
//		}
//
//	}

	ros::init(argc, argv, "ff_waypoint_follower");

	ros::NodeHandle nh;
	string sysStr;
	nh.getParam("/ff_waypoint_follower/system",sysStr );
	if(sysStr.compare("autoware")!=0)
		params.bAutoware = false;

	string signalStr;
	nh.getParam("/ff_waypoint_follower/signal", signalStr );
	if(signalStr.compare("simulation") == 0)
		params.statusSource = FFSteerControlNS::SIMULATION_STATUS;
	else if(signalStr.compare("vehicle") == 0)
		params.statusSource = FFSteerControlNS::CONTROL_BOX_STATUS;
	else if(signalStr.compare("autoware") == 0)
		params.statusSource = FFSteerControlNS::AUTOWARE_STATUS;
	else if(signalStr.compare("segway") == 0)
		params.statusSource = FFSteerControlNS::SEGWAY_STATUS;

	string steerModeStr;
	nh.getParam("/ff_waypoint_follower/steerMode", steerModeStr );
	if(steerModeStr.compare("angle") == 0)
		params.bAngleMode = true;
	else if(steerModeStr.compare("torque") == 0)
		params.bAngleMode = false;

	string driveModeStr;
	nh.getParam("/ff_waypoint_follower/driveMode", driveModeStr );
	if(driveModeStr.compare("velocity") == 0)
		params.bVelocityMode = true;
	else if(driveModeStr.compare("stroke") == 0)
		params.bVelocityMode = false;

	string mapRecStr;
	nh.getParam("/ff_waypoint_follower/mapRecorder", params.iMapping );
//	if(mapRecStr.compare("0") == 0)
//		params.iMapping = 0;
//	else if(mapRecStr.compare("1") == 0)
//		params.iMapping = 1;

	string distanceStr;
	nh.getParam("/ff_waypoint_follower/mapDistance", params.recordDistance );
	//params.recordDistance = atof(distanceStr.c_str());

	string densityStr;
	nh.getParam("/ff_waypoint_follower/mapDensity", params.recordDensity );
	//params.recordDensity = atof(densityStr.c_str());

	cout << "Initialize Planning System .. " << sysStr << ", "
			<< signalStr << "," << steerModeStr << ", "
			<< driveModeStr << ", " << params.iMapping << ", "
			<< params.recordDistance << ", " << params.recordDensity << endl;


	FFSteerControlNS::FFSteerControl controller_x(params);
	controller_x.PlannerMainLoop();
	return 0;
}
