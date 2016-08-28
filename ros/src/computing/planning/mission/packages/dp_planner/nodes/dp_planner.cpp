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
#include "dp_planner_core.h"
#include <iostream>

using namespace std;

void printHelp()
{
	cout << "-planner" << endl;
	cout << "      'DP'" << endl;
	cout << "      'Free'" << endl;
	cout << "-system" << endl;
	cout << "      'autoware'" << endl;
	cout << "      'segway'" << endl;
	cout << "-map" << endl;
	cout << "      'kml'" << endl;
	cout << "      map path" << endl;
	cout << "-h" << endl;
	cout << "      Help" << endl;
}

int main(int argc, char **argv)
{
	std::string plannerType = "DP";
	bool bAutoware = true;
	std::string kmlMapPath = "";
	bool bKML_Map = false;

//	vector<string> cmdData;
//		for ( unsigned int i = 1 ;  i < (unsigned int)argc; i++)
//		{
//			cmdData.push_back(std::string(argv[i]));
//		}
//
//		for(unsigned int i=0; i < cmdData.size(); i++)
//		{
//			if(cmdData.at(i).compare("-planner") == 0)
//			{
//				i++;
//				if(i < cmdData.size())
//				{
//					if(cmdData.at(i).compare("DP") == 0 || cmdData.at(i).compare("Free")==0)
//						plannerType = cmdData.at(i);
//					else
//					{
//						printHelp();
//						return 0;
//					}
//				}
//				else
//				{
//					cout << "Missing parameters ! " << endl;
//					printHelp();
//					return 0;
//				}
//			}
//			else if(cmdData.at(i).compare("-system") == 0)
//			{
//				i++;
//				if(i < cmdData.size())
//				{
//					if(cmdData.at(i).compare("autoware") == 0)
//						bAutoware = true;
//					else if (cmdData.at(i).compare("segway")==0)
//						bAutoware  = false;
//					else
//					{
//						printHelp();
//						return 0;
//					}
//				}
//				else
//				{
//					cout << "Missing parameters ! " << endl;
//					printHelp();
//					return 0;
//				}
//
//			}
//			else if(cmdData.at(i).compare("-map") == 0)
//			{
//				i++;
//				if(i < cmdData.size())
//				{
//					if(cmdData.at(i).compare("kml") == 0)
//					{
//						i++;
//						if(i < cmdData.size())
//						{
//							bKML_Map = true;
//							kmlMapPath = cmdData.at(i);
//						}
//						else
//						{
//							cout << "Map Path is needed ! " << endl;
//							printHelp();
//							return 0;
//						}
//					}
//					else
//					{
//						bKML_Map = false;
//						kmlMapPath = cmdData.at(i);
//					}
//				}
//				else
//				{
//					cout << "Map Path is needed ! " << endl;
//					printHelp();
//					return 0;
//				}
//			}
//			else if(cmdData.at(i).compare("-h") == 0)
//			{
//				printHelp();
//				return 0;
//			}
//		}

	ros::init(argc, argv, "dp_planner");

	ros::NodeHandle nh;
	nh.getParam("/dp_planner/planner", plannerType);
	string sysStr;
	nh.getParam("/dp_planner/system",sysStr );
	if(sysStr.compare("autoware")!=0)
		bAutoware = false;

	string kmlStr;
	nh.getParam("/dp_planner/map", kmlStr);
	if(kmlStr.compare("kml")==0)
		bKML_Map = true;

	nh.getParam("/dp_planner/mapDirectory", kmlMapPath);

	cout << "Initialize Planning System .. " << plannerType << ", " << sysStr << "," << kmlStr << ", " << kmlMapPath << endl;

	PlannerXNS::PlannerX dp_planner(plannerType,bAutoware, bKML_Map, kmlMapPath);
	dp_planner.PlannerMainLoop();
	return 0;
}
