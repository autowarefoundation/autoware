/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  XXX: Licensing has not been cleared yet.
*/

#include <iostream>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "System.h"


using namespace std;
using namespace ORB_SLAM2;


class ORB_Mapper
{
public:
	ORB_Mapper ()
	{ }

	~ORB_Mapper ()
	{ }


	void externalLocalizerGrab ()
	{ }

private:
    // External localization
    tf::TransformListener *extListener;
    tf::StampedTransform extPose;
    string extFrame1, extFrame2;
    bool doStop;
    bool doDebayer;
    bool offlineMode;
    int offsetKeyframe;
};


int main (int argc, char *argv[])
{
//	const ORB_SLAM2::System::operationMode opMode = ORB_SLAM2::System::MAPPING;
	const string mapPath = (argc==3) ? argv[2] : string();
	const string orbVocabFile (ORB_SLAM_VOCABULARY);

	ros::init(argc, argv, "orb_mapping");
	ros::start();
	ros::NodeHandle nodeHandler;

    ORB_SLAM2::System SLAM(orbVocabFile,
    	argv[1],
		ORB_SLAM2::System::MONOCULAR,
		true,
		mapPath,
    	System::MAPPING);




    return 0;
}
