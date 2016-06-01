/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <libgen.h>
#include <iostream>
#include <string>
#include "ImageGrabber.h"



using namespace std;
using ORB_SLAM2::Frame;
namespace enc = sensor_msgs::image_encodings;





int main(int argc, char **argv)
{
	string myname (basename(argv[0]));
	ORB_SLAM2::System::operationMode opMode;
	// Which name was we called by ?
	if (myname.compare(myname.size()-6, 6, "mapper")==0) {
		opMode = ORB_SLAM2::System::MAPPING;
		cout << "Mode: Mapper" << endl;
	}
	else {
		opMode = ORB_SLAM2::System::LOCALIZATION;
		cout << "Mode: Localizer" << endl;
	}

    ros::init(argc, argv, "orb_slam_mapper");
    ros::start();

    if(argc < 2)
    {
        cerr << endl << "Usage: " << myname << " path_to_settings [path_to_map]\n" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    string mapPath = (argc==3) ? argv[2] : string();

    // This macro should be set by Cmake
    string orbVocabFile (ORB_SLAM_VOCABULARY);
    ORB_SLAM2::System SLAM(orbVocabFile,argv[1],ORB_SLAM2::System::MONOCULAR,true, mapPath,
    	opMode);

    ImageGrabber igb(&SLAM);
    std::thread* extLocalizerThd = new std::thread (&ImageGrabber::externalLocalizerGrab, &igb);

    ros::NodeHandle nodeHandler;
    image_transport::TransportHints th;
    if ((int)SLAM.fsSettings["Camera.compressed"]==0) {
    	th = image_transport::TransportHints ("raw");
    }
    else if ((int)SLAM.fsSettings["Camera.compressed"]==1) {
    	th = image_transport::TransportHints ("compressed");
    }
    image_transport::ImageTransport it (nodeHandler);
    image_transport::Subscriber sub = it.subscribe ((string)SLAM.fsSettings["Camera.topic"], 1, &ImageGrabber::GrabImage, &igb, th);

    cout << endl << "Mono Camera topic: " << (string)SLAM.fsSettings["Camera.topic"] << endl;
    cout << "Compressed images? " << ((int)SLAM.fsSettings["Camera.compressed"]==1 ? "True" : "False") << endl;

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
    igb.doStop = true;
    extLocalizerThd->join();

    // Save camera trajectory
//    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

