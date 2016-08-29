/*
 * bag_mapper.cc
 *
 *  Created on: Jun 3, 2016
 *      Author: sujiwo
 */

#include <cstdlib>
#include <signal.h>
#include <iostream>
#include <string>
#include "ImageGrabber.h"
#include "utils.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <boost/foreach.hpp>


using namespace std;


#define foreach BOOST_FOREACH


bool doStop = false;


void stopHandler (int signum)
{
	cout << "Stopping..." << endl;
	doStop = true;
}



int main (int argc, char *argv[])
{
	if (argc < 4) {
		cerr << "\nUsage: bag_mapper path_to_settings image_bag ground_truth [second_skip] [path_to_map]\n" << endl;
		exit(1);
	}

	// Set signal handler
	signal (SIGINT, stopHandler);
	signal (SIGTERM, stopHandler);

	string mapPath = (argc>=6 ? argv[5] : string());
	double secondToSkip = argc>=5 ? atof(argv[4]) : 0.0;
	string bagPath (argv[2]);
	string groundTruth (argv[3]);
	string orbVocabFile (ORB_SLAM_VOCABULARY);

	// Necessary Resource
	ORB_SLAM2::System SLAM (orbVocabFile,
		argv[1],
		ORB_SLAM2::System::MONOCULAR,
		true,
		mapPath,
		ORB_SLAM2::System::MAPPING);
	ImageGrabber GrabBag (&SLAM, NULL, true);
	TfTimeTree TfSource (groundTruth, SLAM.fsSettings["ExternalLocalization.frame1"], SLAM.fsSettings["ExternalLocalization.frame2"]);

	// Build ROSBag Query
	rosbag::Bag bagSrc;
	bagSrc.open (bagPath, rosbag::bagmode::Read);
	const string imageTopic (SLAM.fsSettings["Camera.topic"]);
	rosbag::View viewx(bagSrc, rosbag::TopicQuery(imageTopic));
	ros::Time startTime = viewx.getBeginTime();
	startTime.sec += secondToSkip;
	rosbag::View view(bagSrc, rosbag::TopicQuery(imageTopic), startTime);

	const double mappingStartTime = view.getBeginTime().toSec(),
		mappingStopTime = view.getEndTime().toSec();
	cout << "Starting at " << mappingStartTime << ", ending at " << mappingStopTime << endl;

	bool tracked = false;
	uint32_t frameCounter = 0;
	SLAM.getTracker()->setFps(10);

	foreach (rosbag::MessageInstance const msg, view) {

		frameCounter ++;
		if (frameCounter % 2 == 1)
			continue;

		double timestamp;

		if (doStop==true)
			break;

		// Put mutual exclusion from here ...
		// We intend to pause LoopClosing and LocalMapper
		// Until frame processing is done.
		// Also, we pause frame input until LoopClosing and LocalMapper finish their rounds
		std::lock (SLAM.getLocalMapper()->localMappingRunMutex, SLAM.getLoopCloser()->loopCloserRunMutex);

		if ((int)SLAM.fsSettings["Camera.compressed"]==1) {
			sensor_msgs::CompressedImageConstPtr imgc = msg.instantiate<sensor_msgs::CompressedImage>();
			// XXX: Not finished!
		}

		else {
			sensor_msgs::ImageConstPtr img = msg.instantiate<sensor_msgs::Image>();
			timestamp = img->header.stamp.toSec();
			GrabBag.GrabImage(img);
		}
		// to here
		SLAM.getLocalMapper()->localMappingRunMutex.unlock();
		SLAM.getLoopCloser()->loopCloserRunMutex.unlock();

		// put ground truth
		tf::Transform currentNdtPose;
		Frame &cframe = SLAM.getTracker()->mCurrentFrame;

		if (cframe.mTcw.empty()==false)
			tracked = true;
		else {
			if (tracked==true) {
//				cout << "Frame Counter: " << frameCounter << endl;
				cout << "Stopping due to lost after " << frameCounter << " frames, timestamp: " << timestamp << endl;
				break;
			}
		}

		try {
			currentNdtPose = TfSource.search(timestamp);
			tfToCV(currentNdtPose, cframe.mpReferenceKF->extPosition, cframe.mpReferenceKF->extOrientation);
		} catch (const std::out_of_range &e) {
			currentNdtPose.setOrigin (tf::Vector3());
			currentNdtPose.setRotation (tf::Quaternion());
		}

		frameCounter ++;
	}

	SLAM.Shutdown();
}
