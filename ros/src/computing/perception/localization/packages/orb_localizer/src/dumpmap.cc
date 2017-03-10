/*
 * dumpmap.cc
 *
 *  Created on: Nov 27, 2015
 *      Author: sujiwo
 */

#include <string>
#include <iostream>
#include "Map.h"
#include "KeyFrame.h"
#include "Converter.h"


using namespace std;
using ORB_SLAM2::Map;
using ORB_SLAM2::KeyFrame;


int main (int argc, char **argv)
{
	string mapfile (argv[1]);
	ORB_SLAM2::Map World;
	World.loadFromDisk (mapfile);

	vector<KeyFrame*> allKeyFrames = World.kfListSorted;

	cout << std::fixed << setprecision(7) << "First keyframe time: " << allKeyFrames[0]->mTimeStamp << endl;
	cout << "Last keyframe time: " << allKeyFrames.back()->mTimeStamp << endl;

	for (vector<KeyFrame*>::const_iterator it=allKeyFrames.begin(); it!=allKeyFrames.end(); it++) {

		KeyFrame *kf = *it;
		cv::Mat pos = kf->GetCameraCenter();
		cv::Mat orient = kf->GetRotation().t();
		vector<float> q = ORB_SLAM2::Converter::toQuaternion(orient);

//		cout << kf->mnId << " ";
        cout << std::fixed << setprecision(6) << kf->mTimeStamp
        		<< setprecision(7) << " " << pos.at<float>(0) << " " << pos.at<float>(1) << " " << pos.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3];
        if (kf->extPosition.empty()) {
        	cout << " x x x x x x x ";
        }
        else {
			cout << " " << kf->extPosition.at<double>(0) << " " << kf->extPosition.at<double>(1) << " " << kf->extPosition.at<double>(2);
			cout << " " << kf->extOrientation.at<double>(0) << " " << kf->extOrientation.at<double>(1) << " " << kf->extOrientation.at<double>(2) << " " << kf->extOrientation.at<double>(3);
        }
		cout << endl;

	}
}
