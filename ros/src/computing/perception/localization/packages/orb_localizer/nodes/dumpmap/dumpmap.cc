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
#include "MapPoint.h"
#include "Converter.h"
#include "ORBVocabulary.h"


using namespace std;
using ORB_SLAM2::Map;
using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::MapPoint;


const string mapvoc = "/tmp/orbvocz.txt";


void dumpVocabularyX (const ORB_SLAM2::Map &Map, const string &resultingVocTextfile)
{
	ORB_SLAM2::ORBVocabulary mapVoc(10, 6);
	vector<vector<DBoW2::FORB::TDescriptor> > keymapFeatures;
	keymapFeatures.reserve(Map.kfListSorted.size());

	vector<KeyFrame*> allKeyFrames = Map.kfListSorted;
	for (vector<KeyFrame*>::const_iterator it=allKeyFrames.begin(); it!=allKeyFrames.end(); it++) {

		KeyFrame *kf = *it;
		vector<cv::Mat> vCurrentDesc = ORB_SLAM2::Converter::toDescriptorVector(kf->mDescriptors);

		keymapFeatures.push_back(vCurrentDesc);
	}

	mapVoc.create(keymapFeatures);
	mapVoc.saveToTextFile(resultingVocTextfile);
}


int main (int argc, char **argv)
{
	string mapfile (argv[1]);
	ORB_SLAM2::Map World;
	World.loadFromDisk (mapfile);

	vector<KeyFrame*> allKeyFrames = World.kfListSorted;

	cout << std::fixed << setprecision(7);
//	cout << "First keyframe time: " << allKeyFrames[0]->mTimeStamp << endl;
//	cout << "Last keyframe time: " << allKeyFrames.back()->mTimeStamp << endl;

	for (vector<KeyFrame*>::const_iterator it=allKeyFrames.begin(); it!=allKeyFrames.end(); it++) {

		KeyFrame *kf = *it;
		cv::Mat pos = kf->GetCameraCenter();
		const float
			x = pos.at<float>(0),
			y = pos.at<float>(1),
			z = pos.at<float>(2);

		cv::Mat orient = kf->GetRotation().t();
		Eigen::Quaterniond q = ORB_SLAM2::Converter::toQuaternion(orient);


//		cout << kf->mnId << " ";
        cout << std::fixed << setprecision(6) <<
        		// First column: timestamp
        		kf->mnId << " "
        		<< kf->mTimeStamp
        		<< setprecision(7) << " "
				// Columns 1-3: ORB coordinate
				<< pos.at<float>(0) << " " << pos.at<float>(1) << " " << pos.at<float>(2)
				<< " "
				// Columns 4-7: ORB Orientation in quaternion
				<< q.x() << " " << q.y() << " " << q.z() << " " << q.w();
        if (kf->extPosition.empty()) {
        	cout << " x x x x x x x ";
        }
        else {
			cout << " "
				// Columns 8-10: Reference coordinate
				<< kf->extPosition.at<double>(0) << " " << kf->extPosition.at<double>(1) << " " << kf->extPosition.at<double>(2);
			cout << " "
				// Columns 11-14: Reference orientation in quaternion
				<< kf->extOrientation.at<double>(0) << " " << kf->extOrientation.at<double>(1) << " " << kf->extOrientation.at<double>(2) << " " << kf->extOrientation.at<double>(3);
        }
		cout << endl;

	}

//	cerr << "Saving vocabulary to " << mapvoc << endl;
//	dumpVocabulary(World, mapvoc);

	return 0;
}
