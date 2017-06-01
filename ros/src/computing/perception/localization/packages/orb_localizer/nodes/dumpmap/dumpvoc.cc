/*
 * dumpmap.cc
 *
 *  Created on: Nov 27, 2015
 *      Author: sujiwo
 */

#include <string>
#include <iostream>
#include <cstdio>
#include "Map.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Converter.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"


using namespace std;
using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::MapPoint;
using ORB_SLAM2::KeyFrameDatabase;


const string mapvoc = "/tmp/orbvocz.txt";


template<class T>
vector<T> set2vector (const set<T> &st)
{
	vector<T> rt;
	for (auto &smember: st) {
		rt.push_back(smember);
	}
	return rt;
}


//void dumpVocabulary2 (const ORB_SLAM2::Map &Map, const string &resultingVocTextfile)
void dumpVocabulary2 (const string &mapFilePath, const string &resultingVocTextfile)
{
	ORB_SLAM2::ORBVocabulary mapVoc(10, 6);
	KeyFrameDatabase keyframeDB (mapVoc);
	ORB_SLAM2::Map World;

	World.loadFromDisk(mapFilePath, &keyframeDB);

	vector<vector<DBoW2::FORB::TDescriptor> > keymapFeatures;
	keymapFeatures.reserve(World.kfListSorted.size());

	vector<KeyFrame*> allKeyFrames = World.kfListSorted;
	for (vector<KeyFrame*>::const_iterator it=allKeyFrames.begin(); it!=allKeyFrames.end(); it++) {

		KeyFrame *kf = *it;
		vector<cv::Mat> kfDescriptor;

		// take map points that are belong to this keyframe
		set<MapPoint*> mapSet = kf->GetMapPoints();
		vector<MapPoint*> mapPointList = set2vector(mapSet);

		// for each map points, pick best descriptor and add to descriptors of this keyframe
		for (auto &mpp: mapPointList) {
			cv::Mat mpDescriptor = mpp->GetDescriptor();
			kfDescriptor.push_back(mpDescriptor);
		}

		keymapFeatures.push_back (kfDescriptor);
		fprintf (stderr, "%d descriptors in %d\n", kfDescriptor.size(), kf->mnId);
	}

	mapVoc.create (keymapFeatures);
	fprintf (stderr, "Saving...\n");
	mapVoc.saveToTextFile(resultingVocTextfile);
	fprintf (stderr, "Done\n");
}


int main (int argc, char **argv)
{
	string mapfile (argv[1]);

	dumpVocabulary2 (mapfile, mapvoc);

	return 0;
}
