/*
 * offline_offset.cc
 *
 *  Created on: Jun 5, 2016
 *      Author: sujiwo
 */


#include <iostream>
#include <vector>
#include <cstdlib>
#include <cstdio>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <boost/foreach.hpp>

#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/impl/octree_search.hpp>

#include "System.h"
#include "Map.h"
#include "KeyFrame.h"
#include "ImageGrabber.h"


using namespace std;
using ORB_SLAM2::System;
using ORB_SLAM2::Map;
using ORB_SLAM2::KeyFrame;


#define foreach BOOST_FOREACH


struct MapPt {
	PCL_ADD_POINT4D;
	uint32_t kfId;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16 ;


struct FakeMap
{
    pcl::PointCloud<MapPt>::Ptr kfCloud;
    pcl::octree::OctreePointCloudSearch<MapPt>::Ptr kfOctree;

    vector<tf::Transform> orbPoints;
    vector<tf::Transform> realPoints;

	FakeMap (const string &csvPath)
	{
		FILE *mapfd;
		mapfd = fopen (csvPath.c_str(), "r");
		double
			timestamp,
			xo,
			yo,
			zo,
			qxo,
			qyo,
			qzo,
			qwo,
			xr,
			yr,
			zr,
			qxr,
			qyr,
			qzr,
			qwr;
		uint32_t pos = 0;

		while (fscanf(mapfd, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
			&timestamp,
			&xo,
			&yo,
			&zo,
			&qxo,
			&qyo,
			&qzo,
			&qwo,
			&xr,
			&yr,
			&zr,
			&qxr,
			&qyr,
			&qzr,
			&qwr) != EOF) {

			tf::Transform orbPt, realMapPt;
			orbPt.setOrigin(tf::Vector3(xo, yo, zo));
			orbPt.setRotation(tf::Quaternion(qxo, qyo, qzo, qwo));
			realMapPt.setOrigin(tf::Vector3(xr, yr, zr));
			realMapPt.setRotation(tf::Quaternion(qxr, qyr, qzr, qwr));

			orbPoints.push_back(orbPt);
			realPoints.push_back(realMapPt);
		}

		fclose (mapfd);

		kfCloud = pcl::PointCloud<MapPt>::Ptr (new pcl::PointCloud<MapPt>);
		kfCloud->width = orbPoints.size();
		kfCloud->height = 1;
		kfCloud->resize(kfCloud->width);
		for (int p=0; p<orbPoints.size(); p++) {
			const tf::Transform &kf = orbPoints[p];
			kfCloud->at(p).x = kf.getOrigin().x();
			kfCloud->at(p).y = kf.getOrigin().y();
			kfCloud->at(p).z = kf.getOrigin().z();
			kfCloud->at(p).kfId = p;
		}
		kfOctree = pcl::octree::OctreePointCloudSearch<MapPt>::Ptr (new pcl::octree::OctreePointCloudSearch<MapPt> (1.0));
		kfOctree->setInputCloud(kfCloud);
		kfOctree->addPointsFromInputCloud();
	}

	int search (const double x, const double y, const double z,
		tf::Transform &orbMapPos, tf::Transform &realMapPos)
	{
		MapPt queryPoint;
		queryPoint.x = x, queryPoint.y = y, queryPoint.z = z;

		const int k = 2;
		vector<int> idcs;
		vector<float> sqrDist;
		idcs.resize(k);
		sqrDist.resize(k);

		int r = kfOctree->nearestKSearch(queryPoint, k, idcs, sqrDist);
		if (r==0)
			return -1;
		uint32_t ptId = kfCloud->at(idcs[0]).kfId;
		orbMapPos = orbMapAt (ptId);
		realMapPos = realMapAt (ptId);
		return ptId;
	}

	tf::Transform orbMapAt (const uint32_t &i)
	{ return orbPoints.at(i); }

	tf::Transform realMapAt (const uint32_t &i)
	{ return realPoints.at(i); }

};



int main (int argc, char *argv[])
{
	FakeMap *fakeMap = NULL;
	double secondToSkip = 0.0;

	if (argc<3) {
		cout << "\nUsage:\n";
		cout << "offline_offset path_to_settings orb_result_bag map_file [secondToSkip]\n" << endl;
		exit(1);
	}

	string emptyStr;
	System SLAM(emptyStr, argv[1], System::MONOCULAR,true, argv[3], System::LOCALIZATION);
	Map *mapSrc = SLAM.getMap();
	int offsetKeyframe = (int)SLAM.fsSettings["ExternalLocalization.OffsetKeyframes"];

	if (mapSrc->KeyFramesInMap()==0) {
		fakeMap = new FakeMap (argv[3]);
	}

	if (argc>=4)
		secondToSkip = atof (argv[4]);

	// Build ROSBag Query
	rosbag::Bag bagSrc;
	bagSrc.open (argv[2], rosbag::bagmode::Read);
	rosbag::View viewx(bagSrc, rosbag::TopicQuery("/tf"));
	ros::Time startTime = viewx.getBeginTime();
	startTime.sec += secondToSkip;
	rosbag::View view(bagSrc, rosbag::TopicQuery("/tf"), startTime);
	cout << "Fetching..." << endl;

	const string orbSrcFrame = "/ORB_SLAM/World",
		orbTgFrame = "/ORB_SLAM/Camera";

	// prepare Stdout
    cout << std::fixed << setprecision(6);

	foreach (rosbag::MessageInstance const msg, view) {

		// take current position from bag
		tf::tfMessageConstPtr curPosMsg = msg.instantiate<tf::tfMessage>();
		const double timestamp = curPosMsg->transforms[0].header.stamp.toSec();

		if (curPosMsg->transforms[0].header.frame_id != orbSrcFrame or
				curPosMsg->transforms[0].child_frame_id != orbTgFrame)
			continue;

		// find keyframe position in map
		tf::Transform
			orbPos,
			orbMapPos,
			realMapPos,
			realPos;

		geometry_msgs::Transform pose = curPosMsg->transforms[0].transform;
		double x = pose.translation.x,
			y = pose.translation.y,
			z = pose.translation.z;
		orbPos.setOrigin(tf::Vector3(x, y, z));
		orbPos.setRotation(tf::Quaternion(
			pose.rotation.x,
			pose.rotation.y,
			pose.rotation.z,
			pose.rotation.w));

		if (fakeMap != NULL) {
			int ptId = fakeMap->search(x, y, z, orbMapPos, realMapPos);
			ptId -= offsetKeyframe;
			if (ptId<0)
				continue;
			tf::Transform orbMapOff = fakeMap->orbMapAt(ptId);
			tf::Transform realMapOff = fakeMap->realMapAt(ptId);
			realPos = ImageGrabber::localizeByReference(
				orbPos,
				orbMapPos, orbMapOff,
				realMapPos, realMapOff);
		}
		else {
			KeyFrame *kfReference = SLAM.getMap()->getNearestKeyFrame(x, y, z),
				*kfOffset = SLAM.getMap()->offsetKeyframe(kfReference, offsetKeyframe);

			orbMapPos = ImageGrabber::KeyFramePoseToTf(kfReference);
			realMapPos = ImageGrabber::getKeyFrameExtPose(kfReference);
			tf::Transform orbMapOff = ImageGrabber::KeyFramePoseToTf(kfOffset);
		}

		cout << timestamp << " " << realPos.getOrigin().x() << " " << realPos.getOrigin().y() << " " << realPos.getOrigin().z() << " ";
		cout << realPos.getRotation().x() << " " << realPos.getRotation().y() << " " << realPos.getRotation().z() << " " << realPos.getRotation().w() << endl;
	}

	SLAM.Shutdown();
	if (fakeMap != NULL)
		delete (fakeMap);
	cout << "Done" << endl;
	exit(0);
}


