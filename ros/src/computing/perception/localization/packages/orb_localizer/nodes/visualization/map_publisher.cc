/*
 * map_publisher.cc
 *
 *  Created on: Jul 11, 2016
 *      Author: sujiwo
 */


#include <string>
#include <iostream>
#include <vector>
#include "Map.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Converter.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>
#include "../common.h"

//#include "../__nodes/ImageGrabber.h"


using namespace std;
using ORB_SLAM2::Map;
using ORB_SLAM2::KeyFrame;
using ORB_SLAM2::MapPoint;



class MapPublisher
{
public:
	enum coordMode {
		ORB_COORD = 0,
		EXT_COORD = 1
	};


	MapPublisher (Map &mapsrc,
			ros::NodeHandle &nh,
			coordMode cmode=ORB_COORD) :
		mMap (mapsrc),
		rosNode (nh),
		pubMode (cmode)
	{
	    string MAP_FRAME_ID;
	    if (cmode==EXT_COORD)
	    	MAP_FRAME_ID = "world";
	    else
	    	MAP_FRAME_ID = "ORB_SLAM/World";

	    const char* POINTS_NAMESPACE = "MapPoints";
	    const char* KEYFRAMES_NAMESPACE = "KeyFrames";
	    const char* GRAPH_NAMESPACE = "Graph";

	    // Configure MapPoints
	    fPointSize = 0.03;
	    fCameraSize = 0.07;
	    mPoints.header.frame_id = MAP_FRAME_ID;
	    mPoints.ns = POINTS_NAMESPACE;
	    mPoints.id = 0;
	    mPoints.type = visualization_msgs::Marker::POINTS;
	    mPoints.scale.x=fPointSize;
		mPoints.scale.y=fPointSize;
		mPoints.pose.orientation.w=1.0;
		mPoints.action=visualization_msgs::Marker::ADD;
		mPoints.color.b = 1.0;
		mPoints.color.a = 1.0;

	    //Configure KeyFrames
		mKeyFrames.header.frame_id = MAP_FRAME_ID;
//		mKeyFrames.ns = KEYFRAMES_NAMESPACE;
//		mKeyFrames.id=1;
//		mKeyFrames.type = visualization_msgs::Marker::LINE_LIST;
//		mKeyFrames.scale.x=0.005;
//		mKeyFrames.pose.orientation.w=1.0;
//		mKeyFrames.action=visualization_msgs::Marker::ADD;
//	    mKeyFrames.color.b = 1.0f;
//	    mKeyFrames.color.a = 1.0;

	    //Configure Covisibility Graph
		mCovisibilityGraph.header.frame_id = MAP_FRAME_ID;
		mCovisibilityGraph.ns = GRAPH_NAMESPACE;
		mCovisibilityGraph.id=2;
		mCovisibilityGraph.type = visualization_msgs::Marker::LINE_LIST;
		mCovisibilityGraph.scale.x=0.002;
		mCovisibilityGraph.pose.orientation.w=1.0;
		mCovisibilityGraph.action=visualization_msgs::Marker::ADD;
		mCovisibilityGraph.color.b=0.7f;
		mCovisibilityGraph.color.g=0.7f;
		mCovisibilityGraph.color.a = 0.3;

		mappointPublisher = rosNode.advertise<visualization_msgs::Marker> ("ORB_SLAM/Map/MapPoint", 10);
//		mappointPublisher = rosNode.advertise<
		keyframePosePublisher = rosNode.advertise<geometry_msgs::PoseArray> ("ORB_SLAM/Map/KeyFrame", 10);
		mapPointList = mMap.GetAllMapPoints();
		keyFrameList = mMap.GetAllKeyFrames();
	}


	void paint ()
	{
		if (pubMode==ORB_COORD)
			publishMapPoints();
		publishKeyFrames();
//		publishGraph();

		cout << "Painted" << endl;
	}


protected:
	ros::NodeHandle &rosNode;
	Map &mMap;
	const coordMode pubMode;
	ros::Publisher mappointPublisher;
	ros::Publisher keyframePosePublisher;

	vector <MapPoint*> mapPointList;
	vector <KeyFrame*> keyFrameList;

	visualization_msgs::Marker mPoints;
//	visualization_msgs::Marker mKeyFrames;
	geometry_msgs::PoseArray mKeyFrames;
	visualization_msgs::Marker mCovisibilityGraph;

	float fPointSize,
		fCameraSize;


	void publishMapPoints ()
	{
		mPoints.points.clear();

		for (MapPoint *cmp: mapPointList) {
//			cerr << "P" << endl;
			if (cmp->isBad()) {
				continue;
			}
			geometry_msgs::Point p;
			cv::Mat pos = cmp->GetWorldPos();
			p.x = pos.at<float>(0);
			p.y = pos.at<float>(1);
			p.z = pos.at<float>(2);
			mPoints.points.push_back(p);
		}

		mPoints.header.stamp = ros::Time::now();
		mappointPublisher.publish (mPoints);
	}


	void publishKeyFrames ()
	{
		mKeyFrames.poses.clear();

		for (KeyFrame *keyframe: keyFrameList) {

			tf::Transform keyframePose;

			if (pubMode==ORB_COORD) {
				keyframePose = KeyFramePoseToTf (keyframe);
			}

			else {
				if (keyframe->extPosition.empty())
					continue;
				keyframePose = getKeyFrameExtPose (keyframe);
			}

			geometry_msgs::Point kfCenter;
			geometry_msgs::Quaternion kfDirection;

			kfCenter.x = keyframePose.getOrigin().x();
			kfCenter.y = keyframePose.getOrigin().y();
			kfCenter.z = keyframePose.getOrigin().z();
			kfDirection.x = keyframePose.getRotation().x();
			kfDirection.y = keyframePose.getRotation().x();
			kfDirection.z = keyframePose.getRotation().x();
			kfDirection.w = keyframePose.getRotation().x();

			geometry_msgs::Pose kfPose;
			kfPose.position = kfCenter;
			kfPose.orientation = kfDirection;
			mKeyFrames.poses.push_back (kfPose);

		}

		keyframePosePublisher.publish(mKeyFrames);
	}


	void publishGraph ()
	{

	}
};



int main (int argc, char *argv[])
{
	if (argc<2) {
		cerr << "Please specify map file" << endl;
		exit(-1);
	}
	string mapfile (argv[1]);
	ORB_SLAM2::Map World;
	World.loadFromDisk (mapfile);

	// ROS Stuff
	ros::init(argc, argv, "orb_map_publisher");
	ros::start();
	ros::NodeHandle nodeHandler;

	cout << "** Ready" << endl;
	MapPublisher mpPub (
		World,
		nodeHandler,
		argc>2 ? MapPublisher::EXT_COORD : MapPublisher::ORB_COORD);

	ros::Rate r(1.0);
	while (ros::ok()) {
		mpPub.paint();
		r.sleep();
	}
}
