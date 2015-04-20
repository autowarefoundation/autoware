/*
 * signals.cpp
 *
 *  Created on: Apr 9, 2015
 *      Author: sujiwo
 */


#include <iostream>
#include <ros/ros.h>
#include "Rate.h"
#include "vector_map.h"
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <signal.h>
#include <cstdio>
#include "Math.h"
#include <Eigen/Eigen>
#include "traffic_light_detector/Signals.h"



VectorMap vmap;

Eigen::Vector3f position;
Eigen::Quaternionf orientation;
float	fx,
		fy,
		imageWidth,
		imageHeight,
		cx,
		cy;

tf::StampedTransform trf;


#define SignalLampRadius 0.3


void cameraInfoCallback (const sensor_msgs::CameraInfo::ConstPtr camInfoMsg)
{
	fx = camInfoMsg->P[0],
	fy = camInfoMsg->P[5],
	imageWidth = camInfoMsg->width, imageHeight = camInfoMsg->height,
	cx = camInfoMsg->P[2],
	cy = camInfoMsg->P[6];
	std::cout << fx << " " << fy << " " << cx << " " << cy << std::endl;
}


void getTransform (Eigen::Quaternionf &ori, Point3 &pos)
{
	static tf::TransformListener listener;

    std::cout << "listener before" << std::endl;

	// target_frame    source_frame
//	listener.waitForTransform ("japan_7", "camera", ros::Time(), ros::Duration(10.0));
//	listener.lookupTransform ("japan_7", "camera", ros::Time(), trf);
	listener.waitForTransform ("camera", "japan_7", ros::Time(), ros::Duration(10.0));
	listener.lookupTransform ("camera", "japan_7", ros::Time(), trf);

    std::cout << "listener after" << std::endl;

	tf::Vector3 &p = trf.getOrigin();
	tf::Quaternion o = trf.getRotation();
	pos.x()=p.x(); pos.y()=p.y(); pos.z()=p.z();
	ori.w()=o.w(); ori.x()=o.x(); ori.y()=o.y(); ori.z()=o.z();
}


Point3 transform (const Point3 &psrc, tf::StampedTransform &tfsource)
{
	tf::Vector3 pt3 (psrc.x(), psrc.y(), psrc.z());
	tf::Vector3 pt3s = tfsource * pt3;
	return Point3 (pt3s.x(), pt3s.y(), pt3s.z());
}


/*
 * Project a point from world coordinate to image plane
 */
bool project2 (const Point3 &pt, int &u, int &v, bool useOpenGLCoord=false)
{
	float nearPlane = 1.0;
	float farPlane = 100.0;

	Point3 _pt = transform (pt, trf);
	float _u = _pt.x()*fx/_pt.z() + cx;
	float _v = _pt.y()*fy/_pt.z() + cy;
	u = (int)_u;
	v = (int)_v;
	if (u<0 or u>imageWidth or v<0 or v>imageHeight or _pt.z()<nearPlane or _pt.z()>farPlane) {
		u = -1, v = -1;
		return false;
	}

	if (useOpenGLCoord) {
		v = imageHeight - v;
	}

	return true;
}


void echoSignals2 (ros::Publisher &pub, bool useOpenGLCoord=false)
{
	int countPoint = 0;
    //	signal_pub_node::Signals signalsInFrame;
    traffic_light_detector::Signals signalsInFrame;

	for (int i=1; i<vmap.signals.size()-1; i++) {
		Signal signal = vmap.signals[i];
		int pid = vmap.vectors[signal.vid].pid;

		Point3 signalcenter = vmap.getPoint(pid);
		Point3 signalcenterx (signalcenter.x(), signalcenter.y(), signalcenter.z()+SignalLampRadius);

		int u, v;
		if (project2 (signalcenter, u, v, useOpenGLCoord) == true) {
			countPoint++;

			int radius;
			int ux, vx;
			project2 (signalcenterx, ux, vx, useOpenGLCoord);
			radius = (int)distance (ux, vx, u, v);

//			printf ("%d,%d,%d,%d,%f,%f,%f,%f,%d,%d\n",
//				signal.id,
//				u, v, 		// in pixel
//				radius,		// in pixel
//				signalcenter.x(), signalcenter.y(), signalcenter.z(),	// in japan_7
//				vmap.vectors[signal.vid].hang,
//				signal.type,
//				signal.linkid
//			);
			traffic_light_detector::ExtractedPosition sign;
			sign.signalId = signal.id;
			sign.u = u, sign.v = v, sign.radius = radius;
			sign.x = signalcenter.x(), sign.y = signalcenter.y(), sign.z = signalcenter.z();
			sign.hang = vmap.vectors[signal.vid].hang;
			sign.type = signal.type, sign.linkId = signal.linkid;
			signalsInFrame.Signals.push_back (sign);
		}
	}

	signalsInFrame.header.stamp = ros::Time::now();
	pub.publish (signalsInFrame);

	printf ("There are %d out of %u signals in frame\n", countPoint, vmap.signals.size());
}


void interrupt (int s)
{
	exit(1);
}


int main (int argc, char *argv[])
{
	vmap.loadAll(argv[1]);
	ros::init(argc, argv, "signals", ros::init_options::AnonymousName);
	ros::NodeHandle rosnode;

	ros::Subscriber cameraInfoSubscriber = rosnode.subscribe ("/camera/camera_info", 100, cameraInfoCallback);
	//ros::Publisher signalPublisher = rosnode.advertise <traffic_light_detector::Signals> ("SignalProjector", 100);
    ros::Publisher signalPublisher = rosnode.advertise <traffic_light_detector::Signals> ("traffic_light_pixel_xy", 100);
	signal (SIGINT, interrupt);

    Rate loop (25);
    while (true) {

      std::cout << "spinOnce before" << std::endl;
      ros::spinOnce();
      std::cout << "spinOnce after" << std::endl;

      try {
        getTransform (orientation, position);
      } catch (tf::TransformException &exc) {
      }

      std::cout << "try after" << std::endl;

      echoSignals2 (signalPublisher, true);
      loop.sleep();
    }


}
