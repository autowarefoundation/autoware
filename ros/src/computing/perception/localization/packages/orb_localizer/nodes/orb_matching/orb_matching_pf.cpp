/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  XXX: Licensing has not been cleared yet.
 */


#include <iostream>
#include "Matcher.h"
#include "FusionOdometry.h"


geometry_msgs::PoseWithCovarianceStamped
tfToPoseCovarianceStamped (const tf::Transform &pose)
{
	geometry_msgs::PoseWithCovarianceStamped pocv;
	pocv.pose.pose.position.x = pose.getOrigin().x();
	pocv.pose.pose.position.y = pose.getOrigin().y();
	pocv.pose.pose.position.z = pose.getOrigin().z();
	pocv.pose.pose.orientation.x = pose.getRotation().x();
	pocv.pose.pose.orientation.y = pose.getRotation().y();
	pocv.pose.pose.orientation.z = pose.getRotation().z();
	pocv.pose.pose.orientation.w = pose.getRotation().w();
	return pocv;
}


int main (int argc, char *argv[])
{
	ros::init(argc, argv, "orb_matching", ros::init_options::AnonymousName);
	ros::start();
	ros::NodeHandle nodeHandler ("~");

	Matcher orb_matcher (nodeHandler, false);

	string odomTopic;
	nodeHandler.getParam("odometry_topic", odomTopic);
	FusionOdometry odom_pf (nodeHandler, odomTopic);

	odom_pf.setPoseFunction (
		[&]() -> ObservationList {
			ObservationList obs;
			tf::Transform visPose = orb_matcher.getCurrentRealPose();
			geometry_msgs::PoseWithCovarianceStamped pwv = tfToPoseCovarianceStamped (visPose);
			pwv.header.stamp = ros::Time (orb_matcher.getLastLocalizationTimestamp());
			obs.push_back(pwv);
			return obs;
		}
	);

	ros::spin();

	ros::shutdown();
}
