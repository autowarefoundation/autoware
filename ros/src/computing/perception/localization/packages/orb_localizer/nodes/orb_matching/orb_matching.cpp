/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  XXX: Licensing has not been cleared yet.
 */


#include <iostream>
#include "Matcher.h"



int main (int argc, char *argv[])
{
	ros::init(argc, argv, "orb_matching", ros::init_options::AnonymousName);
	ros::start();
	ros::NodeHandle nodeHandler ("~");

	Matcher orb_matching (nodeHandler);
	ros::spin();

	ros::shutdown();
}
