/*
 * RosHelpers.cpp
 *
 *  Created on: Jun 30, 2016
 *      Author: ai-driver
 */

#include "RosHelpers.h"

#include <iostream>
#include <sstream>
#include <fstream>

RosHelpers::RosHelpers() {

}

RosHelpers::~RosHelpers() {
}

void RosHelpers::getTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform)
{
  static tf::TransformListener listener;

  while (1)
  {
    try
    {
      listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
      break;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }
}
