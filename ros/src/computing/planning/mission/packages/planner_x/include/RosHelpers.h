/*
 * RosHelpers.h
 *
 *  Created on: Jun 30, 2016
 *      Author: ai-driver
 */

#ifndef ROSHELPERS_H_
#define ROSHELPERS_H_

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

namespace PlannerXNS
{

class RosHelpers
{
public:
	RosHelpers();
	virtual ~RosHelpers();
	static void GetTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform);
};

}
#endif /* ROSHELPERS_H_ */
