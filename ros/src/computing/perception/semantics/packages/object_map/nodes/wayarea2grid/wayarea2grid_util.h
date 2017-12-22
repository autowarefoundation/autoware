#ifndef WAYAREA_TO_GRID_UTIL_H
#define WAYAREA_TO_GRID_UTIL_H

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <vector_map/vector_map.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_cv/grid_map_cv.hpp>

namespace object_map
{
  
geometry_msgs::Point transformPoint(const geometry_msgs::Point &point, const tf::Transform &tf)
{
  // Convert ROS pose to TF pose
  tf::Point tf_point;
  tf::pointMsgToTF(point, tf_point);

  // Transform pose
  tf_point = tf * tf_point;

  // Convert TF pose to ROS pose
  geometry_msgs::Point ros_point;
  tf::pointTFToMsg(tf_point, ros_point);

  return ros_point;
}

void publishMap(const grid_map::GridMap& map, const ros::Publisher& pub)
{
  grid_map_msgs::GridMap message;
  grid_map::GridMapRosConverter::toMessage(map, message);
  pub.publish(message);
  ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec()); 
}

tf::StampedTransform findTransform(const std::string& frame1, const std::string& frame2, const tf::TransformListener& tf_listener)
{
  tf::StampedTransform transform;

  try
  {
    tf_listener.lookupTransform(frame1, frame2, ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return transform;
  }

  return transform;
}


} // namespace object_map

#endif // WAYAREA_TO_GRID_UTIL_H
