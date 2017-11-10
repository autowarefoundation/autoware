#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <tf/transform_listener.h>

// lib
#include <euclidean_space.hpp>
#include <state.hpp>
#include <state_context.hpp>

#include <decision_maker_node.hpp>
//#include <vector_map/vector_map.h>

#include <autoware_msgs/lane.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <random>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace decision_maker
{
CrossRoadArea *DecisionMakerNode::findClosestCrossRoad(void)
{
  CrossRoadArea *_area = nullptr;

  euclidean_space::point _pa;
  euclidean_space::point _pb;

  double _min_distance = DBL_MAX;

  int _label = 1;

  if (!current_finalwaypoints_.waypoints.empty())
  {
    _pa.x = current_finalwaypoints_.waypoints[param_target_waypoint_].pose.pose.position.x;
    _pa.y = current_finalwaypoints_.waypoints[param_target_waypoint_].pose.pose.position.y;
    _pa.z = 0.0;
  }

  for (size_t i = 0; i < intersects.size(); i++)
  {
    _pb.x = intersects[i].bbox.pose.position.x;
    _pb.y = intersects[i].bbox.pose.position.y;

    _pb.z = 0.0;

    double __temp_dis = euclidean_space::EuclideanSpace::find_distance(&_pa, &_pb);

    intersects[i].bbox.label = 0;
    if (_min_distance >= __temp_dis)
    {
      _area = &intersects[i];
      _min_distance = __temp_dis;  //
    }
  }

  if (_area)
  {
    _area->bbox.label = 3;
  }

  return _area;
}

bool DecisionMakerNode::isInsideArea(geometry_msgs::Point pt)
{
  // simply implementation
  //
  if (ClosestArea_ != nullptr)
  {
    double x1 = ClosestArea_->bbox.pose.position.x - (ClosestArea_->bbox.dimensions.x / 2);
    double x2 = ClosestArea_->bbox.pose.position.x + (ClosestArea_->bbox.dimensions.x / 2);

    double y1 = ClosestArea_->bbox.pose.position.y - (ClosestArea_->bbox.dimensions.y / 2);
    double y2 = ClosestArea_->bbox.pose.position.y + (ClosestArea_->bbox.dimensions.y / 2);

    if ((x1 <= pt.x && pt.x <= x2))
    {
      if (y1 <= pt.y && pt.y <= y2)
      {
        return true;
      }
    }
  }
  return false;
}

bool DecisionMakerNode::isCrossRoadByVectorMapServer(const autoware_msgs::lane &lane_msg,
                                                     const geometry_msgs::PoseStamped &pose_msg)
{
#ifdef USE_VMAP_SERVER  // this is not successfully run
  cross_road_srv.request.pose = pose_msg;
  cross_road_srv.request.waypoints.waypoints.clear();

  for (int i = 0; i < 50; i++)
  {
    cross_road_srv.request.waypoints.waypoints.push_back(lane_msg.waypoints[i]);
  }
  for (const auto &wayp : lane_msg.waypoints)
    cross_road_srv.request.waypoints.waypoints.push_back(wayp);

  cross_road_cli.call(cross_road_srv);

  for (const auto &cross_road_d : cross_road_srv.response.objects.data)
  {
    //   std::cout << "EEEEEEEEE" << cross_road_d.linkid << std::endl;
  }
#endif
}

double DecisionMakerNode::calcIntersectWayAngle(const autoware_msgs::lane &lane_msg,
                                                const geometry_msgs::PoseStamped &pose_msg)
{
  if (vMap_CrossRoads_flag)
  {
    int FirstPoint = 0;
    int EndPoint = 0;
    int index = 0;
    int PrevPoint = 0;
    double diff = 0.0;
    inside_points_.clear();
    for (int index = 0; index < lane_msg.waypoints.size(); index++)
    {
      if (isInsideArea(lane_msg.waypoints[index].pose.pose.position))
      {
        if (!FirstPoint)
          FirstPoint = index;

        inside_points_.push_back(lane_msg.waypoints[index].pose.pose.position);
      }
      else if (FirstPoint && !EndPoint)
      {
        EndPoint = PrevPoint;
        break;
      }
      PrevPoint = index;
    }

    if (EndPoint == 0)
    {
      std::cerr << "Not inside Cross Road" << std::endl;
    }
    else
    {
      geometry_msgs::Pose _end_point;
      _end_point = lane_msg.waypoints[EndPoint].pose.pose;

      double r, p, y, _y;

      tf::Quaternion quat_end(_end_point.orientation.x, _end_point.orientation.y, _end_point.orientation.z,
                              _end_point.orientation.w);
      tf::Quaternion quat_in(pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z,
                             pose_msg.pose.orientation.w);

      tf::Matrix3x3(quat_end).getRPY(r, p, y);
      tf::Matrix3x3(quat_in).getRPY(r, p, _y);

      diff = std::floor(_y - y) * 180.0 / M_PI;

#ifdef DEBUG_PRINT
      std::cout << "Yaw:" << _y << "-" << y << ":" << _y - y << std::endl;
      if (diff > 50)
      {
        std::cout << "Right Turn" << diff << std::endl;
      }
      else if (diff < -50)
      {
        std::cout << "Left Turn" << diff << std::endl;
      }
      else
      {
        std::cout << "Straight" << diff << std::endl;
      }
      std::cout << "Size:" << lane_msg.waypoints.size() << ":"
                << "First Point = " << FirstPoint << "/ End Point = " << EndPoint << std::endl;
#endif
    }

    return diff;
  }
  else
  {
    return 0.0;
  }
}

void DecisionMakerNode::update(void)
{
  update_msgs();
}

void DecisionMakerNode::run(void)
{
  ros::Rate loop_rate(0.3);

  // for subscribe callback function
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while (ros::ok())
  {
    update();
    if (enableDisplayMarker)
      displayMarker();
    loop_rate.sleep();
  }
}
}
