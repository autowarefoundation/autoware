#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <autoware_msgs/lane.h>
#include <autoware_msgs/traffic_light.h>

#include <cross_road_area.hpp>
#include <state.hpp>
#include <state_context.hpp>
#include <decision_maker_node.hpp>

namespace decision_maker
{
#define VEL_AVERAGE_COUNT 10

void DecisionMakerNode::callbackFromCurrentPose(const geometry_msgs::PoseStamped &msg)
{
  geometry_msgs::PoseStamped _pose = current_pose_ = msg;
  bool initLocalizationFlag = ctx->isState(state_machine::INITIAL_LOCATEVEHICLE_STATE);

  if (initLocalizationFlag &&
      ctx->handleCurrentPose(_pose.pose.position.x, _pose.pose.position.y, _pose.pose.position.z,
                             _pose.pose.orientation.x, _pose.pose.orientation.y, _pose.pose.orientation.z))
  {
    ROS_INFO("Localization was convergence");
  }
}

void DecisionMakerNode::callbackFromLightColor(const autoware_msgs::traffic_light &msg)
{
  ROS_INFO("Light color callback");
  CurrentTrafficlight = msg.traffic_light;
  //ctx->handleTrafficLight(CurrentTrafficlight);
}

//
void DecisionMakerNode::callbackFromPointsRaw(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if (ctx->handlePointsRaw(true))
  {
    Subs["points_raw"].shutdown();
  }
}

void DecisionMakerNode::callbackFromFinalWaypoint(const autoware_msgs::lane &msg)
{
  if (!hasvMap())
  {
    std::cerr << "Not found vmap subscribe" << std::endl;
    // return;
  }

  current_finalwaypoints_ = msg;
  ClosestArea_ = CrossRoadArea::findClosestCrossRoad(current_finalwaypoints_, intersects);

  if (ctx->inState(state_machine::DRIVE_STATE))
  {
    double intersect_wayangle = calcIntersectWayAngle(current_finalwaypoints_, current_pose_);
    ctx->handleIntersection(true, intersect_wayangle);
  }

  double _temp_sum = 0;
  for (int i = 0; i < VEL_AVERAGE_COUNT; i++)
  {
    _temp_sum += mps2kmph(msg.waypoints[i].twist.twist.linear.x);
  }
  average_velocity_ = _temp_sum / VEL_AVERAGE_COUNT;
  
  if (std::fabs(average_velocity_ - current_velocity_) <= 2.0)
  {
    TextOffset = "Keep";
  }
  else if (average_velocity_ - current_velocity_)
  {
    TextOffset = "Accelerate";
  }
  else
  {
    TextOffset = "Decelerate";
  }

  // for publish plan of velocity
  publishToVelocityArray();

  std::cout << "Velocity: " << current_velocity_ << " to " << average_velocity_ << std::endl;
}
void DecisionMakerNode::callbackFromTwistCmd(const geometry_msgs::TwistStamped &msg)
{
  static bool Twistflag = false;

  if (Twistflag)
    ctx->handleTwistCmd(false);
  else
    Twistflag = true;
}

void DecisionMakerNode::callbackFromVectorMapArea(const vector_map_msgs::AreaArray &msg)
{
  vMap_Areas = msg;
  vMap_Areas_flag = true;
}
void DecisionMakerNode::callbackFromVectorMapPoint(const vector_map_msgs::PointArray &msg)
{
  vMap_Points = msg;
  vMap_Points_flag = true;
}
void DecisionMakerNode::callbackFromVectorMapLine(const vector_map_msgs::LineArray &msg)
{
  vMap_Lines = msg;
  vMap_Lines_flag = true;
}
void DecisionMakerNode::callbackFromVectorMapCrossRoad(const vector_map_msgs::CrossRoadArray &msg)
{
  vMap_CrossRoads = msg;
  vMap_CrossRoads_flag = true;
}

void DecisionMakerNode::callbackFromCurrentVelocity(const geometry_msgs::TwistStamped &msg)
{
  current_velocity_ = mps2kmph(msg.twist.linear.x);
}
#if 0
	void DecisionMakerNode::callbackFromDynamicReconfigure(decision_maker::decision_makerConfig &config, uint32_t level){
		ROS_INFO("Reconfigure Request: %d ", config.TARGET_WAYPOINT_COUNT);
	}
#endif
}
