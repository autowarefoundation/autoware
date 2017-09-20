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
#include <euclidean_space.hpp>

namespace decision_maker
{
#define VEL_AVERAGE_COUNT 10
#define CONV_NUM 10
#define CONVERGENCE_THRESHOLD 0.01

bool DecisionMakerNode::isLocalizationConvergence(double _x, double _y, double _z, double _roll, double _pitch, double _yaw)
{
  static int _init_count = 0;
  static euclidean_space::point *a = new euclidean_space::point();
  static euclidean_space::point *b = new euclidean_space::point();

  static double distances[CONV_NUM] = { 0.0 };
  double avg_distances = 0.0;

  for (int i = 1; i < CONV_NUM; i++)
  {
    distances[i] = distances[i - 1];
    avg_distances += distances[i];
  }

  a->x = b->x;
  a->y = b->y;
  a->z = b->z;

  b->x = _x;
  b->y = _y;
  b->z = _z;

  distances[0] = euclidean_space::EuclideanSpace::find_distance(a, b);

  if (++_init_count <= CONV_NUM)
  {
    return false;
  }else
  {
    avg_distances = (avg_distances + distances[0]) / CONV_NUM;
    if (avg_distances <= CONVERGENCE_THRESHOLD){
      return ctx->setCurrentState(state_machine::DRIVE_STATE);
    }else
    {
      return false;
    }
  }
}
    
void DecisionMakerNode::callbackFromCurrentPose(const geometry_msgs::PoseStamped &msg)
{
  geometry_msgs::PoseStamped _pose = current_pose_ = msg;
  bool initLocalizationFlag = ctx->isState(state_machine::INITIAL_LOCATEVEHICLE_STATE);
  if (initLocalizationFlag &&
      isLocalizationConvergence(_pose.pose.position.x, _pose.pose.position.y, _pose.pose.position.z,
                             _pose.pose.orientation.x, _pose.pose.orientation.y, _pose.pose.orientation.z))
  {
    ROS_INFO("Localization was convergence");
  }
}

void DecisionMakerNode::callbackFromLightColor(const autoware_msgs::traffic_light &msg)
{
  ROS_INFO("Light color callback");
  current_traffic_light = msg.traffic_light;
  if(current_traffic_light == state_machine::E_RED ||
	current_traffic_light == state_machine::E_YELLOW ){
	ctx->setCurrentState(state_machine::DRIVE_DETECT_TRAFFICLIGHT_RED_STATE);
  }
  //ctx->handleTrafficLight(CurrentTrafficlight);
}

//
void DecisionMakerNode::callbackFromPointsRaw(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
	if(ctx->setCurrentState(state_machine::INITIAL_LOCATEVEHICLE_STATE))
		Subs["points_raw"].shutdown();
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
