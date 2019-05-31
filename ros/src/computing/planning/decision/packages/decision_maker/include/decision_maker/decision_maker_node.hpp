#ifndef __DECISION_MAKER_NODE__
#define __DECISION_MAKER_NODE__

#include <unordered_map>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <random>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <autoware_config_msgs/ConfigDecisionMaker.h>
#include <autoware_msgs/CloudClusterArray.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/State.h>
#include <autoware_msgs/TrafficLight.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/VehicleLocation.h>
#include <autoware_msgs/Waypoint.h>
#include <autoware_msgs/WaypointState.h>
#include <vector_map/vector_map.h>

#include <amathutils_lib/amathutils.hpp>
#include <cross_road_area.hpp>
#include <decision_maker_param.hpp>
#include <state_machine_lib/state_context.hpp>

namespace decision_maker
{
using namespace vector_map;
using cstring_t = const std::string;

enum class E_Lamp : int32_t
{
  LAMP_EMPTY = -1,
  LAMP_CLEAR = 0,
  LAMP_RIGHT = 1,
  LAMP_LEFT = 2,
  LAMP_HAZARD = 3
};
enum class E_Control : int32_t
{
  KEEP = -1,
  STOP = 1,
  DECELERATE = 2,
  ACCELERATE = 3,
  OTHERS = 4,
};

enum class E_ChangeFlags : int32_t
{
  STRAIGHT,
  RIGHT,
  LEFT,

  UNKNOWN = -1,
};

inline bool hasvMap(void)
{
  return true;
}

template <class T>
typename std::underlying_type<T>::type enumToInteger(T t)
{
  return static_cast<typename std::underlying_type<T>::type>(t);
}

struct AutowareStatus
{
  std::map<std::string, bool> EventFlags;

  // planning status
  autoware_msgs::LaneArray using_lane_array;  // with wpstate
  autoware_msgs::LaneArray based_lane_array;
  autoware_msgs::Lane finalwaypoints;
  int closest_waypoint;
  int obstacle_waypoint;
  int change_flag;

  // vehicle status
  geometry_msgs::Pose pose;
  double velocity;  // kmph

  int found_stopsign_idx;
  int prev_stopped_wpidx;
  int ordered_stop_idx;
  int prev_ordered_idx;

  AutowareStatus(void) : closest_waypoint(-1), obstacle_waypoint(-1), velocity(0), found_stopsign_idx(-1), prev_stopped_wpidx(-1), ordered_stop_idx(-1), prev_ordered_idx(-1)
  {
  }

  // control status
};

class DecisionMakerNode
{
  friend class TestClass;
private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Publishers
  std::unordered_map<std::string, ros::Publisher> Pubs;
  // Subscribers
  std::unordered_map<std::string, ros::Subscriber> Subs;

  std::shared_ptr<ros::AsyncSpinner> spinners;

  AutowareStatus current_status_;

  std::vector<CrossRoadArea> intersects;

  class DetectionArea
  {
  public:
    double x1, x2;
    double y1, y2;

    DetectionArea()
    {
    }
  };
  DetectionArea detectionArea_;

  bool isManualLight;

  // Param
  bool auto_mission_reload_;
  bool auto_engage_;
  bool auto_mission_change_;
  bool use_fms_;
  bool disuse_vector_map_;
  int param_num_of_steer_behind_;
  double change_threshold_dist_;
  double change_threshold_angle_;
  double goal_threshold_dist_;
  double goal_threshold_vel_;
  double stopped_vel_;
  int stopline_reset_count_;

  // initialization method
  void initROS();
  void initVectorMap(void);

  void createSubscriber(void);
  void createPublisher(void);

  // looping method
  void update(void);
  void update_msgs(void);

  void publishToVelocityArray();

  void publishOperatorHelpMessage(const cstring_t& message);
  void publishLampCmd(const E_Lamp& status);
  void publishStoplineWaypointIdx(const int wp_idx);
  void publishLightColor(const int status);

  /* decision */
  void tryNextState(cstring_t& key);
  bool isArrivedGoal(void) const;
  bool isLocalizationConvergence(const geometry_msgs::Point& _current_point) const;
  void insertPointWithinCrossRoad(const std::vector<CrossRoadArea>& _intersects, autoware_msgs::LaneArray& lane_array);
  void setWaypointState(autoware_msgs::LaneArray& lane_array);
  bool waitForEvent(cstring_t& key, const bool& flag);
  bool waitForEvent(cstring_t& key, const bool& flag, const double& timeout);
  bool drivingMissionCheck(void);

  double calcIntersectWayAngle(const autoware_msgs::Lane& laneinArea);
  double getDistToWaypointIdx(const int wpidx) const;
  double calcRequiredDistForStop(void) const;

  uint8_t getSteeringStateFromWaypoint(void);
  uint8_t getEventStateFromWaypoint(void);
  std::pair<uint8_t, int> getStopSignStateFromWaypoint(void);

  // current decision maker is support only lane area
  bool isVehicleOnLaneArea(void)
  {
    return true;
  }
  bool isVehicleOnFreeArea(void)
  {
    return false;
  }

  void setupStateCallback(void);

  /*
   * state callback
   **/

  /*** state vehicle ***/
  // entry callback
  void entryInitState(cstring_t& state_name, int status);
  void entrySensorInitState(cstring_t& state_name, int status);
  void entryMapInitState(cstring_t& state_name, int status);
  void entryLocalizationInitState(cstring_t& state_name, int status);
  void entryPlanningInitState(cstring_t& state_name, int status);
  void entryVehicleInitState(cstring_t& state_name, int status);
  void entryVehicleReadyState(cstring_t& state_name, int status);
  void entryVehicleEmergencyState(cstring_t& state_name, int status);
  // update callback
  void updateInitState(cstring_t& state_name, int status);
  void updateSensorInitState(cstring_t& state_name, int status);
  void updateMapInitState(cstring_t& state_name, int status);
  void updateLocalizationInitState(cstring_t& state_name, int status);
  void updatePlanningInitState(cstring_t& state_name, int status);
  void updateVehicleInitState(cstring_t& state_name, int status);
  void updateVehicleReadyState(cstring_t& state_name, int status);
  void updateBatteryChargingState(cstring_t& state_name, int status);
  void updateVehicleEmergencyState(cstring_t& state_name, int status);
  // exit callback

  /*** state mission ***/
  // entry callback
  void entryMissionInitState(cstring_t& state_name, int status);
  void entryWaitOrderState(cstring_t& state_name, int status);
  void entryMissionCheckState(cstring_t& state_name, int status);
  void entryDriveReadyState(cstring_t& state_name, int status);
  void entryDrivingState(cstring_t& state_name, int status);
  void entryDrivingMissionChangeState(cstring_t& state_name, int status);
  void entryMissionAbortedState(cstring_t& state_name, int status);
  void entryMissionCompleteState(cstring_t& state_name, int status);
  // update callback
  void updateMissionInitState(cstring_t& state_name, int status);
  void updateWaitOrderState(cstring_t& state_name, int status);
  void updateMissionCheckState(cstring_t& state_name, int status);
  void updateDriveReadyState(cstring_t& state_name, int status);
  void updateDrivingState(cstring_t& state_name, int status);
  void updateDrivingMissionChangeState(cstring_t& state_name, int status);
  void updateMissionChangeSucceededState(cstring_t& state_name, int status);
  void updateMissionChangeFailedState(cstring_t& state_name, int status);
  void updateMissionCompleteState(cstring_t& state_name, int status);
  void updateMissionAbortedState(cstring_t& state_name, int status);
  // exit callback
  void exitWaitOrderState(cstring_t& state_name, int status);
  void exitDrivingState(cstring_t& state_name, int status);
  // void exitWaitMissionOrderState(cstring_t& state_name, int status);

  /*** state behavior ***/
  // entry callback
  void entryTurnState(cstring_t& state_name, int status);
  void entryLaneChangeState(cstring_t& state_name, int status);
  // update callback
  void updateStoppingState(cstring_t& state_name, int status);
  void updateBehaviorEmergencyState(cstring_t& state_name, int status);
  void updateMovingState(cstring_t& state_name, int status);
  void updateLaneAreaState(cstring_t& state_name, int status);
  void updateFreeAreaState(cstring_t& state_name, int status);
  void updateCruiseState(cstring_t& state_name, int status);
  void updateBusStopState(cstring_t& state_name, int status);
  void updateParkingState(cstring_t& state_name, int status);
  void updateLeftTurnState(cstring_t& state_name, int status);
  void updateRightTurnState(cstring_t& state_name, int status);
  void updateStraightState(cstring_t& state_name, int status);
  void updateBackState(cstring_t& state_name, int status);
  void updateLeftLaneChangeState(cstring_t& state_name, int status);
  void updateRightLaneChangeState(cstring_t& state_name, int status);
  void updatePullInState(cstring_t& state_name, int status);
  void updatePullOutState(cstring_t& state_name, int status);
  void updateCheckLeftLaneState(cstring_t& state_name, int status);
  void updateCheckRightLaneState(cstring_t& state_name, int status);
  void updateChangeToLeftState(cstring_t& state_name, int status);
  void updateChangeToRightState(cstring_t& state_name, int status);
  // exit callback
  void exitBehaviorEmergencyState(cstring_t& state_name, int status);

  /*** state motion ***/
  // entry callback
  void entryDriveState(cstring_t& state_name, int status);
  void entryGoState(cstring_t& state_name, int status);
  // update callback
  void updateWaitDriveReadyState(cstring_t& state_name, int status);
  void updateWaitEngageState(cstring_t& state_name, int status);
  void updateDriveState(cstring_t& state_name, int status);
  void updateMotionEmergencyState(cstring_t& state_name, int status);
  void updateGoState(cstring_t& state_name, int status);
  void updateWaitState(cstring_t& state_name, int status);
  void updateStopState(cstring_t& state_name, int status);
  void updateStoplineState(cstring_t& state_name, int status);
  void updateOrderedStopState(cstring_t& state_name, int status);
  void updateReservedStopState(cstring_t& state_name, int status);
  // exit callback
  void exitWaitState(cstring_t& state_name, int status);
  void exitStopState(cstring_t& state_name, int status);
  void exitOrderedStopState(cstring_t& state_name, int status);
  void exitReservedStopState(cstring_t& state_name, int status);

  // callback by topic subscribing
  void callbackFromFilteredPoints(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void callbackFromCurrentVelocity(const geometry_msgs::TwistStamped& msg);
  void callbackFromCurrentPose(const geometry_msgs::PoseStamped& msg);
  void callbackFromClosestWaypoint(const std_msgs::Int32& msg);
  void callbackFromLightColor(const ros::MessageEvent<autoware_msgs::TrafficLight const>& event);
  void callbackFromLaneChangeFlag(const std_msgs::Int32& msg);
  void callbackFromFinalWaypoint(const autoware_msgs::Lane& msg);
  void callbackFromLaneWaypoint(const autoware_msgs::LaneArray& msg);
  void callbackFromSimPose(const geometry_msgs::PoseStamped& msg);
  void callbackFromConfig(const autoware_config_msgs::ConfigDecisionMaker& msg);
  void callbackFromStateCmd(const std_msgs::String& msg);
  void callbackFromObstacleWaypoint(const std_msgs::Int32& msg);
  void callbackFromStopOrder(const std_msgs::Int32& msg);
  void callbackFromClearOrder(const std_msgs::Int32& msg);

  void setEventFlag(cstring_t& key, const bool& value)
  {
    current_status_.EventFlags[key] = value;
  }

  bool isEventFlagTrue(std::string key)
  {
    if (current_status_.EventFlags.count(key) == 0)
    {
      current_status_.EventFlags[key] = false;
    }
    return current_status_.EventFlags[key];
  }

public:
  state_machine::StateContext* ctx_vehicle;
  state_machine::StateContext* ctx_mission;
  state_machine::StateContext* ctx_behavior;
  state_machine::StateContext* ctx_motion;
  VectorMap g_vmap;

  DecisionMakerNode(int argc, char** argv)
    : private_nh_("~")
    , auto_mission_reload_(false)
    , auto_engage_(false)
    , auto_mission_change_(false)
    , use_fms_(false)
    , disuse_vector_map_(false)
    , param_num_of_steer_behind_(30)
    , change_threshold_dist_(1.0)
    , change_threshold_angle_(15)
    , goal_threshold_dist_(3.0)
    , goal_threshold_vel_(0.1)
    , stopped_vel_(0.1)
    , stopline_reset_count_(20)
  {
    std::string file_name_mission;
    std::string file_name_vehicle;
    std::string file_name_behavior;
    std::string file_name_motion;
    private_nh_.getParam("state_vehicle_file_name", file_name_vehicle);
    private_nh_.getParam("state_mission_file_name", file_name_mission);
    private_nh_.getParam("state_behavior_file_name", file_name_behavior);
    private_nh_.getParam("state_motion_file_name", file_name_motion);

    ctx_vehicle = new state_machine::StateContext(file_name_vehicle, "autoware_states_vehicle");
    ctx_mission = new state_machine::StateContext(file_name_mission, "autoware_states_mission");
    ctx_behavior = new state_machine::StateContext(file_name_behavior, "autoware_states_behavior");
    ctx_motion = new state_machine::StateContext(file_name_motion, "autoware_states_motion");
    init();
    setupStateCallback();

    private_nh_.getParam("auto_mission_reload", auto_mission_reload_);
    private_nh_.getParam("auto_engage", auto_engage_);
    private_nh_.getParam("auto_mission_change", auto_mission_change_);
    private_nh_.getParam("use_fms", use_fms_);
    private_nh_.getParam("disuse_vector_map", disuse_vector_map_);
    private_nh_.getParam("param_num_of_steer_behind", param_num_of_steer_behind_);
    private_nh_.getParam("change_threshold_dist", change_threshold_dist_);
    private_nh_.getParam("change_threshold_angle", change_threshold_angle_);
    private_nh_.getParam("goal_threshold_dist", goal_threshold_dist_);
    private_nh_.getParam("goal_threshold_vel", goal_threshold_vel_);
    private_nh_.getParam("stopped_vel", stopped_vel_);
    private_nh_.getParam("stopline_reset_count", stopline_reset_count_);
    current_status_.prev_stopped_wpidx = -1;
  }

  void init(void);
  void run(void);

  bool isSubscriberRegistered(cstring_t& topic_name)
  {
    return Subs.count(topic_name) ? true : false;
  }

  static geometry_msgs::Point VMPoint2GeoPoint(const vector_map_msgs::Point& vp)
  {
    geometry_msgs::Point gp;
    gp.x = vp.ly;
    gp.y = vp.bx;
    gp.z = vp.h;
    return gp;
  }
};

}  // namespace decision_maker

#endif
