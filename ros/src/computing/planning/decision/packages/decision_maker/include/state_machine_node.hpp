#ifndef __STATE_MACHINE_NODE__
#define __STATE_MACHINE_NODE__

#include <unordered_map>

#include <autoware_msgs/lane.h>
#include <autoware_msgs/traffic_light.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <sensor_msgs/PointCloud2.h>

//#include <vector_map_server/GetCrossRoad.h>

#include <vector_map_msgs/AreaArray.h>
#include <vector_map_msgs/CrossRoadArray.h>
#include <vector_map_msgs/LineArray.h>
#include <vector_map_msgs/PointArray.h>

#include <geometry_msgs/Point.h>

#include <cross_road_area.hpp>

//#include <dynamic_reconfigure/server.h>
//#include <state_machine/state_machineConfig.h>

namespace state_machine
{
enum class EControl
{
  KEEP = -1,
  STOP = 1,
  DECELERATE = 2,
  ACCELERATE = 3,
  OTHERS = 4,
};

inline bool hasvMap(void)
{
  return true;
}

inline double mps2kmph(double _mpsval)
{
  return (_mpsval * 60 * 60) / 1000;  // mps * 60sec * 60m / 1000m
}

class StateMachineNode
{
private:
  ros::NodeHandle nh_;
  // Publishers
  std::unordered_map<std::string, ros::Publisher> Pubs;
  // Subscribers
  std::unordered_map<std::string, ros::Subscriber> Subs;

  // ROS Messages
  std_msgs::String state_string_msg;
  geometry_msgs::PoseStamped current_pose_;

  jsk_rviz_plugins::OverlayText state_text_msg;
  autoware_msgs::lane current_finalwaypoints_;

  std::vector<geometry_msgs::Point> inside_points_;

  vector_map_msgs::AreaArray vMap_Areas;
  vector_map_msgs::PointArray vMap_Points;
  vector_map_msgs::LineArray vMap_Lines;
  vector_map_msgs::CrossRoadArray vMap_CrossRoads;

  // Current way/behavior status
  double current_velocity_;
  double average_velocity_;
  int CurrentTrafficlight;
  CrossRoadArea *ClosestArea_;
  std::string CurrentStateName;
  std::string TextOffset;
  std::vector<CrossRoadArea> intersects;

  // for vectormap server
  // ros::ServiceClient cross_road_cli;
  // vector_map_server::GetCrossRoad cross_road_srv;

  // initialization flags for initialized by callback
  bool vector_map_init;
  bool vMap_Areas_flag;
  bool vMap_Points_flag;
  bool vMap_Lines_flag;
  bool vMap_CrossRoads_flag;
  bool SimulationMode;

  // initialization method
  void initROS(int argc, char **argv);
  void initVectorMap(void);
  void initStateMsgs(void);
  bool initVectorMapClient(void);

  // looping method
  void update(void);
  void update_msgs(void);
  void update_pubsub(void);
  void displayMarker(void);

  //
  bool isInsideArea(geometry_msgs::Point pt);
  bool isCrossRoadByVectorMapServer(const autoware_msgs::lane &lane_msg, const geometry_msgs::PoseStamped &pose_msg);

  double calcIntersectWayAngle(const autoware_msgs::lane &lane_msg, const geometry_msgs::PoseStamped &pose_msg);

  // callback by topic subscribing
  void callbackFromCurrentVelocity(const geometry_msgs::TwistStamped &msg);
  void callbackFromCurrentPose(const geometry_msgs::PoseStamped &msg);
  void callbackFromLightColor(const autoware_msgs::traffic_light &msg);
  void callbackFromPointsRaw(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void callbackFromFinalWaypoint(const autoware_msgs::lane &msg);
  void callbackFromTwistCmd(const geometry_msgs::TwistStamped &msg);
  void callbackFromSimPose(const geometry_msgs::PoseStamped &msg);

  void callbackFromVectorMapArea(const vector_map_msgs::AreaArray &msg);
  void callbackFromVectorMapPoint(const vector_map_msgs::PointArray &msg);
  void callbackFromVectorMapLine(const vector_map_msgs::LineArray &msg);
  void callbackFromVectorMapCrossRoad(const vector_map_msgs::CrossRoadArray &msg);

  // for ros dynamic reconfigure
  // Currently. this feature is not working.
  // static void callbackFromDynamicReconfigure(state_machine::state_machineConfig &config, uint32_t level);

  // in near future, these function will be deprecate
  CrossRoadArea *findClosestCrossRoad(void);

public:
  StateContext *ctx;

  StateMachineNode(int argc, char **argv)
  {
    SimulationMode = false;

    ctx = new StateContext();
    this->initROS(argc, argv);

    vector_map_init = false;
    vMap_Areas_flag = vMap_Lines_flag = vMap_Points_flag = vMap_CrossRoads_flag = false;

    ClosestArea_ = nullptr;
  }

  void run(void);
};

}  // namespace state_machine

#endif
