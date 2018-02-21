#ifndef __DECISION_MAKER_NODE__
#define __DECISION_MAKER_NODE__

#include <mutex>
#include <unordered_map>

#include <autoware_msgs/ConfigDecisionMaker.h>
#include <autoware_msgs/LaneArray.h>
#include <autoware_msgs/lane.h>
#include <autoware_msgs/traffic_light.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <visualization_msgs/MarkerArray.h>

#include <vector_map_msgs/AreaArray.h>
#include <vector_map_msgs/CrossRoadArray.h>
#include <vector_map_msgs/LineArray.h>
#include <vector_map_msgs/PointArray.h>

#include <vector_map/vector_map.h>

#include <geometry_msgs/Point.h>

// lib
#include <amathutils.hpp>
#include <cross_road_area.hpp>
#include <decision_maker_param.hpp>
#include <state.hpp>
#include <state_context.hpp>

namespace decision_maker
{
using namespace vector_map;


enum class EControl : int32_t
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

class DecisionMakerNode
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

  // ROS Messages(Autoware)
  autoware_msgs::lane current_finalwaypoints_;
  vector_map_msgs::AreaArray vMap_Areas;
  vector_map_msgs::PointArray vMap_Points;
  vector_map_msgs::LineArray vMap_Lines;
  vector_map_msgs::CrossRoadArray vMap_CrossRoads;

  std::vector<geometry_msgs::Point> inside_points_;

  autoware_msgs::LaneArray current_based_lane_array_;       // with wpstate
  autoware_msgs::LaneArray current_shifted_lane_array_;     // with shiftedLane
  autoware_msgs::LaneArray current_controlled_lane_array_;  // modified lane
  autoware_msgs::LaneArray current_stopped_lane_array_;     // 0velocity

  // Current way/behavior status
  double current_velocity_;
  double average_velocity_;
  int current_traffic_light_;
  int closest_waypoint_;
  CrossRoadArea *ClosestArea_;
  std::string CurrentStateName;
  std::string TextOffset;
  std::vector<CrossRoadArea> intersects;
  double displacement_from_path_;

  bool isManualLight;

  // Param
  bool enableDisplayMarker;
  bool enableForceStateChange;
  uint32_t param_convergence_count_;
  uint32_t param_target_waypoint_;
  double param_convergence_threshold_;
  uint32_t param_stopline_target_waypoint_;
  double param_shift_width_;
  double param_crawl_velocity_;

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
  std::mutex vMap_mutex;
  bool created_shift_lane_flag_;

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

  void publishToVelocityArray();
  std::string createStateMessageText();
  int createCrossRoadAreaMarker(visualization_msgs::Marker &crossroad_marker, double scale);

  // judge method
  // in near future, these methods will be deprecate to decision_maker library
  bool isCrossRoadByVectorMapServer(const autoware_msgs::lane &lane_msg, const geometry_msgs::PoseStamped &pose_msg);
  bool isLocalizationConvergence(double _x, double _y, double _z, double _roll, double _pitch, double _yaw);
  bool handleStateCmd(const uint64_t _state_num);
  // double calcIntersectWayAngle(const CrossRoadArea& area);
  double calcIntersectWayAngle(const autoware_msgs::lane &laneinArea);

  void insertPointWithinCrossRoad(const std::vector<CrossRoadArea> &_intersects, autoware_msgs::LaneArray &lane_array);

  void setWaypointState(autoware_msgs::LaneArray &lane_array);
  double calcPosesAngleDiff(const geometry_msgs::Pose &p_from, const geometry_msgs::Pose &p_to);
  double calcPosesAngleDiffN(const geometry_msgs::Pose &p_from, const geometry_msgs::Pose &p_to);
  double getPoseAngle(const geometry_msgs::Pose &p);

  void publishStoppedLaneArray(void);
  void publishControlledLaneArray(void);
  void updateLaneWaypointsArray(void);
  void changeVelocityBasedLane(void);
  void changeVelocityLane(int dir);
  void createShiftLane(void);
  void changeShiftLane(void);
  void removeShiftLane(void);


  void setAllStoplineStop(void);
  void StoplinePlanIn(int status);
  void StoplinePlanOut(int status);
  void publishLightColor(int status);
  void callbackInStateObstacleAvoid(int status);
  void callbackOutStateObstacleAvoid(int status);
  void updateStateObstacleAvoid(int status);
  void updateStateSTR(int status);
  void updateStateStop(int status);
  void callbackInStateStop(int status);
  void callbackInStateAcc(int status);
  void callbackInStateDec(int status);
  void callbackInStateKeep(int status);
  void setupStateCallback(void);
  // callback by topic subscribing
  void callbackFromCurrentVelocity(const geometry_msgs::TwistStamped &msg);
  void callbackFromCurrentPose(const geometry_msgs::PoseStamped &msg);
  void callbackFromClosestWaypoint(const std_msgs::Int32 &msg);
  void callbackFromLightColor(const ros::MessageEvent<autoware_msgs::traffic_light const> &event);
  void callbackFromLaneChangeFlag(const std_msgs::Int32 &msg);
  void callbackFromPointsRaw(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void callbackFromFinalWaypoint(const autoware_msgs::lane &msg);
  void callbackFromLaneWaypoint(const autoware_msgs::LaneArray &msg);
  void callbackFromTwistCmd(const geometry_msgs::TwistStamped &msg);
  void callbackFromSimPose(const geometry_msgs::PoseStamped &msg);
  void callbackFromStateCmd(const std_msgs::Int32 &msg);
  void callbackFromConfig(const autoware_msgs::ConfigDecisionMaker &msg);

  void callbackFromVectorMapArea(const vector_map_msgs::AreaArray &msg);
  void callbackFromVectorMapPoint(const vector_map_msgs::PointArray &msg);
  void callbackFromVectorMapLine(const vector_map_msgs::LineArray &msg);
  void callbackFromVectorMapCrossRoad(const vector_map_msgs::CrossRoadArray &msg);

public:
  state_machine::StateContext *ctx;
  VectorMap g_vmap;

  DecisionMakerNode(int argc, char **argv)
  {
    SimulationMode = false;
    enableDisplayMarker = DEFAULT_DISPLAY_FLAG;
    param_convergence_threshold_ = DEFAULT_CONVERGENCE_THRESHOLD;
    param_convergence_count_ = DEFAULT_CONVERGENCE_COUNT;
    param_target_waypoint_ = DEFAULT_TARGET_WAYPOINT;
    param_shift_width_ = DEFAULT_SHIFT_WIDTH;
    param_stopline_target_waypoint_ = DEFAULT_STOPLINE_TARGET_WAYPOINT;
    param_crawl_velocity_ = DEFAULT_CRAWL_VELOCITY;

    ctx = new state_machine::StateContext();
    this->initROS(argc, argv);

    vector_map_init = false;
    created_shift_lane_flag_ = false;
    closest_waypoint_ = 0;

    ClosestArea_ = nullptr;
    displacement_from_path_ = 0.0;
    isManualLight = false;
  }

  void run(void);
  geometry_msgs::Point to_geoPoint(const vector_map_msgs::Point &vp)
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
