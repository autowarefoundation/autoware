#include <decision_maker_node.hpp>

namespace decision_maker
{
void DecisionMakerNode::init(void)
{
  initROS();
}

void DecisionMakerNode::setupStateCallback(void)
{
  /*INIT*/
  /*** state vehicle ***/
  ctx_vehicle->setCallback(state_machine::CallbackType::ENTRY, "Init",
                           std::bind(&DecisionMakerNode::entryInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::UPDATE, "Init",
                           std::bind(&DecisionMakerNode::updateInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::ENTRY, "SensorInit",
                           std::bind(&DecisionMakerNode::entrySensorInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::UPDATE, "SensorInit",
                           std::bind(&DecisionMakerNode::updateSensorInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::ENTRY, "MapInit",
                           std::bind(&DecisionMakerNode::entryMapInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::UPDATE, "MapInit",
                           std::bind(&DecisionMakerNode::updateMapInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::ENTRY, "LocalizationInit",
                           std::bind(&DecisionMakerNode::entryLocalizationInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::UPDATE, "LocalizationInit",
                           std::bind(&DecisionMakerNode::updateLocalizationInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::ENTRY, "PlanningInit",
                           std::bind(&DecisionMakerNode::entryPlanningInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::UPDATE, "PlanningInit",
                           std::bind(&DecisionMakerNode::updatePlanningInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::ENTRY, "VehicleInit",
                           std::bind(&DecisionMakerNode::entryVehicleInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::UPDATE, "VehicleInit",
                           std::bind(&DecisionMakerNode::updateVehicleInitState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::ENTRY, "VehicleReady",
                           std::bind(&DecisionMakerNode::entryVehicleReadyState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::UPDATE, "VehicleReady",
                           std::bind(&DecisionMakerNode::updateVehicleReadyState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::ENTRY, "VehicleEmergency",
                           std::bind(&DecisionMakerNode::entryVehicleEmergencyState, this, std::placeholders::_1, 0));
  ctx_vehicle->setCallback(state_machine::CallbackType::UPDATE, "VehicleEmergency",
                           std::bind(&DecisionMakerNode::updateVehicleEmergencyState, this, std::placeholders::_1, 0));

  /*** state mission ***/
  ctx_mission->setCallback(state_machine::CallbackType::ENTRY, "WaitVehicleReady",
                           std::bind(&DecisionMakerNode::entryWaitVehicleReadyState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(state_machine::CallbackType::UPDATE, "WaitVehicleReady",
                           std::bind(&DecisionMakerNode::updateWaitVehicleReadyState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(state_machine::CallbackType::ENTRY, "WaitOrder",
                           std::bind(&DecisionMakerNode::entryWaitOrderState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(state_machine::CallbackType::UPDATE, "WaitOrder",
                           std::bind(&DecisionMakerNode::updateWaitOrderState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(state_machine::CallbackType::EXIT, "WaitOrder",
                           std::bind(&DecisionMakerNode::exitWaitOrderState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(state_machine::CallbackType::ENTRY, "MissionCheck",
                           std::bind(&DecisionMakerNode::entryMissionCheckState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(state_machine::CallbackType::UPDATE, "MissionCheck",
                           std::bind(&DecisionMakerNode::updateMissionCheckState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(state_machine::CallbackType::ENTRY, "DriveReady",
                           std::bind(&DecisionMakerNode::entryDriveReadyState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(state_machine::CallbackType::UPDATE, "DriveReady",
                           std::bind(&DecisionMakerNode::updateDriveReadyState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(state_machine::CallbackType::ENTRY, "Driving",
                           std::bind(&DecisionMakerNode::entryDrivingState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(state_machine::CallbackType::UPDATE, "Driving",
                           std::bind(&DecisionMakerNode::updateDrivingState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(state_machine::CallbackType::EXIT, "Driving",
                           std::bind(&DecisionMakerNode::exitDrivingState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
      state_machine::CallbackType::ENTRY, "DrivingMissionChange",
      std::bind(&DecisionMakerNode::entryDrivingMissionChangeState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
      state_machine::CallbackType::UPDATE, "DrivingMissionChange",
      std::bind(&DecisionMakerNode::updateDrivingMissionChangeState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
      state_machine::CallbackType::UPDATE, "MissionChangeSucceeded",
      std::bind(&DecisionMakerNode::updateMissionChangeSucceededState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(
      state_machine::CallbackType::UPDATE, "MissionChangeFailed",
      std::bind(&DecisionMakerNode::updateMissionChangeFailedState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(state_machine::CallbackType::ENTRY, "MissionComplete",
                           std::bind(&DecisionMakerNode::entryMissionCompleteState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(state_machine::CallbackType::UPDATE, "MissionComplete",
                           std::bind(&DecisionMakerNode::updateMissionCompleteState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(state_machine::CallbackType::ENTRY, "MissionAborted",
                           std::bind(&DecisionMakerNode::entryMissionAbortedState, this, std::placeholders::_1, 0));
  ctx_mission->setCallback(state_machine::CallbackType::UPDATE, "MissionAborted",
                           std::bind(&DecisionMakerNode::updateMissionAbortedState, this, std::placeholders::_1, 0));

  /*** state drive ***/
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "WaitEngage",
                         std::bind(&DecisionMakerNode::updateWaitEngageState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::ENTRY, "Drive",
                         std::bind(&DecisionMakerNode::entryDriveState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "Drive",
                         std::bind(&DecisionMakerNode::updateDriveState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "LaneArea",
                         std::bind(&DecisionMakerNode::updateLaneAreaState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "FreeArea",
                         std::bind(&DecisionMakerNode::updateFreeAreaState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "Cruise",
                         std::bind(&DecisionMakerNode::updateCruiseState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "BusStop",
                         std::bind(&DecisionMakerNode::updateBusStopState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "Parking",
                         std::bind(&DecisionMakerNode::updateParkingState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::ENTRY, "DriveEmergency",
                         std::bind(&DecisionMakerNode::entryDriveEmergencyState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "DriveEmergency",
                         std::bind(&DecisionMakerNode::updateDriveEmergencyState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::EXIT, "DriveEmergency",
                         std::bind(&DecisionMakerNode::exitDriveEmergencyState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::ENTRY, "LeftTurn",
                         std::bind(&DecisionMakerNode::entryTurnState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "LeftTurn",
                         std::bind(&DecisionMakerNode::updateLeftTurnState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::ENTRY, "RightTurn",
                         std::bind(&DecisionMakerNode::entryTurnState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "RightTurn",
                         std::bind(&DecisionMakerNode::updateRightTurnState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::ENTRY, "Straight",
                         std::bind(&DecisionMakerNode::entryTurnState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "Straight",
                         std::bind(&DecisionMakerNode::updateStraightState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::ENTRY, "Back",
                         std::bind(&DecisionMakerNode::entryTurnState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "Back",
                         std::bind(&DecisionMakerNode::updateBackState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::ENTRY, "LeftLaneChange",
                         std::bind(&DecisionMakerNode::entryLaneChangeState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "LeftLaneChange",
                         std::bind(&DecisionMakerNode::updateLeftLaneChangeState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::ENTRY, "RightLaneChange",
                         std::bind(&DecisionMakerNode::entryLaneChangeState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "RightLaneChange",
                         std::bind(&DecisionMakerNode::updateRightLaneChangeState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "PullOver",
                         std::bind(&DecisionMakerNode::updatePullOverState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "PullOut",
                         std::bind(&DecisionMakerNode::updatePullOutState, this, std::placeholders::_1, 0));

  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "CheckLeftLane",
                         std::bind(&DecisionMakerNode::updateCheckLeftLaneState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "CheckRightLane",
                         std::bind(&DecisionMakerNode::updateCheckRightLaneState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "ChangeToLeft",
                         std::bind(&DecisionMakerNode::updateChangeToLeftState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "ChangeToRight",
                         std::bind(&DecisionMakerNode::updateChangeToRightState, this, std::placeholders::_1, 0));

  ctx_drive->setCallback(state_machine::CallbackType::ENTRY, "Go",
                         std::bind(&DecisionMakerNode::entryGoState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::ENTRY, "L_Go",
                         std::bind(&DecisionMakerNode::entryGoState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::ENTRY, "R_Go",
                         std::bind(&DecisionMakerNode::entryGoState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::ENTRY, "B_Go",
                         std::bind(&DecisionMakerNode::entryGoState, this, std::placeholders::_1, 0));

  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "Go",
                         std::bind(&DecisionMakerNode::updateGoState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "L_Go",
                         std::bind(&DecisionMakerNode::updateGoState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "R_Go",
                         std::bind(&DecisionMakerNode::updateGoState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "B_Go",
                         std::bind(&DecisionMakerNode::updateGoState, this, std::placeholders::_1, 0));

  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "Wait",
                         std::bind(&DecisionMakerNode::updateWaitState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "L_Wait",
                         std::bind(&DecisionMakerNode::updateWaitState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "R_Wait",
                         std::bind(&DecisionMakerNode::updateWaitState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "B_Wait",
                         std::bind(&DecisionMakerNode::updateWaitState, this, std::placeholders::_1, 0));

  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "StopLine",
                         std::bind(&DecisionMakerNode::updateStoplineState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "L_StopLine",
                         std::bind(&DecisionMakerNode::updateStoplineState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "R_StopLine",
                         std::bind(&DecisionMakerNode::updateStoplineState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "B_StopLine",
                         std::bind(&DecisionMakerNode::updateStoplineState, this, std::placeholders::_1, 0));

  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "Stop",
                         std::bind(&DecisionMakerNode::updateStopState, this, std::placeholders::_1, 1));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "L_Stop",
                         std::bind(&DecisionMakerNode::updateStopState, this, std::placeholders::_1, 1));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "R_Stop",
                         std::bind(&DecisionMakerNode::updateStopState, this, std::placeholders::_1, 1));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "B_Stop",
                         std::bind(&DecisionMakerNode::updateStopState, this, std::placeholders::_1, 1));

  ctx_drive->setCallback(state_machine::CallbackType::EXIT, "Wait",
                         std::bind(&DecisionMakerNode::exitStopState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::EXIT, "L_Wait",
                         std::bind(&DecisionMakerNode::exitStopState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::EXIT, "R_Wait",
                         std::bind(&DecisionMakerNode::exitStopState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::EXIT, "B_Wait",
                         std::bind(&DecisionMakerNode::exitStopState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::EXIT, "StopLine",
                         std::bind(&DecisionMakerNode::exitStopState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::EXIT, "L_StopLine",
                         std::bind(&DecisionMakerNode::exitStopState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::EXIT, "R_StopLine",
                         std::bind(&DecisionMakerNode::exitStopState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::EXIT, "B_StopLine",
                         std::bind(&DecisionMakerNode::exitStopState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::EXIT, "Stop",
                         std::bind(&DecisionMakerNode::exitStopState, this, std::placeholders::_1, 1));
  ctx_drive->setCallback(state_machine::CallbackType::EXIT, "L_Stop",
                         std::bind(&DecisionMakerNode::exitStopState, this, std::placeholders::_1, 1));
  ctx_drive->setCallback(state_machine::CallbackType::EXIT, "R_Stop",
                         std::bind(&DecisionMakerNode::exitStopState, this, std::placeholders::_1, 1));
  ctx_drive->setCallback(state_machine::CallbackType::EXIT, "B_Stop",
                         std::bind(&DecisionMakerNode::exitStopState, this, std::placeholders::_1, 1));

  ctx_drive->setCallback(state_machine::CallbackType::ENTRY, "TryAvoidance",
                         std::bind(&DecisionMakerNode::entryTryAvoidanceState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "TryAvoidance",
                         std::bind(&DecisionMakerNode::updateTryAvoidanceState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::ENTRY, "CheckAvoidance",
                         std::bind(&DecisionMakerNode::entryCheckAvoidanceState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "CheckAvoidance",
                         std::bind(&DecisionMakerNode::updateCheckAvoidanceState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::ENTRY, "Avoidance",
                         std::bind(&DecisionMakerNode::entryAvoidanceState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "Avoidance",
                         std::bind(&DecisionMakerNode::updateAvoidanceState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::ENTRY, "ReturnToLane",
                         std::bind(&DecisionMakerNode::entryReturnToLaneState, this, std::placeholders::_1, 0));
  ctx_drive->setCallback(state_machine::CallbackType::UPDATE, "ReturnToLane",
                         std::bind(&DecisionMakerNode::updateReturnToLaneState, this, std::placeholders::_1, 0));

  ctx_vehicle->nextState("started");
  ctx_mission->nextState("started");
  ctx_drive->nextState("started");
}

void DecisionMakerNode::createSubscriber(void)
{
  // Config subscriber
  Subs["config/decision_maker"] =
      nh_.subscribe("/config/decision_maker", 3, &DecisionMakerNode::callbackFromConfig, this);

  Subs["state_cmd"] = nh_.subscribe("/state_cmd", 1, &DecisionMakerNode::callbackFromStateCmd, this);
  Subs["current_velocity"] =
      nh_.subscribe("/current_velocity", 1, &DecisionMakerNode::callbackFromCurrentVelocity, this);
  Subs["obstacle_waypoint"] =
      nh_.subscribe("/obstacle_waypoint", 1, &DecisionMakerNode::callbackFromObstacleWaypoint, this);
  Subs["change_flag"] = nh_.subscribe("/change_flag", 1, &DecisionMakerNode::callbackFromLaneChangeFlag, this);
}
void DecisionMakerNode::createPublisher(void)
{
  // pub
  Pubs["state/stopline_wpidx"] = nh_.advertise<std_msgs::Int32>("/state/stopline_wpidx", 1, false);

  // for controlling other planner
  Pubs["lane_waypoints_array"] = nh_.advertise<autoware_msgs::LaneArray>(TPNAME_CONTROL_LANE_WAYPOINTS_ARRAY, 10, true);
  Pubs["light_color"] = nh_.advertise<autoware_msgs::TrafficLight>("/light_color_managed", 1);

  // for controlling vehicle
  Pubs["lamp_cmd"] = nh_.advertise<autoware_msgs::LampCmd>("/lamp_cmd", 1);

  // for visualize status
  Pubs["crossroad_marker"] = nh_.advertise<visualization_msgs::MarkerArray>("/state/cross_road_marker", 1);
  Pubs["stopline_target"] = nh_.advertise<visualization_msgs::Marker>("/state/stopline_target", 1);

  Pubs["crossroad_inside_marker"] = private_nh_.advertise<visualization_msgs::Marker>("/state/cross_inside_marker", 1);
  Pubs["crossroad_bbox"] = private_nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/state/crossroad_bbox", 10);

  Pubs["state"] = private_nh_.advertise<std_msgs::String>("state", 1, true);
  Pubs["state_overlay"] = private_nh_.advertise<jsk_rviz_plugins::OverlayText>("/state/overlay_text", 1);
  Pubs["available_transition"] = private_nh_.advertise<std_msgs::String>("available_transition", 1, true);

  // for debug
  Pubs["target_velocity_array"] = nh_.advertise<std_msgs::Float64MultiArray>("/target_velocity_array", 1);
  Pubs["operator_help_text"] = private_nh_.advertise<jsk_rviz_plugins::OverlayText>("operator_help_text", 1, true);
}

void DecisionMakerNode::initROS()
{
  // for subscribe callback function

  createSubscriber();
  createPublisher();

  spinners = std::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(3));
  spinners->start();

#if 0
  // message setup
  state_text_msg.width = 400;
  state_text_msg.height = 500;
  state_text_msg.top = 10;
  state_text_msg.left = 10;
  state_text_msg.bg_color.r = 0;
  state_text_msg.bg_color.g = 0;
  state_text_msg.bg_color.b = 0;
  state_text_msg.bg_color.a = 0.8;

  state_text_msg.line_width = 2;
  state_text_msg.text_size = 18;
  state_text_msg.font = "DejaVu Sans Mono";
  state_text_msg.fg_color.r = 0.1;
  state_text_msg.fg_color.g = 1.0;
  state_text_msg.fg_color.b = 0.94;
  state_text_msg.fg_color.a = 0.8;
  state_text_msg.text = "UNDEFINED";
#endif
  update_msgs();

#if 0
  // status subscriber
  Subs["sim_pose"] = nh_.subscribe("sim_pose", 20, &DecisionMakerNode::callbackFromSimPose, this);
  Subs["current_pose"] = nh_.subscribe("current_pose", 20, &DecisionMakerNode::callbackFromCurrentPose, this);
  Subs["current_velocity"] =
      nh_.subscribe("current_velocity", 20, &DecisionMakerNode::callbackFromCurrentVelocity, this);
  Subs["light_color"] = nh_.subscribe("light_color", 10, &DecisionMakerNode::callbackFromLightColor, this);
  Subs["final_waypoints"] = nh_.subscribe("final_waypoints", 100, &DecisionMakerNode::callbackFromFinalWaypoint, this);
  Subs["twist_cmd"] = nh_.subscribe("twist_cmd", 10, &DecisionMakerNode::callbackFromTwistCmd, this);
  Subs["change_flag"] = nh_.subscribe("change_flag", 1, &DecisionMakerNode::callbackFromLaneChangeFlag, this);
  Subs["state_cmd"] = nh_.subscribe("state_cmd", 1, &DecisionMakerNode::callbackFromStateCmd, this);
  Subs["closest_waypoint"] =
      nh_.subscribe("closest_waypoint", 1, &DecisionMakerNode::callbackFromClosestWaypoint, this);
  Subs["cloud_clusters"] = nh_.subscribe("cloud_clusters", 1, &DecisionMakerNode::callbackFromObjectDetector, this);


  // pub
  Pubs["state/stopline_wpidx"] = nh_.advertise<std_msgs::Int32>("/state/stopline_wpidx", 1, false);

  // for controlling other planner
  Pubs["state"] = nh_.advertise<std_msgs::String>("state", 1);
  Pubs["lane_waypoints_array"] = nh_.advertise<autoware_msgs::LaneArray>(TPNAME_CONTROL_LANE_WAYPOINTS_ARRAY, 10, true);
  Pubs["states"] = nh_.advertise<autoware_msgs::State>("/decisionmaker/states", 1, true);
  Pubs["light_color"] = nh_.advertise<autoware_msgs::TrafficLight>("/light_color_managed", 1);

  // for controlling vehicle
  Pubs["lamp_cmd"] = nh_.advertise<autoware_msgs::LampCmd>("/lamp_cmd", 1);

  // for visualize status
  Pubs["state_overlay"] = nh_.advertise<jsk_rviz_plugins::OverlayText>("/state/overlay_text", 1);
  Pubs["crossroad_marker"] = nh_.advertise<visualization_msgs::MarkerArray>("/state/cross_road_marker", 1);
  Pubs["crossroad_inside_marker"] = nh_.advertise<visualization_msgs::Marker>("/state/cross_inside_marker", 1);
  Pubs["crossroad_bbox"] = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/state/crossroad_bbox", 10);
  Pubs["detection_area"] = nh_.advertise<visualization_msgs::Marker>("/state/detection_area", 1);
  Pubs["stopline_target"] = nh_.advertise<visualization_msgs::Marker>("/state/stopline_target", 1);

  // for debug
  Pubs["target_velocity_array"] = nh_.advertise<std_msgs::Float64MultiArray>("/target_velocity_array", 1);
  Pubs["state_local_diffdistance"] = nh_.advertise<std_msgs::Float64>("/state/val_diff_distance", 1);
  Pubs["exectime"] = nh_.advertise<std_msgs::Float64>("/state/exectime", 1);

  // message setup
  state_text_msg.width = 400;
  state_text_msg.height = 500;
  state_text_msg.top = 10;
  state_text_msg.left = 10;
  state_text_msg.bg_color.r = 0;
  state_text_msg.bg_color.g = 0;
  state_text_msg.bg_color.b = 0;
  state_text_msg.bg_color.a = 0.8;

  state_text_msg.line_width = 2;
  state_text_msg.text_size = 18;
  state_text_msg.font = "DejaVu Sans Mono";
  state_text_msg.fg_color.r = 0.1;
  state_text_msg.fg_color.g = 1.0;
  state_text_msg.fg_color.b = 0.94;
  state_text_msg.fg_color.a = 0.8;
  state_text_msg.text = "UNDEFINED";

  // initial publishing state message
  update_msgs();

  // setup a callback for state update();
  setupStateCallback();

  ctx->nextState("started");

  {
    if (enableDisplayMarker)
      displayMarker();
  }

  ROS_INFO("Initialized OUT\n");
  // ctx->setCurrentState(state_machine::LOCATEVEHICLE_STATE);

  Subs["lane_waypoints_array"] =
      nh_.subscribe(TPNAME_BASED_LANE_WAYPOINTS_ARRAY, 100, &DecisionMakerNode::callbackFromLaneWaypoint, this);
#endif
}

void DecisionMakerNode::initVectorMap(void)
{
  int _index = 0;

  const std::vector<CrossRoad> crossroads = g_vmap.findByFilter([](const CrossRoad& crossroad) { return true; });
  if (crossroads.empty())
  {
    ROS_INFO("crossroads have not found\n");
    return;
  }

  for (const auto& cross_road : crossroads)
  {
    geometry_msgs::Point _prev_point;
    Area area = g_vmap.findByKey(Key<Area>(cross_road.aid));
    CrossRoadArea carea;
    carea.id = _index++;
    carea.area_id = area.aid;

    double x_avg = 0.0, x_min = 0.0, x_max = 0.0;
    double y_avg = 0.0, y_min = 0.0, y_max = 0.0;
    double z = 0.0;
    int points_count = 0;

    const std::vector<Line> lines =
        g_vmap.findByFilter([&area](const Line& line) { return area.slid <= line.lid && line.lid <= area.elid; });
    for (const auto& line : lines)
    {
      const std::vector<Point> points =
          g_vmap.findByFilter([&line](const Point& point) { return line.bpid == point.pid; });
      for (const auto& point : points)
      {
        geometry_msgs::Point _point;
        _point.x = point.ly;
        _point.y = point.bx;
        _point.z = point.h;

        if (_prev_point.x == _point.x && _prev_point.y == _point.y)
          continue;

        _prev_point = _point;
        points_count++;
        carea.points.push_back(_point);

        // calc a centroid point and about intersects size
        x_avg += _point.x;
        y_avg += _point.y;
        x_min = (x_min == 0.0) ? _point.x : std::min(_point.x, x_min);
        x_max = (x_max == 0.0) ? _point.x : std::max(_point.x, x_max);
        y_min = (y_min == 0.0) ? _point.y : std::min(_point.y, y_min);
        y_max = (y_max == 0.0) ? _point.y : std::max(_point.y, y_max);
        z = _point.z;
      }  // points iter
    }    // line iter
    carea.bbox.pose.position.x = x_avg / (double)points_count * 1.5 /* expanding rate */;
    carea.bbox.pose.position.y = y_avg / (double)points_count * 1.5;
    carea.bbox.pose.position.z = z;
    carea.bbox.dimensions.x = x_max - x_min;
    carea.bbox.dimensions.y = y_max - y_min;
    carea.bbox.dimensions.z = 2;
    carea.bbox.label = 1;
    intersects.push_back(carea);
  }
}
}
