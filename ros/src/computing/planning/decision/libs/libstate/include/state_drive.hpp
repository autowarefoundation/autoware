#ifndef __STATE_DRIVE_HPP__
#define __STATE_DRIVE_HPP__

/**
 * @file state_drive.hpp
 * @brief Declaration drive state
 * @author Yusuke Fujii
 * @date 2017/07/27
 */

#include "state.hpp"

namespace state_machine
{
// DriveState
class DriveState : public State<DriveState>
{
private:
  friend class State<DriveState>;

  DriveState(void)
  {
    StateName = "Drive";
    StateNum = StateTransMask = DRIVE_STATE;
    StateTransMask |= INITIAL_LOCATEVEHICLE_STATE;
    StateKind = MAIN_STATE;
  }

public:
};

class DriveAccAccelerationState : public State<DriveAccAccelerationState>
{
private:
  friend class State<DriveAccAccelerationState>;
  DriveAccAccelerationState(void)
  {
    StateName = "Accelerate";
    StateNum = DRIVE_ACC_ACCELERATION_STATE;
    StateKind = ACC_STATE;
  }

public:
  void ShowStateName(void)
  {
    std::cout << StateName << "-";
  }
};

class DriveAccDecelerationState : public State<DriveAccDecelerationState>
{
private:
  friend class State<DriveAccDecelerationState>;
  DriveAccDecelerationState(void)
  {
    StateName = "Decelerate";
    StateNum = DRIVE_ACC_DECELERATION_STATE;
    StateKind = ACC_STATE;
  }

public:
};

class DriveAccKeepState : public State<DriveAccKeepState>
{
private:
  friend class State<DriveAccKeepState>;
  DriveAccKeepState(void)
  {
    StateName = "Keep";
    StateNum = DRIVE_ACC_KEEP_STATE;
    StateKind = ACC_STATE;
  }

public:
};

class DriveAccCrawlState : public State<DriveAccCrawlState>
{
private:
  friend class State<DriveAccCrawlState>;
  DriveAccCrawlState(void)
  {
    StateName = "Crawl";
    StateNum = DRIVE_ACC_CRAWL_STATE;
    StateKind = ACC_STATE;
  }

public:
};

class DriveAccStopState : public State<DriveAccStopState>
{
private:
  friend class State<DriveAccStopState>;
  DriveAccStopState(void)
  {
    StateName = "Stop";
    StateNum = DRIVE_ACC_STOP_STATE;
    StateKind = ACC_STATE;
  }

public:
};

class DriveAccStopLineState : public State<DriveAccStopLineState>
{
private:
  friend class State<DriveAccStopLineState>;
  DriveAccStopLineState(void)
  {
    StateName = "StopLine";
    StateNum = DRIVE_ACC_STOPLINE_STATE;
    StateKind = ACC_STATE;
  }

public:
};

class DriveStrLeftState : public State<DriveStrLeftState>
{
private:
  friend class State<DriveStrLeftState>;
  DriveStrLeftState(void)
  {
    StateName = "Left Turn";
    StateNum = DRIVE_STR_LEFT_STATE;
    StateKind = STR_STATE;
  }

public:
};
class DriveStrRightState : public State<DriveStrRightState>
{
private:
  friend class State<DriveStrRightState>;
  DriveStrRightState(void)
  {
    StateName = "Right Turn";
    StateNum = DRIVE_STR_RIGHT_STATE;
    StateKind = STR_STATE;
  }

public:
};
class DriveStrStraightState : public State<DriveStrStraightState>
{
private:
  friend class State<DriveStrStraightState>;
  DriveStrStraightState(void)
  {
    StateName = "Straight";
    StateNum = DRIVE_STR_STRAIGHT_STATE;
    StateKind = STR_STATE;
  }

public:
};

// Planning to change str state
class DriveBehaviorLaneChangeLeftState : public State<DriveBehaviorLaneChangeLeftState>
{
private:
  friend class State<DriveBehaviorLaneChangeLeftState>;
  DriveBehaviorLaneChangeLeftState(void)
  {
    StateName = "LaneChangeLeft";
    StateNum = DRIVE_BEHAVIOR_LANECHANGE_LEFT_STATE;
    StateKind = BEHAVIOR_STATE;
  }

public:
};

// Planning to change str state
class DriveBehaviorLaneChangeRightState : public State<DriveBehaviorLaneChangeRightState>
{
private:
  friend class State<DriveBehaviorLaneChangeRightState>;
  DriveBehaviorLaneChangeRightState(void)
  {
    StateName = "LaneChangeRight";
    StateNum = DRIVE_BEHAVIOR_LANECHANGE_RIGHT_STATE;
    StateKind = BEHAVIOR_STATE;
  }

public:
};

class DriveBehaviorObstacleAvoidanceState : public State<DriveBehaviorObstacleAvoidanceState>
{
private:
  friend class State<DriveBehaviorObstacleAvoidanceState>;
  DriveBehaviorObstacleAvoidanceState(void)
  {
    StateName = "ObstacleAvoidance";
    StateNum = DRIVE_BEHAVIOR_OBSTACLE_AVOIDANCE_STATE;
    StateKind = BEHAVIOR_STATE;
  }

public:
};

class DriveBehaviorTrafficLightRedState : public State<DriveBehaviorTrafficLightRedState>
{
private:
  friend class State<DriveBehaviorTrafficLightRedState>;
  DriveBehaviorTrafficLightRedState(void)
  {
    StateName = "TrafficLightRed";
    StateNum = DRIVE_BEHAVIOR_TRAFFICLIGHT_RED_STATE;
    StateKind = BEHAVIOR_STATE;
  }

public:
};

class DriveBehaviorTrafficLightGreenState : public State<DriveBehaviorTrafficLightGreenState>
{
private:
  friend class State<DriveBehaviorTrafficLightGreenState>;
  DriveBehaviorTrafficLightGreenState(void)
  {
    StateName = "TrafficLightGreen";
    StateNum = DRIVE_BEHAVIOR_TRAFFICLIGHT_GREEN_STATE;
    StateKind = BEHAVIOR_STATE;
  }

public:
};

class DriveBehaviorStoplinePlanState : public State<DriveBehaviorStoplinePlanState>
{
private:
  friend class State<DriveBehaviorStoplinePlanState>;
  DriveBehaviorStoplinePlanState(void)
  {
    StateName = "STOPLINE_PLAN";
    StateNum = DRIVE_BEHAVIOR_STOPLINE_PLAN_STATE;
    StateKind = BEHAVIOR_STATE;
  }

public:
};

class DriveBehaviorAcceptLanechangeState : public State<DriveBehaviorAcceptLanechangeState>
{
private:
  friend class State<DriveBehaviorAcceptLanechangeState>;
  DriveBehaviorAcceptLanechangeState(void)
  {
    StateName = "AcceptLane-Change";
    StateNum = DRIVE_BEHAVIOR_ACCEPT_LANECHANGE_STATE;
    StateKind = BEHAVIOR_STATE;
  }

public:
};
}
#endif
