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
    StateName = "Acceleration";
    StateNum = DRIVE_STATE | DRIVE_ACC_ACCELERATION_STATE;
    StateTransMask = DRIVE_STATE;
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
    StateName = "Deceleration";
    StateNum = DRIVE_STATE | DRIVE_ACC_DECELERATION_STATE;
    StateTransMask = DRIVE_STATE;
    StateKind = ACC_STATE;
  }

public:
  void ShowStateName(void)
  {
    std::cout << StateName << "-";
  }
};

class DriveAccKeepState : public State<DriveAccKeepState>
{
private:
  friend class State<DriveAccKeepState>;
  DriveAccKeepState(void)
  {
    StateName = "Keep";
    StateNum = DRIVE_STATE | DRIVE_ACC_KEEP_STATE;
    StateTransMask = DRIVE_STATE;
    StateKind = ACC_STATE;
  }

public:
  void ShowStateName(void)
  {
    std::cout << StateName << "-";
  }
};

class DriveAccStopState : public State<DriveAccStopState>
{
private:
  friend class State<DriveAccStopState>;
  DriveAccStopState(void)
  {
    StateName = "Stop";
    StateNum = DRIVE_STATE | DRIVE_ACC_STOP_STATE;
    StateTransMask = DRIVE_STATE;
    StateKind = ACC_STATE;
  }

public:
  void ShowStateName(void)
  {
    std::cout << StateName << "-";
  }
};

class DriveStrLeftState : public State<DriveStrLeftState>
{
private:
  friend class State<DriveStrLeftState>;
  DriveStrLeftState(void)
  {
    StateName = "Left Turn";
    StateNum = DRIVE_STATE | DRIVE_STR_LEFT_STATE;
    StateTransMask = DRIVE_STATE;
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
    StateNum = DRIVE_STATE | DRIVE_STR_RIGHT_STATE;
    StateTransMask = DRIVE_STATE;
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
    StateNum = DRIVE_STATE | DRIVE_STR_STRAIGHT_STATE;
    StateTransMask = DRIVE_STATE;
    StateKind = STR_STATE;
  }

public:
};

class DriveBehaviorLaneChangeLeftState : public State<DriveBehaviorLaneChangeLeftState>
{
private:
  friend class State<DriveBehaviorLaneChangeLeftState>;
  DriveBehaviorLaneChangeLeftState(void)
  {
    StateName = "LaneChangeLeft";
    StateNum = DRIVE_STATE | DRIVE_BEHAVIOR_LANECHANGE_LEFT_STATE;
    StateTransMask = DRIVE_STATE;
    StateKind = BEHAVIOR_STATE;
  }

public:
};

class DriveBehaviorLaneChangeRightState : public State<DriveBehaviorLaneChangeRightState>
{
private:
  friend class State<DriveBehaviorLaneChangeRightState>;
  DriveBehaviorLaneChangeRightState(void)
  {
    StateName = "LaneChangeRight";
    StateNum = DRIVE_STATE | DRIVE_BEHAVIOR_LANECHANGE_RIGHT_STATE;
    StateTransMask = DRIVE_STATE;
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
    StateTransMask = DRIVE_STATE;
    StateKind = BEHAVIOR_STATE;
  }

public:
};

class DriveDetectObstacleState : public State<DriveDetectObstacleState>
{
private:
  friend class State<DriveDetectObstacleState>;
  DriveDetectObstacleState(void)
  {
    StateName = "DetectObstacle";
    StateNum = DRIVE_STATE | DRIVE_DETECT_OBSTACLE_STATE;
    StateTransMask = DRIVE_STATE;
    StateKind = PERCEPTION_STATE;
  }

public:
};

class DriveDetectStoplineState : public State<DriveDetectStoplineState>
{
private:
  friend class State<DriveDetectStoplineState>;
  DriveDetectStoplineState(void)
  {
    StateName = "DetectStopline";
    StateNum = DRIVE_STATE | DRIVE_DETECT_STOPLINE_STATE;
    StateTransMask = DRIVE_STATE;
    StateKind = PERCEPTION_STATE;
  }

public:
};

class DriveDetectTrafficlightRedState : public State<DriveDetectTrafficlightRedState>
{
private:
  friend class State<DriveDetectTrafficlightRedState>;
  DriveDetectTrafficlightRedState(void)
  {
    StateName = "DetectTrafficLightRed";
    StateNum = DRIVE_STATE | DRIVE_DETECT_TRAFFICLIGHT_RED_STATE;
    StateTransMask = DRIVE_STATE;
    StateKind = PERCEPTION_STATE;
  }

public:
};
}

#endif
