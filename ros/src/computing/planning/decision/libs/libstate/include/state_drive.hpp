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
  }

public:
};

class DriveMoveFwdState : public State<DriveMoveFwdState>
{
private:
  friend class State<DriveMoveFwdState>;
  DriveMoveFwdState(void)
  {
    StateName = "MoveFwd";
    StateNum = DRIVE_STATE | DRIVE_MOVEFWD_STATE;
    StateTransMask = 0;
  }

public:
  void ShowStateName(void)
  {
    std::cout << StateName << "-";
  }
};

class DriveMoveFwdLeftState : public State<DriveMoveFwdLeftState>
{
private:
  friend class State<DriveMoveFwdLeftState>;
  DriveMoveFwdLeftState(void)
  {
    StateName = "MoveFwdLeft";
    StateNum = DRIVE_STATE | DRIVE_MOVEFWD_LEFT_STATE;
    StateTransMask = DRIVE_STATE | DRIVE_MOVEFWD_STATE;
  }

public:
};
class DriveMoveFwdRightState : public State<DriveMoveFwdRightState>
{
private:
  friend class State<DriveMoveFwdRightState>;
  DriveMoveFwdRightState(void)
  {
    StateName = "MoveFwdRight";
    StateNum = DRIVE_STATE | DRIVE_MOVEFWD_STATE | DRIVE_MOVEFWD_RIGHT_STATE;
    StateTransMask = DRIVE_STATE | DRIVE_MOVEFWD_STATE;
  }

public:
};
class DriveMoveFwdStraightState : public State<DriveMoveFwdStraightState>
{
private:
  friend class State<DriveMoveFwdStraightState>;
  DriveMoveFwdStraightState(void)
  {
    StateName = "MoveFwdStraight";
    StateNum = DRIVE_STATE | DRIVE_MOVEFWD_STATE | DRIVE_MOVEFWD_STRAIGHT_STATE;
    StateTransMask = DRIVE_STATE;
  }

public:
};

class DriveLaneChangeState : public State<DriveLaneChangeState>
{
private:
  friend class State<DriveLaneChangeState>;
  DriveLaneChangeState(void)
  {
    StateName = "LaneChange";
    StateNum = DRIVE_STATE | DRIVE_LANECHANGE_STATE;
    StateTransMask = DRIVE_STATE;
  }

public:
};

class DriveLaneChangeLeftState : public State<DriveLaneChangeLeftState>
{
private:
  friend class State<DriveLaneChangeLeftState>;
  DriveLaneChangeLeftState(void)
  {
    StateName = "LaneChangeLeft";
    StateNum = DRIVE_STATE | DRIVE_LANECHANGE_STATE | DRIVE_LANECHANGE_LEFT_STATE;
    StateTransMask = DRIVE_STATE | DRIVE_LANECHANGE_STATE;
  }

public:
};

class DriveLaneChangeRightState : public State<DriveLaneChangeRightState>
{
private:
  friend class State<DriveLaneChangeRightState>;
  DriveLaneChangeRightState(void)
  {
    StateName = "LaneChangeRight";
    StateNum = DRIVE_STATE | DRIVE_LANECHANGE_STATE | DRIVE_LANECHANGE_RIGHT_STATE;
    StateTransMask = DRIVE_STATE | DRIVE_LANECHANGE_STATE;
  }

public:
};

class DriveLaneChangeRightAvoidanceState : public State<DriveLaneChangeRightAvoidanceState>
{
private:
  friend class State<DriveLaneChangeRightAvoidanceState>;
  DriveLaneChangeRightAvoidanceState(void)
  {
    StateName = "LaneChangeRightAvoidanceState";
    StateNum = DRIVE_STATE | DRIVE_LANECHANGE_STATE | DRIVE_LANECHANGE_RIGHT_STATE |  DRIVE_LANECHANGE_RIGHT_AVOIDANCE_STATE;
    StateTransMask = DRIVE_STATE | DRIVE_LANECHANGE_STATE;
  }

public:
};

class DriveObstacleAvoidanceState : public State<DriveObstacleAvoidanceState>
{
private:
  friend class State<DriveObstacleAvoidanceState>;
  DriveObstacleAvoidanceState(void)
  {
    StateName = "ObstacleAvoidance";
    StateTransMask = 0;
  }

public:
};

class DriveObstacleAvoidanceStaticState : public State<DriveObstacleAvoidanceStaticState>
{
private:
  friend class State<DriveObstacleAvoidanceStaticState>;
  DriveObstacleAvoidanceStaticState(void)
  {
    StateName = "ObstacleAvoidanceStatic";
    StateTransMask = 0;
  }

public:
};

class DriveObstacleAvoidanceDynamicState : public State<DriveObstacleAvoidanceDynamicState>
{
private:
  friend class State<DriveObstacleAvoidanceDynamicState>;
  DriveObstacleAvoidanceDynamicState(void)
  {
    StateName = "ObstacleAvoidanceDynamic";
    StateTransMask = 0;
  }

public:
};

// DriveStopState
class DriveStopState : public State<DriveStopState>
{
private:
  friend class State<DriveStopState>;
  DriveStopState(void)
  {
    StateName = "DriveStop";
    StateNum  = DRIVE_STATE | DRIVE_STOP_STATE;
    StateTransMask = DRIVE_STATE;
  }

public:
};

// DriveStopAvoidanceState
class DriveStopAvoidanceState : public State<DriveStopAvoidanceState>
{
private:
  friend class State<DriveStopAvoidanceState>;
  DriveStopAvoidanceState(void)
  {
    StateName = "DriveStopAvoidance";
    StateNum  = DRIVE_STATE | DRIVE_STOP_STATE | DRIVE_STOP_AVOIDANCE_STATE;
    StateTransMask = DRIVE_STATE;
  }

public:
};

// DriveStopStopLineState
class DriveStopStopLineState : public State<DriveStopStopLineState>
{
private:
  friend class State<DriveStopStopLineState>;
  DriveStopStopLineState(void)
  {
    StateName = "DriveStopStopLine";
    StateNum  = DRIVE_STATE | DRIVE_STOP_STATE | DRIVE_STOP_STOPLINE_STATE;
    StateTransMask = DRIVE_STATE;
  }

public:
};
class DriveStopTrafficLightState : public State<DriveStopTrafficLightState>
{
private:
  friend class State<DriveStopTrafficLightState>;
  DriveStopTrafficLightState(void)
  {
    StateName = "DriveStopTrafficLight";
    StateNum  = DRIVE_STATE | DRIVE_STOP_STATE | DRIVE_STOP_TRAFFICLIGHT_STATE;
    StateTransMask = DRIVE_STATE;
  }

public:
};
}

#endif
