#ifndef __STATE_HPP__
#define __STATE_HPP__

#include <iostream>
#include <memory>
#include <vector>

#include <state_flags.hpp>

namespace state_machine
{
class StartState;
class InitialState;
class LocateVehicleState;

class DriveState;
// Accel/Brake subState
class DriveAccAccelerationState;
class DriveAccDecelerationState;
class DriveAccKeepState;
class DriveAccStopState;
// Steering subState
class DriveStrStraightState;
class DriveStrLeftTurnState;
class DriveStrRightTurnState;

// Behavior subState
class DriveBehaviorLaneChangeLeftState;
class DriveBehaviorLaneChangeRightState;
class DriveBehaviorObstacleAvoidanceState;

// Perception subState
class DriveDetectObstacleState;
class DriveDetectStoplineState;
class DriveDetectTrafficlightRedState;

class MissionCompleteState;
class EmergencyState;

// base class
class BaseState
{
protected:
public:
  BaseState()
  {
  }
  virtual void showStateName(void) = 0;
  virtual unsigned long long getStateTransMask() = 0;
  virtual unsigned long long getStateNum() = 0;
  virtual std::string getStateName() = 0;
  virtual unsigned char getStateKind() = 0;
};

// Interface
template <class T>
class State : public BaseState
{
protected:
  std::string StateName = "Base";
  unsigned long long StateNum;
  unsigned long long StateTransMask;
  unsigned char StateKind;

public:
  State()
  {
    StateNum = 0;
    StateTransMask = (unsigned long long)STATE_END - 1;
    StateKind = UNKNOWN_STATE;
  }
  void showStateName(void)
  {
    std::cout << StateName << "-";
  }

  static T* getInstance(void)
  {
    static T singleton;
    return &singleton;
  }

  std::string getStateName(void)
  {
    return std::string(StateName);
  }

  unsigned char getStateKind(void)
  {
    return StateKind;
  }

  unsigned long long getStateTransMask(void)
  {
    return StateTransMask;
  }
  unsigned long long getStateNum(void)
  {
    return StateNum;
  }
};

// StartState
class StartState : public State<StartState>
{
private:
  friend class State<StartState>;
  StartState(void)
  {
    StateName = "Start";
    StateNum = START_STATE;
    StateTransMask = (unsigned long long)STATE_END - 1;
    StateKind = MAIN_STATE;
  }

public:
};

// InitialState
class InitialState : public State<InitialState>
{
private:
  friend class State<InitialState>;
  InitialState(void)
  {
    StateName = "Initial";
    StateNum = StateTransMask = INITIAL_STATE;
    StateTransMask |= START_STATE | EMERGENCY_STATE | MISSION_COMPLETE_STATE;
    StateKind = MAIN_STATE;
  }

public:
};
class LocateVehicleState : public State<LocateVehicleState>
{
private:
  friend class State<LocateVehicleState>;
  LocateVehicleState(void)
  {
    StateName = "Locate Vehicle";
    StateNum = StateTransMask = INITIAL_LOCATEVEHICLE_STATE;
    StateTransMask |= INITIAL_STATE;
    StateKind = MAIN_STATE;
  }

public:
};
// MissionCompleteState
class MissionCompleteState : public State<MissionCompleteState>
{
private:
  friend class State<MissionCompleteState>;
  MissionCompleteState(void)
  {
    StateName = "MissionComplete";
    StateNum = MISSION_COMPLETE_STATE;
    StateTransMask = DRIVE_STATE;
    StateKind = MAIN_STATE;
  }

public:
};
}

#endif
