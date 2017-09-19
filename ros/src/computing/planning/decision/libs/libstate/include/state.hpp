#ifndef __STATE_HPP__
#define __STATE_HPP__

#include <iostream>
#include <memory>
#include <vector>

#include "state_flags.hpp"

using namespace std;

namespace state_machine
{
class StartState;
class InitialState;
class LocateVehicleState;
class DriveState;
class DriveMoveFwdState;
class DriveMoveFwdLeftState;
class DriveMoveFwdRightState;
class DriveMoveFwdStraightState;
class DriveLaneChangeState;
class DriveLaneChangeLeftState;
class DriveLaneChangeRightState;
class DriveLaneChangeRightAvoidanceState;
class DriveObstacleAvoidanceState;
class DriveObstacleAvoidanceStaticState;
class DriveObstacleAvoidanceDynamicState;
class DriveStopState;
class DriveStopAvoidanceState;
class DriveStopStopLineState;
class DriveStopTrafficLightState;
class MissionCompleteState;
class EmergencyState;
class EmergencyHWState;
class EmergencyHWVehicleState;
class EmergencyHWControllerState;
class EmergencySWState;
class EmergencySWAutowareState;
class EmergencySWControllerState;

// base class
class BaseState
{
protected:
public:
  BaseState()
  {
  }
  virtual void ShowStateName(void) = 0;
  virtual unsigned long long GetStateTransMask() = 0;
  virtual unsigned long long GetStateNum() = 0;
  virtual std::unique_ptr<std::string> GetStateName() = 0;
};

// Interface
template <class T>
class State : public BaseState
{
protected:
  std::string StateName = "Base";
  unsigned long long StateNum;
  unsigned long long StateTransMask;

public:
  State()
  {
    StateNum = 0;
    StateTransMask = (unsigned long long)STATE_END - 1;
  }
  void ShowStateName(void)
  {
    std::cout << StateName << "-";
  }

  static T* GetInstance(void)
  {
    static T singleton;
    return &singleton;
  }

  std::unique_ptr<std::string> GetStateName(void)
  {
    return std::unique_ptr<std::string>(new std::string(StateName));
  }

  unsigned long long GetStateTransMask(void)
  {
    return StateTransMask;
  }
  unsigned long long GetStateNum(void)
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
  }

public:
};
}

#endif
