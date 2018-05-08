#ifndef __STATE_MAIN_HPP__
#define __STATE_MAIN_HPP__

#include <iostream>
#include <memory>
#include <vector>

#include <state_machine_lib/state.hpp>

namespace state_machine
{
// StartState
class StartState : public State<StartState>
{
private:
  friend class State<StartState>;
  StartState(void)
  {
    StateName = "Start";
    StateNum = START_STATE;
    StateTransMask = (uint64_t)STATE_END - 1;
    StateKind = MAIN_STATE;
  }

public:
  virtual void update(void) override
  {
  }
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

// EmergencyState
class EmergencyState : public State<EmergencyState>
{
private:
  friend class State<EmergencyState>;
  EmergencyState(void)
  {
    StateName = "Emergency";
    StateTransMask = 0;
  }

public:
};
}

#endif
