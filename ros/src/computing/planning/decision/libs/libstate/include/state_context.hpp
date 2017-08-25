#ifndef __STATE_CONTEXT_HPP__
#define __STATE_CONTEXT_HPP__

#include <atomic>
#include <iostream>
#include <thread>
#include <unordered_map>
#include <utility>
#include "state.hpp"
#include "state_drive.hpp"
#include "state_emg.hpp"

namespace state_machine
{
class StateContext
{
private:
  BaseState *state_;
  BaseState *sub_state;
  BaseState *sub_sub_state;
  std::unordered_map<uint64_t, BaseState *> StateStores;
  unsigned long long st;
  unsigned long long ChangeStateFlags;
  std::atomic<bool> thread_loop;

  std::thread *thr_state_dec;

  void ShowStateMove(unsigned long long _state_num)
  {
    std::cout << "State will be [" << *StateStores[_state_num]->GetStateName() << "]" << std::endl;
  }

public:
  void debug_next_state()
  {
    if (st >= STATE_END)
      st = 1;
    state_ = StateStores[st];
    st = st << 1;
  }
  StateContext(void)
  {
    st = 1;
    state_ = nullptr;
    sub_state = nullptr;
    sub_sub_state = nullptr;

    StateStores[START_STATE] = StartState::GetInstance();
    StateStores[INITIAL_STATE] = InitialState::GetInstance();
    StateStores[INITIAL_LOCATEVEHICLE_STATE] = LocateVehicleState::GetInstance();
    StateStores[DRIVE_STATE] = DriveState::GetInstance();
    StateStores[DRIVE_MOVEFWD_STATE] = DriveMoveFwdState::GetInstance();
    StateStores[DRIVE_MOVEFWD_LEFT_STATE] = DriveMoveFwdLeftState::GetInstance();
    StateStores[DRIVE_MOVEFWD_RIGHT_STATE] = DriveMoveFwdRightState::GetInstance();
    StateStores[DRIVE_MOVEFWD_STRAIGHT_STATE] = DriveMoveFwdStraightState::GetInstance();
    StateStores[DRIVE_LANECHANGE_STATE] = DriveLaneChangeState::GetInstance();
    StateStores[DRIVE_LANECHANGE_LEFT_STATE] = DriveLaneChangeLeftState::GetInstance();
    StateStores[DRIVE_LANECHANGE_RIGHT_STATE] = DriveLaneChangeRightState::GetInstance();
    StateStores[DRIVE_LANECHANGE_RIGHT_AVOIDANCE_STATE] = DriveLaneChangeRightAvoidanceState::GetInstance();
    StateStores[DRIVE_OBSTACLE_AVOIDANCE_STATE] = DriveObstacleAvoidanceState::GetInstance();
    StateStores[DRIVE_OBSTACLE_AVOIDANCE_STATIC_STATE] = DriveObstacleAvoidanceStaticState::GetInstance();
    StateStores[DRIVE_OBSTACLE_AVOIDANCE_DYNAMIC_STATE] = DriveObstacleAvoidanceDynamicState::GetInstance();
    StateStores[DRIVE_STOP_STATE] = DriveStopState::GetInstance();
    StateStores[DRIVE_STOP_AVOIDANCE_STATE] = DriveStopAvoidanceState::GetInstance();
    StateStores[DRIVE_STOP_STOPLINE_STATE] = DriveStopStopLineState::GetInstance();
    StateStores[DRIVE_STOP_TRAFFICLIGHT_STATE] = DriveStopTrafficLightState::GetInstance();
    StateStores[MISSION_COMPLETE_STATE] = MissionCompleteState::GetInstance();
    StateStores[EMERGENCY_STATE] = EmergencyState::GetInstance();
    StateStores[EMERGENCY_HW_STATE] = EmergencyHWState::GetInstance();
    StateStores[EMERGENCY_HWVEHICLE_STATE] = EmergencyHWVehicleState::GetInstance();
    StateStores[EMERGENCY_HWCONTROLLER_STATE] = EmergencyHWControllerState::GetInstance();
    StateStores[EMERGENCY_SW_STATE] = EmergencySWState::GetInstance();
    StateStores[EMERGENCY_SWAUTOWARE_STATE] = EmergencySWAutowareState::GetInstance();
    StateStores[EMERGENCY_SWCONTROLLER_STATE] = EmergencySWControllerState::GetInstance();

    ChangeStateFlags = 0;
    thread_loop = true;

    this->InitContext();
  }
  ~StateContext()
  {
    thread_loop = false;
  }

  bool isState(unsigned long long _state_num);
  bool inState(unsigned long long _state_num);
  
  void StateDecider(void);

 
  bool setCurrentState(BaseState *state);
  bool setCurrentState(BaseState *state, BaseState *substate);
  bool setCurrentState(BaseState *state, BaseState *substate, BaseState *subsubstate);
 
  BaseState *getCurrentState(void);
  std::unique_ptr<std::string> getCurrentStateName(void);
  void showCurrentStateName(void);
  
  BaseState *getStateObject(unsigned long long _state_num);
  void InitContext(void);

  bool TFInitialized(void);

  void handleTrafficLight(uint32_t _light_color);
  bool handleCurrentPose(double x, double y, double z, double roll, double pitch, double yaw);
  bool handlePointsRaw(bool _hasLidarData);

  bool handleIntersection(bool _hasIntersection, double _angle);
  bool handleTwistCmd(bool _hasTwistCmd);
};
}

#endif
