/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <memory>
#include <iostream>

namespace state_machine
{
enum class StateList : int32_t
{
  MOVE_FORWARD,
  TRAFFIC_LIGHT_STOP,
  LANE_CHANGE,
  STOP_SIGN_STOP,
  OBSTACLE_AVOIDANCE,

  MISSION_COMPLETE = 100,
  EMERGENCY = -1,
};

enum class TrafficLight : int32_t
{
  RED,
  GREEN,
  UNKNOWN,
};

enum class ChangeFlag : int32_t
{
  straight,
  right,
  left,

  unknown = -1,
};

template <class T>
typename std::underlying_type<T>::type enumToInteger(T t)
{
  return static_cast<typename std::underlying_type<T>::type>(t);
}

// Forward Decralation
class StateContext;

// abstract class for states
class BaseState
{
public:
  virtual ~BaseState() = default;
  virtual void update(StateContext *context) = 0;
  virtual int32_t getStateName()
  {
    return 0;
  };
  
  virtual std::unique_ptr<std::string> getStateNameString()
  {
    return 0;
  };
};

// State : MOVE_FORWARD
class StateMoveForward : public BaseState
{
public:
  void update(StateContext *context) override;
  int32_t getStateName() override
  {
    return enumToInteger(StateList::MOVE_FORWARD);
  }
  std::unique_ptr<std::string> getStateNameString() override
  {
    return std::unique_ptr<std::string>(new std::string("MOVE_FORWARD"));
  }
  static std::unique_ptr<BaseState> create()
  {
    return std::unique_ptr<BaseState>(new StateMoveForward);
  };

private:
  StateMoveForward() = default;
};

// State : TRAFFIC_LIGHT_STOP
class StateTrafficLightStop : public BaseState
{
public:
  void update(StateContext *context) override;
  int32_t getStateName() override
  {
    return enumToInteger(StateList::TRAFFIC_LIGHT_STOP);
  }
  std::unique_ptr<std::string> getStateNameString() override
  {
    return std::unique_ptr<std::string>(new std::string("TRAFFIC_LIGHT_STOP"));
  }
  static std::unique_ptr<BaseState> create()
  {
    return std::unique_ptr<BaseState>(new StateTrafficLightStop);
  };

private:
  StateTrafficLightStop() = default;
};

// State : LANE_CHANGE
class StateLaneChange : public BaseState
{
public:
  void update(StateContext *context) override;
  int32_t getStateName() override
  {
    return enumToInteger(StateList::LANE_CHANGE);
  }
  std::unique_ptr<std::string> getStateNameString() override
  {
    return std::unique_ptr<std::string>(new std::string("LANE_CHANGE"));
  }
  static std::unique_ptr<BaseState> create()
  {
    return std::unique_ptr<BaseState>(new StateLaneChange);
  };

private:
  StateLaneChange() = default;
};

// State : STOP_SIGN_STOP
class StateStopSignStop : public BaseState
{
 public:
  void update(StateContext *context) override;
  int32_t getStateName() override
  {
    return enumToInteger(StateList::STOP_SIGN_STOP);
  }
  std::unique_ptr<std::string> getStateNameString() override
  {
    return std::unique_ptr<std::string>(new std::string("STOP_SIGN_STOP"));
  }
  static std::unique_ptr<BaseState> create()
  {
    return std::unique_ptr<BaseState>(new StateStopSignStop);
  };

 private:
  StateStopSignStop() = default;
};

// State : Obstacle Avoidance
class StateObstacleAvoidance : public BaseState
{
 public:
  void update(StateContext *context) override;
  int32_t getStateName() override
  {
    return enumToInteger(StateList::STOP_SIGN_STOP);
  }
  std::unique_ptr<std::string> getStateNameString() override
  {
    return std::unique_ptr<std::string>(new std::string("OBSTACLE_AVOIDANCE"));
  }
  static std::unique_ptr<BaseState> create()
  {
    return std::unique_ptr<BaseState>(new StateObstacleAvoidance);
  };

 private:
  StateObstacleAvoidance() = default;
};

// State : EMERGENCY
class StateEmergency : public BaseState
{
public:
  void update(StateContext *context) override;
  int32_t getStateName() override
  {
    return enumToInteger(StateList::EMERGENCY);
  }
  std::unique_ptr<std::string> getStateNameString() override
  {
    return std::unique_ptr<std::string>(new std::string("EMERGENCY"));
  }
  static std::unique_ptr<BaseState> create()
  {
    return std::unique_ptr<BaseState>(new StateEmergency);
  };

private:
  StateEmergency() = default;
};

// State : MISSION_COMPLETE
class StateMissionComplete : public BaseState
{
public:
  void update(StateContext *context) override;
  int32_t getStateName() override
  {
    return enumToInteger(StateList::MISSION_COMPLETE);
  }
  std::unique_ptr<std::string> getStateNameString() override
  {
    return std::unique_ptr<std::string>(new std::string("MISSION_COMPLETE"));
  }
  static std::unique_ptr<BaseState> create()
  {
    return std::unique_ptr<BaseState>(new StateMissionComplete);
  };

private:
  StateMissionComplete() = default;
};

class StateContext
{
public:
  StateContext()
    : state_(StateMoveForward::create()), light_color_(TrafficLight::UNKNOWN), change_flag_(ChangeFlag::unknown){};
  void setState(std::unique_ptr<BaseState> newState)
  {
    state_ = std::move(newState);
  };
  void update()
  {
    state_->update(this);
  }
  void setLightColor(const int32_t &msg)
  {
    light_color_ = static_cast<TrafficLight>(msg);
  }
  void setChangeFlag(const int32_t &msg)
  {
    change_flag_ = static_cast<ChangeFlag>(msg);
  }

  TrafficLight getLightColor() const
  {
    return light_color_;
  }
  ChangeFlag getChangeFlag() const
  {
    return change_flag_;
  }
  int32_t getCurrentState() const
  {
    return state_->getStateName();
  }
  std::unique_ptr<std::string> getCurrentStateString() const
  {
    return state_->getStateNameString();
  }

private:
  std::unique_ptr<BaseState> state_;
  TrafficLight light_color_;
  ChangeFlag change_flag_;
};

}  // state_machine
#endif  // STATE_MACHINE_H
