/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
  MISSION_COMPLETE,

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

//Forward Decralation
class StateContext;

// abstract class for states
class BaseState
{
public:
  virtual ~BaseState() = default;
  virtual void update(StateContext *context) = 0;
  virtual int32_t getStateName() { return 0;};
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
  static std::unique_ptr<BaseState> create()
  {
    return std::unique_ptr<BaseState>(new StateLaneChange);
  };

 private:
  StateLaneChange() = default;
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
    : state_(StateMoveForward::create())
    , light_color_(TrafficLight::UNKNOWN)
  {};
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
  int32_t getCurrentState() const
  {
    return state_->getStateName();
  }

private:
  std::unique_ptr<BaseState> state_;
  TrafficLight light_color_;
  ChangeFlag change_flag_;
};

}  // state_machine
#endif  // STATE_MACHINE_H