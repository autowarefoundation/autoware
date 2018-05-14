#include <sched.h>
#include <sys/types.h>
#include <unistd.h>
#include <thread>
#include <vector>

#include <cassert>
#include <mutex>

#include <state_machine_lib/state_context.hpp>

#include <state_machine_lib/state.hpp>
#include <state_machine_lib/state_common.hpp>

/**
 *
 * @file StateContext.cpp
 * @brief state context class
 * @author Yusuke Fujii
 * @date  2017/07/25
 *
 **/

namespace state_machine
{
std::vector<BaseState *> StateContext::getMultipleStates(uint64_t _state_num_set)
{
  std::vector<BaseState *> ret;
  for (uint64_t mask = STATE_SUB_END; _state_num_set != 0 && mask != 0; mask >>= 1)
  {
    if (mask & _state_num_set)
    {
      ret.push_back(getStateObject(mask));
      _state_num_set &= ~mask;
    }
  }
  return ret;
}

void StateContext::update(void)
{
  for (auto &p : HolderMap)
  {
    if (p.first == BEHAVIOR_STATE)
    {
      for (auto &&state : getMultipleStates(p.second))
      {
        state->update();
      }
    }
    else
    {
      if (p.second)
        if (getStateObject(p.second))
          getStateObject(p.second)->update();
    }
  }
}

void StateContext::inState(uint8_t _kind, uint64_t _prev_state_num)
{
  if (_kind > UNKNOWN_STATE)
  {
    return;
  }
  else if (_kind == BEHAVIOR_STATE)
  {
    for (auto &&state : getMultipleStates(HolderMap[_kind]))
    {
      if (!(_prev_state_num & getStateNum(state)))
        state->inState();
    }
  }
  else
  {
    if (getStateObject(HolderMap[_kind]))
      getStateObject(HolderMap[_kind])->inState();
  }
}

BaseState *StateContext::getStateObject(const uint64_t &_state_num)
{
  if (_state_num)
  {
    if (StateStores[_state_num])
    {
      return StateStores[_state_num];
    }
  }
  return nullptr;
}

bool StateContext::setCallbackUpdateFunc(const uint64_t &_state_num, const std::function<void(void)> &_f)
{
  bool ret = false;
  if (getStateObject(_state_num))
  {
    getStateObject(_state_num)->setCallbackUpdateFunc(_f);
    ret = true;
  }

  return ret;
}
bool StateContext::setCallbackInFunc(const uint64_t &_state_num, const std::function<void(void)> &_f)
{
  bool ret = false;
  if (getStateObject(_state_num))
  {
    getStateObject(_state_num)->setCallbackInFunc(_f);
    ret = true;
  }
  return ret;
}
bool StateContext::setCallbackOutFunc(const uint64_t &_state_num, const std::function<void(void)> &_f)
{
  bool ret = false;
  if (getStateObject(_state_num))
  {
    getStateObject(_state_num)->setCallbackOutFunc(_f);
    ret = true;
  }
  return ret;
}

std::string StateContext::getStateName(const uint64_t &_state_num)
{
  if (getStateObject(_state_num))
  {
    return getStateObject(_state_num)->getStateName();
  }
  else
    return "";
}

uint64_t StateContext::getStateNum(BaseState *_state)
{
  if (_state)
  {
    return _state->getStateNum();
  }
  else
    return 0;
}

uint8_t StateContext::getStateKind(BaseState *_state)
{
  if (_state)
  {
    return _state->getStateKind();
  }
  else
    return UNKNOWN_STATE;
}

uint8_t StateContext::getStateKind(const uint64_t &_state_num)
{
  if (_state_num)
  {
    return getStateKind(getStateObject(_state_num));
  }
  return UNKNOWN_STATE;
}

void StateContext::showCurrentStateName(void)
{
  for (auto &p : HolderMap)
  {
    if (p.second)
      if (getStateObject(p.second))
        getStateObject(p.second)->showStateName();
  }
}

bool StateContext::isDifferentState(uint64_t _state_a, uint64_t _state_b)
{
  return !(_state_a & _state_b);
}

bool StateContext::isEmptyMainState(void)
{
  if (HolderMap[MAIN_STATE])
    return false;
  return true;
}

uint64_t StateContext::getStateTransMask(BaseState *_state)
{
  if (_state)
    return _state->getStateTransMask();
  else
    return 0;
}

bool StateContext::isMainState(BaseState *_state)
{
  return getStateKind(_state) == MAIN_STATE;
}

bool StateContext::setCurrentState(BaseState *_state)
{
  change_state_mutex.lock();
  bool ret = true;
  if (_state)
  {
    uint64_t prev_state = HolderMap[getStateKind(_state)];
    bool diff = isDifferentState(getStateNum(_state), HolderMap[getStateKind(_state)]);
    if (isMainState(_state))
    {
      if (isEmptyMainState() || enableForceSetState || (getStateTransMask(_state) & HolderMap[MAIN_STATE]))
      {
        for (auto &state : HolderMap)
        {
          if (state.first == MAIN_STATE)
          {
            state.second = getStateNum(_state);
          }
          else
          {
            state.second = 0ULL;
          }
        }
      }
      else
      {
        ret = false;
      }
    }
    else
    {
      if (getStateKind(_state) == BEHAVIOR_STATE)
      {
        HolderMap[getStateKind(_state)] |= getStateNum(_state);
      }
      else
      {
        HolderMap[getStateKind(_state)] = getStateNum(_state);
      }
    }
    change_state_mutex.unlock();
    if (ret && diff)
    {
      this->inState(getStateKind(_state), prev_state);

      if (getStateKind(_state) == getStateKind(prev_state) && getStateKind(_state) != BEHAVIOR_STATE)
        if (getStateObject(prev_state))
          getStateObject(prev_state)->outState();
    }
  }
  else
  {
    change_state_mutex.unlock();
    ret = false;
  }
  return ret;
}

bool StateContext::setCurrentState(uint64_t flag)
{
  bool ret = this->setCurrentState(getStateObject(flag));
  return ret;
}

bool StateContext::setEnableForceSetState(bool force_flag)
{
  enableForceSetState = force_flag;
  return true;
}

std::string StateContext::getCurrentStateName(void)
{
  return this->getCurrentStateName(MAIN_STATE);
}

std::string StateContext::getCurrentStateName(uint8_t _kind)
{
  if (HolderMap[_kind])
  {
    if (_kind == BEHAVIOR_STATE)
    {
      uint64_t _current_state = HolderMap[_kind];
      std::string ret = "";
      for (uint64_t mask = STATE_SUB_END; _current_state != 0 && mask != 0; mask >>= 1)
      {
        if (mask & _current_state)
        {
          ret += "\n" + getStateName(mask);
          _current_state &= ~mask;
        }
      }
      return ret;
    }
    else
    {
      return getStateName(HolderMap[_kind]);
    }
  }
  return std::string("");
}

BaseState *StateContext::getCurrentMainState(void)
{
  return getStateObject(HolderMap[MAIN_STATE]);
}

bool StateContext::disableCurrentState(uint64_t _state_num)
{
  if (isMainState(getStateObject(_state_num)))
  {
    return false;
  }
  if (isCurrentState(_state_num))
  {
    HolderMap[getStateKind(_state_num)] &= ~_state_num;
    getStateObject(_state_num)->outState();
    return true;
  }
  else
  {
    return false;
  }
}

bool StateContext::isCurrentState(uint64_t _state_num)
{
  if (_state_num)
    return HolderMap[getStateKind(_state_num)] & _state_num;
  else
    return false;
}

bool StateContext::isState(BaseState *base, uint64_t _state_num)
{
  return base ? base->getStateNum() & _state_num ? true : false : false;
}

bool StateContext::handleIntersection(bool _hasIntersection, double _angle)
{
  /* deprecated */
  return false;
}

std::string StateContext::createStateMessageText(void)
{
  std::string ret;

  for (auto &p : HolderMap)
  {
    if (p.second)
    {
      ret = ret + "\n" + getStateName(p.second);
    }
  }
  return ret;
}

bool StateContext::handleTwistCmd(bool _hasTwistCmd)
{
  if (_hasTwistCmd)
    return this->setCurrentState(DRIVE_STATE);
  else
    return false;
}

void StateContext::stateDecider(void)
{
  // not running
}

void StateContext::InitContext(void)
{
  thr_state_dec = new std::thread(&StateContext::stateDecider, this);
  thr_state_dec->detach();
  this->setCurrentState(START_STATE);
  return;
}
bool StateContext::TFInitialized(void)
{
  return this->setCurrentState(INITIAL_STATE);
}
}
