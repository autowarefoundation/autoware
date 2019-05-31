#ifndef __STATE_HPP__
#define __STATE_HPP__

#include <cassert>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <state_machine_lib/state_common.hpp>
#include <state_machine_lib/state_flags.hpp>

namespace state_machine
{
class State
{
private:
  std::shared_ptr<State> child_state_;

  std::string state_name_;
  uint64_t state_id_;

  std::shared_ptr<State> parent_state_;

  std::function<void(const std::string&)> CallbackUpdateFunc;
  std::function<void(const std::string&)> CallbackEntryFunc;
  std::function<void(const std::string&)> CallbackExitFunc;

  std::map<std::string, uint64_t> transition_map_;

  std::string entered_key_;

  void parseChildState(YAML::Node& node);

public:
  State(std::string _state_name, uint64_t _state_id, std::shared_ptr<State> _parent_state = NULL)
    : child_state_(NULL), state_name_(_state_name), state_id_(_state_id), parent_state_(_parent_state), entered_key_("")
  {
#if 0
    CallbackUpdateFunc =
        [&](const std::string &arg) { /*DEBUG_PRINT("[%s]:%s is not registered\n", state_name_.c_str(), "update");*/ };
    CallbackEntryFunc =
        [&](const std::string &arg) { /*DEBUG_PRINT("[%s]:%s is not registered\n", state_name_.c_str(), "entry");*/ };
    CallbackExitFunc =
        [&](const std::string &arg) { /*DEBUG_PRINT("[%s]:%s is not registered\n", state_name_.c_str(), "exit");*/ };
#endif
  }
  ~State()
  {
  }

  void onUpdate(void)
  {
    if (child_state_)
      child_state_->onUpdate();

    if (CallbackUpdateFunc)
      CallbackUpdateFunc(state_name_);
  }

  void onEntry(void)
  {
    DEBUG_PRINT("[%s:Entry]\n", state_name_.c_str());

    if (CallbackEntryFunc)
      CallbackEntryFunc(state_name_);
  }
  void onExit(void)
  {
    DEBUG_PRINT("[%s:Exit]\n", state_name_.c_str());
    if (child_state_)
      child_state_->onExit();

    setChild(nullptr);

    if (CallbackExitFunc)
      CallbackExitFunc(state_name_);
  }
  void setCallbackUpdate(std::function<void(const std::string&)> _f)
  {
    CallbackUpdateFunc = _f;
  }
  void setCallbackEntry(std::function<void(const std::string&)> _f)
  {
    CallbackEntryFunc = _f;
  }

  void setCallbackExit(std::function<void(const std::string&)> _f)
  {
    CallbackExitFunc = _f;
  }

  void showStateName(void)
  {
    std::cout << state_name_ << "-";
  }
  void setParent(std::shared_ptr<State> _parent)
  {
    assert(this != _parent.get());
    parent_state_ = _parent;
  }

  void setChild(std::shared_ptr<State> _child)
  {
    assert(this != _child.get());
    child_state_ = _child;
  }

  std::shared_ptr<State> getParent()
  {
    return parent_state_;
  }
  std::shared_ptr<State> getChild()
  {
    return child_state_;
  }
  std::string getStateName(void)
  {
    return std::string(state_name_);
  }

  void addTransition(const std::string key, const uint64_t val)
  {
    transition_map_[key] = val;
  }

  uint64_t getTansitionVal(std::string key) const
  {
    return transition_map_.at(key);
  }

  const std::map<std::string, uint64_t>& getTransitionMap(void)
  {
    return transition_map_;
  }

  uint64_t getStateID(void)
  {
    return state_id_;
  }
  void setEnteredKey(const std::string& key)
  {
    entered_key_ = key;
  }
  std::string getEnteredKey(void)
  {
    return entered_key_;
  }
};
}

#endif
