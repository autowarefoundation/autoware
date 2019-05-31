#ifndef __STATE_CONTEXT_HPP__
#define __STATE_CONTEXT_HPP__

#include <atomic>
#include <iostream>
#include <map>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <utility>

#include <state_machine_lib/state.hpp>

namespace state_machine
{
class StateContext
{
private:
  std::shared_ptr<State> root_state_;
  std::map<uint64_t, std::shared_ptr<State>> state_map_;
  std::mutex change_state_mutex_;

  void showStateMove(uint64_t _state_id)
  {
    std::cout << "State will be [" << state_map_[_state_id]->getStateName() << "]" << std::endl;
  }
  bool setCurrentState(State* state);

  void setParent(uint64_t child, uint64_t parent)
  {
    state_map_[child]->setParent(state_map_[parent]);
  }
  uint64_t parseChildState(const YAML::Node& node, uint64_t _id_counter, uint64_t _parent_id);
  int32_t getStateIDbyName(std::string _name);
  void setTransitionMap(const YAML::Node& node, const std::shared_ptr<State>& _state);

  std::shared_ptr<State> getStatePtr(const YAML::Node& node);
  std::shared_ptr<State> getStatePtr(const std::string& _state_name);
  std::shared_ptr<State> getStatePtr(const uint64_t& _state_id);

  bool isCurrentState(const std::string& state_name);

  std::string dot_output_name;

public:
  StateContext(const std::string& file_name, const std::string& msg_name)
  {
    createStateMap(file_name, msg_name);
    root_state_ = getStartState();
    dot_output_name = "/tmp/" + msg_name + ".dot";
    createDOTGraph(dot_output_name);
  }

  ~StateContext()
  {
  }

  void createStateMap(std::string _state_file_name, std::string _msg_name);
  std::shared_ptr<State> getStartState(void);

  void onUpdate(void);

  bool setCallback(const CallbackType& _type, const std::string& _state_name,
                   const std::function<void(const std::string&)>& _f);

  // visualize
  void createGraphTransitionList(std::ofstream& outputfile, int idx,
                                 std::map<uint64_t, std::vector<uint64_t>>& sublist);
  void createDOTGraph(std::string _file_name);

  std::string getStateText();
  std::string getAvailableTransition(void);
  void showStateName();
  void nextState(const std::string& transition_key);
};
}

#endif
