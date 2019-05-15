#include <sched.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <cassert>
#include <memory>
#include <mutex>
#include <vector>
#include <limits>

#include <fstream>

#include <state_machine_lib/state.hpp>
#include <state_machine_lib/state_context.hpp>

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
bool StateContext::setCallback(const CallbackType& _type, const std::string& _state_name,
                               const std::function<void(const std::string&)>& _f)
{
  bool ret = false;
  int32_t _state_id = getStateIDbyName(_state_name);
  if (_state_id != -1 && getStatePtr(static_cast<uint64_t>(_state_id)))
  {
    switch (_type)
    {
      case CallbackType::UPDATE:
        getStatePtr(static_cast<uint64_t>(_state_id))->setCallbackUpdate(_f);
        ret = true;
        break;
      case CallbackType::ENTRY:
        getStatePtr(static_cast<uint64_t>(_state_id))->setCallbackEntry(_f);
        ret = true;
        break;
      case CallbackType::EXIT:
        getStatePtr(static_cast<uint64_t>(_state_id))->setCallbackExit(_f);
        ret = true;
        break;
      default:
        break;
    }
  }
  return ret;
}
/*****************************/
/*  Callback                 */
/*****************************/

void StateContext::onUpdate(void)
{
  root_state_->onUpdate();
}

bool StateContext::isCurrentState(const std::string& state_name)
{
  bool ret = false;
  std::shared_ptr<State> state = root_state_;
  do
  {
    if (state->getStateName() == state_name)
    {
      ret = true;
    }
    state = state->getChild();
  } while (state != nullptr);
  return ret;
}

void StateContext::nextState(const std::string& transition_key)
{
  std::shared_ptr<State> state = root_state_;
  std::string _target_state_name;
  std::vector<std::string> key_list;

  while (state)
  {
    if (state->getTransitionMap().count(transition_key) != 0)
    {
      const uint64_t transition_state_id = state->getTransitionMap().at(transition_key);
      _target_state_name = state_map_.at(transition_state_id)->getStateName();

      if (isCurrentState(_target_state_name))
      {
        return;
      }

      if (state_map_.at(transition_state_id)->getParent())
      {
        DEBUG_PRINT("[Child]:TransitionState: %d -> %d\n", state->getStateID(), transition_state_id);

        std::shared_ptr<State> in_state = root_state_;

        do
        {
          if (in_state == state_map_.at(transition_state_id)->getParent())
          {
            if (in_state->getChild())
            {
              key_list.push_back(in_state->getChild()->getEnteredKey());
              in_state->getChild()->onExit();
            }
            in_state->setChild(state_map_.at(transition_state_id));
            break;
          }
          in_state = in_state->getChild();
        } while (in_state);

#ifdef DEBUG
        createDOTGraph(dot_output_name);
#endif
        state_map_.at(transition_state_id)->setEnteredKey(transition_key);
        state_map_.at(transition_state_id)->onEntry();
      }
      else
      {
        DEBUG_PRINT("[Root]:TransitionState: %d -> %d\n", state->getStateID(), transition_state_id);
        state->onExit();

        root_state_ = state_map_.at(transition_state_id);
        root_state_->setChild(nullptr);
        root_state_->setParent(nullptr);
        root_state_->setEnteredKey(transition_key);
#ifdef DEBUG
        createDOTGraph(dot_output_name);
#endif

        root_state_->onEntry();
      }
      break;
    }
    state = state->getChild();
  }

  if (isCurrentState(_target_state_name))
  {
    showStateName();
  }
}

/*****************************/
/* Getter/Setter             */
/*****************************/

void StateContext::createGraphTransitionList(std::ofstream& outputfile, int idx,
                                             std::map<uint64_t, std::vector<uint64_t>>& sublist)
{
  if (!sublist[idx].empty() || state_map_.at(idx)->getParent() == NULL)
  {
    outputfile << "subgraph cluster_" << idx << "{\n"
               << "label = \"" << state_map_.at(idx)->getStateName() << "\";\n";
    if (!state_map_.at(idx)->getParent())
    {
      outputfile << "group = 1;\n";
    }

    for (auto& state : sublist[idx])
    {
      createGraphTransitionList(outputfile, state, sublist);
    }
  }

  for (auto& map : state_map_.at(idx)->getTransitionMap())
  {
    if ((state_map_.at(map.second)->getParent() == state_map_.at(idx)->getParent() ||
         state_map_.at(map.second)->getParent() == state_map_.at(idx)) &&
        state_map_.at(map.second)->getParent() != NULL)
    {
      outputfile << idx << "->" << map.second << " [label=\"" << map.first << "\"];\n";
    }
  }
  if (!sublist[idx].empty() || state_map_.at(idx)->getParent() == NULL)
  {
    outputfile << "}\n";
  }
  for (auto& map : state_map_.at(idx)->getTransitionMap())
  {
    if ((state_map_.at(map.second)->getParent() != state_map_.at(idx)->getParent() &&
         state_map_.at(map.second)->getParent() != state_map_.at(idx)) ||
        state_map_.at(map.second)->getParent() == NULL)
    {
      outputfile << idx << "->" << map.second << " [label=\"" << map.first << "\"];\n";
    }
  }
}

void StateContext::createDOTGraph(std::string _file_name)
{
  std::ofstream outputfile(_file_name);
  outputfile << "digraph state_machine_graph {\n"
             << "dpi = \"192\";\n node [style=filled];\n";
  std::vector<uint64_t> rootlist;
  std::map<uint64_t, std::vector<uint64_t>> sublist;
  std::map<uint64_t, int> layer_map;

  // create child list
  for (auto& state : state_map_)
  {
    outputfile << state.second->getStateID() << "[label=\"" << state.second->getStateName() << "\"";

    {
      std::shared_ptr<State> temp = root_state_;
      while (temp)
      {
        if (temp->getStateID() == state.second->getStateID())
        {
          outputfile << ",color = \"crimson\"";
        }
        temp = temp->getChild();
      }
    }
    if (state.second->getParent())
    {
      sublist[state.second->getParent()->getStateID()].push_back(state.second->getStateID());
    }
    else
    {
      outputfile << ", group = 1";
      rootlist.push_back(state.second->getStateID());
    }
    outputfile << "];\n";
  }

  for (auto& root_idx : rootlist)
  {
    int idx = root_idx;
    createGraphTransitionList(outputfile, idx, sublist);
  }

  outputfile << "}";
}

std::shared_ptr<State> StateContext::getStartState()
{
  for (auto& state : state_map_)
  {
    if (state.second->getStateName() == "Start")
    {
      return state.second;
    }
  }
  return nullptr;
}

int32_t StateContext::getStateIDbyName(std::string _name)
{
  for (const auto& i : state_map_)
  {
    if (i.second->getStateName() == _name)
    {
      return static_cast<int32_t>(i.second->getStateID());
    }
  }
  return -1;
}

std::string StateContext::getAvailableTransition(void)
{
  std::string text;

  std::shared_ptr<State> state = root_state_;
  do
  {
    for (const auto& keyval : state->getTransitionMap())
    {
      text += keyval.first + ":" + state_map_.at(keyval.second)->getStateName() + ",";
    }
    state = state->getChild();
  } while (state != nullptr);

  return text;
}

std::string StateContext::getStateText(void)
{
  std::string text;

  std::shared_ptr<State> state = root_state_;
  do
  {
    text += state->getStateName() + "\n";
    state = state->getChild();
  } while (state != nullptr);

  return text;
}

void StateContext::setTransitionMap(const YAML::Node& node, const std::shared_ptr<State>& _state)
{
  for (unsigned int j = 0; j < node.size(); j++)
  {
    int32_t state_id = getStateIDbyName(node[j]["Target"].as<std::string>());
    if (state_id == -1)
      continue;

    _state->addTransition(node[j]["Key"].as<std::string>(), static_cast<uint64_t>(state_id));
  }
}

void StateContext::showStateName()
{
  std::shared_ptr<State> state = root_state_;
  do
  {
    state = state->getChild();
  } while (state != nullptr);
}

std::shared_ptr<State> StateContext::getStatePtr(const YAML::Node& node)
{
  return getStatePtr(node["StateName"].as<std::string>());
}

std::shared_ptr<State> StateContext::getStatePtr(const std::string& _state_name)
{
  int32_t state_id = getStateIDbyName(_state_name);

  if (_state_name == "~" || state_id == -1)
    return nullptr;
  else
    return getStatePtr(static_cast<uint64_t>(state_id));
}

std::shared_ptr<State> StateContext::getStatePtr(const uint64_t& _state_id)
{
  return state_map_.at(_state_id);
}

void StateContext::createStateMap(std::string _state_file_name, std::string _msg_name)
{
  const YAML::Node StateYAML = YAML::LoadFile(_state_file_name)[_msg_name];

  // create state
  for (unsigned int i = 0; i < StateYAML.size(); i++)
  {
    state_map_[i] = std::shared_ptr<State>(new State(StateYAML[i]["StateName"].as<std::string>(), i));
  }

  // set Parent
  // set transition
  for (unsigned int i = 0; i < StateYAML.size(); i++)
  {
    getStatePtr(StateYAML[i])->setParent(getStatePtr(StateYAML[i]["Parent"].as<std::string>()));
    setTransitionMap(StateYAML[i]["Transition"], getStatePtr(StateYAML[i]));
  }
}
}
