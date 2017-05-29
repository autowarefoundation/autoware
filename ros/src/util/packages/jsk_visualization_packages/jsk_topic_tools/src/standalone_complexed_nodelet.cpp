// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <nodelet/loader.h>
#include "jsk_topic_tools/rosparam_utils.h"
#include "jsk_topic_tools/log_utils.h"
#include <boost/algorithm/string.hpp>
#include <list>
// Parameter structure is
// nodelets:
//   -  name: node_name
//      type: nodelet_type
//      remappings:
//        - from: from_topic
//        - to: to_topic
//        - from: from_topic
//        - to: to_topic

std::string parentName(const std::string& name)
{
  std::list<std::string> list_string;
  std::string delim("/");
  boost::split(list_string, name, boost::is_any_of(delim));

  if (list_string.size() > 0) {
    list_string.pop_back();
  }
  return boost::algorithm::join(list_string, "/");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "standalone_complexed_nodelet");
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;
  nodelet::Loader manager(false); // Don't bring up the manager ROS API
  nodelet::V_string my_argv;  
  std::vector<std::string> candidate_root;
  candidate_root.push_back("nodelets");
  int candidate_num;
  private_nh.param("candidate_num", candidate_num, 100);
  for (size_t i = 0; i < candidate_num; i++) {
    candidate_root.push_back((boost::format("nodelets_%lu") % i).str());
  }
  for (size_t i_candidate = 0; i_candidate < candidate_root.size(); i_candidate++) {
    std::string root_name = candidate_root[i_candidate];
    if (private_nh.hasParam(root_name)) {
      XmlRpc::XmlRpcValue nodelets_values;
      private_nh.param(root_name, nodelets_values, nodelets_values);
      if (nodelets_values.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        for (size_t i_nodelet = 0; i_nodelet < nodelets_values.size(); i_nodelet++) {
          JSK_ROS_INFO("i_nodelet %lu", i_nodelet);
          XmlRpc::XmlRpcValue onenodelet_param = nodelets_values[i_nodelet];
          if (onenodelet_param.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
            std::string name, type;
            nodelet::M_string remappings;
            if (onenodelet_param.hasMember("if") && !(bool)onenodelet_param["if"]) {
              continue;
            }
            if (onenodelet_param.hasMember("unless") && (bool)onenodelet_param["unless"]) {
              continue;
            }
            if (onenodelet_param.hasMember("name")) {
              name = nh.resolveName((std::string)onenodelet_param["name"]);
            }
            else {
              JSK_ROS_FATAL("element ~nodelets should have name field");
              return 1;
            }
            if (onenodelet_param.hasMember("type")) {
              type = (std::string)onenodelet_param["type"];
            }
            else {
              JSK_ROS_FATAL("element ~nodelets should have type field");
              return 1;
            }
            if (onenodelet_param.hasMember("remappings")) {
              XmlRpc::XmlRpcValue remappings_params
                = onenodelet_param["remappings"];
              if (remappings_params.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                for (size_t remappings_i = 0; remappings_i < remappings_params.size(); remappings_i++) {
                  XmlRpc::XmlRpcValue remapping_element_param = remappings_params[remappings_i];
                  if (remapping_element_param.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                    if (remapping_element_param.hasMember("from") && remapping_element_param.hasMember("to")) {
                      std::string from = (std::string)remapping_element_param["from"];
                      std::string to = (std::string)remapping_element_param["to"];
                      if (from.size() > 0 && from[0] == '~') {
                        ros::NodeHandle nodelet_private_nh = ros::NodeHandle(name);
                        from = nodelet_private_nh.resolveName(from.substr(1, from.size() - 1));
                      }
                      else {
                        ros::NodeHandle nodelet_nh = ros::NodeHandle(parentName(name));
                        from = nodelet_nh.resolveName(from);
                      }
                      if (to.size() > 0 && to[0] == '~') {
                        ros::NodeHandle nodelet_private_nh = ros::NodeHandle(name);
                        to = nodelet_private_nh.resolveName(to.substr(1, to.size() - 1));
                      }
                      else {
                        ros::NodeHandle nodelet_nh = ros::NodeHandle(parentName(name));
                        to = nodelet_nh.resolveName(to);
                      }
                      JSK_ROS_INFO("remapping: %s => %s", from.c_str(), to.c_str());
                      remappings[from] = to;
                    }
                    else {
                      JSK_ROS_FATAL("remappings parameter requires from and to fields");
                      return 1;
                    }
                  }
                  else {
                    JSK_ROS_FATAL("remappings should be an array");
                    return 1;
                  }
                }
              }
              else {
                JSK_ROS_FATAL("remappings should be an array");
                return 1;
              }
            }
            // Done reading parmaeter for one nodelet
          
            if (!manager.load(name, type, remappings, my_argv)) {
              JSK_ROS_ERROR("Failed to load nodelet [%s -- %s]", name.c_str(), type.c_str());
            }
            else {
              JSK_ROS_INFO("Succeeded to load nodelet [%s -- %s]", name.c_str(), type.c_str());
            }
          }
          else {
            JSK_ROS_FATAL("element ~nodelets should be a dictionay");
            return 1;
          }
        }
      }
      else {
        JSK_ROS_FATAL("~nodelets should be a list");
        return 1;
      }
    }
  }
  JSK_ROS_INFO("done reading parmaeters");
  std::vector<std::string> loaded_nodelets = manager.listLoadedNodelets();
  JSK_ROS_INFO("loaded nodelets: %lu", loaded_nodelets.size());
  for (size_t i = 0; i < loaded_nodelets.size(); i++) {
    JSK_ROS_INFO("loaded nodelet: %s", loaded_nodelets[i].c_str());
  }
  ros::spin();
  return 0;
}
