// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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

#include "jsk_recognition_utils/pcl_util.h"
#include <set>
#include <algorithm>

namespace jsk_recognition_utils
{
  //static boost::mutex global_chull_mutex;
  boost::mutex global_chull_mutex;

  Eigen::Affine3f affineFromYAMLNode(const YAML::Node& pose)
  {
    float x, y, z, rx, ry, rz, rw;
#ifdef USE_OLD_YAML
    pose[0] >> x; pose[1] >> y; pose[2] >> z;
    pose[3] >> rx; pose[4] >> ry; pose[5] >> rz; pose[6] >> rw;
#else
    x = pose[0].as<float>(); y = pose[1].as<float>(); z = pose[2].as<float>();
    rx= pose[3].as<float>(); ry= pose[4].as<float>(); rz= pose[5].as<float>(); rw = pose[6].as<float>();
#endif
    Eigen::Vector3f p(x, y, z);
    Eigen::Quaternionf q(rw, rx, ry, rz);
    Eigen::Affine3f trans = Eigen::Translation3f(p) * Eigen::AngleAxisf(q);
    return trans;
  }
  
  
  std::vector<int> addIndices(const std::vector<int>& a,
                              const std::vector<int>& b)
  {
    std::set<int> all(b.begin(), b.end());
    for (size_t i = 0; i < a.size(); i++) {
      all.insert(a[i]);
    }
    return std::vector<int>(all.begin(), all.end());
  }

  pcl::PointIndices::Ptr addIndices(const pcl::PointIndices& a,
                                    const pcl::PointIndices& b)
  {
    std::vector<int> indices = addIndices(a.indices, b.indices);
    pcl::PointIndices::Ptr ret(new pcl::PointIndices);
    ret->indices = indices;
    return ret;
  }

  std::vector<int> subIndices(const std::vector<int>& a,
                              const std::vector<int>& b)
  {
    std::set<int> all(a.begin(), a.end());
    for (size_t i = 0; i < b.size(); i++) {
      std::set<int>::iterator it = all.find(b[i]);
      if (it != all.end()) {
        all.erase(it);
      }
    }
    return std::vector<int>(all.begin(), all.end());
  }
  
  pcl::PointIndices::Ptr subIndices(const pcl::PointIndices& a,
                                    const pcl::PointIndices& b)
  {
    std::vector<int> indices = subIndices(a.indices, b.indices);
    pcl::PointIndices::Ptr ret(new pcl::PointIndices);
    ret->indices = indices;
    return ret;
  }
  
  void Counter::add(double v)
  {
    acc_(v);
  }
  
  double Counter::mean()
  {
    return boost::accumulators::mean(acc_);
  }

  double Counter::min()
  {
    return boost::accumulators::min(acc_);
  }

  double Counter::max()
  {
    return boost::accumulators::max(acc_);
  }

  int Counter::count()
  {
    return boost::accumulators::count(acc_);
  }
  
  double Counter::variance()
  {
    return boost::accumulators::variance(acc_);
  }

  void buildGroupFromGraphMap(IntegerGraphMap graph_map,
                              const int from_index_arg,
                              std::vector<int>& to_indices_arg,
                              std::set<int>& output_set)
  {
    // convert graph_map into one-directional representation
    IntegerGraphMap onedirectional_map(graph_map);
    for (IntegerGraphMap::iterator it = onedirectional_map.begin();
         it != onedirectional_map.end();
         ++it) {
      int from_index = it->first;
      std::vector<int> to_indices = it->second;
      for (size_t i = 0; i < to_indices.size(); i++) {
        int to_index = to_indices[i];
        if (onedirectional_map.find(to_index) == onedirectional_map.end()) {
          // not yet initialized
          onedirectional_map[to_index] = std::vector<int>(); 
        }
        if (std::find(onedirectional_map[to_index].begin(),
                      onedirectional_map[to_index].end(),
                      from_index) == onedirectional_map[to_index].end()) {
          onedirectional_map[to_index].push_back(from_index);
        }
      }
    }
    _buildGroupFromGraphMap(onedirectional_map,
                            from_index_arg,
                            to_indices_arg,
                            output_set);
  }
  
  void _buildGroupFromGraphMap(IntegerGraphMap graph_map,
                               const int from_index,
                               std::vector<int>& to_indices,
                               std::set<int>& output_set)
  {    
    output_set.insert(from_index);
    for (size_t i = 0; i < to_indices.size(); i++) {
      int to_index = to_indices[i];
      if (output_set.find(to_index) == output_set.end()) {
        output_set.insert(to_index);
        //std::cout << "__connection__: " << from_index << " --> " << to_index << std::endl;
        std::vector<int> next_indices = graph_map[to_index];
        _buildGroupFromGraphMap(graph_map,
                               to_index,
                               next_indices,
                               output_set);
      }
    }
  }

  void buildAllGroupsSetFromGraphMap(IntegerGraphMap graph_map,
                                     std::vector<std::set<int> >& output_sets)
  {
    std::set<int> duplication_check_set;
    for (IntegerGraphMap::iterator it = graph_map.begin();
         it != graph_map.end();
         ++it) {
      int from_index = it->first;
      if (duplication_check_set.find(from_index)
          == duplication_check_set.end()) {
        std::set<int> new_graph_set;
        buildGroupFromGraphMap(graph_map, from_index, it->second,
                               new_graph_set);
        output_sets.push_back(new_graph_set);
        // update duplication_check_set
        addSet<int>(duplication_check_set, new_graph_set);
      }
    }
  }

  void addDiagnosticInformation(
    const std::string& string_prefix,
    jsk_topic_tools::TimeAccumulator& accumulator,
    diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    stat.add(string_prefix + " (Avg.)", accumulator.mean());
    if (accumulator.mean() != 0.0) {
      stat.add(string_prefix + " (Avg., fps)", 1.0 / accumulator.mean());
    }
    stat.add(string_prefix + " (Max)", accumulator.max());
    stat.add(string_prefix + " (Min)", accumulator.min());
    stat.add(string_prefix + " (Var.)", accumulator.variance());
  }

  void addDiagnosticErrorSummary(
    const std::string& string_prefix,
    jsk_topic_tools::VitalChecker::Ptr vital_checker,
    diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    stat.summary(
      diagnostic_msgs::DiagnosticStatus::ERROR,
      (boost::format("%s not running for %f sec")
       % string_prefix % vital_checker->deadSec()).str());
  }
  
  void addDiagnosticBooleanStat(
    const std::string& string_prefix,
    const bool value,
    diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    if (value) {
      stat.add(string_prefix, "True");
    }
    else {
      stat.add(string_prefix, "False");
    }
  }

  SeriesedBoolean::SeriesedBoolean(const int buf_len):
    buf_(buf_len)
  {
  }
  
  SeriesedBoolean::~SeriesedBoolean()
  {
  }

  void SeriesedBoolean::addValue(bool val)
  {
    buf_.push_front(val);
  }
  
  bool SeriesedBoolean::getValue()
  {
    if (buf_.size() == 0) {
      return false;
    }
    else {
      for (boost::circular_buffer<bool>::iterator it = buf_.begin();
           it != buf_.end();
           ++it) {
        if (!*it) {
          return false;
        }
      }
      return true;
    }
  }

  TimeredDiagnosticUpdater::TimeredDiagnosticUpdater(
    ros::NodeHandle& nh,
    const ros::Duration& timer_duration):
    diagnostic_updater_(new diagnostic_updater::Updater)
  {
    timer_ = nh.createTimer(
      timer_duration, boost::bind(
        &TimeredDiagnosticUpdater::timerCallback,
        this,
        _1));
    timer_.stop();
  }
  
  void TimeredDiagnosticUpdater::start()
  {
    timer_.start();
  }

  TimeredDiagnosticUpdater::~TimeredDiagnosticUpdater()
  {
  }

  void TimeredDiagnosticUpdater::setHardwareID(const std::string& name)
  {
    diagnostic_updater_->setHardwareID(name);
  }
  
  void TimeredDiagnosticUpdater::add(const std::string& name,
                                     diagnostic_updater::TaskFunction f)
  {
    diagnostic_updater_->add(name, f);
  }
  
  // void TimeredDiagnosticUpdater::add(diagnostic_updater::DiagnosticTask task)
  // {
  //   diagnostic_updater_->add(task);
  // }

  void TimeredDiagnosticUpdater::update()
  {
    diagnostic_updater_->update();
  }
  
  void TimeredDiagnosticUpdater::timerCallback(const ros::TimerEvent& event)
  {
    update();
  }
  
}

