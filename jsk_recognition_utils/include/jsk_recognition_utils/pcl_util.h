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

#ifndef JSK_RECOGNITION_UTILS_PCL_UTIL_H_
#define JSK_RECOGNITION_UTILS_PCL_UTIL_H_
#include <pcl/point_types.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/count.hpp>

#include <set>
#include <map>

#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <pcl/PointIndices.h>
#include <std_msgs/ColorRGBA.h>

#include <jsk_topic_tools/time_accumulator.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <boost/circular_buffer.hpp>
#include <jsk_topic_tools/vital_checker.h>

#include <pcl/filters/extract_indices.h>
#include <yaml-cpp/yaml.h>

namespace jsk_recognition_utils
{
  Eigen::Affine3f affineFromYAMLNode(const YAML::Node& pose);
  
  std::vector<int> addIndices(const std::vector<int>& a,
                              const std::vector<int>& b);
  pcl::PointIndices::Ptr addIndices(const pcl::PointIndices& a,
                                    const pcl::PointIndices& b);
  // substract indices like: a - b
  std::vector<int> subIndices(const std::vector<int>& a,
                              const std::vector<int>& b);
  pcl::PointIndices::Ptr subIndices(const pcl::PointIndices& a,
                                    const pcl::PointIndices& b);

  template <class PointT>
  std::vector<typename pcl::PointCloud<PointT> ::Ptr>
  convertToPointCloudArray(const typename pcl::PointCloud<PointT>::Ptr& cloud,
                           const std::vector<pcl::PointIndices::Ptr>& indices)
  {
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    std::vector<typename pcl::PointCloud<PointT> ::Ptr> cloud_array;
    for (size_t i = 0; i < indices.size(); i++) {
      typename pcl::PointCloud<PointT> ::Ptr
        segment (new pcl::PointCloud<PointT>);
      extract.setIndices(indices[i]);
      extract.filter(*segment);
      cloud_array.push_back(segment);
    }
    return cloud_array;
  }


  template <class T>
  pcl::PointCloud<pcl::PointXYZ>::Ptr convertToXYZCloud(const pcl::PointCloud<T>& cloud)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
    output->points.resize(cloud.points.size());
    for (size_t i = 0; i < cloud.points.size(); i++) {
      pcl::PointXYZ p;
      p.x = cloud.points[i].x;
      p.y = cloud.points[i].y;
      p.z = cloud.points[i].z;
      output->points[i] = p;
    }
    return output;
  }
  
  template<class T>
  void appendVector(std::vector<T>& a, const std::vector<T>& b)
  {
    for (size_t i = 0; i < b.size(); i++) {
      a.push_back(b[i]);
    }
  }
  
  // select color out of 20 colors
  std_msgs::ColorRGBA colorCategory20(int i);

  class Counter
  {
  public:
    typedef boost::accumulators::accumulator_set<
    double,
    boost::accumulators::stats<boost::accumulators::tag::count,
                               boost::accumulators::tag::mean,
                               boost::accumulators::tag::min,
                               boost::accumulators::tag::max,
                               boost::accumulators::tag::variance> > Accumulator;
    virtual void add(double v);
    virtual double mean();
    virtual double min();
    virtual double max();
    virtual int count();
    virtual double variance();
  protected:
    Accumulator acc_;
  };

  ////////////////////////////////////////////////////////
  // Graph utility function
  ////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////
  // buildGroupFromGraphMap (recursive function)
  //   This function retrieves a directional graph, and build
  //   a set of the indices where can be arrived from a specified
  //   vertex.
  // 
  //   graph_map := A map representeing edges of the graph.
  //                The graph is bidirectional graph.
  //                the key means "from vertex" and the value
  //                means the "to indices" from the key vertex.
  //   from_index := The index to pay attension
  //   to_indices := The "to indices" from from_index
  //   output_set := result
  ////////////////////////////////////////////////////////
  typedef std::map<int, std::vector<int> > IntegerGraphMap;
  
  void buildGroupFromGraphMap(IntegerGraphMap graph_map,
                              const int from_index,
                              std::vector<int>& to_indices,
                              std::set<int>& output_set);
  void _buildGroupFromGraphMap(IntegerGraphMap graph_map,
                              const int from_index,
                              std::vector<int>& to_indices,
                              std::set<int>& output_set);
  
  ////////////////////////////////////////////////////////
  // buildAllGraphSetFromGraphMap
  //   get all the list of set represented in graph_map
  ////////////////////////////////////////////////////////
  void buildAllGroupsSetFromGraphMap(IntegerGraphMap graph_map,
                                     std::vector<std::set<int> >& output_sets);
  
  ////////////////////////////////////////////////////////
  // addSet<class>(A, B)
  //   add two set like A = A + B
  ////////////////////////////////////////////////////////
  template <class T>
  void addSet(std::set<T>& output,
              const std::set<T>& new_set)
  {
    typedef typename std::set<T> Set;
    typedef typename Set::iterator Iterator;
    for (Iterator it = new_set.begin();
         it != new_set.end();
         ++it) {
      output.insert(*it);
    }
  }

  ////////////////////////////////////////////////////////
  // add TimeAcumulator information to Diagnostics
  ////////////////////////////////////////////////////////
  void addDiagnosticInformation(
    const std::string& string_prefix,
    jsk_topic_tools::TimeAccumulator& accumulator,
    diagnostic_updater::DiagnosticStatusWrapper& stat);

  ////////////////////////////////////////////////////////
  // set error string to 
  ////////////////////////////////////////////////////////
  void addDiagnosticErrorSummary(
    const std::string& string_prefix,
    jsk_topic_tools::VitalChecker::Ptr vital_checker,
    diagnostic_updater::DiagnosticStatusWrapper& stat);

  ////////////////////////////////////////////////////////
  // add Boolean string to stat
  ////////////////////////////////////////////////////////
  void addDiagnosticBooleanStat(
    const std::string& string_prefix,
    const bool value,
    diagnostic_updater::DiagnosticStatusWrapper& stat);
  
  ////////////////////////////////////////////////////////
  // SeriesedBoolean
  //   store boolean value to limited buffer
  //   and return true if all the values are true.
  ////////////////////////////////////////////////////////
  class SeriesedBoolean
  {
  public:
    typedef boost::shared_ptr<SeriesedBoolean> Ptr;
    SeriesedBoolean(const int buf_len);
    virtual ~SeriesedBoolean();
    virtual void addValue(bool val);
    virtual bool getValue();
  protected:
  private:
    boost::circular_buffer<bool> buf_;
  };

  ////////////////////////////////////////////////////////
  // TimeredDiagnosticUpdater
  //   useful wrapper of DiagnosticUpdater.
  ////////////////////////////////////////////////////////
  class TimeredDiagnosticUpdater
  {
  public:
    typedef boost::shared_ptr<TimeredDiagnosticUpdater> Ptr;
    TimeredDiagnosticUpdater(ros::NodeHandle& nh,
                             const ros::Duration& timer_duration);
    virtual ~TimeredDiagnosticUpdater();
    // wrapper methods of diagnostic_updater::Updater
    virtual void add(const std::string& name,
                     diagnostic_updater::TaskFunction f);
    //virtual void add(diagnostic_updater::DiagnosticTask task);
    virtual void start();
    virtual void setHardwareID(const std::string& name);
    virtual void update();
  protected:
    virtual void timerCallback(const ros::TimerEvent& event);
    ros::Timer timer_;
    boost::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  private:
    
  };
  
  extern boost::mutex global_chull_mutex;
  
}

#endif
