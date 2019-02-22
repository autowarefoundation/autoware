/*
 *  Copyright (c) 2019, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
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
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 *  THE POSSIBILITY OF SUCH DAMAGE.
 */


/*
 * RandomAccessBag.h
 *
 *  Created on: Aug 15, 2018
 *      Author: sujiwo
 */

#ifndef _RANDOMACCESSBAG_H_
#define _RANDOMACCESSBAG_H_

#include <exception>
#include <map>
#include <memory>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <string>
#include <utility>
#include <vector>

class time_exception : public std::runtime_error {
  virtual const char *what() const throw() {
    return "Invalid out-of-time requested";
  }
};

class RandomAccessBag : public rosbag::View {
public:
  typedef std::shared_ptr<RandomAccessBag> Ptr;

  RandomAccessBag(const rosbag::Bag &bag, const std::string &topic);

  RandomAccessBag(const rosbag::Bag &bag, const std::string &topic,
                  const ros::Time &t1, const ros::Time &t2);

  RandomAccessBag(rosbag::Bag const &bag, const std::string &topic,
                  const double seconds1FromOffset,
                  const double seconds2FromOffset);

  virtual ~RandomAccessBag();

  void setTimeConstraint(const ros::Time &t1, const ros::Time &t2);

  // Set time constraint to default (start & stop bag)
  void resetTimeConstraint();

  /*
   * A note about seconds in this function:
   * These parameters are time (in seconds) from start of the bag
   */
  void setTimeConstraint(const double seconds1FromOffset,
                         const double seconds2FromOffset);

  template <typename T> boost::shared_ptr<T> at(int position) {
    assert(position >= 0 and position < size());
    return instantiate<T>(msgPtr.at(position));
  }

  //	RandomAccessBag subset(const ros::Time &start, ros::Duration &d) const;

  ros::Time timeAt(const int i) const { return msgPtr.at(i).time; }

  template <typename T> boost::shared_ptr<T> atDurationSecond(const double S) {
    return at<T>(getPositionAtDurationSecond(S));
  }

  std::string getTopic() { return conn->topic; }

  size_t size() const { return static_cast<size_t>(size_cache_); }

  uint32_t getPositionAtDurationSecond(const double S) const;

  uint32_t getPositionAtTime(const ros::Time &tx) const;

  /*
   * Duration of this view in ros::Duration
   */
  ros::Duration length() const { return stopTime() - startTime(); }

  ros::Time startTime() const { return msgPtr.front().time; }

  ros::Time stopTime() const { return msgPtr.back().time; }

  inline bool isTimeInside(const ros::Time &t) const {
    return (t >= msgPtr.front().time and t <= msgPtr.back().time);
  }

  /*
   * Convert time as represented by seconds from offset
   */
  ros::Time timeFromOffset(const double secondsFromStart) const;

  /*
   * Convert time as represented by seconds from start of bag
   */
  ros::Time timeFromStart(const double seconds) const;

  inline ros::Time getBagStartTime() const { return bagStartTime; }

  inline ros::Time getBagStopTime() const { return bagStopTime; }

  inline bool isTimeConstrained() const { return mIsTimeConstrained; }

  inline std::string topic() const { return viewTopic; }

  std::string messageType() const;

  static std::map<std::string, std::string>
  getTopicList(const rosbag::Bag &bag);

protected:
  void createCache();

  const rosbag::Bag &bagstore;
  const rosbag::ConnectionInfo *conn;
  std::vector<rosbag::IndexEntry> msgPtr;
  bool mIsTimeConstrained = false;

  template <class T>
  boost::shared_ptr<T> instantiate(const rosbag::IndexEntry &index_entry) {
    rosbag::MessageInstance *m =
        newMessageInstance(conn, index_entry, bagstore);
    return m->instantiate<T>();
  }

  ros::Time bagStartTime, bagStopTime;
  const std::string viewTopic;
};

#endif /* _RANDOMACCESSBAG_H_ */
