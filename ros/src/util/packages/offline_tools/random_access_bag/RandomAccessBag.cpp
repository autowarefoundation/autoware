/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

/*
 * RandomAccessBag.cpp
 *
 *  Created on: Aug 15, 2018
 *      Author: sujiwo
 */

#include <algorithm>
#include <exception>
#include <rosbag/query.h>
#include <stdexcept>

#include "RandomAccessBag.h"

#include "access_private.hpp"

using namespace std;

ACCESS_PRIVATE_FIELD(rosbag::MessageInstance, rosbag::IndexEntry const,
                     index_entry_);

/*
 * A Notes about ROS Bags: TODO
 */

/*
 * Only allows one single topic
 */
RandomAccessBag::RandomAccessBag(const rosbag::Bag &_bag,
                                 const std::string &topic)
    :

      bagstore(_bag),
      viewTopic(topic), rosbag::View::View(_bag, rosbag::TopicQuery(topic))

{
  bagStartTime = getBeginTime();
  bagStopTime = getEndTime();

  createCache();
}

RandomAccessBag::RandomAccessBag(const rosbag::Bag &_bag,
                                 const std::string &topic, const ros::Time &t1,
                                 const ros::Time &t2)
    :

      bagstore(_bag),
      viewTopic(topic), rosbag::View::View(_bag)

{
  bagStartTime = getBeginTime();
  bagStopTime = getEndTime();

  ros::Time t1x, t2x;
  t1x = (t1 == ros::TIME_MIN ? bagStartTime : t1);
  t2x = (t2 == ros::TIME_MAX ? bagStopTime : t2);

  setTimeConstraint(t1x, t2x);
}

RandomAccessBag::RandomAccessBag(rosbag::Bag const &_bag,
                                 const std::string &topic,
                                 const double seconds1, const double seconds2)
    :

      bagstore(_bag),
      viewTopic(topic), rosbag::View::View(_bag)

{
  bagStartTime = getBeginTime();
  bagStopTime = getEndTime();

  setTimeConstraint(seconds1, seconds2);
}

RandomAccessBag::~RandomAccessBag() {}

void RandomAccessBag::createCache() {
  update();
  rosbag::View::size();
  iterator it = begin();
  size_t sz = this->size();
  conn = getConnections()[0];
  msgPtr.resize(sz);

  for (uint32_t p = 0; p < sz; p++) {
    rosbag::MessageInstance &m = *it;
    rosbag::IndexEntry const ie = access_private::index_entry_(m);
    msgPtr.at(p) = ie;
    ++it;
  }
}

void RandomAccessBag::setTimeConstraint(const double seconds1,
                                        const double seconds2) {
  assert(seconds1 <= seconds2);

  ros::Duration td1(seconds1), td2(seconds2);

  assert(bagStartTime + td2 <= bagStopTime);

  ros::Time t1 = bagStartTime + td1, t2 = bagStartTime + td2;

  setTimeConstraint(t1, t2);
}

void RandomAccessBag::setTimeConstraint(const ros::Time &t1,
                                        const ros::Time &t2) {
  if (t1 < bagStartTime or t1 > bagStopTime)
    throw out_of_range("Requested start time is out of range");

  if (t2 < bagStartTime or t2 > bagStopTime)
    throw out_of_range("Requested stop time is out of range");

  queries_.clear();
  ranges_.clear();
  addQuery(bagstore, rosbag::TopicQuery(viewTopic), t1, t2);
  createCache();
  mIsTimeConstrained = true;
}

void RandomAccessBag::resetTimeConstraint() {
  queries_.clear();
  ranges_.clear();
  addQuery(bagstore, rosbag::TopicQuery(viewTopic));
  createCache();
  mIsTimeConstrained = false;
}

uint32_t RandomAccessBag::getPositionAtDurationSecond(const double S) const {
  ros::Duration Sd(S);
  ros::Time Tx = msgPtr.at(0).time + Sd;
  if (Tx < msgPtr.at(0).time or Tx > msgPtr.back().time)
    throw out_of_range("Requested time is out of bag");

  return getPositionAtTime(Tx);
}

uint32_t RandomAccessBag::getPositionAtTime(const ros::Time &tx) const {
  ros::Time txi;
  const ros::Time bstart = msgPtr.front().time, bstop = msgPtr.back().time;

  // Check whether tx is inside the range, with tolerance of 1 microsecond
  if (tx < bstart) {
    if (abs((tx - bstart).toNSec()) > 1000)
      throw std::out_of_range("Requested time is below the range");
    else
      txi = bstart;
  }

  else if (tx > bstop) {
    if (abs((tx - bstop).toNSec()) > 1000)
      throw std::out_of_range("Requested time is over the range");
    else
      txi = bstop;
  }

  else
    txi = tx;

  auto it =
      std::lower_bound(msgPtr.begin(), msgPtr.end(), txi,
                       [](const rosbag::IndexEntry &iptr, const ros::Time &t) {
                         return (iptr.time < t);
                       });

  // check if either solution p or p-1 is closer to our requested time
  uint32_t p = it - msgPtr.begin();
  if (p > 0 and p < size() - 1) {
    auto td1 = msgPtr.at(p).time - tx, td0 = txi - msgPtr.at(p - 1).time;
    return (td1 > td0 ? p - 1 : p);
  } else
    return p;
}

/*
 * Convert time as represented by seconds from offset
 */
ros::Time RandomAccessBag::timeFromOffset(const double secondsFromStart) const {
  assert(secondsFromStart >= 0 and secondsFromStart <= length().toSec());

  ros::Duration td(secondsFromStart);
  return msgPtr.at(0).time + td;
}

/*
 * Convert time as represented by seconds from start of bag
 */
ros::Time RandomAccessBag::timeFromStart(const double seconds) const {
  assert(seconds >= 0 and seconds <= (bagStopTime - bagStartTime).toSec());

  ros::Duration td(seconds);
  return bagStartTime + td;
}

typedef std::map<uint32_t, rosbag::ConnectionInfo *> connectionListT;
ACCESS_PRIVATE_FIELD(rosbag::Bag, connectionListT, connections_);

std::map<std::string, std::string>
RandomAccessBag::getTopicList(const rosbag::Bag &bag) {
  std::map<std::string, std::string> topicList;

  const connectionListT &bagConnList = access_private::connections_(bag);

  for (auto &conp : bagConnList) {
    auto conn = conp.second;
    topicList.insert(make_pair(conn->topic, conn->datatype));
  }

  return topicList;
}

std::string RandomAccessBag::messageType() const { return conn->datatype; }
