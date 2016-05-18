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


#ifndef JSK_TOPIC_TOOLS_LOG_UTILS_H_
#define JSK_TOPIC_TOOLS_LOG_UTILS_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#define JSK_NODELET_DEBUG(str,...) \
  NODELET_DEBUG("[%s] " str, __PRETTY_FUNCTION__, ##__VA_ARGS__)
#define JSK_NODELET_INFO(str,...) \
  NODELET_INFO("[%s] " str, __PRETTY_FUNCTION__, ##__VA_ARGS__)
#define JSK_NODELET_WARN(str,...) \
  NODELET_WARN("[%s] " str, __PRETTY_FUNCTION__, ##__VA_ARGS__)
#define JSK_NODELET_ERROR(str,...) \
  NODELET_ERROR("[%s] " str, __PRETTY_FUNCTION__, ##__VA_ARGS__)
#define JSK_NODELET_FATAL(str,...) \
  NODELET_FATAL("[%s] " str, __PRETTY_FUNCTION__, ##__VA_ARGS__)

#define JSK_NODELET_DEBUG_STREAM(...) \
  NODELET_DEBUG_STREAM("[" << __PRETTY_FUNCTION__ << "] " << __VA_ARGS__)
#define JSK_NODELET_INFO_STREAM(...) \
  NODELET_INFO_STREAM("[" << __PRETTY_FUNCTION__ << "] " << __VA_ARGS__)
#define JSK_NODELET_WARN_STREAM(...) \
  NODELET_WARN_STREAM("[" << __PRETTY_FUNCTION__ << "] " << __VA_ARGS__)
#define JSK_NODELET_ERROR_STREAM(...) \
  NODELET_ERROR_STREAM("[" << __PRETTY_FUNCTION__ << "] " << __VA_ARGS__)
#define JSK_NODELET_FATAL_STREAM(...) \
  NODELET_FATAL_STREAM("[" << __PRETTY_FUNCTION__ << "] " << __VA_ARGS__)

#define JSK_ROS_DEBUG(str,...) \
  ROS_DEBUG("[%s] " str, __PRETTY_FUNCTION__, ##__VA_ARGS__)
#define JSK_ROS_INFO(str,...) \
  ROS_INFO("[%s] " str, __PRETTY_FUNCTION__, ##__VA_ARGS__)
#define JSK_ROS_WARN(str,...) \
  ROS_WARN("[%s] " str, __PRETTY_FUNCTION__, ##__VA_ARGS__)
#define JSK_ROS_ERROR(str,...) \
  ROS_ERROR("[%s] " str, __PRETTY_FUNCTION__, ##__VA_ARGS__)
#define JSK_ROS_FATAL(str,...) \
  ROS_FATAL("[%s] " str, __PRETTY_FUNCTION__, ##__VA_ARGS__)

#define JSK_ROS_DEBUG_STREAM(...) \
  ROS_DEBUG_STREAM("[" << __PRETTY_FUNCTION__ << "] " << __VA_ARGS__)
#define JSK_ROS_INFO_STREAM(...) \
  ROS_INFO_STREAM("[" << __PRETTY_FUNCTION__ << "] " << __VA_ARGS__)
#define JSK_ROS_WARN_STREAM(...) \
  ROS_WARN_STREAM("[" << __PRETTY_FUNCTION__ << "] " << __VA_ARGS__)
#define JSK_ROS_ERROR_STREAM(...) \
  ROS_ERROR_STREAM("[" << __PRETTY_FUNCTION__ << "] " << __VA_ARGS__)
#define JSK_ROS_FATAL_STREAM(...) \
  ROS_FATAL_STREAM("[" << __PRETTY_FUNCTION__ << "] " << __VA_ARGS__)

#endif
