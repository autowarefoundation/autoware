/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
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
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
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

#ifndef ROSBAG_CONTROL_TIME_TRANSLATOR_H
#define ROSBAG_CONTROL_TIME_TRANSLATOR_H

#include "ros/time.h"
#include "rosbag/macros.h"

namespace rosbag_control {

//! Helper class for translating between two times
/*!
 * The time translator can be configured with a Real start time, a
 * Translated start time, and a time scale.
 * 
 * It will convert a time from a series starting at realStartTime to a
 * comparable time series instead starting at translatedStartTime.
 * All durations in the time-sequence as scaled by 1/(timeScale).
 *
 * That is, a time-sequence with time-scale 2 will finish twice as
 * quickly.
 */
class ROSBAG_DECL TimeTranslator
{
public:
    TimeTranslator();

    void      setTimeScale(double const& s);
    void      setRealStartTime(ros::Time const& t);
    void      setTranslatedStartTime(ros::Time const& t);  //!< Increments the translated start time by shift.  Useful for pausing.
    void      shift(ros::Duration const& d);               //!< Increments the translated start time by shift.  Useful for pausing.
    ros::Time translate(ros::Time const& t);

private:
    double    time_scale_;
    ros::Time real_start_;
    ros::Time translated_start_;
};

} // namespace rosbag_control

#endif
