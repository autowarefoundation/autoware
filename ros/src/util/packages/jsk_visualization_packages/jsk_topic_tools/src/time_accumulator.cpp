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

#include "jsk_topic_tools/time_accumulator.h"

namespace jsk_topic_tools
{
  TimeAccumulator::TimeAccumulator()
  {

  }

  TimeAccumulator::~TimeAccumulator()
  {

  }
  
  ScopedTimer TimeAccumulator::scopedTimer()
  {
    return ScopedTimer(this);
  }

  void TimeAccumulator::registerTime(double time)
  {
    acc_(time);
  }

  double TimeAccumulator::mean()
  {
    return boost::accumulators::mean(acc_);
  }

  double TimeAccumulator::min()
  {
    return boost::accumulators::min(acc_);
  }

  double TimeAccumulator::max()
  {
    return boost::accumulators::max(acc_);
  }

  int TimeAccumulator::count()
  {
    return boost::accumulators::count(acc_);
  }
  
  double TimeAccumulator::variance()
  {
    return boost::accumulators::variance(acc_);
  }
  
  ScopedTimer::ScopedTimer(TimeAccumulator* parent):
    parent_(parent), start_time_(ros::WallTime::now())
  {
  }

  ScopedTimer::~ScopedTimer()
  {
    parent_->registerTime((ros::WallTime::now() - start_time_).toSec());
  }
}

