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


#include "jsk_topic_tools/diagnostic_utils.h"

namespace jsk_topic_tools
{
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
  
}
