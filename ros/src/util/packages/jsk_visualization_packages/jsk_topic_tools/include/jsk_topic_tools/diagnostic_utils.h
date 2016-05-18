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


#ifndef JSK_TOPIC_TOOLS_DIAGNOSTIC_UTIL_H_
#define JSK_TOPIC_TOOLS_DIAGNOSTIC_UTIL_H_

#include <string>
#include <diagnostic_updater/diagnostic_updater.h>
#include "jsk_topic_tools/time_accumulator.h"
#include "jsk_topic_tools/vital_checker.h"

namespace jsk_topic_tools
{
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
  
}

#endif
