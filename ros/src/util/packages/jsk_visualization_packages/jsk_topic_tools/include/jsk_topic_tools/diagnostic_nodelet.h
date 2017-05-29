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


#ifndef JSK_TOPIC_TOOLS_DIAGNOSTIC_NODELET_
#define JSK_TOPIC_TOOLS_DIAGNOSTIC_NODELET_

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <jsk_topic_tools/vital_checker.h>

#include "jsk_topic_tools/connection_based_nodelet.h"
#include "jsk_topic_tools/diagnostic_utils.h"
#include "jsk_topic_tools/timered_diagnostic_updater.h"

namespace jsk_topic_tools
{
  
  /** @brief
   * Nodelet class based on ConnectionBasedNodelet and
   * publish diagnostic infromation periodicaly.
   *
   * In default, diagnostic status is determined by vital_checker_
   * status.
   * If vital_checker_ is not poked for seconds, which is specified by
   * ~vital_rate param and defaults to 1.0 second, diagnostic status
   * will be ERROR.
   *
   * In order to utilize this, need to call VitalChecker::poke method
   * in your callback function.
   *
   */
  class DiagnosticNodelet: public ConnectionBasedNodelet
  {
  public:
    typedef boost::shared_ptr<DiagnosticNodelet> Ptr;

    /** @brief
     * Constructor and subclass need to call this.
     *
     * @param name name of subclass
     */
    DiagnosticNodelet(const std::string& name);

  protected:
    
    /** @brief
     * Initialize method. Subclass should call this method
     * in its onInit method.
     */
    virtual void onInit();

    /** @brief
     * Method which is called periodically.
     *
     * In default, it check vitality of vital_checker_ and if vital_checker_
     * is not poked for seconds, diagnostic status will be ERROR.
     * @param stat Modofy stat to change status of diagnostic information.
     */
    virtual void updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat);

    /** @brief
     * Name of subclass. It is set by constructor and used as
     * Diagnostic method.
     */
    const std::string name_;
    
    /** @brief
     * Pointer to TimeredDiagnosticUpdater to call
     * updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper&)
     * periodically.
     */
    TimeredDiagnosticUpdater::Ptr diagnostic_updater_;
    
    /** @brief
     * VitalChecker object wihch is used in default configuration.
     */
    jsk_topic_tools::VitalChecker::Ptr vital_checker_;

  private:
    
  };
}

#endif
