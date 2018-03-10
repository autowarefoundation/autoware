/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *
 * $Id$
 *
 */
#ifndef FAST_PCL_REGISTRATION_CORRESPONDENCE_TYPES_H_
#define FAST_PCL_REGISTRATION_CORRESPONDENCE_TYPES_H_

#include <pcl/correspondence.h>

namespace pcl
{
  namespace registration
  {
    /** \brief calculates the mean and standard deviation of descriptor distances from correspondences
      * \param[in] correspondences list of correspondences
      * \param[out] mean the mean descriptor distance of correspondences
      * \param[out] stddev the standard deviation of descriptor distances.
      * \note The sample varaiance is used to determine the standard deviation
      */
    inline void 
    getCorDistMeanStd (const pcl::Correspondences& correspondences, double &mean, double &stddev);

    /** \brief extracts the query indices
      * \param[in] correspondences list of correspondences
      * \param[out] indices array of extracted indices.
      * \note order of indices corresponds to input list of descriptor correspondences
      */
    inline void 
    getQueryIndices (const pcl::Correspondences& correspondences, std::vector<int>& indices);

    /** \brief extracts the match indices
      * \param[in] correspondences list of correspondences
      * \param[out] indices array of extracted indices.
      * \note order of indices corresponds to input list of descriptor correspondences
      */
    inline void 
    getMatchIndices (const pcl::Correspondences& correspondences, std::vector<int>& indices);

  }
}

//#include <pcl/registration/impl/correspondence_types.hpp>
#include "fast_pcl/registration/impl/correspondence_types.hpp"

#endif /* FAST_PCL_REGISTRATION_CORRESPONDENCE_TYPES_H_ */
