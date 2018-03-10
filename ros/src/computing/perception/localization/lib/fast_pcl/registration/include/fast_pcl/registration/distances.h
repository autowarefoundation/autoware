/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
#ifndef FAST_PCL_REGISTRATION_DISTANCES_H
#define FAST_PCL_REGISTRATION_DISTANCES_H

//#include <pcl/registration/eigen.h>
#include "fast_pcl/registration/eigen.h"
#include <vector>

namespace pcl
{
  namespace distances
  {

    /** \brief Compute the median value from a set of doubles
      * \param[in] fvec the set of doubles
      * \param[in] m the number of doubles in the set
      */
    inline double 
    computeMedian (double *fvec, int m)
    {
      // Copy the values to vectors for faster sorting
      std::vector<double> data (m);
      memcpy (&data[0], fvec, sizeof (double) * m);
      
      std::nth_element(data.begin(), data.begin() + (data.size () >> 1), data.end());
      return (data[data.size () >> 1]);
    }

    /** \brief Use a Huber kernel to estimate the distance between two vectors
      * \param[in] p_src the first eigen vector
      * \param[in] p_tgt the second eigen vector
      * \param[in] sigma the sigma value
      */
    inline double
    huber (const Eigen::Vector4f &p_src, const Eigen::Vector4f &p_tgt, double sigma) 
    {
      Eigen::Array4f diff = (p_tgt.array () - p_src.array ()).abs ();
      double norm = 0.0;
      for (int i = 0; i < 3; ++i)
      {
        if (diff[i] < sigma)
          norm += diff[i] * diff[i];
        else
          norm += 2.0 * sigma * diff[i] - sigma * sigma;
      }
      return (norm);
    }

    /** \brief Use a Huber kernel to estimate the distance between two vectors
      * \param[in] diff the norm difference between two vectors
      * \param[in] sigma the sigma value
      */
    inline double
    huber (double diff, double sigma) 
    {
      double norm = 0.0;
      if (diff < sigma)
        norm += diff * diff;
      else
        norm += 2.0 * sigma * diff - sigma * sigma;
      return (norm);
    }

    /** \brief Use a Gedikli kernel to estimate the distance between two vectors
      * (for more information, see 
      * \param[in] val the norm difference between two vectors
      * \param[in] clipping the clipping value
      * \param[in] slope the slope. Default: 4
      */
    inline double
    gedikli (double val, double clipping, double slope = 4) 
    {
      return (1.0 / (1.0 + pow (fabs(val) / clipping, slope)));
    }

    /** \brief Compute the Manhattan distance between two eigen vectors.
      * \param[in] p_src the first eigen vector
      * \param[in] p_tgt the second eigen vector
      */
    inline double
    l1 (const Eigen::Vector4f &p_src, const Eigen::Vector4f &p_tgt) 
    {
      return ((p_src.array () - p_tgt.array ()).abs ().sum ());
    }

    /** \brief Compute the Euclidean distance between two eigen vectors.
      * \param[in] p_src the first eigen vector
      * \param[in] p_tgt the second eigen vector
      */
    inline double
    l2 (const Eigen::Vector4f &p_src, const Eigen::Vector4f &p_tgt) 
    {
      return ((p_src - p_tgt).norm ());
    }

    /** \brief Compute the squared Euclidean distance between two eigen vectors.
      * \param[in] p_src the first eigen vector
      * \param[in] p_tgt the second eigen vector
      */
    inline double
    l2Sqr (const Eigen::Vector4f &p_src, const Eigen::Vector4f &p_tgt) 
    {
      return ((p_src - p_tgt).squaredNorm ());
    }
  }
}

#endif
