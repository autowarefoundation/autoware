/*
 *  Copyright (c) 2017, Tier IV, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
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
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef LIBICP_SLAM_BASE_H
#define LIBICP_SLAM_BASE_H

#include "points_localizer/localizer.h"

template <class PointSource, class PointTarget>
class LibIcpSlamBase : public LibLocalizer <PointSource, PointTarget>
{
    public:
        virtual ~LibIcpSlamBase() = default;

        virtual void setTransformationEpsilon(double epsilon) = 0;
        virtual void setEuclideanFitnessEpsilon(double epsilon) = 0;
        virtual void setMaxCorrespondenceDistance(double distance_threshold) = 0;
        virtual void setRANSACOutlierRejectionThreshold(double inlier_threshold) = 0;
        virtual void setMaximumIterations(int nr_iterations) = 0;

        virtual double getTransformationEpsilon() = 0;
        virtual double getEuclideanFitnessEpsilon() = 0;
        virtual double getMaxCorrespondenceDistance() = 0;
        virtual double getRANSACOutlierRejectionThreshold() = 0;
        virtual int getMaximumIterations() = 0;
};

#endif
