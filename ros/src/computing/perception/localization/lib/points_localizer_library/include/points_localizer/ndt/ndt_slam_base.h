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

#ifndef LIBNDT_SLAM_BASE_H
#define LIBNDT_SLAM_BASE_H

#include "points_localizer/localizer.h"

template <class PointSource, class PointTarget>
class LibNdtSlamBase : public LibLocalizer<PointSource, PointTarget>
{
    public:
        LibNdtSlamBase();
        virtual ~LibNdtSlamBase() = default;

        virtual void setTransformationEpsilon(double trans_eps) = 0;
        virtual void setStepSize(double step_size) = 0;
        virtual void setResolution(float res) = 0;
        virtual void setMaximumIterations(int max_iter) = 0;

        virtual double getTransformationEpsilon() = 0;
        virtual double getStepSize() const = 0;
        virtual float getResolution() const = 0;
        virtual int getMaximumIterations() = 0;
        virtual std::vector<Eigen::Vector3d> getCentroid() const {
            std::vector<Eigen::Vector3d> tmp;
            return tmp;
        };

        virtual std::vector<Eigen::Vector3d> getEval() const {
            std::vector<Eigen::Vector3d> tmp;
            return tmp;
        };

        virtual std::vector<Eigen::Matrix3d> getEvec() const {
            std::vector<Eigen::Matrix3d> tmp;
            return tmp;
        };

        virtual std::vector<Eigen::Matrix3d> getCovariance() const {
            std::vector<Eigen::Matrix3d> tmp;
            return tmp;
        };

        virtual double getTransformationProbability() const = 0;

        virtual double getFitnessScore() = 0;
        //TODO
        virtual double getFitnessScore(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& source_cloud, int* const nr, const double max_range)
        {
            return getFitnessScore();
        };

        virtual std::stringstream logFileContent() const override;
};

template <class PointSource, class PointTarget>
LibNdtSlamBase<PointSource, PointTarget>::LibNdtSlamBase()
{

}

template <class PointSource, class PointTarget>
std::stringstream LibNdtSlamBase<PointSource, PointTarget>::logFileContent() const
{
    std::stringstream content = LibLocalizer<PointSource, PointTarget>::logFileContent();
    content << ","
            << getTransformationProbability();
    return content;
}

#endif
