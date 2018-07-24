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

#ifndef LIBNDT_SLAM_DUMMY_H
#define LIBNDT_SLAM_DUMMY_H

#include "points_localizer/ndt/ndt_slam_base.h"

template <class PointSource, class PointTarget>
class LibNdtSlamDummy
    : public LibNdtSlamBase <PointSource, PointTarget>
{
    public:
        LibNdtSlamDummy() = default;
        ~LibNdtSlamDummy() = default;

        void setTransformationEpsilon(double trans_eps) override {};
        void setStepSize(double step_size)  override {};
        void setResolution(float res) override {};
        void setMaximumIterations(int max_iter) override {};

        double getTransformationEpsilon() override {return 0;};
        double getStepSize() const override {return 0;};
        float getResolution() const override {return 0;};
        int getMaximumIterations() override {return 0;};
        double getTransformationProbability() const override {return 0;};

    protected:
        void align(const Pose& predict_pose) override {};
        double getFitnessScore() override {return 0;};
        void setInputTarget(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_ptr) override {};
        void setInputSource(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& scan_ptr) override {};
        Pose getFinalPose() override {return Pose();};
        void updateVoxelGrid(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& scan_ptr, const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_ptr) override {};

};

#endif
