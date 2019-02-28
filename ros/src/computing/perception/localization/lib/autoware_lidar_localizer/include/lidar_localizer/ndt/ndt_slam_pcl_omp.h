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

#ifndef NDT_SLAM_PCL_OMP_H
#define NDT_SLAM_PCL_OMP_H

#ifndef OPENMP_FOUND

    #include "lidar_localizer/ndt/ndt_slam_dummy.h"

    template <class PointSource, class PointTarget>
    class NdtSlamPCLOMP
        : public NdtSlamDummy<PointSource, PointTarget>
    {
        public:
            NdtSlamPCLOMP();
            ~NdtSlamPCLOMP() = default;
    };

#else

    #include "lidar_localizer/ndt/ndt_slam_base.h"

    #include <pcl/io/io.h>
    #include <pcl/io/pcd_io.h>
    #include <pcl/point_types.h>
    #include <pcl_omp_registration/ndt.h>


    template <class PointSource, class PointTarget>
    class NdtSlamPCLOMP
        : public NdtSlamBase<PointSource, PointTarget>
    {
        public:
            NdtSlamPCLOMP();
            ~NdtSlamPCLOMP() = default;

            void setTransformationEpsilon(double trans_eps) override;
            void setStepSize(double step_size) override;
            void setResolution(float res) override;
            void setMaximumIterations(int max_iter) override;

            double getTransformationEpsilon() override;
            double getStepSize() const override;
            float getResolution() const override;
            int getMaximumIterations() override;
            double getTransformationProbability() const override;

        protected:
            void align(const Pose& predict_pose) override;
            double getFitnessScore() override;
            void setInputTarget(const boost::shared_ptr< pcl::PointCloud<PointSource> >& map_ptr) override;
            void setInputSource(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& scan_ptr) override;
            Pose getFinalPose() override;
            void buildMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& map_ptr) override;
            void swapInstance() override;

        private:
            boost::shared_ptr< pcl_omp::NormalDistributionsTransform<PointSource, PointTarget> > ndt_ptr_;
            boost::shared_ptr< pcl_omp::NormalDistributionsTransform<PointSource, PointTarget> > swap_ndt_ptr_;
    };

#endif

#endif
