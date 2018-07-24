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

#ifndef LIBNDT_SLAM_PCL_ANH_GPU_H
#define LIBNDT_SLAM_PCL_ANH_GPU_H

#ifndef CUDA_FOUND

    #include "points_localizer/ndt/ndt_slam_dummy.h"

    template <class PointSource, class PointTarget>
    class LibNdtSlamPCLANHGPU
        : public LibNdtSlamDummy<PointSource, PointTarget>
    {
        public:
            LibNdtSlamPCLANHGPU();
            ~LibNdtSlamPCLANHGPU() = default;
    };

    template <class PointSource, class PointTarget>
    LibNdtSlamPCLANHGPU<PointSource, PointTarget>::LibNdtSlamPCLANHGPU()
    {
        std::cerr << "**************************************************************" << std::endl;
        std::cerr << "[ERROR]PCL_ANH_GPU is not built. Please use other method type." << std::endl;
        std::cerr << "**************************************************************" << std::endl;
        exit(1);
    }

#else

    #include "points_localizer/ndt/ndt_slam_base.h"

    #include <pcl/io/io.h>
    #include <pcl/io/pcd_io.h>
    #include <pcl/point_types.h>
    #include <ndt_gpu/NormalDistributionsTransform.h>

    template <class PointSource, class PointTarget>
    class LibNdtSlamPCLANHGPU
        : public LibNdtSlamBase<PointSource, PointTarget>
    {
        public:
            LibNdtSlamPCLANHGPU();
            ~LibNdtSlamPCLANHGPU() = default;

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
            void setInputTarget(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_ptr) override;
            void setInputSource(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& scan_ptr) override;
            Pose getFinalPose() override;
            void buildMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_ptr) override;
            void swapInstance() override;

        private:
            boost::shared_ptr< gpu::GNormalDistributionsTransform > ndt_ptr_;
            boost::shared_ptr< gpu::GNormalDistributionsTransform > swap_ndt_ptr_;
        };

    template <class PointSource, class PointTarget>
    LibNdtSlamPCLANHGPU<PointSource, PointTarget>::LibNdtSlamPCLANHGPU()
        : ndt_ptr_(new gpu::GNormalDistributionsTransform)
        , swap_ndt_ptr_(ndt_ptr_)
    {
    }

    template <class PointSource, class PointTarget>
    void LibNdtSlamPCLANHGPU<PointSource, PointTarget>::setTransformationEpsilon(double trans_eps)
    {
        ndt_ptr_->setTransformationEpsilon(trans_eps);
    }

    template <class PointSource, class PointTarget>
    void LibNdtSlamPCLANHGPU<PointSource, PointTarget>::setStepSize(double step_size)
    {
        ndt_ptr_->setStepSize(step_size);
    }

    template <class PointSource, class PointTarget>
    void LibNdtSlamPCLANHGPU<PointSource, PointTarget>::setResolution(float res)
    {
        ndt_ptr_->setResolution(res);
    }

    template <class PointSource, class PointTarget>
    void LibNdtSlamPCLANHGPU<PointSource, PointTarget>::setMaximumIterations(int max_iter)
    {
        ndt_ptr_->setMaximumIterations(max_iter);
    }

    template <class PointSource, class PointTarget>
    double LibNdtSlamPCLANHGPU<PointSource, PointTarget>::getTransformationEpsilon()
    {
        return ndt_ptr_->getTransformationEpsilon();
    }

    template <class PointSource, class PointTarget>
    double LibNdtSlamPCLANHGPU<PointSource, PointTarget>::getStepSize() const
    {
        return ndt_ptr_->getStepSize();
    }

    template <class PointSource, class PointTarget>
    float LibNdtSlamPCLANHGPU<PointSource, PointTarget>::getResolution() const
    {
        return ndt_ptr_->getResolution();
    }

    template <class PointSource, class PointTarget>
    int LibNdtSlamPCLANHGPU<PointSource, PointTarget>::getMaximumIterations()
    {
        return ndt_ptr_->getMaximumIterations();
    }

    template <class PointSource, class PointTarget>
    double LibNdtSlamPCLANHGPU<PointSource, PointTarget>::getTransformationProbability() const
    {
        return ndt_ptr_->getTransformationProbability();
    }

    template <class PointSource, class PointTarget>
    void LibNdtSlamPCLANHGPU<PointSource, PointTarget>::align(const Pose& predict_pose)
    {
        const auto predict_matrix = convertToEigenMatrix4f(predict_pose);
        ndt_ptr_->align(predict_matrix);
    }

    template <class PointSource, class PointTarget>
    void LibNdtSlamPCLANHGPU<PointSource, PointTarget>::setInputTarget(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_ptr)
    {
        boost::shared_ptr< pcl::PointCloud<PointTarget> > non_const_map_ptr(new pcl::PointCloud<PointTarget>(*map_ptr));
        ndt_ptr_->setInputTarget(non_const_map_ptr);
    }

    template <class PointSource, class PointTarget>
    void LibNdtSlamPCLANHGPU<PointSource, PointTarget>::setInputSource(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& scan_ptr)
    {
        boost::shared_ptr< pcl::PointCloud<PointSource> > non_const_scan_ptr(new pcl::PointCloud<PointSource>(*scan_ptr));
        ndt_ptr_->setInputSource(non_const_scan_ptr);
    }

    template <class PointSource, class PointTarget>
    double LibNdtSlamPCLANHGPU<PointSource, PointTarget>::getFitnessScore()
    {
        return ndt_ptr_->getFitnessScore();
    }

    template <class PointSource, class PointTarget>
    Pose LibNdtSlamPCLANHGPU<PointSource, PointTarget>::getFinalPose()
    {
        return convertToPose(ndt_ptr_->getFinalTransformation());
    }

    template <class PointSource, class PointTarget>
    void LibNdtSlamPCLANHGPU<PointSource, PointTarget>::buildMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_ptr)
    {
        const auto trans_estimation = getTransformationEpsilon();
        const auto step_size = getStepSize();
        const auto resolution = getResolution();
        const auto max_iter = getMaximumIterations();

        boost::shared_ptr< gpu::GNormalDistributionsTransform > tmp_ndt_ptr(new gpu::GNormalDistributionsTransform);
        tmp_ndt_ptr->setTransformationEpsilon(trans_estimation);
        tmp_ndt_ptr->setStepSize(step_size);
        tmp_ndt_ptr->setResolution(resolution);
        tmp_ndt_ptr->setMaximumIterations(max_iter);

        boost::shared_ptr< pcl::PointCloud<PointTarget> > non_const_map_ptr(new pcl::PointCloud<PointTarget>(*map_ptr));
        tmp_ndt_ptr->setInputTarget(non_const_map_ptr);

        boost::shared_ptr< pcl::PointCloud<PointSource> > dummy_scan_ptr(new pcl::PointCloud<PointSource>());
        PointSource dummy_point;
        dummy_scan_ptr->push_back(dummy_point);
        tmp_ndt_ptr->setInputSource(dummy_scan_ptr);

        const auto identity_matrix = Eigen::Matrix4f::Identity();
        tmp_ndt_ptr->align(identity_matrix);

        swap_ndt_ptr_ = tmp_ndt_ptr;
    }


    template <class PointSource, class PointTarget>
    void LibNdtSlamPCLANHGPU<PointSource, PointTarget>::swapInstance()
    {
        ndt_ptr_ = swap_ndt_ptr_;
    }

#endif

#endif
