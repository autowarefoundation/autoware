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

#ifndef ICP_SLAM_PCL_H
#define ICP_SLAM_PCL_H

#include "lidar_localizer/icp/icp_slam_base.h"

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

template <class PointSource, class PointTarget>
class IcpSlamPCL
    : public IcpSlamBase<PointSource, PointTarget>
{
    public:
        IcpSlamPCL();
        ~IcpSlamPCL() = default;
        void setTransformationEpsilon(double epsilon) override;
        void setEuclideanFitnessEpsilon(double epsilon) override;
        void setMaxCorrespondenceDistance(double distance_threshold) override;
        void setRANSACOutlierRejectionThreshold(double inlier_threshold) override;
        void setMaximumIterations(int nr_iterations) override;

        double getTransformationEpsilon() override;
        double getEuclideanFitnessEpsilon() override;
        double getMaxCorrespondenceDistance() override;
        double getRANSACOutlierRejectionThreshold() override;
        int getMaximumIterations() override;

    protected:
        void align(const Pose& predict_pose) override;
        double getFitnessScore() override;
        void setInputTarget(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_ptr) override;
        void setInputSource(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& scan_ptr) override;
        Pose getFinalPose() override;

    private:
        pcl::IterativeClosestPoint<PointSource, PointTarget> icp_;
};

template <class PointSource, class PointTarget>
IcpSlamPCL<PointSource, PointTarget>::IcpSlamPCL()
{
}

template <class PointSource, class PointTarget>
void IcpSlamPCL<PointSource, PointTarget>::setTransformationEpsilon(double epsilon)
{
    icp_.setTransformationEpsilon(epsilon);
}

template <class PointSource, class PointTarget>
void IcpSlamPCL<PointSource, PointTarget>::setEuclideanFitnessEpsilon(double epsilon)
{
    icp_.setEuclideanFitnessEpsilon(epsilon);
}

template <class PointSource, class PointTarget>
void IcpSlamPCL<PointSource, PointTarget>::setMaxCorrespondenceDistance(double distance_threshold)
{
    icp_.setMaxCorrespondenceDistance(distance_threshold);
}

template <class PointSource, class PointTarget>
void IcpSlamPCL<PointSource, PointTarget>::setRANSACOutlierRejectionThreshold(double inlier_threshold)
{
    icp_.setRANSACOutlierRejectionThreshold(inlier_threshold);
}

template <class PointSource, class PointTarget>
void IcpSlamPCL<PointSource, PointTarget>::setMaximumIterations(int nr_iterations)
{
    icp_.setMaximumIterations(nr_iterations);
}

template <class PointSource, class PointTarget>
double IcpSlamPCL<PointSource, PointTarget>::getTransformationEpsilon()
{
    return icp_.getTransformationEpsilon();
}

template <class PointSource, class PointTarget>
double IcpSlamPCL<PointSource, PointTarget>::getEuclideanFitnessEpsilon()
{
    return icp_.getEuclideanFitnessEpsilon();
}

template <class PointSource, class PointTarget>
double IcpSlamPCL<PointSource, PointTarget>::getMaxCorrespondenceDistance()
{
    return icp_.getMaxCorrespondenceDistance();
}

template <class PointSource, class PointTarget>
double IcpSlamPCL<PointSource, PointTarget>::getRANSACOutlierRejectionThreshold()
{
    return icp_.getRANSACOutlierRejectionThreshold();
}

template <class PointSource, class PointTarget>
int IcpSlamPCL<PointSource, PointTarget>::getMaximumIterations()
{
    return icp_.getMaximumIterations();
}


template <class PointSource, class PointTarget>
void IcpSlamPCL<PointSource, PointTarget>::align(const Pose& predict_pose)
{
    const auto predict_matrix = convertToEigenMatrix4f(predict_pose);
    pcl::PointCloud<PointSource> output_cloud;
    icp_.align(output_cloud, predict_matrix);
}

template <class PointSource, class PointTarget>
void IcpSlamPCL<PointSource, PointTarget>::setInputTarget(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_ptr)
{
    icp_.setInputTarget(map_ptr);
}

template <class PointSource, class PointTarget>
void IcpSlamPCL<PointSource, PointTarget>::setInputSource(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& scan_ptr)
{
    icp_.setInputSource(scan_ptr);
}

template <class PointSource, class PointTarget>
double IcpSlamPCL<PointSource, PointTarget>::getFitnessScore()
{
    return icp_.getFitnessScore();
}

template <class PointSource, class PointTarget>
Pose IcpSlamPCL<PointSource, PointTarget>::getFinalPose()
{
    return convertToPose(icp_.getFinalTransformation());
}


#endif
