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


#ifndef MATCHING_RELIABILITY_H
#define MATCHING_RELIABILITY_H

#include <pcl/kdtree/kdtree_flann.h>

#include "lidar_localizer/util/data_structs.h"


template<class PointType>
struct PointsWithDistance
{
    PointCloudWithDistance() : distance(0) {};
    PointType point;
    double distacne;
};

class MachingReliability
{
    public:
        MachingReliability();
        ~MachingReliability() = default;
        void setSigma(double sigma);
        void setInputTarget(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& pointcloud_ptr);
        double calcMatchingReleliability(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& pointcloud_ptr);
        std::vector< PointCloudWithDistance<PointSource> > getPointCloudWithDistance() const
        {
            return point_with_distance_array_;
        }

    private:
        std::vector< PointCloudWithDistance<PointSource> > convertPointCloudWithDistance(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& pointcloud_ptr);
        double calcProbabilityDensity(double x, double average, double sigma);
        pcl::KdTreeFLANN<pcl::PointXYZI> tree_;
        std::vector< PointCloudWithDistance<PointSource> > point_with_distance_array_;

        double sigma_;
        double min_clipping_height_;
        double max_clipping_height_;
        double min_clipping_range_;
        double max_clipping_range_;
};

void MachingReliability::setInputTarget(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& pointcloud_ptr)
{
  tree_.setInputCloud(pointcloud_ptr);
}

void MachingReliability::calcMatchingReleliability(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& pointcloud_ptr)
{
    point_with_distance_array_ = convertPointCloudWithDistance(pointcloud_ptr);
    double k = calcProbabilityDensity(0, 0, sigma);
    double sum = 0;
    for(const auto point_with_distance : point_with_distance_array_)
    {
        double d = calcProbabilityDensity(point_with_distance.distance, 0, sigma);
        sum += d/k;
    }
    double score = point_with_distance_array_.empty()
                     ? 0
                     : sum / point_with_distance_array_.size();
    return score;
}

std::vector< PointCloudWithDistance<PointSource> > MachingReliability::convertPointCloudWithDistance(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& pointcloud_ptr)
{
    std::vector< PointCloudWithDistance<PointSource> > point_with_distance_array;
    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);
    for (const auto& point : pointcloud_ptr->points)
    {
      PointCloudWithDistance<PointSource> point_with_distance;
      tree_.nearestKSearch(point, 1, nn_indices, nn_dists);
      point_with_distance.point = point;
      point_with_distance.distance = nn_dists[0];
      point_with_distance_array.push_back(point_with_distance);
    }
    return point_with_distance_array;
}

#endif
