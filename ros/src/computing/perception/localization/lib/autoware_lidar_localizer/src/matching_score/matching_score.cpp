/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autoware_lidar_localizer/matching_score/matching_score.h"

#include <pcl/point_types.h>

template<class PointType>
MatchingScore<PointType>::MatchingScore()
    : fermi_kT_(0.05)
    , fermi_mu_(0.25)
{
}

template<class PointType>
void MatchingScore<PointType>::setInputTarget(const boost::shared_ptr< pcl::PointCloud<PointType> const>& pointcloud_ptr)
{
    static size_t points_size = 0;
    if (points_size != pointcloud_ptr->points.size()) {
        tree_ptr_->setInputCloud(pointcloud_ptr);
        points_size = pointcloud_ptr->points.size();
    }
}

template<class PointType>
void MatchingScore<PointType>::setSearchMethodTarget(const boost::shared_ptr<pcl::search::KdTree<PointType> >& tree_ptr)
{
    tree_ptr_ = tree_ptr;
}

template<class PointType>
double MatchingScore<PointType>::calcMatchingScore(const boost::shared_ptr< pcl::PointCloud<PointType> const>& pointcloud_ptr)
{
    point_with_distance_array_ = convertPointWithDistance(pointcloud_ptr);
    const double s0 = calcFermiDistributionFunction(0, fermi_kT_, fermi_mu_);  //to normalize to 1
    double score_sum = 0;
    for(const auto point_with_distance : point_with_distance_array_) {
        const double s = calcFermiDistributionFunction(point_with_distance.distance, fermi_kT_, fermi_mu_);
        score_sum += s / s0;
    }
    const double score = point_with_distance_array_.empty()
                     ? 0
                     : score_sum / point_with_distance_array_.size();
    return score;
}


template<class PointType>
double MatchingScore<PointType>::calcFermiDistributionFunction(const double x, const double kT, const double mu)
{
    return 1.0 / (std::exp((x-mu)/kT)+1.0);
}

template<class PointType>
std::vector< PointWithDistance<PointType> > MatchingScore<PointType>::convertPointWithDistance(const boost::shared_ptr< pcl::PointCloud<PointType> const>& pointcloud_ptr)
{
    std::vector< PointWithDistance<PointType> > point_with_distance_array;
    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);
    for (const auto& point : pointcloud_ptr->points) {
        PointWithDistance<PointType> point_with_distance;
        tree_ptr_->nearestKSearch(point, 1, nn_indices, nn_dists);
        point_with_distance.point = point;
        point_with_distance.distance = std::sqrt(nn_dists[0]);
        point_with_distance_array.push_back(point_with_distance);
    }
    return point_with_distance_array;
}

template class MatchingScore<pcl::PointXYZ>;
template class MatchingScore<pcl::PointXYZI>;
template class MatchingScore<pcl::PointXYZRGB>;
