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


#ifndef MATCHING_SCORE_H
#define MATCHING_SCORE_H

#include <pcl/search/kdtree.h>

#include "autoware_lidar_localizer/util/data_structs.h"


template<class PointType>
class MatchingScore
{
    public:
        MatchingScore();
        ~MatchingScore() = default;
        void setInputTarget(const boost::shared_ptr< pcl::PointCloud<PointType> const>& pointcloud_ptr);
        void setSearchMethodTarget(const boost::shared_ptr<pcl::search::KdTree<PointType> >& tree_ptr);
        double calcMatchingScore(const boost::shared_ptr< pcl::PointCloud<PointType> const>& pointcloud_ptr);

        void setFermikT(const double fermi_kT)
        {
            fermi_kT_ = fermi_kT;
        }

        double getFermikT() const
        {
            return fermi_kT_;
        }

        void setFermiMu(const double fermi_mu)
        {
            fermi_mu_ = fermi_mu;
        }

        double getFermiMu() const
        {
            return fermi_mu_;
        }

        std::vector< PointWithDistance<PointType> > getPointWithDistanceArray() const
        {
            return point_with_distance_array_;
        }

    private:
        std::vector< PointWithDistance<PointType> > convertPointWithDistance(const boost::shared_ptr< pcl::PointCloud<PointType> const>& pointcloud_ptr);
        double calcFermiDistributionFunction(const double x, const double kT, const double mu);
        boost::shared_ptr<pcl::search::KdTree<PointType>> tree_ptr_;
        std::vector< PointWithDistance<PointType> > point_with_distance_array_;

        double fermi_kT_;
        double fermi_mu_;
};

#endif
