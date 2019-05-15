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


#ifndef MATCHING_SCORE_HISTOGRAM_H
#define MATCHING_SCORE_HISTOGRAM_H

#include "autoware_lidar_localizer/util/data_structs.h"

class MatchingScoreHistogram
{
    public:
        MatchingScoreHistogram();
        ~MatchingScoreHistogram() = default;

        template<class PointType>
        std::vector<HistogramWithRangeBin> createHistogramWithRangeBinArray(const std::vector< PointWithDistance<PointType> >& point_with_distance_array);

        void setNumberOfBins(const size_t number_of_bins)
        {
            number_of_bins_ = number_of_bins;
        }

        void setBinWidth(const double bin_width)
        {
            bin_width_ = bin_width;
        }

        void setBinMaxHeight(const double bin_max_height)
        {
            bin_max_height_ = bin_max_height;
        }

    private:
        void initHistogram();

        std::vector<HistogramWithRangeBin> histgram_bin_array_;
        size_t number_of_bins_;
        double bin_width_;
        double bin_max_height_;
};

#endif
