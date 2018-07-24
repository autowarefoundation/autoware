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

#include "points_localizer/reliability/slam_reliability.h"

#include <ctime>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>


double average(const std::deque<double> &deque)
{
    const double sum = std::accumulate(std::begin(deque), std::end(deque), 0.0);
    const double ave = (deque.size() > 0) ? (sum / deque.size()) : 0;
    return ave;
}


double variance(const std::deque<double> &deque, const double ave)
{
    double sum = 0;
    for(const auto &val : deque) {
        sum += std::pow(val-ave, 2.0);
    }
    const double var = (deque.size() > 0) ? (sum / deque.size()) : 0;
    return var;
}

double variance(const std::deque<double> &deque)
{
    const double ave = average(deque);
    return variance(deque, ave);
}

LibSlamReliability::LibSlamReliability()
    : window_size_(10)
{
}

void LibSlamReliability::setScore(const double score)
{
    if(score_deque_.size() > window_size_) {
        score_deque_.pop_front();
    }
    score_deque_.push_back(score);
}

double LibSlamReliability::getAverage() const
{
    return average(score_deque_);
}

double LibSlamReliability::getVariance() const
{
    return variance(score_deque_);
}
