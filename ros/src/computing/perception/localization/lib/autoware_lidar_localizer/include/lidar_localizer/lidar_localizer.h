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

#ifndef LIDAR_LOCALIZER_H
#define LIDAR_LOCALIZER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>
#include <thread>
#include <sys/stat.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include "lidar_localizer/util/data_structs.h"
#include "lidar_localizer/util/util_functions.h"

template <class PointSource, class PointTarget>
class LidarLocalizer
{
    enum class ThreadStatus{
        Sleeping,
        Running,
        Finished,
    };

    public:
        LidarLocalizer();
        virtual ~LidarLocalizer();
        void initPointsMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& pointcloud_ptr);
        void updatePointsMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& pointcloud_ptr);

        bool alignMap(const boost::shared_ptr< pcl::PointCloud<PointSource> >& pointcloud_ptr, const Pose& predict_pose);
        void writeLogFile(const std::string& log_file_directory_path);

        Pose getLocalizerPose() const;
        double getAlignTime() const;

    protected:
        virtual void align(const Pose& predict_pose) = 0;
        virtual void setInputTarget(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& map_ptr) = 0;
        virtual void setInputSource(const boost::shared_ptr< pcl::PointCloud<PointSource> >& scan_ptr) = 0;
        virtual Pose getFinalPose() = 0;

        virtual void buildMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& map_ptr) = 0;
        virtual void swapInstance() = 0;

        virtual std::stringstream logFileContent() const;

    private:
        void buildMapThread(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& map_ptr);
        bool swapMap();

        Pose localizer_pose_;

        bool is_init_map_;
        size_t map_point_size_;
        double align_time_;
        double fitness_score_;

        ThreadStatus thread_status_;
        std::chrono::system_clock::time_point thread_begin_time_;
};

#endif
