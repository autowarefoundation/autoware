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

#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H

#include <deque>
#include <thread>
#include <mutex>
#include <functional>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include "lidar_localizer/util/data_structs.h"
#include "lidar_localizer/util/util_functions.h"

template <class PointTarget>
class MapManager
{
    public:
        MapManager();
        virtual ~MapManager();

        void addPointCloudMapThread(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& points_raw_ptr);
        void downsampleMapThread();
        void saveSingleMapThread();
        void saveSeparateMapThread();
        void loadAroundMapThread(const Pose& localizer_pose);

        void setMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& map_ptr);
        boost::shared_ptr< pcl::PointCloud<PointTarget> > getMap();
        void setSaveSeparateMapSize(const double separate_map_size);
        void setSaveMapLeafSize(const double save_map_leaf_size);
        double getSaveSeparateMapSize() const;
        void setFileDirectoryPath(const std::string& directory_path);

    private:
        void runProcess();

        void addPointCloudMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& points_raw_ptr);
        void downsampleMap();
        void saveSingleMap();
        void saveSeparateMap();
        void loadAroundMap(const Pose& localizer_pose);

        std::string directory_path_;
        double separate_map_size_;
        double save_map_leaf_size_;
        double default_reserve_size_;

        boost::shared_ptr< pcl::PointCloud<PointTarget> > map_ptr_;

        std::deque< std::function<void()> > process_queue_;
        std::thread process_thread_;
        std::mutex mtx_;
        bool is_thread_run_ok_;
};

#endif
