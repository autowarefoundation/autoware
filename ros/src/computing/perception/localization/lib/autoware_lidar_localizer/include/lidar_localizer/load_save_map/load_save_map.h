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

#ifndef LOAD_SAVE_MAP_H
#define LOAD_SAVE_MAP_H

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include "lidar_localizer/util/data_structs.h"
#include "lidar_localizer/util/util_functions.h"

template <class PointSource, class PointTarget>
class LoadSaveMap
{
    public:
        LoadSaveMap();
//        virtual ~LoadSaveMap() = default;
        virtual ~LoadSaveMap();
        void saveSingleMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_raw_ptr) const;
        void saveSeparateMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_raw_ptr) const;
        boost::shared_ptr< pcl::PointCloud<PointTarget> > loadAroundMap(const Pose& localizer_pose) const;

        void setSaveSeparateMapSize(const double separate_map_size);
        void setSaveMapLeafSize(const double save_map_leaf_size);
        double getSaveSeparateMapSize() const;
        void setFileDirectoryPath(const std::string& directory_path);

    private:
        std::string directory_path_;
        double separate_map_size_;
        double save_map_leaf_size_;
};

template <class PointSource, class PointTarget>
LoadSaveMap<PointSource, PointTarget>::LoadSaveMap()
    :separate_map_size_(100.0)
    ,save_map_leaf_size_(0.2)
{
}

template <class PointSource, class PointTarget>
LoadSaveMap<PointSource, PointTarget>::~LoadSaveMap()
{
}

template <class PointSource, class PointTarget>
void LoadSaveMap<PointSource, PointTarget>::saveSingleMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_raw_ptr) const
{
    boost::shared_ptr< pcl::PointCloud<PointTarget> > map_filtered_tmp_ptr(new pcl::PointCloud<PointTarget>);

    const auto down_start = std::chrono::system_clock::now();
    pcl::VoxelGrid<PointTarget> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(save_map_leaf_size_, save_map_leaf_size_, save_map_leaf_size_);
    voxel_grid_filter.setInputCloud(map_raw_ptr);
    voxel_grid_filter.filter(*map_filtered_tmp_ptr);
    const auto down_end = std::chrono::system_clock::now();
    const auto down_time = std::chrono::duration_cast<std::chrono::microseconds>(down_end - down_start).count() / 1000.0;
    std::cout << "down_time: " << down_time << "ms" << std::endl;

    std::string path = directory_path_ + "/pointcloud_map.pcd";
    pcl::io::savePCDFileBinary(path, *map_filtered_tmp_ptr);
}

template <class PointSource, class PointTarget>
void LoadSaveMap<PointSource, PointTarget>::saveSeparateMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_raw_ptr) const
{
    // double min_x_m =  calcMinX(map_raw_ptr);
    // double max_x_m =  calcMaxX(map_raw_ptr);
    // double min_y_m =  calcMinY(map_raw_ptr);
    // double max_y_m =  calcMaxY(map_raw_ptr);

    double min_x_m = 0;
    double max_x_m = 0;
    double min_y_m = 0;
    double max_y_m = 0;

    //TODO bug?
    const int min_x = std::floor(min_x_m / separate_map_size_);
    const int max_x = std::floor(max_x_m / separate_map_size_);
    const int min_y = std::floor(min_y_m / separate_map_size_);
    const int max_y = std::floor(max_y_m / separate_map_size_);

    // std::cout << "min_x_m:" << min_x_m << " max_x_m:" << max_x_m << std::endl;
    // std::cout << "min_y_m:" << min_y_m << " max_y_m:" << max_y_m << std::endl;
    // std::cout << "min_x:" << min_x << " max_x:" << max_x << std::endl;
    // std::cout << "min_y:" << min_y << " max_y:" << max_y << std::endl;

    const auto save_start = std::chrono::system_clock::now();
    //TODO
    for(int i = min_x; i <= max_x; ++i) {
        for(int j = min_y; j <= max_y; ++j) {
            boost::shared_ptr< pcl::PointCloud<PointTarget> > map_region_tmp_ptr(new pcl::PointCloud<PointTarget>);
            passThroughPointCloud<PointTarget>(map_raw_ptr, map_region_tmp_ptr, i*separate_map_size_, j*separate_map_size_, separate_map_size_);

            if(map_region_tmp_ptr->width == 0) {
                continue;
            }

            std::string path = directory_path_ + "/pointcloud_map_" + std::to_string(i*separate_map_size_) + "_" + std::to_string(j*separate_map_size_) + ".pcd";
            boost::shared_ptr< pcl::PointCloud<PointTarget> > load_cloud(new pcl::PointCloud<PointTarget>);

            // if(map_raw_ptr_map_.count(path)) {
            //     //std::cout << "exist " << path << std::endl;
            //     load_cloud = map_raw_ptr_map_.at(path);
            // }
            //TODO check exist File
            if (pcl::io::loadPCDFile(path.c_str(), *load_cloud) == -1) {
                std::cerr << "load failed " << path << std::endl;
            }

            map_region_tmp_ptr->height = 1;
            map_region_tmp_ptr->width += load_cloud->width;

            if(map_region_tmp_ptr->points.capacity() < map_region_tmp_ptr->width) {
                map_region_tmp_ptr->points.reserve(map_region_tmp_ptr->width);
            }

            for(const auto& point : load_cloud->points) {
                map_region_tmp_ptr->points.push_back(point);
            }

            const auto down_start = std::chrono::system_clock::now();
            boost::shared_ptr< pcl::PointCloud<PointTarget> > map_region_filtered_tmp_ptr(new pcl::PointCloud<PointTarget>);

            pcl::VoxelGrid<PointTarget> voxel_grid_filter;
            voxel_grid_filter.setLeafSize(save_map_leaf_size_, save_map_leaf_size_, save_map_leaf_size_);
            voxel_grid_filter.setInputCloud(map_region_tmp_ptr);
            voxel_grid_filter.filter(*map_region_filtered_tmp_ptr);
            const auto down_end = std::chrono::system_clock::now();
            const auto down_time = std::chrono::duration_cast<std::chrono::microseconds>(down_end - down_start).count() / 1000.0;
            std::cout << "down_time: " << down_time << "ms" << std::endl;

            // if(map_raw_ptr_map_.count(path)) {
            //     map_raw_ptr_map_.at(path) = map_region_filtered_tmp_ptr;
            // }
            // else {
            //     map_raw_ptr_map_.emplace(path, map_region_filtered_tmp_ptr);
            // }

            //TODO create area_list?
            pcl::io::savePCDFileBinary(path, *map_region_filtered_tmp_ptr);
        }
    }

    const auto save_end = std::chrono::system_clock::now();
    const auto save_time = std::chrono::duration_cast<std::chrono::microseconds>(save_end - save_start).count() / 1000.0;
    std::cout << "save_time: " << save_time << "ms" << std::endl;

}

template <class PointSource, class PointTarget>
boost::shared_ptr< pcl::PointCloud<PointTarget> > LoadSaveMap<PointSource, PointTarget>::loadAroundMap(const Pose& localizer_pose) const
{
    std::vector<std::string> path_arrary;

    const int x = std::floor(localizer_pose.x / separate_map_size_);
    const int y = std::floor(localizer_pose.y / separate_map_size_);

    for(int i = -1; i <= 1; ++i) {
        for(int j = -1; j <= 1; ++j) {
            std::string path = directory_path_ + "/pointcloud_map_" + std::to_string((i+x)*separate_map_size_) + "_" + std::to_string((j+y)*separate_map_size_) + ".pcd";
            std::cout << path << std::endl;
            path_arrary.push_back(path);
        }
    }

    std::map< std::string , boost::shared_ptr< pcl::PointCloud<PointTarget> > > map_raw_ptr_map_tmp;
    for(const auto& path : path_arrary) {
        pcl::PointCloud<PointTarget> pointcloud;
        // if(map_raw_ptr_map_.count(path)) {
        //     std::cout << "exist " << path << std::endl;
        //     map_raw_ptr_map_tmp.emplace(path, map_raw_ptr_map_.at(path));
        //     continue;
        // }
        //TODO check file
        if (pcl::io::loadPCDFile(path.c_str(), pointcloud) == -1) {
            std::cerr << "load failed " << path << std::endl;
            continue;
        }
        map_raw_ptr_map_tmp.emplace(path, pointcloud.makeShared());
    }
    // map_raw_ptr_map_ = map_raw_ptr_map_tmp;

    pcl::PointCloud<PointTarget> pointclouds;
    pointclouds.height = 1;
    for(const auto& pointcloud_map : map_raw_ptr_map_tmp)
    {
        pointclouds.width += pointcloud_map.second->width;
        if(pointclouds.points.capacity() < pointclouds.width) {
            pointclouds.points.reserve(pointclouds.width);
        }
        for(const auto& point : pointcloud_map.second->points) {
            pointclouds.points.push_back(point);
        }
    }
    return pointclouds.makeShared();
}

template <class PointSource, class PointTarget>
void LoadSaveMap<PointSource, PointTarget>::setSaveSeparateMapSize(const double separate_map_size)
{
    separate_map_size_ = separate_map_size;
}

template <class PointSource, class PointTarget>
double LoadSaveMap<PointSource, PointTarget>::getSaveSeparateMapSize() const
{
    return separate_map_size_;
}

template <class PointSource, class PointTarget>
void LoadSaveMap<PointSource, PointTarget>::setSaveMapLeafSize(const double save_map_leaf_size)
{
    save_map_leaf_size_ = save_map_leaf_size;
}
template <class PointSource, class PointTarget>
void LoadSaveMap<PointSource, PointTarget>::setFileDirectoryPath(const std::string& directory_path)
{
    directory_path_ = directory_path;
}

#endif
