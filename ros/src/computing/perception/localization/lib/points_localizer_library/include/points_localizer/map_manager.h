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

#ifndef MAPMANAGER_H
#define MAPMANAGER_H

#include <sensor_msgs/PointCloud2.h>

class MapManager
{
    public:
        MapManager();

    private:
        void callbackPointsMapUpdated(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2_msg_ptr);
        void saveMap(const double x, const double y, const double width);
        void loadMap(const double x, const double y, const double width);

        double save_map_size_;
        double save_map_leaf_size_;

};

template <class PointSource, class PointTarget>
MapManager<PointSource, PointTarget>::MapManager()
    : save_map_size_(100.0)
    , save_map_leaf_size_(0.2)
{

}

template <class PointSource, class PointTarget>
void MapManager<PointSource, PointTarget>::callbackPointsMapUpdated(const sensor_msgs::PointCloud2::ConstPtr& pointcloud2_msg_ptr)
{
    pcl::PointCloud<PointSource> pointcloud;
    pcl::fromROSMsg(*pointcloud2_msg_ptr, pointcloud);
    updateMap(pointcloud);
}

template <class PointSource, class PointTarget>
void MapManager<PointSource, PointTarget>::saveMap(const double x, const double y, const double width)
{
    const auto save_start = std::chrono::system_clock::now();
    //TODO
    for(int i = -1; i <= 1; ++i) {
        for(int j = -1; j <= 1; ++j) {
            boost::shared_ptr< pcl::PointCloud<PointTarget> > map_region_tmp_ptr(new pcl::PointCloud<PointTarget>);
            passThroughPointCloud(map_raw_ptr_, map_region_tmp_ptr, i*width+x, j*width+y, width);

            std::string path = director_path_ + "/pointcloud_map_" + std::to_string(i*width+x) + "_" + std::to_string(j*width+y) + ".pcd";
            boost::shared_ptr< pcl::PointCloud<PointTarget> > load_cloud(new pcl::PointCloud<PointTarget>);

            if(map_raw_ptr_map_.count(path)) {
                std::cout << "exist " << path << std::endl;
                //load_cloud = map_raw_ptr_map_.at(path);
            }
            else if (pcl::io::loadPCDFile(path.c_str(), *load_cloud) == -1) {
                std::cerr << "load failed " << path << std::endl;
            }

            if(map_region_tmp_ptr->width == 0) {
                continue;
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

            if(map_raw_ptr_map_.count(path)) {
                map_raw_ptr_map_.at(path) = map_region_filtered_tmp_ptr;
            }
            else {
                map_raw_ptr_map_.emplace(path, map_region_filtered_tmp_ptr);
            }

            //TODO create area_list?
            pcl::io::savePCDFileBinary(path, *map_region_filtered_tmp_ptr);
        }
    }

    const auto save_end = std::chrono::system_clock::now();
    const auto save_time = std::chrono::duration_cast<std::chrono::microseconds>(save_end - save_start).count() / 1000.0;
    std::cout << "save_time: " << save_time << "ms" << std::endl;

}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::loadMap(const double x, const double y, const double width)
{
    std::vector<std::string> path_arrary;
    static std::vector<std::string> prev_path_arrary;

    for(int i = -1; i <= 1; ++i) {
        for(int j = -1; j <= 1; ++j) {
            std::string path = director_path_ + "/pointcloud_map_" + std::to_string(i*width+x) + "_" + std::to_string(j*width+y) + ".pcd";
            std::cout << path << std::endl;
            path_arrary.push_back(path);
        }
    }

    // for(const auto& prev_path : prev_path_arrary) {
    //     const auto it = std::find(std::begin(path_arrary), std::end(path_arrary), prev_path);
    //     //if not exist
    //     if(it == std::end(path_arrary)) {
    //         path_arrary.push_back(prev_path);
    //     }
    // }

    std::map< std::string , boost::shared_ptr< pcl::PointCloud<PointTarget> > > map_raw_ptr_map_tmp;
    for(const auto& path : path_arrary) {
        pcl::PointCloud<PointTarget> pointcloud;
        if(map_raw_ptr_map_.count(path)) {
            std::cout << "exist " << path << std::endl;
            map_raw_ptr_map_tmp.emplace(path, map_raw_ptr_map_.at(path));
            continue;
        }
        //TODO check file
        else if (pcl::io::loadPCDFile(path.c_str(), pointcloud) == -1) {
            std::cerr << "load failed " << path << std::endl;
            continue;
        }
        map_raw_ptr_map_tmp.emplace(path, pointcloud.makeShared());
    }
    map_raw_ptr_map_ = map_raw_ptr_map_tmp;

    pcl::PointCloud<PointTarget> pointclouds;
    pointclouds.height = 1;
    //pointclouds.points.reserve(30000000);
    for(const auto& pointcloud_map : map_raw_ptr_map_)
    {
        pointclouds.width += pointcloud_map.second->width;
        if(pointclouds.points.capacity() < pointclouds.width) {
            pointclouds.points.reserve(pointclouds.width);
        }
        for(const auto& point : pointcloud_map.second->points) {
            pointclouds.points.push_back(point);
        }
    }

    // pcl::PointCloud<PointTarget> pointclouds;
    // pointclouds.height = 1;
    // // //pointclouds.points.reserve(30000000);
    // for(const auto& path : path_arrary) {
    //     pcl::PointCloud<PointTarget> pointcloud;
    //     if (pcl::io::loadPCDFile(path.c_str(), pointcloud) == -1) {
    //         std::cerr << "load failed " << path << std::endl;
    //         continue;
    //     }
    //     pointclouds.width = pointclouds.width + pointcloud.width;
    //     if(pointclouds.points.capacity() < pointclouds.width) {
    //         pointclouds.points.reserve(pointclouds.width);
    //     }
    //     for(const auto& point : pointcloud.points) {
    //         pointclouds.points.push_back(point);
    //     }
    // }

    prev_path_arrary = path_arrary;

    if(map_raw_ptr_ == nullptr || pointclouds.points.size() != map_raw_ptr_->points.size()) {
        map_raw_ptr_ = pointclouds.makeShared();
        //map_filtered_ptr_ = boost::make_shared< pcl::PointCloud<PointTarget> >(pointclouds);
        map_raw_ptr_->points.reserve(30000000); //TODO
        buildMapThread(map_raw_ptr_);
    }
}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::updateMap(const pcl::PointCloud<PointSource>& pointcloud)
{
    std::cout << __func__ << std::endl;

    const int x = std::floor(localizer_pose_.x / save_map_size_);
    const int y = std::floor(localizer_pose_.y / save_map_size_);
    static Pose reload_pose(x, y, 0, 0, 0, 0);

    if(std::abs(x-reload_pose.x) > 0 || std::abs(y-reload_pose.y) > 0)
    {
        std::cout << "saveMap & loadMap" << std::endl;
        const auto save_start = std::chrono::system_clock::now();
        saveMap(reload_pose.x*save_map_size_, reload_pose.y*save_map_size_, save_map_size_);
        const auto save_end = std::chrono::system_clock::now();
        const auto save_time = std::chrono::duration_cast<std::chrono::microseconds>(save_end - save_start).count() / 1000.0;
        std::cout << "save_time: " << save_time << "ms" << std::endl;

        const auto load_start = std::chrono::system_clock::now();
        loadMap(x*save_map_size_, y*save_map_size_, save_map_size_);
        const auto load_end = std::chrono::system_clock::now();
        const auto load_time = std::chrono::duration_cast<std::chrono::microseconds>(load_end - load_start).count() / 1000.0;
        std::cout << "load_time: " << load_time << "ms" << std::endl;

        reload_pose.x = x;
        reload_pose.y = y;
    }
}

#endif
