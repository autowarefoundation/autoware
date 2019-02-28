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

#include "lidar_localizer/map_manager/map_manager.h"

#include <boost/filesystem.hpp>

template <class PointTarget>
MapManager<PointTarget>::MapManager()
    : separate_map_size_(100.0)
    , save_map_leaf_size_(0.2)
    , default_reserve_size_(100000000)
    , is_thread_run_ok_(true)
{
    runProcess();
}

template <class PointTarget>
MapManager<PointTarget>::~MapManager()
{
    std::cout << __func__ << std::endl;
    is_thread_run_ok_ = false;
    process_thread_.join();
}

template <class PointTarget>
void MapManager<PointTarget>::runProcess()
{
    std::cout << "process_queue_size: " << process_queue_.size() << std::endl;

    process_thread_ = std::thread([this]() {
        while(is_thread_run_ok_) {
            if(process_queue_.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            std::cout << "process_queue_size threadA: " << process_queue_.size() << std::endl;

            queue_mtx_.lock();
            auto process = process_queue_.front();
            process_queue_.pop_front();
            queue_mtx_.unlock();

            process();

            std::cout << "process_queue_size threadB: " << process_queue_.size() << std::endl;
            std::cout << "process end" << std::endl;
    }
    });

}

template <class PointTarget>
void MapManager<PointTarget>::addPointCloudMapThread(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& points_raw_ptr)
{
    auto process = std::bind(&MapManager<PointTarget>::addPointCloudMap, this, points_raw_ptr);
    std::lock_guard<std::mutex> lock(queue_mtx_);
    process_queue_.push_back(process);
}

template <class PointTarget>
void MapManager<PointTarget>::downsampleMapThread()
{
    auto process = std::bind(&MapManager<PointTarget>::downsampleMap, this);
    std::lock_guard<std::mutex> lock(queue_mtx_);
    process_queue_.push_back(process);
}

template <class PointTarget>
void MapManager<PointTarget>::saveSingleMapThread()
{
    auto process = std::bind(&MapManager<PointTarget>::saveSingleMap, this);
    std::lock_guard<std::mutex> lock(queue_mtx_);
    process_queue_.push_back(process);
}

template <class PointTarget>
void MapManager<PointTarget>::saveSeparateMapThread()
{
    auto process = std::bind(&MapManager<PointTarget>::saveSeparateMap, this);
    std::lock_guard<std::mutex> lock(queue_mtx_);
    process_queue_.push_back(process);
}

template <class PointTarget>
void MapManager<PointTarget>::loadAroundMapThread(const Pose& localizer_pose)
{
    auto process = std::bind(&MapManager<PointTarget>::loadAroundMap, this, localizer_pose);
    std::lock_guard<std::mutex> lock(queue_mtx_);
    process_queue_.push_back(process);
}

template <class PointTarget>
void MapManager<PointTarget>::addPointCloudMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& points_raw_ptr)
{
    std::cout << __func__ << " start" << std::endl;
    const auto start_time = std::chrono::system_clock::now();

    if(thread_map_ptr_ == nullptr) {
        thread_map_ptr_ = boost::make_shared< pcl::PointCloud<PointTarget> >();
    }

    map_mtx_.lock();
    addPointCloud(points_raw_ptr, thread_map_ptr_);
    map_mtx_.unlock();

    const auto end_time = std::chrono::system_clock::now();
    const auto exe_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0;
    std::cout << __func__ << " time: " << exe_time << "ms" << std::endl;
}

template <class PointTarget>
void MapManager<PointTarget>::downsampleMap()
{
    std::cout << __func__ << " start" << std::endl;

    if(thread_map_ptr_ == nullptr) {
        std::cout << __func__ << "[ERROR] thread_map_ptr_ is nullptr" << std::endl;
        return;
    }

    const auto start_time = std::chrono::system_clock::now();

    pcl::PointCloud<PointTarget> filterd_map;
    pcl::VoxelGrid<PointTarget> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(save_map_leaf_size_, save_map_leaf_size_, save_map_leaf_size_);
    voxel_grid_filter.setInputCloud(thread_map_ptr_);
    voxel_grid_filter.filter(filterd_map);

    map_mtx_.lock();
    thread_map_ptr_ = filterd_map.makeShared();
    if(thread_map_ptr_->points.capacity() < default_reserve_size_) {
        thread_map_ptr_->points.reserve(default_reserve_size_);
    }
    map_mtx_.unlock();

    const auto end_time = std::chrono::system_clock::now();
    const auto exe_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0;
    std::cout << __func__ << " time: " << exe_time << "ms" << std::endl;
}

template <class PointTarget>
void MapManager<PointTarget>::saveSingleMap()
{
    std::cout << __func__ << " start" << std::endl;

    if(thread_map_ptr_ == nullptr) {
        std::cout << __func__ << "[ERROR] thread_map_ptr_ is nullptr" << std::endl;
        return;
    }

    const auto start_time = std::chrono::system_clock::now();

    std::string path = directory_path_ + "/pointcloud_map.pcd";
    pcl::io::savePCDFileBinary(path, *thread_map_ptr_);

    const auto end_time = std::chrono::system_clock::now();
    const auto exe_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0;
    std::cout << __func__ << " time: " << exe_time << "ms" << std::endl;
}

template <class PointTarget>
void MapManager<PointTarget>::saveSeparateMap()
{
    std::cout << __func__ << " start" << std::endl;

    if(thread_map_ptr_ == nullptr) {
        std::cout << __func__ << "[ERROR] thread_map_ptr_ is nullptr" << std::endl;
        return;
    }

    const auto start_time = std::chrono::system_clock::now();

    double min_x_m = calcMinX(thread_map_ptr_);
    double max_x_m = calcMaxX(thread_map_ptr_);
    double min_y_m = calcMinY(thread_map_ptr_);
    double max_y_m = calcMaxY(thread_map_ptr_);

    //TODO bug?
    const int min_x = std::floor(min_x_m / separate_map_size_);
    const int max_x = std::floor(max_x_m / separate_map_size_);
    const int min_y = std::floor(min_y_m / separate_map_size_);
    const int max_y = std::floor(max_y_m / separate_map_size_);

    // std::cout << "min_x_m:" << min_x_m << " max_x_m:" << max_x_m << std::endl;
    // std::cout << "min_y_m:" << min_y_m << " max_y_m:" << max_y_m << std::endl;
    // std::cout << "min_x:" << min_x << " max_x:" << max_x << std::endl;
    // std::cout << "min_y:" << min_y << " max_y:" << max_y << std::endl;

    //TODO
    for(int i = min_x; i <= max_x; ++i) {
        for(int j = min_y; j <= max_y; ++j) {
            boost::shared_ptr< pcl::PointCloud<PointTarget> > map_region_tmp_ptr(new pcl::PointCloud<PointTarget>);
            passThroughPointCloud<PointTarget>(thread_map_ptr_, map_region_tmp_ptr, i*separate_map_size_, j*separate_map_size_, separate_map_size_);

            if(map_region_tmp_ptr->width == 0) {
                continue;
            }

            std::string path = directory_path_ + "/pointcloud_map_" + std::to_string(i*separate_map_size_) + "_" + std::to_string(j*separate_map_size_) + ".pcd";
            boost::shared_ptr< pcl::PointCloud<PointTarget> > load_cloud(new pcl::PointCloud<PointTarget>);

            // if(map_ptr_map_.count(path)) {
            //     //std::cout << "exist " << path << std::endl;
            //     load_cloud = map_ptr_map_.at(path);
            // }
            boost::system::error_code file_error;
            const bool exist_file = boost::filesystem::exists(path.c_str(), file_error);
            if(!exist_file || file_error){
                std::cout << "no exist " << path << std::endl;
            }
            else if (pcl::io::loadPCDFile(path.c_str(), *load_cloud) == -1) {
                std::cout << "load failed " << path << std::endl;
            }

            map_region_tmp_ptr->height = 1;
            map_region_tmp_ptr->width += load_cloud->width;

            if(map_region_tmp_ptr->points.capacity() < map_region_tmp_ptr->width) {
                map_region_tmp_ptr->points.reserve(map_region_tmp_ptr->width);
            }

            //TODO
            for(const auto& point : load_cloud->points) {
                map_region_tmp_ptr->points.push_back(point);
            }

            // if(map_ptr_map_.count(path)) {
            //     map_ptr_map_.at(path) = map_region_filtered_tmp_ptr;
            // }
            // else {
            //     map_ptr_map_.emplace(path, map_region_filtered_tmp_ptr);
            // }

            //TODO create area_list?
            pcl::io::savePCDFileBinary(path, *map_region_tmp_ptr);
        }
    }

    const auto end_time = std::chrono::system_clock::now();
    const auto exe_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0;
    std::cout << __func__ << " time: " << exe_time << "ms" << std::endl;
}

template <class PointTarget>
void MapManager<PointTarget>::loadAroundMap(const Pose& localizer_pose)
{
    if(thread_map_ptr_ == nullptr) {
        std::cout << __func__ << "[ERROR] thread_map_ptr_ is nullptr" << std::endl;
        return;
    }

    std::cout << __func__ << " start" << std::endl;
    const auto start_time = std::chrono::system_clock::now();

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

    std::map< std::string , boost::shared_ptr< pcl::PointCloud<PointTarget> > > tmp_map_ptr;
    for(const auto& path : path_arrary) {
        pcl::PointCloud<PointTarget> pointcloud;
        // if(tmp_map_ptr.count(path)) {
        //     std::cout << "exist " << path << std::endl;
        //     tmp_map_ptr.emplace(path, tmp_map_ptr.at(path));
        //     continue;
        // }
        //TODO check file
        boost::system::error_code file_error;
        const bool exist_file = boost::filesystem::exists(path.c_str(), file_error);
        if(!exist_file || file_error){
            std::cout << "no exist " << path << std::endl;
            continue;
        }
        if (pcl::io::loadPCDFile(path.c_str(), pointcloud) == -1) {
            std::cout << "load failed " << path << std::endl;
            continue;
        }
        tmp_map_ptr.emplace(path, pointcloud.makeShared());
    }
    // tmp_map_ptr = tmp_map_ptr;

    boost::shared_ptr< pcl::PointCloud<PointTarget> > map_ptr(new pcl::PointCloud<PointTarget>);
    map_ptr->height = 1;
    for(const auto& pointcloud_map : tmp_map_ptr)
    {
        // pointclouds.width += pointcloud_map.second->width;
        // if(pointclouds.points.capacity() < pointclouds.width) {
        //     pointclouds.points.reserve(pointclouds.width);
        // }
        // for(const auto& point : pointcloud_map.second->points) {
        //     pointclouds.points.push_back(point);
        // }
        //
        addPointCloud(pointcloud_map.second, map_ptr);
    }

    map_mtx_.lock();
    thread_map_ptr_ = map_ptr;
    if(thread_map_ptr_->points.capacity() < default_reserve_size_) {
        thread_map_ptr_->points.reserve(default_reserve_size_);
    }
    map_mtx_.unlock();


    const auto end_time = std::chrono::system_clock::now();
    const auto exe_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0;
    std::cout << __func__ << " time: " << exe_time << "ms " << thread_map_ptr_->points.size() << std::endl;
}

template <class PointTarget>
void MapManager<PointTarget>::setMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& map_ptr)
{
    map_mtx_.lock();
    map_ptr_ = map_ptr;
    thread_map_ptr_ = map_ptr;
    map_mtx_.unlock();
}

template <class PointTarget>
boost::shared_ptr< pcl::PointCloud<PointTarget> >  MapManager<PointTarget>::getMap()
{
    map_mtx_.lock();
    map_ptr_ = thread_map_ptr_;
    map_mtx_.unlock();
    return map_ptr_;
}

template <class PointTarget>
void MapManager<PointTarget>::setSaveSeparateMapSize(const double separate_map_size)
{
    separate_map_size_ = separate_map_size;
}

template <class PointTarget>
double MapManager<PointTarget>::getSaveSeparateMapSize() const
{
    return separate_map_size_;
}

template <class PointTarget>
void MapManager<PointTarget>::setSaveMapLeafSize(const double save_map_leaf_size)
{
    save_map_leaf_size_ = save_map_leaf_size;
}

template <class PointTarget>
void MapManager<PointTarget>::setFileDirectoryPath(const std::string& directory_path)
{
    directory_path_ = directory_path;
}


template class MapManager<pcl::PointXYZ>;
template class MapManager<pcl::PointXYZI>;
