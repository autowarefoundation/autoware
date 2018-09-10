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

#ifndef LidarLocalizer_H
#define LidarLocalizer_H

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

static Eigen::Matrix4f convertToEigenMatrix4f(const Pose& pose)
{
    const Eigen::Translation3f translation(pose.x, pose.y, pose.z);
    const Eigen::AngleAxisf rotation_x(pose.roll, Eigen::Vector3f::UnitX());
    const Eigen::AngleAxisf rotation_y(pose.pitch, Eigen::Vector3f::UnitY());
    const Eigen::AngleAxisf rotation_z(pose.yaw, Eigen::Vector3f::UnitZ());
    const Eigen::Matrix4f m = (translation * rotation_z * rotation_y * rotation_x).matrix();
    return m;
}

static Pose convertToPose(const Eigen::Matrix4f& m)
{
  Pose pose;
  pose.x = m(0, 3);
  pose.y = m(1, 3);
  pose.z = m(2, 3);

  //reference to tf::getEulerYPR()
  if (std::fabs(m(2,0)) >= 1)
  {
    pose.yaw = 0;
    if (m(2,0) < 0)
    {
      pose.pitch = M_PI / 2.0;
      pose.roll = std::atan2(m(0,1),m(0,2));
    }
    else
    {
      pose.pitch = -M_PI / 2.0;
      pose.roll = std::atan2(-m(0,1),-m(0,2));
    }
  }
  else
  {
    pose.pitch = -std::asin(m(2,0));
    pose.roll  = std::atan2(m(2,1)/std::cos(pose.pitch),
                            m(2,2)/std::cos(pose.pitch));
    pose.yaw   = std::atan2(m(1,0)/std::cos(pose.pitch),
                            m(0,0)/std::cos(pose.pitch));
  }

  return pose;
}

static Pose transformToPose(const Pose& pose, const Eigen::Matrix4f& m)
{
  Eigen::Matrix4f eigen_pose = convertToEigenMatrix4f(pose);
  Eigen::Matrix4f trans_pose = eigen_pose * m;

  return convertToPose(trans_pose);
}

template <class PointType>
static void passThroughPointCloud(const boost::shared_ptr< pcl::PointCloud<PointType> const> &input_point_cloud_ptr, const boost::shared_ptr< pcl::PointCloud<PointType> > output_point_cloud_ptr, const double x, const double y, const double width)
{
    output_point_cloud_ptr->points.reserve(output_point_cloud_ptr->width);
    for(const auto& point : input_point_cloud_ptr->points)
    {
        if(  point.x >= x && point.x <= x+width
          && point.y >= y && point.y <= y+width) {
              output_point_cloud_ptr->points.push_back(point);
        }
    }
    output_point_cloud_ptr->width = output_point_cloud_ptr->points.size();
    output_point_cloud_ptr->height = 1;
}

template <class PointType>
static void addPointCloud(const boost::shared_ptr< pcl::PointCloud<PointType> const>& input_ptr, const boost::shared_ptr< pcl::PointCloud<PointType> >& output_ptr, const Pose& pose)
{
    boost::shared_ptr< pcl::PointCloud<PointType> > transformed_input_ptr(new pcl::PointCloud<PointType>);
    const auto eigen_pose = convertToEigenMatrix4f(pose);
    pcl::transformPointCloud(*input_ptr, *transformed_input_ptr, eigen_pose);
    const auto need_points_size = output_ptr->points.size()+transformed_input_ptr->points.size();
    if(output_ptr->points.capacity() < need_points_size) {
        output_ptr->points.reserve(need_points_size*2);
    }
    *output_ptr += *transformed_input_ptr;
}


//TODO use std::minmax_element
template <class T>
double calcMaxX(const boost::shared_ptr< pcl::PointCloud<T> > &cloud)
{
    if (cloud->empty())
        return 0;
    double height = std::max_element(cloud->begin(), cloud->end(), [](const T & lhs, const T & rhs)
    {
        return lhs.x < rhs.x;
    })->x;
    return height;
}

template <class T>
double calcMinX(const boost::shared_ptr< pcl::PointCloud<T> > &cloud)
{
    if (cloud->empty())
        return 0;
    double height = std::min_element(cloud->begin(), cloud->end(), [](const T & lhs, const T & rhs)
    {
        return lhs.x < rhs.x;
    })->x;
    return height;
}

template <class T>
double calcMaxY(const boost::shared_ptr< pcl::PointCloud<T> > &cloud)
{
    if (cloud->empty())
        return 0;
    double height = std::max_element(cloud->begin(), cloud->end(), [](const T & lhs, const T & rhs)
    {
        return lhs.y < rhs.y;
    })->y;
    return height;
}

template <class T>
double calcMinY(const boost::shared_ptr< pcl::PointCloud<T> > &cloud)
{
    if (cloud->empty())
        return 0;
    double height = std::min_element(cloud->begin(), cloud->end(), [](const T & lhs, const T & rhs)
    {
        return lhs.y < rhs.y;
    })->y;
    return height;
}

template <class PointSource, class PointTarget>
class LidarLocalizer
{
    public:
        LidarLocalizer();
//        virtual ~LidarLocalizer() = default;
        virtual ~LidarLocalizer();
        void setPointsMap(boost::shared_ptr< pcl::PointCloud<PointTarget> >& pointcloud_ptr);
        void updatePointsMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& pointcloud_ptr);
        void updateManualPose(const Pose& pose, const double current_time_sec);
        void updateStaticPose(const Pose& pose, const double current_time_sec);
        void updateGnssPose(const Pose& pose, const double current_time_sec);
        void updateDeadReconing(const Velocity& velocity, const double current_time_sec);
        bool updateLocalizer(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& pointcloud_ptr, const double current_time_sec);
        void addMap(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& pointcloud_ptr);
        void saveSingleMap();
        void saveSeparateMap();
        void loadAroundMap();
        void downsampleMap();
        void writeLogFile();

        void setSaveSeparateMapSize(const double separate_map_size);
        void setSaveMapLeafSize(const double save_map_leaf_size);
        void setMinScanRange(const double min_scan_range);
        void setMinAddScanShift(const double min_add_scan_shift);
        double getSaveSeparateMapSize() const;
        Pose getLocalizerPose() const;
        Velocity getLocalizerVelocity() const;
        const boost::shared_ptr< pcl::PointCloud<PointTarget> > getMapPtr() const;
        double getAlignTime() const;


    protected:
        virtual void align(const Pose& predict_pose) = 0;
        virtual void setInputTarget(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_ptr) = 0;
        virtual void setInputSource(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& scan_ptr) = 0;
        virtual Pose getFinalPose() = 0;

        virtual void buildMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_ptr) = 0;
        virtual void swapInstance() = 0;

        virtual std::stringstream logFileContent() const;

    private:
        void buildMapThread(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_ptr);
        bool swapMap();
        Velocity computeVelocity(const Pose& previous_pose, const double previous_time_sec, const Pose& current_pose, const double current_time_sec) const;
        Pose predictNextPose(const Pose& previous_pose, const double previous_time_sec, const Velocity& velocity, const double next_time_sec) const;

        Pose localizer_pose_;
        Pose previous_localizer_pose_;
        Velocity localizer_velocity_;

        boost::shared_ptr< pcl::PointCloud<PointTarget> > map_raw_ptr_;
        std::map< std::string , boost::shared_ptr< pcl::PointCloud<PointTarget> > > map_raw_ptr_map_;
        boost::shared_ptr< pcl::PointCloud<PointTarget> > map_filtered_ptr_;

        bool is_initial_pose_set_;
        double current_time_sec;
        double previous_time_sec;
        double align_time_;
        double fitness_score_;
        double separate_map_size_;
        double save_map_leaf_size_;
        std::string director_path_;

        enum class ThreadStatus{sleeping, running, finished};
        ThreadStatus thread_status_;
        std::chrono::system_clock::time_point thread_begin_time_;
};

template <class PointSource, class PointTarget>
LidarLocalizer<PointSource, PointTarget>::LidarLocalizer()
    :is_initial_pose_set_(false)
    ,current_time_sec(0)
    ,previous_time_sec(0)
    ,align_time_(0)
    ,fitness_score_(0)
    ,separate_map_size_(100.0)
    ,save_map_leaf_size_(0.2)
    ,thread_status_(ThreadStatus::sleeping)
{
    const std::time_t now = std::time(NULL);
    const std::tm* pnow = std::localtime(&now);
    char buffer[80];
    std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
    director_path_ = "lidar_localizer_" + std::string(buffer);
    mkdir(director_path_.c_str(), 0755);
}

template <class PointSource, class PointTarget>
LidarLocalizer<PointSource, PointTarget>::~LidarLocalizer()
{
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::setPointsMap(boost::shared_ptr< pcl::PointCloud<PointTarget> >& pointcloud_ptr)
{
    std::cout << __func__ << std::endl;
    if(map_raw_ptr_ == nullptr || pointcloud_ptr->points.size() != map_raw_ptr_->points.size()) {
        map_raw_ptr_ = pointcloud_ptr;
        setInputTarget(map_raw_ptr_);
        is_initial_pose_set_ = true; //TODO
    }
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::updatePointsMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& pointcloud_ptr)
{
    std::cout << __func__ << std::endl;
    if(map_raw_ptr_ == nullptr) {
        map_raw_ptr_ = pointcloud_ptr;
        setInputTarget(map_raw_ptr_);
    }
    else if (pointcloud_ptr->points.size() != map_raw_ptr_->points.size()){
        map_raw_ptr_ = pointcloud_ptr;
        buildMapThread(map_raw_ptr_);

    }
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::buildMapThread(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_ptr)
{
    //not want to make many threads
    if(thread_status_ != ThreadStatus::sleeping) {
        return;
    }
    thread_begin_time_ =  std::chrono::system_clock::now();
    thread_status_ = ThreadStatus::running;

    std::thread build_map_thread([this, map_ptr](){
        buildMap(map_ptr);
        thread_status_ = ThreadStatus::finished;
    });

    build_map_thread.detach();

}

template <class PointSource, class PointTarget>
bool LidarLocalizer<PointSource, PointTarget>::swapMap()
{
    //if it takes a lot of time to generate map, wait for thread
    auto thread_now_time =  std::chrono::system_clock::now();
    auto thread_processing_time_msec = std::chrono::duration_cast<std::chrono::microseconds>(thread_now_time - thread_begin_time_).count() / 1000.0;
    const double time_threshold_msec = 1000.0;
    if(thread_status_ == ThreadStatus::running && thread_processing_time_msec > time_threshold_msec) {
        std::cout << "waiting for finish thread" << std::endl;
        while(thread_status_ != ThreadStatus::finished) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    else if(thread_status_ != ThreadStatus::finished) {
         return false;
    }

    swapInstance();
    thread_status_ = ThreadStatus::sleeping;
    return true;
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::updateManualPose(const Pose& pose, const double current_time_sec)
{
    std::cout << __func__ << std::endl;

    previous_time_sec = previous_time_sec == 0 ? current_time_sec : this->current_time_sec;
    this->current_time_sec = current_time_sec;

    localizer_pose_ = pose;

    //TODO RANSAC?
    if(map_raw_ptr_ != nullptr) {
      double min_distance = DBL_MAX;
      double nearest_z = localizer_pose_.z;
      for(const auto& p : map_raw_ptr_->points) {
        double distance = hypot(localizer_pose_.x - p.x, localizer_pose_.y - p.y);
        if(distance < min_distance) {
          min_distance = distance;
          nearest_z = p.z;
        }
      }
      localizer_pose_.z = nearest_z;
    }

    previous_localizer_pose_ = localizer_pose_;

    localizer_velocity_.clear();

    is_initial_pose_set_ = true;
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::updateStaticPose(const Pose& pose, const double current_time_sec)
{
    std::cout << __func__ << std::endl;


    previous_time_sec = previous_time_sec == 0 ? current_time_sec : this->current_time_sec;
    this->current_time_sec = current_time_sec;
    localizer_pose_ = pose;

    previous_localizer_pose_ = localizer_pose_;

    localizer_velocity_.clear();

    is_initial_pose_set_ = true;
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::updateGnssPose(const Pose& pose, const double current_time_sec)
{
    std::cout << __func__ << "   " << fitness_score_ <<std::endl;
    //if(is_initial_pose_set_ == false || fitness_score_ >= 500.0) {
    if(is_initial_pose_set_ == false) {
        previous_time_sec = previous_time_sec == 0 ? current_time_sec : this->current_time_sec;
        this->current_time_sec = current_time_sec;

        localizer_pose_ = pose;

        previous_localizer_pose_ = localizer_pose_;

        localizer_velocity_.clear();

        is_initial_pose_set_ = true;
    }
}

template <class PointSource, class PointTarget>
Pose LidarLocalizer<PointSource, PointTarget>::predictNextPose(const Pose& previous_pose, const double previous_time_sec, const Velocity& velocity, const double next_time_sec) const
{
    const double time_diff_sec = next_time_sec - previous_time_sec;

    Pose next_pose;
    next_pose.x = previous_pose.x + velocity.linear.x * time_diff_sec;
    next_pose.y = previous_pose.y + velocity.linear.y * time_diff_sec;
    next_pose.z = previous_pose.z + velocity.linear.z * time_diff_sec;
    next_pose.roll = previous_pose.roll + velocity.angular.x * time_diff_sec;
    next_pose.pitch = previous_pose.pitch + velocity.angular.y * time_diff_sec;
    next_pose.yaw = previous_pose.yaw + velocity.angular.z * time_diff_sec;
    return next_pose;
}



template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::updateDeadReconing(const Velocity& velocity, const double current_time_sec)
{
    previous_time_sec = previous_time_sec == 0 ? current_time_sec : this->current_time_sec;
    this->current_time_sec = current_time_sec;

    Pose next_pose = predictNextPose(localizer_pose_, previous_time_sec, velocity, current_time_sec);

    std::cout << next_pose << " " << current_time_sec << " " << previous_time_sec << std::endl;
    previous_localizer_pose_ = localizer_pose_;
    localizer_pose_ = next_pose;

    localizer_velocity_ = velocity;
}


template <class PointSource, class PointTarget>
Velocity LidarLocalizer<PointSource, PointTarget>::computeVelocity(const Pose& previous_pose, const double previous_time_sec, const Pose& current_pose, const double current_time_sec) const
{
    const double diff_time = current_time_sec - previous_time_sec;
    const auto diff_pose = current_pose - previous_pose;
    Velocity velocity;
    velocity.linear.x = (diff_time > 0) ? (diff_pose.x / diff_time) : 0;
    velocity.linear.y = (diff_time > 0) ? (diff_pose.y / diff_time) : 0;
    velocity.linear.z = (diff_time > 0) ? (diff_pose.z / diff_time) : 0;
    velocity.angular.x =  (diff_time > 0) ? (diff_pose.roll / diff_time) : 0;
    velocity.angular.y =  (diff_time > 0) ? (diff_pose.pitch / diff_time) : 0;
    velocity.angular.z =  (diff_time > 0) ? (diff_pose.yaw / diff_time) : 0;

    return velocity;
}


template <class PointSource, class PointTarget>
bool LidarLocalizer<PointSource, PointTarget>::updateLocalizer(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& pointcloud_ptr, const double current_time_sec)
{
    std::cout << __func__ << std::endl;

    if(map_raw_ptr_ == nullptr) {
        std::cout << "[WARN]received points. But map is not loaded" << std::endl;
        return false;
    }
    if(is_initial_pose_set_ == false) {
        std::cout << "[WARN]received points. But initial pose is not set" << std::endl;
        return false;
    }

    previous_time_sec = previous_time_sec == 0 ? current_time_sec :this->current_time_sec;
    this->current_time_sec = current_time_sec;

    if(swapMap()) {
        std::cout << "map swapped" << std::endl;
    }
    setInputSource(pointcloud_ptr);

    auto predict_pose = predictNextPose(localizer_pose_, previous_time_sec, localizer_velocity_, current_time_sec);

    const auto align_start = std::chrono::system_clock::now();
    align(predict_pose);
    const auto align_end = std::chrono::system_clock::now();
    align_time_ = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;
    std::cout << "align_time: " << align_time_ << "ms" << std::endl;

    localizer_pose_ = getFinalPose();
    localizer_velocity_ = computeVelocity(previous_localizer_pose_, previous_time_sec, localizer_pose_, current_time_sec);

    std::cout << localizer_pose_ << std::endl;
    std::cout << previous_localizer_pose_ << std::endl;
    std::cout << predict_pose << std::endl;
    std::cout << localizer_velocity_ << std::endl;

    previous_localizer_pose_ = localizer_pose_;

    return true;
}

template <class PointSource, class PointTarget>
Pose LidarLocalizer<PointSource, PointTarget>::getLocalizerPose() const
{
    return localizer_pose_;
}

template <class PointSource, class PointTarget>
Velocity LidarLocalizer<PointSource, PointTarget>::getLocalizerVelocity() const
{
    return localizer_velocity_;
}

template <class PointSource, class PointTarget>
double LidarLocalizer<PointSource, PointTarget>::getAlignTime() const
{
    return align_time_;
}

template <class PointSource, class PointTarget>
const boost::shared_ptr< pcl::PointCloud<PointTarget> > LidarLocalizer<PointSource, PointTarget>::getMapPtr() const
{
    return map_raw_ptr_;
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::saveSingleMap()
{
    boost::shared_ptr< pcl::PointCloud<PointTarget> > map_filtered_tmp_ptr(new pcl::PointCloud<PointTarget>);

    const auto down_start = std::chrono::system_clock::now();
    pcl::VoxelGrid<PointTarget> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(save_map_leaf_size_, save_map_leaf_size_, save_map_leaf_size_);
    voxel_grid_filter.setInputCloud(map_raw_ptr_);
    voxel_grid_filter.filter(*map_filtered_tmp_ptr);
    const auto down_end = std::chrono::system_clock::now();
    const auto down_time = std::chrono::duration_cast<std::chrono::microseconds>(down_end - down_start).count() / 1000.0;
    std::cout << "down_time: " << down_time << "ms" << std::endl;

    std::string path = director_path_ + "/pointcloud_map.pcd";
    pcl::io::savePCDFileBinary(path, *map_filtered_tmp_ptr);
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::saveSeparateMap()
{
    double min_x_m =  calcMinX(map_raw_ptr_);
    double max_x_m =  calcMaxX(map_raw_ptr_);
    double min_y_m =  calcMinY(map_raw_ptr_);
    double max_y_m =  calcMaxY(map_raw_ptr_);

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
            passThroughPointCloud<PointTarget>(map_raw_ptr_, map_region_tmp_ptr, i*separate_map_size_, j*separate_map_size_, separate_map_size_);

            std::string path = director_path_ + "/pointcloud_map_" + std::to_string(i*separate_map_size_) + "_" + std::to_string(j*separate_map_size_) + ".pcd";
            boost::shared_ptr< pcl::PointCloud<PointTarget> > load_cloud(new pcl::PointCloud<PointTarget>);

            if(map_raw_ptr_map_.count(path)) {
                //std::cout << "exist " << path << std::endl;
                //load_cloud = map_raw_ptr_map_.at(path);
            }
            //TODO check exist File
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
void LidarLocalizer<PointSource, PointTarget>::loadAroundMap()
{
    std::vector<std::string> path_arrary;
    static std::vector<std::string> prev_path_arrary;

    const int x = std::floor(localizer_pose_.x / separate_map_size_);
    const int y = std::floor(localizer_pose_.y / separate_map_size_);

    for(int i = -1; i <= 1; ++i) {
        for(int j = -1; j <= 1; ++j) {
            std::string path = director_path_ + "/pointcloud_map_" + std::to_string((i+x)*separate_map_size_) + "_" + std::to_string((j+y)*separate_map_size_) + ".pcd";
            std::cout << path << std::endl;
            path_arrary.push_back(path);
        }
    }

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


    prev_path_arrary = path_arrary;

    if(map_raw_ptr_ == nullptr || pointclouds.points.size() != map_raw_ptr_->points.size()) {
        map_raw_ptr_ = pointclouds.makeShared();
        map_raw_ptr_->points.reserve(80000000); //TODO
        buildMapThread(map_raw_ptr_);
    }
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::addMap(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& pointcloud_ptr)
{
    std::cout << __func__ << std::endl;

    const auto add_start = std::chrono::system_clock::now();
    addPointCloud(pointcloud_ptr, map_raw_ptr_, localizer_pose_);
    const auto add_end = std::chrono::system_clock::now();
    const auto add_time = std::chrono::duration_cast<std::chrono::microseconds>(add_end - add_start).count() / 1000.0;
    std::cout << "add_time: " << add_time << "ms" << std::endl;
    buildMapThread(map_raw_ptr_);
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::downsampleMap()
{
    const auto down_start = std::chrono::system_clock::now();
    pcl::PointCloud<PointTarget> filterd_map;

    pcl::VoxelGrid<PointTarget> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(save_map_leaf_size_, save_map_leaf_size_, save_map_leaf_size_);
    voxel_grid_filter.setInputCloud(map_raw_ptr_);
    voxel_grid_filter.filter(filterd_map);
    map_raw_ptr_ = filterd_map.makeShared();
    map_raw_ptr_->points.reserve(80000000); //TODO
    const auto down_end = std::chrono::system_clock::now();
    const auto down_time = std::chrono::duration_cast<std::chrono::microseconds>(down_end - down_start).count() / 1000.0;
    std::cout << "down_time: " << down_time << "ms" << std::endl;
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::writeLogFile()
{
    static std::ofstream log_file_stream;
    static bool is_first_call = true;
    if(is_first_call) {
        const std::string filename = director_path_  + "/log.csv";
        log_file_stream.open(filename.c_str(), std::ios::app);
        is_first_call = false;
    }
    if (!log_file_stream) {
      std::cerr << "Could not open log file." << std::endl;
      //exit(1);
    }

    log_file_stream << logFileContent().str() << std::endl;
}

template <class PointSource, class PointTarget>
std::stringstream LidarLocalizer<PointSource, PointTarget>::logFileContent() const
{
    std::stringstream content;
    content << localizer_pose_.x << ","
            << localizer_pose_.y << ","
            << localizer_pose_.z << ","
            << localizer_pose_.roll << ","
            << localizer_pose_.pitch << ","
            << localizer_pose_.yaw << ","
            << align_time_;
    return content;
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::setSaveSeparateMapSize(const double separate_map_size)
{
    separate_map_size_ = separate_map_size;
}

template <class PointSource, class PointTarget>
double LidarLocalizer<PointSource, PointTarget>::getSaveSeparateMapSize() const
{
    return separate_map_size_;
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::setSaveMapLeafSize(const double save_map_leaf_size)
{
    save_map_leaf_size_ = save_map_leaf_size;
}

#endif
