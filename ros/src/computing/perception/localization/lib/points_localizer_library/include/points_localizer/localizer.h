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

#ifndef LIBLOCALIZER_H
#define LIBLOCALIZER_H

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

#include <tf/tf.h>

#include "points_localizer/util/libdata_structs.h"

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

template <class PointTarget>
static void passThroughPointCloud(const boost::shared_ptr< pcl::PointCloud<PointTarget> > input_point_cloud_ptr, boost::shared_ptr< pcl::PointCloud<PointTarget> > output_point_cloud_ptr, const double x, const double y, const double width)
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

template <class PointTarget>
static void addPointCloud(const boost::shared_ptr< pcl::PointCloud<PointTarget> > input_ptr, boost::shared_ptr< pcl::PointCloud<PointTarget> > &output_ptr, const Pose& pose)
{
    boost::shared_ptr< pcl::PointCloud<PointTarget> > transformed_input_ptr(new pcl::PointCloud<PointTarget>);
    const auto eigen_pose = convertToEigenMatrix4f(pose);
    pcl::transformPointCloud(*input_ptr, *transformed_input_ptr, eigen_pose);
    *output_ptr += *transformed_input_ptr;

        // size_t width = output_ptr->width;
        // output_ptr->width = output_ptr->width + transformed_input_ptr->width;
        // output_ptr->height = 1;
        // // if(output_ptr->points.capacity() < output_ptr->width) {
        // //     output_ptr->points.reserve(output_ptr->width * 2);
        // // }
        // //
        // // for(const auto& point : transformed_input_ptr->points) {
        // //     output_ptr->points.push_back(point);
        // // }
        //
        // output_ptr->resize(output_ptr->width);
        // for(size_t i = width; i < output_ptr->points.size(); ++i) {
        //     output_ptr->points[i] = transformed_input_ptr->points[i -width];
        // }

}

template <class PointTarget>
static void removePointCloudAroundSensor(const boost::shared_ptr< pcl::PointCloud<PointTarget> > input_ptr, boost::shared_ptr< pcl::PointCloud<PointTarget> > output_ptr, double remove_range_meter)
{
    double r = 0;
    for (const auto& p : input_ptr->points) {
      r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
      if (r > remove_range_meter) {
        output_ptr->push_back(p);
      }
    }
}

template <class PointSource, class PointTarget>
class LibLocalizer
{
    public:
        LibLocalizer();
//        virtual ~LibLocalizer() = default;
        virtual ~LibLocalizer();
        void setPointsMap(const pcl::PointCloud<PointTarget>& pointcloud);
        void updatePointsMap(const pcl::PointCloud<PointTarget>& pointcloud);
        void updateManualPose(const Pose& pose, const double current_time_sec);
        void updateStaticPose(const Pose& pose, const double current_time_sec);
        void updateGnssPose(const Pose& pose, const double current_time_sec);
        void updateDeadReconing(const Velocity& velocity, const double current_time_sec);
        bool updateLocalizer(const pcl::PointCloud<PointSource>& pointcloud, const double current_time_sec);
        void updateMap(const pcl::PointCloud<PointSource>& pointcloud);
        void writeLogFile();

        void setSaveMapSize(const double save_map_size);
        void setSaveMapLeafSize(const double save_map_leaf_size);
        void setMinScanRange(const double min_scan_range);
        void setMinAddScanShift(const double min_add_scan_shift);
        Pose getLocalizerPose() const;
        Velocity getLocalizerVelocity() const;
        pcl::PointCloud<PointTarget> getMap() const;
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
        void saveMap(const double x, const double y, const double width);
        void loadMap(const double x, const double y, const double width);

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
        double save_map_size_;
        double save_map_leaf_size_;
        double min_scan_range_;
        double min_add_scan_shift_;
        std::string director_path_;

        enum class ThreadStatus{sleeping, running, finished};
        ThreadStatus thread_status_;
};

template <class PointSource, class PointTarget>
LibLocalizer<PointSource, PointTarget>::LibLocalizer()
    :is_initial_pose_set_(false)
    ,current_time_sec(0)
    ,previous_time_sec(0)
    ,align_time_(0)
    ,fitness_score_(0)
    ,save_map_size_(100.0)
    ,save_map_leaf_size_(0.2)
    ,min_scan_range_(5.0)
    ,min_add_scan_shift_(1.0)
    ,thread_status_(ThreadStatus::sleeping)
{
    const std::time_t now = std::time(NULL);
    const std::tm* pnow = std::localtime(&now);
    char buffer[80];
    std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
    director_path_ = "points_localizer_" + std::string(buffer);
    mkdir(director_path_.c_str(), 0755);
}

template <class PointSource, class PointTarget>
LibLocalizer<PointSource, PointTarget>::~LibLocalizer()
{
    //TODO save last map
}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::setPointsMap(const pcl::PointCloud<PointTarget>& pointcloud)
{
    std::cout << __func__ << std::endl;
    if(map_raw_ptr_ == nullptr || pointcloud.points.size() != map_raw_ptr_->points.size()) {
        map_raw_ptr_ = boost::make_shared< pcl::PointCloud<PointTarget> >();
        removePointCloudAroundSensor(pointcloud.makeShared(), map_raw_ptr_, min_scan_range_);
        setInputTarget(map_raw_ptr_);
        is_initial_pose_set_ = true; //TODO
    }
}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::updatePointsMap(const pcl::PointCloud<PointTarget>& pointcloud)
{
    std::cout << __func__ << std::endl;
    if(map_raw_ptr_ == nullptr) {
        map_raw_ptr_ = boost::make_shared< pcl::PointCloud<PointTarget> >(pointcloud);
        setInputTarget(map_raw_ptr_);
    }
    else if (pointcloud.points.size() != map_raw_ptr_->points.size()){
        map_raw_ptr_ = boost::make_shared< pcl::PointCloud<PointTarget> >(pointcloud);
        buildMapThread(map_raw_ptr_);

    }
}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::buildMapThread(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_ptr)
{
    //not want to make many threads
    if(thread_status_ != ThreadStatus::sleeping) {
        return;
    }

    thread_status_ = ThreadStatus::running;

    std::thread build_map_thread([this, map_ptr](){
        buildMap(map_ptr);
        thread_status_ = ThreadStatus::finished;
    });

    build_map_thread.detach();

}

template <class PointSource, class PointTarget>
bool LibLocalizer<PointSource, PointTarget>::swapMap()
{
    if(thread_status_ != ThreadStatus::finished) {
        return false;
    }

    swapInstance();
    thread_status_ = ThreadStatus::sleeping;
    return true;
}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::updateManualPose(const Pose& pose, const double current_time_sec)
{
    std::cout << __func__ << std::endl;

    previous_time_sec = previous_time_sec == 0 ? current_time_sec : this->current_time_sec;
    this->current_time_sec = current_time_sec;

    localizer_pose_ = pose;

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
void LibLocalizer<PointSource, PointTarget>::updateStaticPose(const Pose& pose, const double current_time_sec)
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
void LibLocalizer<PointSource, PointTarget>::updateGnssPose(const Pose& pose, const double current_time_sec)
{
    std::cout << __func__ << "   " << fitness_score_ <<std::endl;
    if(is_initial_pose_set_ == false || fitness_score_ >= 500.0) {

        previous_time_sec = previous_time_sec == 0 ? current_time_sec : this->current_time_sec;
        this->current_time_sec = current_time_sec;

        localizer_pose_ = pose;

        previous_localizer_pose_ = localizer_pose_;

        localizer_velocity_.clear();

        is_initial_pose_set_ = true;
    }
}

template <class PointSource, class PointTarget>
Pose LibLocalizer<PointSource, PointTarget>::predictNextPose(const Pose& previous_pose, const double previous_time_sec, const Velocity& velocity, const double next_time_sec) const
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
void LibLocalizer<PointSource, PointTarget>::updateDeadReconing(const Velocity& velocity, const double current_time_sec)
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
Velocity LibLocalizer<PointSource, PointTarget>::computeVelocity(const Pose& previous_pose, const double previous_time_sec, const Pose& current_pose, const double current_time_sec) const
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
bool LibLocalizer<PointSource, PointTarget>::updateLocalizer(const pcl::PointCloud<PointSource>& pointcloud, const double current_time_sec)
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

    setInputSource(pointcloud.makeShared());

    auto predict_pose = predictNextPose(localizer_pose_, previous_time_sec, localizer_velocity_, current_time_sec);

    const auto align_start = std::chrono::system_clock::now();
    align(predict_pose);
    const auto align_end = std::chrono::system_clock::now();
    align_time_ = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;
    std::cout << "align_time: " << align_time_ << "ms" << std::endl;

    // const auto calc_fitness_score_start = std::chrono::system_clock::now();
    // fitness_score_ = getFitnessScore();
    // //std::cout << fitness_score_ << std::endl;
    // const auto calc_fitness_score_end = std::chrono::system_clock::now();
    // const auto calc_fitness_score_time = std::chrono::duration_cast<std::chrono::microseconds>(calc_fitness_score_end - calc_fitness_score_start).count() / 1000.0;

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
Pose LibLocalizer<PointSource, PointTarget>::getLocalizerPose() const
{
    return localizer_pose_;
}

template <class PointSource, class PointTarget>
Velocity LibLocalizer<PointSource, PointTarget>::getLocalizerVelocity() const
{
    return localizer_velocity_;
}

template <class PointSource, class PointTarget>
double LibLocalizer<PointSource, PointTarget>::getAlignTime() const
{
    return align_time_;
}

template <class PointSource, class PointTarget>
pcl::PointCloud<PointTarget> LibLocalizer<PointSource, PointTarget>::getMap() const
{
    return *map_raw_ptr_;
}


template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::saveMap(const double x, const double y, const double width)
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

    static Pose added_pose;
    const double add_scan_shift_meter = std::sqrt(std::pow(localizer_pose_.x - added_pose.x, 2.0) + std::pow(localizer_pose_.y - added_pose.y, 2.0));
    if(add_scan_shift_meter >= min_add_scan_shift_) {

        boost::shared_ptr< pcl::PointCloud<PointTarget> > scan_removed_around_sensor_ptr(new pcl::PointCloud<PointTarget>);
        removePointCloudAroundSensor(pointcloud.makeShared(), scan_removed_around_sensor_ptr, min_scan_range_);

        const auto add_start = std::chrono::system_clock::now();
        addPointCloud(scan_removed_around_sensor_ptr, map_raw_ptr_, localizer_pose_);
        const auto add_end = std::chrono::system_clock::now();
        const auto add_time = std::chrono::duration_cast<std::chrono::microseconds>(add_end - add_start).count() / 1000.0;
        std::cout << "add_time: " << add_time << "ms" << std::endl;
        //addPointCloud(scan_removed_around_sensor_ptr, map_filtered_ptr_, localizer_pose_);

        buildMapThread(map_raw_ptr_);
    }

    //TODO
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



template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::writeLogFile()
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
std::stringstream LibLocalizer<PointSource, PointTarget>::logFileContent() const
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
void LibLocalizer<PointSource, PointTarget>::setSaveMapSize(const double save_map_size)
{
    save_map_size_ = save_map_size;
}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::setSaveMapLeafSize(const double save_map_leaf_size)
{
    save_map_leaf_size_ = save_map_leaf_size;
}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::setMinScanRange(const double min_scan_range)
{
    min_scan_range_ = min_scan_range;
}

template <class PointSource, class PointTarget>
void LibLocalizer<PointSource, PointTarget>::setMinAddScanShift(const double min_add_scan_shift)
{
    min_add_scan_shift_ = min_add_scan_shift;
}

#endif
