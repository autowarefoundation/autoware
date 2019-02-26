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
//        virtual ~LidarLocalizer() = default;
        virtual ~LidarLocalizer();
        void initPointsMap(boost::shared_ptr< pcl::PointCloud<PointTarget> >& pointcloud_ptr);
        void updatePointsMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& pointcloud_ptr);

        bool alignMap(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& pointcloud_ptr, const Pose& predict_pose);
        void writeLogFile(const std::string& log_file_directory_path);

        Pose getLocalizerPose() const;
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

        pcl::PointCloud<PointTarget> map_raw_;
        boost::shared_ptr< pcl::PointCloud<PointTarget> > map_raw_ptr_;
        Pose localizer_pose_;

        double align_time_;
        double fitness_score_;
        size_t default_reserve_size_;

        std::thread build_map_thread_;
        ThreadStatus thread_status_;
        std::chrono::system_clock::time_point thread_begin_time_;
};

template <class PointSource, class PointTarget>
LidarLocalizer<PointSource, PointTarget>::LidarLocalizer()
    : localizer_pose_(1.2, 0, 2.0, 0, 0, 0)
    , align_time_(0)
    , fitness_score_(0)
    , default_reserve_size_(100000000)
    , thread_status_(ThreadStatus::Sleeping)
{

}

template <class PointSource, class PointTarget>
LidarLocalizer<PointSource, PointTarget>::~LidarLocalizer()
{
    std::cerr << __func__ << std::endl;
    build_map_thread_.join();
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::initPointsMap(boost::shared_ptr< pcl::PointCloud<PointTarget> >& pointcloud_ptr)
{
    std::cout << __func__ << std::endl;
    map_raw_ptr_ = pointcloud_ptr;
    // map_raw_ptr_->points.reserve(default_reserve_size_);
    setInputTarget(map_raw_ptr_);
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::updatePointsMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& pointcloud_ptr)
{
    std::cout << __func__ << std::endl;

    if(pointcloud_ptr == nullptr){
        std::cout << "[ERROR] pointcloud_ptr is nullptr" << std::endl;
        return;
    }


    if(map_raw_ptr_ == nullptr) {
        map_raw_ptr_ = pointcloud_ptr;
        std::cout << pointcloud_ptr->points.size() << " " <<  map_raw_ptr_->points.size() << std::endl;
        // map_raw_ptr_->points.reserve(default_reserve_size_);
        setInputTarget(map_raw_ptr_);
        // TODO replace
        //initPointsMap(map_raw_ptr_)
    }
    else if (pointcloud_ptr->points.size() != map_raw_ptr_->points.size()){
        std::cout << pointcloud_ptr->points.size() << " " <<  map_raw_ptr_->points.size() << std::endl;
        map_raw_ptr_ = pointcloud_ptr;
        // map_raw_ = *pointcloud_ptr;
        // map_raw_ptr_ = map_raw_.makeShared();
        //map_raw_ptr_->points.reserve(default_reserve_size_);
        buildMapThread(map_raw_ptr_);

    }
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::buildMapThread(const boost::shared_ptr< pcl::PointCloud<PointTarget> const>& map_ptr)
{
    //donot create multiple threads
    if(thread_status_ != ThreadStatus::Sleeping) {
        return;
    }

    thread_begin_time_ =  std::chrono::system_clock::now();
    thread_status_ = ThreadStatus::Running;

    // build_map_thread_ = std::thread([this, map_ptr](){
    //     std::cout << __func__ << " " << map_ptr << std::endl;
    //     buildMap(map_ptr);
    //     std::cout << __func__ << " " << map_ptr << std::endl;
    //     thread_status_ = ThreadStatus::Finished;
    // });

    //TODO
    std::thread build_map_thread([this, map_ptr](){
        buildMap(map_ptr);
        thread_status_ = ThreadStatus::Finished;
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
    if(thread_status_ == ThreadStatus::Running && thread_processing_time_msec > time_threshold_msec) {
        std::cout << "*************************" << std::endl;
        std::cout << "waiting for finish thread" << std::endl;
        std::cout << "*************************" << std::endl;
        //TODO replace? thread.join()
        while(thread_status_ != ThreadStatus::Finished) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    else if(thread_status_ != ThreadStatus::Finished) {
         return false;
    }

    swapInstance();
    thread_status_ = ThreadStatus::Sleeping;
    return true;
}

template <class PointSource, class PointTarget>
bool LidarLocalizer<PointSource, PointTarget>::alignMap(const boost::shared_ptr< pcl::PointCloud<PointSource> const>& pointcloud_ptr, const Pose& predict_pose)
{
    std::cout << __func__ << std::endl;

    if(map_raw_ptr_ == nullptr) {
        std::cout << "[WARN]received points. But map is not loaded" << std::endl;
        return false;
    }

    if(swapMap()) {
        std::cout << "map swapped" << std::endl;
    }

    setInputSource(pointcloud_ptr);

    const auto align_start = std::chrono::system_clock::now();
    align(predict_pose);
    const auto align_end = std::chrono::system_clock::now();
    align_time_ = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;
    std::cout << "align_time: " << align_time_ << "ms" << std::endl;

    localizer_pose_ = getFinalPose();

    return true;
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::writeLogFile(const std::string& log_file_directory_path)
{
    static std::ofstream log_file_stream;
    static bool is_first_call = true;
    if(is_first_call) {
        const std::string filename = log_file_directory_path  + "/log.csv";
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
Pose LidarLocalizer<PointSource, PointTarget>::getLocalizerPose() const
{
    return localizer_pose_;
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

#endif
