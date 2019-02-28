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

#include "lidar_localizer/lidar_localizer.h"

template <class PointSource, class PointTarget>
LidarLocalizer<PointSource, PointTarget>::LidarLocalizer()
    : is_init_map_(false)
    , map_point_size_(0)
    , align_time_(0)
    , fitness_score_(0)
    , thread_status_(ThreadStatus::Sleeping)
{

}

template <class PointSource, class PointTarget>
LidarLocalizer<PointSource, PointTarget>::~LidarLocalizer()
{
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::initPointsMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& pointcloud_ptr)
{
    setInputTarget(pointcloud_ptr);
    map_point_size_ = pointcloud_ptr->points.size();
    is_init_map_ = true;
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::updatePointsMap(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& pointcloud_ptr)
{
    if(pointcloud_ptr == nullptr){
        std::cout << "[ERROR] pointcloud_ptr is nullptr" << std::endl;
        return;
    }

    if(map_point_size_ == 0) {
        initPointsMap(pointcloud_ptr);
    }
    else if(map_point_size_ != pointcloud_ptr->points.size()){
        buildMapThread(pointcloud_ptr);
    }
}

template <class PointSource, class PointTarget>
void LidarLocalizer<PointSource, PointTarget>::buildMapThread(const boost::shared_ptr< pcl::PointCloud<PointTarget> >& map_ptr)
{
    //donot create multiple threads
    if(thread_status_ != ThreadStatus::Sleeping) {
        return;
    }

    thread_begin_time_ =  std::chrono::system_clock::now();
    thread_status_ = ThreadStatus::Running;

    std::thread build_map_thread([this, map_ptr](){
        buildMap(map_ptr);
        thread_status_ = ThreadStatus::Finished;
    });
    build_map_thread.detach();

    map_point_size_ = map_ptr->points.size();
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
bool LidarLocalizer<PointSource, PointTarget>::alignMap(const boost::shared_ptr< pcl::PointCloud<PointSource> >& pointcloud_ptr, const Pose& predict_pose)
{
    if(!is_init_map_) {
        std::cout << "[WARN]received points. But map is not loaded" << std::endl;
        return false;
    }

    swapMap();
    setInputSource(pointcloud_ptr);

    const auto align_start = std::chrono::system_clock::now();
    align(predict_pose);
    const auto align_end = std::chrono::system_clock::now();
    align_time_ = std::chrono::duration_cast<std::chrono::microseconds>(align_end - align_start).count() / 1000.0;

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
      return;
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


template class LidarLocalizer<pcl::PointXYZ, pcl::PointXYZ>;
template class LidarLocalizer<pcl::PointXYZI, pcl::PointXYZI>;
