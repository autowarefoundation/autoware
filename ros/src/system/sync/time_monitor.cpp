/*
 *  Copyright (c) 2015, Nagoya University
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
/* ----header---- */
/* common header */
#include "ros/ros.h"
#include <sstream>
#include <vector>
#include <boost/circular_buffer.hpp>
#include "std_msgs/Time.h"
/* user header */
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "cv_tracker/image_obj.h"
#include "points2image/PointsImage.h"
#include "cv_tracker/image_obj_ranged.h"
#include "cv_tracker/image_obj_tracked.h"
#include "geometry_msgs/PoseStamped.h"
#include "cv_tracker/obj_label.h"
#include "lidar_tracker/centroids.h"
#include "visualization_msgs/MarkerArray.h"

/* ----var---- */
/* common var */

/* user var */
class TimeManager
{
    std::vector<ros::Time> image_raw;
    std::vector<ros::Time> points_raw;
    std::vector<ros::Time> image_obj;
    std::vector<ros::Time> image_obj_ranged;
    std::vector<ros::Time> image_obj_tracked;
    std::vector<ros::Time> current_pose;
    std::vector<ros::Time> obj_label;
    std::vector<ros::Time> cluster_centroids;
    std::vector<ros::Time> obj_pose;

    ros::NodeHandle nh;
    ros::Subscriber image_raw_sub;
    ros::Subscriber points_raw_sub;
    ros::Subscriber image_obj_tracked_sub;
    ros::Subscriber current_pose_sub;
//  ros::Subscriber obj_label_sub;
    ros::Subscriber obj_label_sub;
    ros::Subscriber cluster_centroids_sub;
    ros::Subscriber obj_pose_sub;
    ros::Publisher time_monitor_pub;
    int seq;

public:
    TimeManager(int);
    void image_raw_callback(const sensor_msgs::Image::ConstPtr& image_raw_msg);
    void points_raw_callback(const sensor_msgs::PointCloud2::ConstPtr& points_raw_msg);
    void image_obj_ranged_callback(const cv_tracker::image_obj_ranged::ConstPtr& image_obj_ranged_msg);
    void image_obj_tracked_callback(const cv_tracker::image_obj_tracked::ConstPtr& image_obj_tracked_msg);
    void current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& current_pose_msg);
    void obj_label_callback(const cv_tracker::obj_label::ConstPtr& obj_label_msg) ;
    void cluster_centroids_callback(const lidar_tracker::centroids::ConstPtr& cluster_centroids_msg);
    void obj_pose_callback(const lidar_tracker::centroids::ConstPtr& obj_pose_msg);
    void run();
};

TimeManager::TimeManager(int buffer_size) {
    ros::NodeHandle private_nh("~");
    time_monitor_pub = nh.advertise<std_msgs::Time> ("time_monitor", 1);
    image_raw_sub = nh.subscribe("/image_raw", 1, &TimeManager::image_raw_callback, this);
    points_raw_sub = nh.subscribe("/points_raw", 1, &TimeManager::points_raw_callback, this);
    image_obj_tracked_sub = nh.subscribe("/obj_car/image_obj_tracked", 1, &TimeManager::image_obj_tracked_callback, this);
    current_pose_sub = nh.subscribe("current_pose", 1, &TimeManager::current_pose_callback, this);
//  ros::Subscriber obj_label_sub = nh.subscribe("/obj_person/obj_label", 1, TimeManager::obj_label_callback);
    obj_label_sub = nh.subscribe("/obj_car/obj_label", 1, &TimeManager::obj_label_callback, this);
    cluster_centroids_sub = nh.subscribe("/cluster_centroids", 1, &TimeManager::cluster_centroids_callback, this);
    obj_pose_sub = nh.subscribe("obj_pose", 1, &TimeManager::obj_pose_callback, this);

    image_raw.resize(buffer_size);
    points_raw.resize(buffer_size);
    image_obj.resize(buffer_size);
    image_obj_ranged.resize(buffer_size);
    image_obj_tracked.resize(buffer_size);
    current_pose.resize(buffer_size);
    obj_label.resize(buffer_size);
    cluster_centroids.resize(buffer_size);

    seq = 0;
}

void TimeManager::image_raw_callback(const sensor_msgs::Image::ConstPtr& image_raw_msg) {
    static int i = 0;
    if (i >= image_raw.size()) {
        i = 0;
    }
    image_raw.at(i) = ros::Time::now();
    i++;
}

void TimeManager::points_raw_callback(const sensor_msgs::PointCloud2::ConstPtr& points_raw_msg) {
    static int i = 0;
    if (i >= points_raw.size()) {
        i = 0;
    }
    points_raw.at(i) = ros::Time::now();
    i++;
}

void TimeManager::image_obj_ranged_callback(const cv_tracker::image_obj_ranged::ConstPtr& image_obj_ranged_msg) {
    static int i = 0;
    if (i >= image_obj_ranged.size()) {
        i = 0;
    }
    image_obj_ranged.at(i) = ros::Time::now();
    time_monitor_pub.publish(image_obj_ranged.at(i));
    i++;
}

void TimeManager::image_obj_tracked_callback(const cv_tracker::image_obj_tracked::ConstPtr& image_obj_tracked_msg) {
    static int i = 0;
    if (i >= image_obj_tracked.size()) {
        i = 0;
    }
    image_obj_tracked.at(i) = ros::Time::now();
    i++;
}

void TimeManager::current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& current_pose_msg) {
    static int i = 0;
    if (i >= current_pose.size()) {
        i = 0;
    }
    current_pose.at(i) = ros::Time::now();
    i++;
}

void TimeManager::obj_label_callback(const cv_tracker::obj_label::ConstPtr& obj_label_msg) {
    static int i = 0;
    if (i >= obj_label.size()) {
        i = 0;
    }
    obj_label.at(i) = ros::Time::now();
    i++;
}

void TimeManager::cluster_centroids_callback(const lidar_tracker::centroids::ConstPtr& cluster_centroids_msg) {
    static int i = 0;
    if (i >= cluster_centroids.size()) {
        i = 0;
    }
    cluster_centroids.at(i) = ros::Time::now();
    i++;
}

void TimeManager::obj_pose_callback(const lidar_tracker::centroids::ConstPtr& obj_pose_msg) {
    static int i = 0;
    if (i >= obj_pose.size()) {
        i = 0;
    }
    obj_pose.at(i) = ros::Time::now();
    i++;
}

void TimeManager::run() {
    ros::spin();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "time_monitor");
#if 0
  ros::NodeHandle nh();
  ros::NodeHandle private_nh("~");

  ros::Subscriber image_raw_sub = nh.subscribe("/image_raw", 1, image_raw_callback);
  ros::Subscriber points_raw_sub = nh.subscribe("/points_raw", 1, points_raw_callback);
  ros::Subscriber image_obj_tracked_sub = nh.subscribe("/obj_car/image_obj_tracked", 1, image_obj_tracked_callback);
  ros::Subscriber current_pose_sub = nh.subscribe("current_pose", 1, current_pose_callback);
//  ros::Subscriber obj_label_sub = nh.subscribe("/obj_person/obj_label", 1, obj_label_callback);
  ros::Subscriber obj_label_sub = nh.subscribe("/obj_car/obj_label", 1, obj_label_callback);
  ros::Subscriber cluster_centroids_sub = nh.subscribe("/cluster_centroids", 1, cluster_centroids_callback);
  ros::Subscriber obj_pose_sub = nh.subscribe("obj_pose", 1, obj_pose_callback);

  ros::spin();
#endif
  TimeManager time_manager(32);
  time_manager.run();

  return 0;
}
