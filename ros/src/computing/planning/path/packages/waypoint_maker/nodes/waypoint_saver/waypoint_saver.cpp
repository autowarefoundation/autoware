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

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <vehicle_socket/CanInfo.h>
#include <fstream>
#include <sstream>
#include <cfloat>

#include "geo_pos_conv.hh"

#define NSEC_TO_SEC 0.000000001

bool RecieveOnce = false;

static bool IsNearlyZero(geometry_msgs::Point pose)
{

    double abs_x = fabs(pose.x);
    double abs_y = fabs(pose.y);
    double abs_z = fabs(pose.z);

    if (abs_x < DBL_MIN * 100 && abs_y < DBL_MIN * 100 && abs_z < DBL_MIN * 100)
        return true;
    else
        return false;

}

class WAYPOINT_SAVER {
private:
    ros::NodeHandle node_;
    ros::Subscriber gnss_pose_sub;
    ros::Subscriber ndt_pose_sub;
    ros::Subscriber can_info_sub;

    std::string save_topic;
    ros::Time t1, t2;
    std::ofstream ofs_;

    geometry_msgs::PoseWithCovarianceStamped pose_inv_;
    geometry_msgs::Point current_pose_;
    geometry_msgs::Point last_pose_;
    double velocity_;
    bool no_velocity_;
    int pose_time_sec_;
    int pose_time_nsec_;
    int can_time_sec_;
    int can_time_nsec_;

public:
    void GNSSPoseCB(const sensor_msgs::NavSatFixConstPtr &pose);
    void NDTPoseCB(const geometry_msgs::PoseStampedConstPtr &pose);
    void CanInfoCB(const vehicle_socket::CanInfoConstPtr &info);
    void MainLoop();

    WAYPOINT_SAVER();
    ~WAYPOINT_SAVER();
};

WAYPOINT_SAVER::WAYPOINT_SAVER()
{
    ros::NodeHandle n_private_("~");

    double interval = 1.0;
    n_private_.getParam("interval", interval);
    std::cout << "interval = " << interval << std::endl;

    n_private_.getParam("no_velocity", no_velocity_);
    std::cout << "no_velocity = " << no_velocity_ << std::endl;

    std::string filename = "";
    if (n_private_.getParam("save_filename", filename) == false) {
        std::cout << "error! usage : rosrun lane_follower waypoint_saver _save_topic:=[topic] _interval:=[value] _save_filename:=\"[save file]\"" << std::endl;
        exit(-1);
    }

    save_topic = "ndt";
    n_private_.getParam("save_topic", save_topic);
    gnss_pose_sub = node_.subscribe("fix", 1, &WAYPOINT_SAVER::GNSSPoseCB, this);
    ndt_pose_sub = node_.subscribe("ndt_pose", 1, &WAYPOINT_SAVER::NDTPoseCB, this);
    can_info_sub = node_.subscribe("can_info", 1, &WAYPOINT_SAVER::CanInfoCB, this);
    std::cout << "WAIT...\n";
    ofs_.open(filename.c_str());

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        if (IsNearlyZero(current_pose_) == true)
            continue;

        if (RecieveOnce != true) {

            ofs_ << current_pose_.x << "," << current_pose_.y << "," << current_pose_.z << std::endl;
            RecieveOnce = true;
            last_pose_ = current_pose_;
        } else {
            std::cout << current_pose_.x << "," << current_pose_.y << "," << current_pose_.z << std::endl;
            std::cout << last_pose_.x << "," << last_pose_.y << "," << last_pose_.z << std::endl;

            double distance = sqrt(pow((current_pose_.x - last_pose_.x), 2) + pow((current_pose_.y - last_pose_.y), 2) + pow((current_pose_.z - last_pose_.z), 2));
            std::cout << "distance = " << distance << std::endl;

            std::cout << "can_time_sec = " << can_time_sec_ << std::endl;
            std::cout << "pose_time_sec = " << pose_time_sec_ << std::endl;
            std::cout << "can_time_nsec_ = " << can_time_nsec_ << std::endl;
            std::cout << "pose_time_nsec_ = " << pose_time_nsec_ << std::endl;
            std::cout << "nsec sub = " << fabs(can_time_nsec_ - pose_time_nsec_) * NSEC_TO_SEC << std::endl;

            if (distance > interval) {
                std::cout << "save sequence" << std::endl;
                if (no_velocity_ == false) {
                    if (can_time_sec_ == pose_time_sec_ && fabs(can_time_nsec_ - pose_time_nsec_) * NSEC_TO_SEC < 0.1) {
                        std::cout << "waypoint_velocity_saved" << std::endl;
                        last_pose_ = current_pose_;
                        ofs_ << current_pose_.x << "," << current_pose_.y << "," << current_pose_.z << "," << velocity_ << std::endl;
                    }
                } else {
                    std::cout << "waypoint_saved" << std::endl;
                    last_pose_ = current_pose_;
                    ofs_ << current_pose_.x << "," << current_pose_.y << "," << current_pose_.z << "," << 0 << std::endl;
                }
            }

        }

        loop_rate.sleep();
    }

}

WAYPOINT_SAVER::~WAYPOINT_SAVER()
{
    std::cout << "\nEND!!\n";
}

void WAYPOINT_SAVER::CanInfoCB(const vehicle_socket::CanInfoConstPtr &info)
{
    velocity_ = info->speed;
    can_time_sec_ = info->header.stamp.sec;
    can_time_nsec_ = info->header.stamp.nsec;
}

void WAYPOINT_SAVER::NDTPoseCB(const geometry_msgs::PoseStampedConstPtr &pose)
{

    if (save_topic == "ndt") {
        current_pose_ = pose->pose.position;
        pose_time_sec_ = pose->header.stamp.sec;
        pose_time_nsec_ = pose->header.stamp.nsec;
        //geometry_msgs::Point p(pose->pose.position);

    } else {
        // std::cout << "save_topic is not ndt" << std::endl;
    }
}
void WAYPOINT_SAVER::GNSSPoseCB(const sensor_msgs::NavSatFixConstPtr &pose)
{
    if (save_topic == "gnss") {
        geo_pos_conv geo;
        geo.set_plane(7);
        geo.llh_to_xyz(pose->latitude, pose->longitude, pose->altitude);

        current_pose_.x = geo.x();
        current_pose_.y = geo.y();
        current_pose_.z = geo.z();

        pose_time_sec_ = pose->header.stamp.sec;
        pose_time_nsec_ = pose->header.stamp.nsec;
    } else {
        //  std::cout << "save_topic is not gnss" << std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "WAYPOINT_SAVER");
    WAYPOINT_SAVER as;

    return 0;
}
