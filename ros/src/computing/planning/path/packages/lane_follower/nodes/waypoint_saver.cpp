#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <vehicle_socket/CanInfo.h>
#include <fstream>
#include <sstream>

#include "geo_pos_conv.hh"

bool RecieveOnce = false;

class WAYPOINT_SAVER {
private:
    ros::NodeHandle node_;
    ros::Subscriber pose_sub_;
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
    int pose_time_sec_;
    int pose_time_nsec_;
    int can_time_sec_;
    int can_time_nsec_;

public:
    void PoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose);
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

    double interval = 4.0;
    n_private_.getParam("interval", interval);
    std::cout << "interval = " << interval << std::endl;

    std::string filename = "";
    if (n_private_.getParam("save_filename", filename) == false) {
        std::cout << "error! usage : rosrun lane_follower waypoint_saver _save_topic:=[topic] _interval:=[value] _save_filename:=\"[save file]\"" << std::endl;
        exit(-1);
    }

    save_topic = "amcl";
    n_private_.getParam("save_topic", save_topic);
    pose_sub_ = node_.subscribe("amcl_pose", 1, &WAYPOINT_SAVER::PoseCB, this);
    gnss_pose_sub = node_.subscribe("fix", 1, &WAYPOINT_SAVER::GNSSPoseCB, this);
    ndt_pose_sub = node_.subscribe("ndt_pose", 1, &WAYPOINT_SAVER::NDTPoseCB, this);
    can_info_sub = node_.subscribe("can_info", 1, &WAYPOINT_SAVER::CanInfoCB, this);

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ros::spinOnce();

        if (RecieveOnce != true) {

            ofs_ << current_pose_.x << "," << current_pose_.y << "," << current_pose_.z << std::endl;
            RecieveOnce = true;

        } else {

            double distance = sqrt(pow((current_pose_.x - last_pose_.x), 2) + pow((current_pose_.y - last_pose_.y), 2) + pow((current_pose_.z - last_pose_.z), 2));

            if (distance > interval) {
                if (can_time_sec_ == pose_time_sec_ && fabs(can_time_nsec_ - pose_time_nsec_) < 0.01) {
                    last_pose_ = current_pose_;
                    ofs_ << current_pose_.x << "," << current_pose_.y << "," << current_pose_.z << "," << velocity_ << std::endl;
                }
            }

        }

        loop_rate.sleep();
    }

    std::cout << "WAIT...\n";
    ofs_.open(filename.c_str());

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
        std::cout << "save_topic is not gnss" << std::endl;
    }
}

void WAYPOINT_SAVER::PoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose)
{
    if (save_topic == "amcl") {
        geometry_msgs::Point p(pose->pose.pose.position);
        if (RecieveOnce != true) {

            ofs_ << p.x << "," << p.y << std::endl;
            RecieveOnce = true;
        } else {

            double distance = (p.x - last_pose_.x) * (p.x - last_pose_.x) + (p.y - last_pose_.y) * (p.y - last_pose_.y);

            if (distance > 4.0) {
                last_pose_ = p;

                ofs_ << p.x << "," << p.y << std::endl;
            }
        }
    } else {
        std::cout << "save_topic is not amcl" << std::endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "WAYPOINT_SAVER");
    WAYPOINT_SAVER as;

    return 0;
}
