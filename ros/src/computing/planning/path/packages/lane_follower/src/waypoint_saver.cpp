#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <fstream>
#include <sstream>

#include "geo_pos_conv.hh"

bool RecieveOnce = false;

class WAYPOINT_SAVER {
private:
    ros::NodeHandle node_;
    ros::Subscriber pose_sub_;
    ros::Subscriber gnss_pose_sub;
    //ros::Publisher pose_pub_;
    std::string save_topic;
    ros::Time t1, t2;
    std::ofstream ofs_;

    geometry_msgs::PoseWithCovarianceStamped pose_inv_;
    geometry_msgs::Point last_pose_;

public:
    void PoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose);
    void GNSSPoseCB(const sensor_msgs::NavSatFixConstPtr &pose);
    void MainLoop();

    WAYPOINT_SAVER();
    ~WAYPOINT_SAVER();
};

WAYPOINT_SAVER::WAYPOINT_SAVER()
{
    ros::NodeHandle n_private_("~");

    std::string filename = "/home/pdsljp/auto_ws/path.txt";
    n_private_.getParam("filename", filename);
    save_topic = "amcl";
    n_private_.getParam("save_topic", save_topic);
    pose_sub_ = node_.subscribe("amcl_pose", 1, &WAYPOINT_SAVER::PoseCB, this);
    //gnss_pose_sub = node_.subscribe("fix", 1, &WAYPOINT_SAVER::GNSSPoseCB,
         //   this);
    //pose_pub_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/amcl_pose", 100);

    std::cout << "WAIT...\n";
    ofs_.open(filename.c_str());

    ros::spin();
}

WAYPOINT_SAVER::~WAYPOINT_SAVER()
{
    std::cout << "\nEND!!\n";
}

void WAYPOINT_SAVER::GNSSPoseCB(const sensor_msgs::NavSatFixConstPtr &msg)
{
    if (save_topic == "gnss") {
        geo_pos_conv geo;
        geo.llh_to_xyz(msg->latitude, msg->longitude, msg->altitude);
        geometry_msgs::Point p;
        p.x = geo.x();
        p.y = geo.y();
        p.z = geo.z();

        if (RecieveOnce != true) {

            ofs_ << p.x << "," << p.y << std::endl;
            RecieveOnce = true;
        } else {

            double distance = (p.x - last_pose_.x) * (p.x - last_pose_.x)
                    + (p.y - last_pose_.y) * (p.y - last_pose_.y);

            if (distance > 4.0) {
                last_pose_ = p;

                ofs_ << p.x << "," << p.y << std::endl;
            }
        }
    }else{
        std::cout <<"save_topic is not gnss" << std::endl;
    }
}

void WAYPOINT_SAVER::PoseCB(
        const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose)
{
    if (save_topic == "gnss") {
    geometry_msgs::Point p(pose->pose.pose.position);
    if (RecieveOnce != true) {

        ofs_ << p.x << "," << p.y << std::endl;
        RecieveOnce = true;
    } else {

        double distance = (p.x - last_pose_.x) * (p.x - last_pose_.x)
                + (p.y - last_pose_.y) * (p.y - last_pose_.y);

        if (distance > 4.0) {
            last_pose_ = p;

            ofs_ << p.x << "," << p.y << std::endl;
        }
    }
    }else{
            std::cout <<"save_topic is not amcl" << std::endl;
        }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "WAYPOINT_SAVER");
    WAYPOINT_SAVER as;

    return 0;
}
