#ifndef CSV_LOGGER_H_
#define CSV_LOGGER_H_

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/ndt_stat.h>
#include <autoware_msgs/image_obj_ranged.h>

namespace csv_logger_namespace{
    
struct ndt_matching_struct{
    ros::Time time;
    double pose[7]; //x,y,z,a,b,c,d
    double velocity;
    double acceleration;
};

struct obj_detection_struct{
    ros::Time time;
    int obj_num;
    double rect[20][6]; //x,y,width,height,score,range
};

class csv_logger{
public:
    //Constructor
    csv_logger();

    //Operation function
    void run();
    
private:
    ros::Time time;
    
    //Node handles
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    //Logging mode flags
    bool ndt_matching_flag;
    bool obj_detection_flag;

    //Subscribers
    ros::Subscriber sub_ndt_pose, sub_ndt_stat, sub_obj_detection;
    
    //Variables
    ndt_matching_struct ndt_matching;
    obj_detection_struct obj_detection;
    
    //Log file
    std::string filename;
    std::ofstream ofs;

    //Callback functions
    void ndt_pose_callback(const geometry_msgs::PoseStamped& msg);
    void ndt_stat_callback(const autoware_msgs::ndt_stat& msg);
    void obj_detection_callback(const autoware_msgs::image_obj_ranged& msg);

    void initForROS();
    
    void log_data();
};

}

#endif //CSV_LOGGER_H_