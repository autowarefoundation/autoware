#ifndef VEHICLE_STATUS_CONFIG_H_INCLUDED
#define VEHICLE_STATUS_CONFIG_H_INCLUDED

#include <ros/ros.h>

#define DEFAULT_MONITOR_WIDTH 320

#define KM_PER_HOUR 0
#define M_PER_SEC 1

#define RAD 0
#define DEG 1

#define USE_CTRL_CMD 0
#define USE_ACCEL_STEER_BRAKE_CMD 1

class gear_status
{
    public:
        gear_status(){};
        ~gear_status(){};
        void load_params(){
            ros::param::param<int>("/vehicle_info/gear/D", drive_, 1);
            ros::param::param<int>("/vehicle_info/gear/R", rear_, 2);
            ros::param::param<int>("/vehicle_info/gear/B", brake_, 3);
            ros::param::param<int>("/vehicle_info/gear/R", neutral_, 4);
            ros::param::param<int>("/vehicle_info/gear/R", parking_, 5);
        };
        int get_drive_value(){return drive_;};
        int get_rear_value(){return rear_;};
        int get_brake_value(){return brake_;};
        int get_neutral_value(){return neutral_;};
        int get_parking_value(){return parking_;};
    private:
        ros::NodeHandle nh_;
        int drive_;
        int rear_;
        int brake_;
        int neutral_;
        int parking_;
};

#endif //VEHICLE_STATUS_CONFIG_H_INCLUDED