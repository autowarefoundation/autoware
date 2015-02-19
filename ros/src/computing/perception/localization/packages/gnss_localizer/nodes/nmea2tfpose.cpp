#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

#include "geo_pos_conv.hh"

geo_pos_conv geo;
ros::Publisher pose_publisher;

using namespace std;

void csv_div(string str, std::vector<std::string> *items)
{
    string token;
    istringstream stream(str);

    items->clear();
    while (getline(stream, token, ',')) {
        //printf("%s|  ",token.c_str());
        items->push_back(token);
    }
    //  printf("\n");
}
/*
 double str2double(std::string str)
 {
 stringstream ss;
 double val;
 ss << str;
 ss >> val;
 return val;
 }
 */

void NmeaCallback(const nmea_msgs::Sentence::ConstPtr& msg)
{
    static double qq_time, roll, pitch, yaw;
    //static double gga_time, x, y, z;
    static double gga_time;
    static tf::TransformBroadcaster br;
    static ros::Time pc_time;
    std::vector<std::string> nmea;
    csv_div(msg->sentence, &nmea);

    // printf("%s\n",msg->sentence.c_str());

    if (nmea[0].compare(0, 2, "QQ") == 0) {
        pc_time = msg->header.stamp;
        /*
        qq_time = str2double(nmea[3]);
        roll = str2double(nmea[4]) * M_PI / 180.;
        pitch = -1 * str2double(nmea[5]) * M_PI / 180.;
        yaw = -1 * str2double(nmea[6]) * M_PI / 180. + M_PI / 2;
        */
        qq_time = stod(nmea[3]);
        roll = stod(nmea[4]) * M_PI / 180.;
        pitch = -1 * stod(nmea[5]) * M_PI / 180.;
        yaw = -1 * stod(nmea[6]) * M_PI / 180. + M_PI / 2;
        //printf("angle %f  %f %f %f\n",qq_time,roll,pitch,yaw);
    }

    if (nmea[0] == "$GPGGA") {
        pc_time = msg->header.stamp;
        /*
        gga_time = str2double(nmea[1]);
        double lat = str2double(nmea[2]);
        double lon = str2double(nmea[4]);
        double h = str2double(nmea[9]); //+str2double(nmea[11]);
*/
        gga_time = stod(nmea[1]);
        double lat = stod(nmea[2]);
        double lon = stod(nmea[4]);git s
        double h = stod(nmea[9]);

        geo.set_llh_nmea_degrees(lat, lon, h);
        //    printf("pos %f  %f %f %f\n",gga_time,geo.x,geo.y,geo.z);
    }

   // if (qq_time == gga_time) {
    if(fabs(qq_time - gga_time) <= __FLT_EPSILON__){
        //printf("%f %f %f %f %f  %f %f %f\n", pc_time.toSec(), gga_time, geo.x(), geo.y(), geo.z(), roll, pitch, yaw);

        tf::Transform transform;
        tf::Quaternion q;

        transform.setOrigin(tf::Vector3(geo.y(), geo.x(), geo.z()));
        q.setRPY(roll, pitch, yaw);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, pc_time, "map", "gps"));

        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.header.frame_id = "map";
        pose.pose.position.x = geo.y();
        pose.pose.position.y = geo.x();
        pose.pose.position.z = geo.z();
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        pose_publisher.publish(pose);

    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nmea2tf");
    geo.set_plane(7);

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("nmea_sentence", 1000, NmeaCallback);
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("gnss_pose", 1000);
    ros::spin();

    return 0;
}
