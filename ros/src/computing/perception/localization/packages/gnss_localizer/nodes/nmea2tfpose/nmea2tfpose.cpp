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

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include "geo_pos_conv.hh"

static geo_pos_conv geo;
static ros::Publisher pose_publisher;
static ros::Publisher stat_publisher;
static std_msgs::Bool gnss_stat_msg;
static int _plane = 0;
#define NEW_QQ_SIZE 11

using namespace std;

static void csv_div(string str, std::vector<std::string> *items)
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

static void NmeaCallback(const nmea_msgs::Sentence::ConstPtr& msg)
{
    static double qq_time, roll, pitch, yaw;
    //static double gga_time, x, y, z;
    static double gga_time;
    static tf::TransformBroadcaster br;
    static ros::Time pc_time;
    std::vector<std::string> nmea;
    csv_div(msg->sentence, &nmea);
    //static bool calibration_flag = true;

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

        // new QQ message
   /*     if (nmea.size() == NEW_QQ_SIZE) {
            if (stod(nmea[7]) == 1 && stod(nmea[8]) == 1 && stod(nmea[9]) == 1) {
                calibration_flag = true;
            } else {
                calibration_flag = false;
            }
        }*/
        //printf("angle %f  %f %f %f\n",qq_time,roll,pitch,yaw);
    }

//    if (calibration_flag == true) {
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
            double lon = stod(nmea[4]);
            double h = stod(nmea[9]);

            geo.set_llh_nmea_degrees(lat, lon, h);
            //    printf("pos %f  %f %f %f\n",gga_time,geo.x,geo.y,geo.z);
        }

        // if (qq_time == gga_time) {
        if (fabs(qq_time - gga_time) <= __FLT_EPSILON__) {
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

            // set gnss_stat
            if (pose.pose.position.x == 0.0 || pose.pose.position.y == 0.0 || pose.pose.position.z == 0.0) {
                gnss_stat_msg.data = false;
            } else {
                gnss_stat_msg.data = true;
            }

            pose_publisher.publish(pose);
            stat_publisher.publish(gnss_stat_msg);
        }
 /*   } else {
        std::cout << "not calibrated!!" << std::endl;
    }*/
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nmea2tfpose");

    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    private_nh.getParam("plane", _plane);
    std::cout << "plane number : " << _plane << std::endl;

    geo.set_plane(_plane);
    ros::Subscriber sub = n.subscribe("nmea_sentence", 1000, NmeaCallback);
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("gnss_pose", 1000);
    stat_publisher = n.advertise<std_msgs::Bool>("/gnss_stat", 100);
    ros::spin();

    return 0;
}
