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

#include "nmea2tfpose_core_RTK.h"
#include <std_msgs/Float64.h>
#include <autoware_msgs/gnss_standard_deviation.h>
#include <autoware_msgs/gnss_surface_speed.h>

const unsigned int LAST_GEO_SIZE=10;

namespace gnss_localizer
{
// Constructor
Nmea2TFPoseNode::Nmea2TFPoseNode()
  : private_nh_("~")
  , MAP_FRAME_("map")
  , GPS_FRAME_("gps")
  , roll_(0)
  , pitch_(0)
  , yaw_(0)
  , orientation_time_(0)
  , position_time_(0)
  , current_time_(0)
  , orientation_stamp_(0)
  , last_geo_(LAST_GEO_SIZE)
  , gphdt_value(0)
  , lat_std(0)
  , lon_std(0)
  , alt_std(0)
  , surface_speed_(0)
{
  initForROS();
  geo_.set_plane(plane_number_);
  geo2_.set_plane(plane_number_);
}

// Destructor
Nmea2TFPoseNode::~Nmea2TFPoseNode()
{
}

void Nmea2TFPoseNode::initForROS()
{
  // ros parameter settings
  private_nh_.getParam("plane", plane_number_);

  // setup subscriber
  sub1_ = nh_.subscribe("nmea_sentence", 100, &Nmea2TFPoseNode::callbackFromNmeaSentence, this);
  can_info = nh_.subscribe("can_info",100,&Nmea2TFPoseNode::callbackFromCanInfo, this);

  // setup publisher
  pub1_ = nh_.advertise<geometry_msgs::PoseStamped>("gnss_pose", 10);
  pub_hdt = nh_.advertise<std_msgs::Float64>("gnss_hdt", 10);
  pub_std = nh_.advertise<autoware_msgs::gnss_standard_deviation>("gnss_standard_deviation", 10);
  pub_surface_speed = nh_.advertise<autoware_msgs::gnss_surface_speed>("gnss_surface_speed", 10);
}

void Nmea2TFPoseNode::run()
{
  ros::spin();
}

void Nmea2TFPoseNode::publishPoseStamped()
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = MAP_FRAME_;
  pose.header.stamp = current_time_;
  pose.pose.position.x = geo_.y();
  pose.pose.position.y = geo_.x();
  pose.pose.position.z = geo_.z();
  pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_, pitch_, yaw_);
  pub1_.publish(pose);

  std_msgs::Float64 hdt;
  hdt.data = gphdt_value;
  pub_hdt.publish(hdt);

  autoware_msgs::gnss_standard_deviation gsd;
  gsd.header.frame_id = MAP_FRAME_;
  gsd.header.stamp = current_time_;
  gsd.lat_std = lat_std;
  gsd.lon_std = lon_std;
  gsd.alt_std = alt_std;
  pub_std.publish(gsd);

  autoware_msgs::gnss_surface_speed gss;
  gss.header.frame_id = MAP_FRAME_;
  gss.header.stamp = current_time_;
  gss.surface_speed = surface_speed_;
  pub_surface_speed.publish(gss);
}

void Nmea2TFPoseNode::publishTF()
{
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(geo_.y(), geo_.x(), geo_.z()));
  tf::Quaternion quaternion;
  quaternion.setRPY(roll_, pitch_, yaw_);
  transform.setRotation(quaternion);
  br_.sendTransform(tf::StampedTransform(transform, current_time_, MAP_FRAME_, GPS_FRAME_));
}

void Nmea2TFPoseNode::createOrientation()
{
  //yaw_ = atan2(geo_.x() - last_geo_.x(), geo_.y() - last_geo_.y());
  //yaw_ = last_geo_.returnAngle_movingAverage(2);
  yaw_ = gphdt_value;
  //yaw_ = atan2(geo2_.x() - geo_.x(), geo2_.y() - geo_.y());
  roll_ = 0;
  pitch_ = 0;

  //std::cout<<"atan2 : "<<yaw_*180/M_PI<<std::endl;
  //std::cout<<"hdt   : "<<gphdt_value*180/M_PI<<std::endl;
}

void Nmea2TFPoseNode::convert(std::vector<std::string> nmea, ros::Time current_stamp)
{
    std::cout<<nmea.at(0).c_str()<<std::endl;
  try
  {
    if (nmea.at(0).compare(0, 2, "QQ") == 0)
    {
      orientation_time_ = stod(nmea.at(3));
      roll_ = stod(nmea.at(4)) * M_PI / 180.;
      pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
      yaw_ = -1 * stod(nmea.at(6)) * M_PI / 180. + M_PI / 2;
      orientation_stamp_ = current_stamp;
      ROS_INFO("QQ is subscribed.");
    }
    else if (nmea.at(0) == "$PASHR")
    {
      orientation_time_ = stod(nmea.at(1));
      roll_ = stod(nmea.at(4)) * M_PI / 180.;
      pitch_ = -1 * stod(nmea.at(5)) * M_PI / 180.;
      yaw_ = -1 * stod(nmea.at(2)) * M_PI / 180. + M_PI / 2;
      ROS_INFO("PASHR is subscribed.");
    }
    else if(nmea.at(0).compare(3, 3, "GGA") == 0)
    {
      position_time_ = stod(nmea.at(1));
      double lat = stod(nmea.at(2));
      double lon = stod(nmea.at(4));
      double h = stod(nmea.at(9));
      geo_.set_llh_nmea_degrees(lat, lon, h);

      double fai = gphdt_value;
      const double earth_R = 6.3781E6;
      const double ant_distance = 1.28;
      double lat2 = (ant_distance*cos(fai))/earth_R+lat;
      double lon2 = (ant_distance*sin(fai))/earth_R+lon;
      geo2_.set_llh_nmea_degrees(lat2, lon2, h);
      ROS_INFO("GGA is subscribed.");
    }
    else if(nmea.at(0) == "$GPRMC")
    {
      position_time_ = stoi(nmea.at(1));
      double lat = stod(nmea.at(3));
      double lon = stod(nmea.at(5));
      double h = 0.0;
      geo_.set_llh_nmea_degrees(lat, lon, h);
      ROS_INFO("GPRMC is subscribed.");
    }
    else if(nmea.at(0) == "$GPVTG")
    {
      surface_speed_ = stod(nmea.at(7));
      ROS_INFO("GPVGT is subscribed.");
    }
    else if(nmea.at(0) == "$GPHDT")
    {
      double val=stod(nmea.at(1));
      if(val<0) val+=360;
      else if(val>360) val-=360;
      gphdt_value = -1 * val*M_PI/180.0 + M_PI/2;
      ROS_INFO("GPHDT is subscribed.");
    }
    else if(nmea.at(0) == "$GPGST")
    {
        lat_std = stod(nmea.at(6));
        lon_std = stod(nmea.at(7));
        alt_std = stod(nmea.at(8));
        ROS_INFO("GPGST is subscribed.");
    }
  }catch (const std::exception &e)
  {
    ROS_WARN_STREAM("Message is invalid : " << e.what());
  }
}

void Nmea2TFPoseNode::callbackFromNmeaSentence(const nmea_msgs::Sentence::ConstPtr &msg)
{
  current_time_ = msg->header.stamp;
  convert(split(msg->sentence), msg->header.stamp);

  double timeout = 10.0;
  if (fabs(orientation_stamp_.toSec() - msg->header.stamp.toSec()) > timeout)
  {
    double dt = sqrt(pow(geo_.x() - last_geo_[0].x(), 2) + pow(geo_.y() - last_geo_[0].y(), 2));
    double threshold = 0.10;
    //if (dt >= threshold)
    {   //std::printf("x:%Lf y:%Lf\n",geo_.x(),geo_.y());
        //std::printf("xl:%Lf yl:%Lf\n",last_geo_.x(),last_geo_.y());
        //std::printf("xs:%Lf ys:%Lf yaw:%Lf\n",geo_.x() - last_geo_.x(),geo_.y() - last_geo_.y(),atan2((double)geo_.x() - (double)last_geo_.x(), (double)geo_.y() - (double)last_geo_.y()));
        //std::cout<<"x:"<<geo_.x()<<" y:"<<geo_.y()<<std::endl;
        //std::cout<<"xl:"<<last_geo_.x()<<" yl:"<<last_geo_.y()<<std::endl;
        //std::cout<<"xs:"<<geo_.x() - last_geo_.x()<<" ys:"<<geo_.y() - last_geo_.y()<<" yaw:"<<atan2(geo_.x() - last_geo_.x(), geo_.y() - last_geo_.y())<<std::endl;
        ROS_INFO("QQ is not subscribed. Orientation is created by atan2");
        last_geo_.push_back(geo_,lat_std,lon_std,alt_std,surface_speed_);
        createOrientation();
        publishPoseStamped();
        publishTF();
        //last_geo_ = geo_;
    }
    /*}else
    {
        createOrientation();
        publishPoseStamped();
        publishTF();
    }*/
    return;
  }

  double e = 1e-2;
  if (fabs(orientation_time_ - position_time_) < e)
  {
    publishPoseStamped();
    publishTF();
    return;
  }
}

void Nmea2TFPoseNode::callbackFromCanInfo(const autoware_msgs::CanInfo::ConstPtr &msg)
{

}

std::vector<std::string> split(const std::string &string)
{
  std::vector<std::string> str_vec_ptr;
  std::string token;
  std::stringstream ss(string);

  while (getline(ss, token, ','))
    str_vec_ptr.push_back(token);

  return str_vec_ptr;
}

}  // gnss_localizer
