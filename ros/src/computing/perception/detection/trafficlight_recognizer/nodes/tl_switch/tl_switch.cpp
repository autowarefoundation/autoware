#include "TrafficLight.h"
#include "ros/ros.h"
#include <autoware_msgs/TrafficLight.h>
#include <chrono>
#include <thread>

static const bool ADVERTISE_LATCH = true;

class TLSwitch {
public:
  TLSwitch(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh);
  ~TLSwitch();

private:
  void switch_color();
  void reset_light_msg();
  void watchdog_timer();
  void camera_light_color_callback(
      const autoware_msgs::TrafficLight::ConstPtr &input_msg);
  void ams_light_color_callback(
      const autoware_msgs::TrafficLight::ConstPtr &input_msg);

  std::string light_color_topic_name_;
  std::string camera_light_color_topic_name_;
  std::string ams_light_color_topic_name_;
  autoware_msgs::TrafficLight camera_msg_;
  autoware_msgs::TrafficLight ams_msg_;
  autoware_msgs::TrafficLight state_msg_;
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher traffic_light_pub_;
  ros::Subscriber camera_sub_;
  ros::Subscriber ams_sub_;
  ros::Time ams_msg_time_;
  bool is_ams_timeout_;
  ros::Duration ams_timeout_period_;
  std::thread watchdog_timer_thread_;
};

TLSwitch::TLSwitch(const ros::NodeHandle &nh, const ros::NodeHandle &private_nh)
    : nh_(nh), private_nh_(private_nh), ams_timeout_period_(2.0) {
  private_nh_.param<std::string>("light_color_topic", light_color_topic_name_,
                                 "/light_color");
  private_nh_.param<std::string>("camera_light_color_topic",
                                 camera_light_color_topic_name_,
                                 "/camera_light_color");
  private_nh_.param<std::string>(
      "ams_light_color_topic", ams_light_color_topic_name_, "/ams_light_color");
  traffic_light_pub_ = nh_.advertise<autoware_msgs::TrafficLight>(
      light_color_topic_name_, 1, ADVERTISE_LATCH);
  camera_sub_ = nh_.subscribe(camera_light_color_topic_name_, 1,
                              &TLSwitch::camera_light_color_callback, this);
  ams_sub_ = nh_.subscribe(ams_light_color_topic_name_, 1,
                           &TLSwitch::ams_light_color_callback, this);
  reset_light_msg();

  watchdog_timer_thread_ = std::thread(&TLSwitch::watchdog_timer, this);
  watchdog_timer_thread_.detach();
}

TLSwitch::~TLSwitch() {}

void TLSwitch::reset_light_msg() {
  camera_msg_.traffic_light = TRAFFIC_LIGHT_UNKNOWN;
  ams_msg_.traffic_light = TRAFFIC_LIGHT_UNKNOWN;
  state_msg_.traffic_light = TRAFFIC_LIGHT_UNKNOWN;
  is_ams_timeout_ = true;
}

void TLSwitch::watchdog_timer() {
  while (1) {
    ros::Time now_time = ros::Time::now();

    // if lost Communication
    if (now_time - ams_msg_time_ > ams_timeout_period_ && !is_ams_timeout_) {
      ROS_WARN("Lost Communication! Timeout ams: %f sec",
               (now_time - ams_msg_time_).toSec());
      is_ams_timeout_ = true;
      switch_color();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

void TLSwitch::camera_light_color_callback(
    const autoware_msgs::TrafficLight::ConstPtr &msg) {
  camera_msg_.traffic_light = msg->traffic_light;
  switch_color();
}

void TLSwitch::ams_light_color_callback(
    const autoware_msgs::TrafficLight::ConstPtr &msg) {
  ams_msg_time_ = ros::Time::now();
  is_ams_timeout_ = false;
  ams_msg_.traffic_light = msg->traffic_light;
  switch_color();
}

void TLSwitch::switch_color() {
  if (is_ams_timeout_) {
    state_msg_.traffic_light = camera_msg_.traffic_light;
  } else {
    if (ams_msg_.traffic_light == TRAFFIC_LIGHT_RED ||
        camera_msg_.traffic_light == TRAFFIC_LIGHT_RED) {
      state_msg_.traffic_light = TRAFFIC_LIGHT_RED;
    } else if (ams_msg_.traffic_light == TRAFFIC_LIGHT_UNKNOWN ||
               camera_msg_.traffic_light == TRAFFIC_LIGHT_UNKNOWN) {
      state_msg_.traffic_light = TRAFFIC_LIGHT_UNKNOWN;
    } else if (ams_msg_.traffic_light == TRAFFIC_LIGHT_GREEN &&
               camera_msg_.traffic_light == TRAFFIC_LIGHT_GREEN) {
      state_msg_.traffic_light = TRAFFIC_LIGHT_GREEN;
    }
  }

  traffic_light_pub_.publish(state_msg_);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "light_color_switch");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  TLSwitch light_color_switch(nh, private_nh);

  ros::spin();
  return 0;
}
