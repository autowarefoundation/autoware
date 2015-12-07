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
#include <boost/circular_buffer.hpp>
#include "std_msgs/Header.h"
/* user header */
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "synchronization/time_diff.h"

/* ----mode---- */
#define SELECT_MODE 0
#define PUBLISH_RATE 5
/* ----var---- */
/* common var */
#if SELECT_MODE
bool buf_flag;
pthread_mutex_t mutex;
#endif
/* user var */
#if SELECT_MODE
boost::circular_buffer<sensor_msgs::Image> image_raw_ringbuf(10);
boost::circular_buffer<sensor_msgs::PointCloud2> points_raw_ringbuf(10);
#else
sensor_msgs::Image image_raw_buf;
#endif
ros::Publisher points_raw__pub;
ros::Publisher image_raw__pub;
ros::Publisher time_diff_pub;

double fabs_time_diff(const std_msgs::Header *timespec1, const std_msgs::Header *timespec2)
{
    double time1 = (double)timespec1->stamp.sec + (double)timespec1->stamp.nsec/1000000000L;
    double time2 = (double)timespec2->stamp.sec + (double)timespec2->stamp.nsec/1000000000L;
    return fabs(time1 - time2);
}


void image_raw_callback(sensor_msgs::Image image_raw_msg)
{
#if SELECT_MODE
    pthread_mutex_lock(&mutex);
    image_raw_ringbuf.push_front(*image_raw_msg);
    pthread_mutex_unlock(&mutex);
#else
    image_raw_buf = image_raw_msg;
#endif
}

void points_raw_callback(const sensor_msgs::PointCloud2::ConstPtr& points_raw_msg)
{
#if SELECT_MODE
    pthread_mutex_lock(&mutex);
    points_raw_ringbuf.push_front(*points_raw_msg);
    pthread_mutex_unlock(&mutex);
#else
    synchronization::time_diff time_diff_msg;
    time_diff_msg.header.frame_id = "0";
    time_diff_msg.header.stamp = points_raw_msg->header.stamp;
    time_diff_msg.time_diff = fabs_time_diff(&(points_raw_msg->header), &image_raw_buf.header)*1000.0; //msec
    time_diff_msg.camera = image_raw_buf.header.stamp;
    time_diff_msg.lidar = points_raw_msg->header.stamp;
    time_diff_pub.publish(time_diff_msg);

    image_raw_buf.header.stamp = points_raw_msg->header.stamp;
    image_raw__pub.publish(image_raw_buf);
    points_raw__pub.publish(points_raw_msg);
#endif
}

#if SELECT_MODE
void* thread(void* args) {
    ros::Rate loop_rate(PUBLISH_RATE);
    while (ros::ok()) {
        pthread_mutex_lock(&mutex);
        image_raw__pub.publish(image_raw_ringbuf.front());
        points_raw__pub.publish(points_raw_ringbuf.front());
        pthread_mutex_unlock(&mutex);
        loop_rate.sleep();
    }
    return NULL;
}
#endif

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sync_drivers");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

#if SELECT_MODE
    /* create server thread */
    pthread_t th;
    pthread_create(&th, NULL, thread, (void *)NULL );
#endif

  std::string image_topic;
  std::string points_topic;
  if (private_nh.getParam("image_node", image_topic))
  {
		ROS_INFO("Setting image node to %s", image_topic.c_str());
  }
  else
  {
      ROS_INFO("No image node received, defaulting to image_obj, you can use _image_node:=YOUR_TOPIC");
      image_topic = "/image_raw";
  }
  if (private_nh.getParam("points_node", points_topic))
  {
		ROS_INFO("Setting points node to %s", points_topic.c_str());
  }
  else
  {
      ROS_INFO("No points node received, defaulting to vscan_image, you can use _points_node:=YOUR_TOPIC");
      points_topic = "/points_raw";
  }

  ros::Subscriber image_raw_sub = nh.subscribe(image_topic, 1, image_raw_callback);
  ros::Subscriber points_raw_sub = nh.subscribe(points_topic, 1, points_raw_callback);
  image_raw__pub = private_nh.advertise<sensor_msgs::Image>("image_raw_", 1);
  points_raw__pub = private_nh.advertise<sensor_msgs::PointCloud2>("points_raw_", 1);
  time_diff_pub = nh.advertise<synchronization::time_diff>("/time_difference", 1);

  ros::spin();

#if SELECT_MODE
  /* shutdown child thread */
  ROS_INFO("wait until shutdown a thread");
  pthread_kill(th, SIGINT);
  pthread_join(th, NULL);
#endif

  return 0;
}
