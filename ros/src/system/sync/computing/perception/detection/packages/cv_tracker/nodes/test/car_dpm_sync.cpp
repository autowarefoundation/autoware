/* ----header---- */
/* common header */
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <boost/circular_buffer.hpp>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <mqueue.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include "t_sync_message.h"
/* user header */
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "autoware_msgs/ImageObj.h"
#include "autoware_msgs/PointsImage.h"

/* ----mode---- */
#define _REQ_PUB 1

/* ----var---- */
/* common var */
bool buf_flag;
pthread_mutex_t mutex;
/* user var */
boost::circular_buffer<sensor_msgs::Image> image_raw_ringbuf(3);
boost::circular_buffer<sensor_msgs::PointCloud2> points_raw_ringbuf(3);
ros::Publisher image_raw_pub;
ros::Publisher points_raw_pub;
bool image_obj_flag;
bool points_image_flag;

/* ----function---- */
double fabs_time_diff(std_msgs::Header *timespec1, std_msgs::Header *timespec2) {
    double time1 = (double)timespec1->stamp.sec + (double)timespec1->stamp.nsec/1000000000L;
    double time2 = (double)timespec2->stamp.sec + (double)timespec2->stamp.nsec/1000000000L;

    return fabs(time1 - time2);
}

double get_time(const std_msgs::Header *timespec) {
    return (double)timespec->stamp.sec + (double)timespec->stamp.nsec/1000000000L;
}


#if _REQ_PUB
sensor_msgs::Image* p_image_raw_buf;
sensor_msgs::PointCloud2* p_points_raw_buf;

void image_raw_callback(const sensor_msgs::Image::ConstPtr& image_raw_msg) {
    pthread_mutex_lock(&mutex);
    image_raw_ringbuf.push_front(*image_raw_msg);
    //points_raw is empty
    if (points_raw_ringbuf.begin() == points_raw_ringbuf.end()) {
        pthread_mutex_unlock(&mutex);
        ROS_INFO("points_raw ring buffer is empty");
        return;
    }
    buf_flag = true;
    pthread_mutex_unlock(&mutex);
}

void points_raw_callback(const sensor_msgs::PointCloud2::ConstPtr& points_raw_msg) {
    pthread_mutex_lock(&mutex);
    points_raw_ringbuf.push_front(*points_raw_msg);
    //image_raw is empty
    if (image_raw_ringbuf.begin() == image_raw_ringbuf.end()) {
        ROS_INFO("image_raw ring buffer is empty");
        pthread_mutex_unlock(&mutex);
        return;
    }

    buf_flag = true;
    pthread_mutex_unlock(&mutex);
}

void publish_msg(sensor_msgs::Image* p_image_raw_buf, sensor_msgs::PointCloud2* p_points_raw_buf)
{
    ROS_INFO("publish");
    image_raw_pub.publish(*p_image_raw_buf);
    points_raw_pub.publish(*p_points_raw_buf);
}

bool publish() {
    if (buf_flag) {
        pthread_mutex_lock(&mutex);

        //image_raw is empty
        if (image_raw_ringbuf.begin() == image_raw_ringbuf.end()) {
            pthread_mutex_unlock(&mutex);
            ROS_INFO("image_raw ring buffer is empty");
            return false;
        }

        //points_raw is empty
        if (points_raw_ringbuf.begin() == points_raw_ringbuf.end()) {
            pthread_mutex_unlock(&mutex);
            ROS_INFO("points_raw ring buffer is empty");
            return false;
        }

        // image_raw > points_raw
        if (get_time(&(image_raw_ringbuf.front().header)) >= get_time(&(points_raw_ringbuf.front().header))) {
            p_points_raw_buf = &(points_raw_ringbuf.front());
            boost::circular_buffer<sensor_msgs::Image>::iterator it = image_raw_ringbuf.begin();
            if (image_raw_ringbuf.size() == 1) {
                p_image_raw_buf = &*it;
                publish_msg(p_image_raw_buf, p_points_raw_buf);
                pthread_mutex_unlock(&mutex);
                return true;
            } else {
                for (it++; it != image_raw_ringbuf.end(); it++) {
                    if (fabs_time_diff(&(points_raw_ringbuf.front().header), &((it-1)->header))
                        < fabs_time_diff(&(points_raw_ringbuf.front().header), &(it->header))) {
                        p_image_raw_buf = &*(it-1);
                        break;
                    }
                }
                if (it == image_raw_ringbuf.end()) {
                    p_image_raw_buf = &(image_raw_ringbuf.back());
                }
            }
        }
        // image_raw < points_raw
        else {
            p_image_raw_buf = &(image_raw_ringbuf.front());
            boost::circular_buffer<sensor_msgs::PointCloud2>::iterator it = points_raw_ringbuf.begin();
            if (points_raw_ringbuf.size() == 1) {
                p_points_raw_buf = &*it;
                publish_msg(p_image_raw_buf, p_points_raw_buf);
                pthread_mutex_unlock(&mutex);
                return true;
            }

            for (it++; it != points_raw_ringbuf.end(); it++) {
                if (fabs_time_diff(&(image_raw_ringbuf.front().header), &((it-1)->header))
                    < fabs_time_diff(&(image_raw_ringbuf.front().header), &(it->header))) {
                    p_points_raw_buf = &*(it-1);
                    break;
                }
            }

            if (it == points_raw_ringbuf.end()) {
                p_points_raw_buf = &(points_raw_ringbuf.back());
            }
        }
        publish_msg(p_image_raw_buf, p_points_raw_buf);
        pthread_mutex_unlock(&mutex);
        return true;
    } else {
        return false;
    }
}
#else
sensor_msgs::Image image_raw_buf;
sensor_msgs::PointCloud2 points_raw_buf;

void image_raw_callback(const sensor_msgs::Image::ConstPtr& image_raw_msg) {
    pthread_mutex_lock(&mutex);
    image_raw_ringbuf.push_front(*image_raw_msg);

    //points_raw is empty
    if (points_raw_ringbuf.begin() == points_raw_ringbuf.end()) {
        pthread_mutex_unlock(&mutex);
        ROS_INFO("points_raw ring buffer is empty");
        return;
    }

    buf_flag = true;

    // image_raw > points_raw
    if (get_time(&(image_raw_ringbuf.front().header)) >= get_time(&(points_raw_ringbuf.front().header))) {
        points_raw_buf = points_raw_ringbuf.front();
        boost::circular_buffer<sensor_msgs::Image>::iterator it = image_raw_ringbuf.begin();
        if (image_raw_ringbuf.size() == 1) {
            image_raw_buf = *it;
            pthread_mutex_unlock(&mutex);
            return;
        } else {
            for (it++; it != image_raw_ringbuf.end(); it++) {
                if (fabs_time_diff(&(points_raw_ringbuf.front().header), &((it-1)->header))
                    < fabs_time_diff(&(points_raw_ringbuf.front().header), &(it->header))) {
                    image_raw_buf = *(it-1);
                    break;
                }
            }
            if (it == image_raw_ringbuf.end()) {
                image_raw_buf = image_raw_ringbuf.back();
            }
        }

    } else {
        image_raw_buf = image_raw_ringbuf.front();
        boost::circular_buffer<sensor_msgs::PointCloud2>::iterator it = points_raw_ringbuf.begin();
        if (points_raw_ringbuf.size() == 1) {
            points_raw_buf = *it;
            pthread_mutex_unlock(&mutex);
            return;
        }

        for (it++; it != points_raw_ringbuf.end(); it++) {
            if (fabs_time_diff(&(image_raw_ringbuf.front().header), &((it-1)->header))
                < fabs_time_diff(&(image_raw_ringbuf.front().header), &(it->header))) {
                points_raw_buf = *(it-1);
                break;
            }
        }

        if (it == points_raw_ringbuf.end()) {
            points_raw_buf = points_raw_ringbuf.back();
        }
    }
    pthread_mutex_unlock(&mutex);
}

void points_raw_callback(const sensor_msgs::PointCloud2::ConstPtr& points_raw_msg) {
    pthread_mutex_lock(&mutex);
    points_raw_ringbuf.push_front(*points_raw_msg);
    //image_raw is empty
    if (image_raw_ringbuf.begin() == image_raw_ringbuf.end()) {
        ROS_INFO("image_raw ring buffer is empty");
        pthread_mutex_unlock(&mutex);
        return;
    }

    buf_flag = true;

    // image_raw > points_raw
    if (get_time(&(image_raw_ringbuf.front().header)) >= get_time(&(points_raw_ringbuf.front().header))) {
        points_raw_buf = points_raw_ringbuf.front();
        boost::circular_buffer<sensor_msgs::Image>::iterator it = image_raw_ringbuf.begin();
        if (image_raw_ringbuf.size() == 1) {
            image_raw_buf = *it;
            pthread_mutex_unlock(&mutex);
            return;
        } else {
            for (it++; it != image_raw_ringbuf.end(); it++) {
                if (fabs_time_diff(&(points_raw_ringbuf.front().header), &((it-1)->header))
                    < fabs_time_diff(&(points_raw_ringbuf.front().header), &(it->header))) {
                    image_raw_buf = *(it-1);
                    break;
                }
            }
            if (it == image_raw_ringbuf.end()) {
                image_raw_buf = image_raw_ringbuf.back();
            }
        }

    } else {
        image_raw_buf = image_raw_ringbuf.front();
        boost::circular_buffer<sensor_msgs::PointCloud2>::iterator it = points_raw_ringbuf.begin();
        if (points_raw_ringbuf.size() == 1) {
            points_raw_buf = *it;
            pthread_mutex_unlock(&mutex);
            return;
        }

        for (it++; it != points_raw_ringbuf.end(); it++) {
            if (fabs_time_diff(&(image_raw_ringbuf.front().header), &((it-1)->header))
                < fabs_time_diff(&(image_raw_ringbuf.front().header), &(it->header))) {
                points_raw_buf = *(it-1);
                break;
            }
        }

        if (it == points_raw_ringbuf.end()) {
            points_raw_buf = points_raw_ringbuf.back();
        }
    }
    pthread_mutex_unlock(&mutex);
}

bool publish() {
    if (buf_flag) {
        pthread_mutex_lock(&mutex);
        // scan_ringbuf.clear();
        // image_ringbuf.clear();
        // scan_ringbuf.push_front(scan_buf);
        // image_ringbuf.push_front(image_buf);
        ROS_INFO("publish");
        image_raw_pub.publish(image_raw_buf);
        points_raw_pub.publish(points_raw_buf);
        pthread_mutex_unlock(&mutex);
        return true;
    } else {
        ROS_INFO("publish failed");
        return false;
    }
}
#endif

void image_obj_callback(const autoware_msgs::ImageObj::ConstPtr& image_obj_msg) {
    if (image_obj_flag) {
        image_obj_flag = false;
        points_image_flag = false;
        return;
    }

    image_obj_flag = true;
    if (points_image_flag) {
        ROS_INFO("catch publish request");
        if(!publish()) {
            /* when to publish is failure, republish */
            struct timespec sleep_time;
            sleep_time.tv_sec = 0;
            sleep_time.tv_nsec = 200000000; //5Hz
            while (!publish() || ros::ok())
                nanosleep(&sleep_time, NULL);
        }
        image_obj_flag = false;
        points_image_flag = false;
    }
}
void points_image_callback(const autoware_msgs::PointsImage::ConstPtr& points_image_msg) {
    if (points_image_flag) {
        image_obj_flag = false;
        points_image_flag = false;
        return;
    }

    points_image_flag = true;
    if (image_obj_flag) {
        ROS_INFO("catch publish request");
        if(!publish()) {
            /* when to publish is failure, republish */
            struct timespec sleep_time;
            sleep_time.tv_sec = 0;
            sleep_time.tv_nsec = 200000000; //5Hz
            while (!publish() || ros::ok())
                nanosleep(&sleep_time, NULL);
        }
        image_obj_flag = false;
        points_image_flag = false;
    }
}

void* thread(void* args)
{
    ros::NodeHandle nh_rcv;
    ros::CallbackQueue rcv_callbackqueue;
    nh_rcv.setCallbackQueue(&rcv_callbackqueue);
    ros::Subscriber image_obj_sub = nh_rcv.subscribe("/obj_car/image_obj", 5, image_obj_callback);
    ros::Subscriber points_image_sub = nh_rcv.subscribe("/points_image", 5, points_image_callback);
    bool prev_image_obj_flag;
    bool prev_points_image_flag;

    while (nh_rcv.ok()) {

        prev_points_image_flag = points_image_flag;
        prev_image_obj_flag = image_obj_flag;

        rcv_callbackqueue.callAvailable(ros::WallDuration(1.0f));

        if (image_obj_flag == prev_image_obj_flag && points_image_flag == prev_points_image_flag) {
            ROS_INFO("timeout");
            if(!publish()) {
                /* when to publish is failure, republish */
                struct timespec sleep_time;
                sleep_time.tv_sec = 0;
                sleep_time.tv_nsec = 200000000; //5Hz
                while (!publish() || ros::ok())
                    nanosleep(&sleep_time, NULL);
            }
        }
    }
    return NULL;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sync_server");
    ros::NodeHandle nh;

    ros::Subscriber image_raw_sub = nh.subscribe("/image_raw", 1, image_raw_callback);
    ros::Subscriber points_raw_sub = nh.subscribe("/points_raw", 1, points_raw_callback);
    image_raw_pub = nh.advertise<sensor_msgs::Image>("/image_raw_", 5);
    points_raw_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_raw_", 5);
    while (!buf_flag) {
        ros::spinOnce();
    }
    /* create server thread */
    pthread_t th;
    pthread_create(&th, NULL, thread, (void *)NULL );

    if(!publish()) {
        /* when to publish is failure, republish */
        struct timespec sleep_time;
        sleep_time.tv_sec = 0;
        sleep_time.tv_nsec = 200000000; //5Hz
        while (!publish() || ros::ok())
            nanosleep(&sleep_time, NULL);
    }

    ros::spin();

    /* shutdown server thread */
    ROS_INFO("wait until shutdown a thread");
    pthread_kill(th, SIGINT);
    pthread_join(th, NULL);

    return 0;
}
