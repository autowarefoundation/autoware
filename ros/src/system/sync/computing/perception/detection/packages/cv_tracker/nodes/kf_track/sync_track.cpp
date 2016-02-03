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
#include "cv_tracker/image_obj_tracked.h"
#include "cv_tracker/image_obj_ranged.h"

/* ----mode---- */
#define _REQ_PUB 1

/* ----var---- */
/* common var */
bool buf_flag;
pthread_mutex_t mutex;
/* user var */
boost::circular_buffer<cv_tracker::image_obj_ranged> image_obj_ranged_ringbuf(10);
boost::circular_buffer<sensor_msgs::Image> image_raw_ringbuf(10);
ros::Publisher image_obj_ranged__pub;
ros::Publisher image_raw__pub;
bool image_obj_tracked_flag;

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
cv_tracker::image_obj_ranged* p_image_obj_ranged_buf;
sensor_msgs::Image* p_image_raw_buf;

void publish_msg(cv_tracker::image_obj_ranged* p_image_obj_ranged_buf, sensor_msgs::Image* p_image_raw_buf) {
    ROS_INFO("publish");
    image_obj_ranged__pub.publish(*p_image_obj_ranged_buf);
    image_raw__pub.publish(*p_image_raw_buf);
}

bool publish() {
    if (buf_flag) {
        //image_obj_ranged is empty
        if (image_obj_ranged_ringbuf.begin() == image_obj_ranged_ringbuf.end()) {
            ROS_INFO("image_obj_ranged ring buffer is empty");
            return false;
        }

        //image_raw is empty
        if (image_raw_ringbuf.begin() == image_raw_ringbuf.end()) {
            ROS_INFO("image_raw ring buffer is empty");
            return false;
        }

        // image_obj_ranged > image_raw
        if (get_time(&(image_obj_ranged_ringbuf.front().header)) >= get_time(&(image_raw_ringbuf.front().header))) {
            p_image_raw_buf = &(image_raw_ringbuf.front());
            boost::circular_buffer<cv_tracker::image_obj_ranged>::iterator it = image_obj_ranged_ringbuf.begin();
            if (image_obj_ranged_ringbuf.size() == 1) {
                p_image_obj_ranged_buf = &*it;
                publish_msg(p_image_obj_ranged_buf, p_image_raw_buf);
                if (image_obj_tracked_flag == true){
                    buf_flag = false;
                    image_obj_tracked_flag = false;
                    image_obj_ranged_ringbuf.clear();
                    image_raw_ringbuf.clear();
                }
                return true;
            } else {
                for (it++; it != image_obj_ranged_ringbuf.end(); it++) {
                    if (fabs_time_diff(&(image_raw_ringbuf.front().header), &((it-1)->header))
                        < fabs_time_diff(&(image_raw_ringbuf.front().header), &(it->header))) {
                        p_image_obj_ranged_buf = &*(it-1);
                        break;
                    }
                }
                if (it == image_obj_ranged_ringbuf.end()) {
                    p_image_obj_ranged_buf = &(image_obj_ranged_ringbuf.back());
                }
            }
        }
        // image_obj_ranged < image_raw
        else {
            p_image_obj_ranged_buf = &(image_obj_ranged_ringbuf.front());
            boost::circular_buffer<sensor_msgs::Image>::iterator it = image_raw_ringbuf.begin();
            if (image_raw_ringbuf.size() == 1) {
                p_image_raw_buf = &*it;
                publish_msg(p_image_obj_ranged_buf, p_image_raw_buf);
                if (image_obj_tracked_flag == true){
                    buf_flag = false;
                    image_obj_tracked_flag = false;
                    image_obj_ranged_ringbuf.clear();
                    image_raw_ringbuf.clear();
                }
                return true;
            }

            for (it++; it != image_raw_ringbuf.end(); it++) {
                if (fabs_time_diff(&(image_obj_ranged_ringbuf.front().header), &((it-1)->header))
                    < fabs_time_diff(&(image_obj_ranged_ringbuf.front().header), &(it->header))) {
                    p_image_raw_buf = &*(it-1);
                    break;
                }
            }

            if (it == image_raw_ringbuf.end()) {
                p_image_raw_buf = &(image_raw_ringbuf.back());
            }
        }
        publish_msg(p_image_obj_ranged_buf, p_image_raw_buf);
        if (image_obj_tracked_flag == true){
            buf_flag = false;
            image_obj_tracked_flag = false;
            image_obj_ranged_ringbuf.clear();
            image_raw_ringbuf.clear();
        }

        return true;
    } else {
        return false;
    }
}

void image_obj_ranged_callback(const cv_tracker::image_obj_ranged::ConstPtr& image_obj_ranged_msg) {
    pthread_mutex_lock(&mutex);
    image_obj_ranged_ringbuf.push_front(*image_obj_ranged_msg);
    //image_raw is empty
    if (image_raw_ringbuf.begin() == image_raw_ringbuf.end()) {
        ROS_INFO("image_raw ring buffer is empty");
        buf_flag = false;
        pthread_mutex_unlock(&mutex);
        return;
    }
    buf_flag = true;
    pthread_mutex_unlock(&mutex);
    pthread_mutex_lock(&mutex);
    if (image_obj_tracked_flag == true) {
        publish();
    }
    pthread_mutex_unlock(&mutex);
}

void image_raw_callback(const sensor_msgs::Image::ConstPtr& image_raw_msg) {
    pthread_mutex_lock(&mutex);
    image_raw_ringbuf.push_front(*image_raw_msg);
    //image_obj_ranged is empty
    if (image_obj_ranged_ringbuf.begin() == image_obj_ranged_ringbuf.end()) {
        ROS_INFO("image_obj_ranged ring buffer is empty");
        buf_flag = false;
        pthread_mutex_unlock(&mutex);
        return;
    }
    buf_flag = true;
    pthread_mutex_unlock(&mutex);
    pthread_mutex_lock(&mutex);
    if (image_obj_tracked_flag == true) {
        publish();
    }
    pthread_mutex_unlock(&mutex);
}
#else
#endif

void image_obj_tracked_callback(const cv_tracker::image_obj_tracked::ConstPtr& image_obj_tracked_msg) {
    pthread_mutex_lock(&mutex);
    image_obj_tracked_flag = true;
    ROS_INFO("catch publish request");
    if (publish() == false) {
        ROS_INFO("waitting...");
    }
    pthread_mutex_unlock(&mutex);
}

void* thread(void* args) {
    ros::NodeHandle nh_rcv;
    ros::CallbackQueue rcv_callbackqueue;
    nh_rcv.setCallbackQueue(&rcv_callbackqueue);
    ros::Subscriber image_obj_tracked_sub = nh_rcv.subscribe("/image_obj_tracked", 1, image_obj_tracked_callback);
    while (nh_rcv.ok()) {
        rcv_callbackqueue.callAvailable(ros::WallDuration(3.0f));
        pthread_mutex_lock(&mutex);
        bool flag = (image_obj_tracked_flag == false && buf_flag == true);
        if (flag) {
            ROS_INFO("timeout");
            if(!publish()) {
                /* when to publish is failure, republish */
                struct timespec sleep_time;
                sleep_time.tv_sec = 0;
                sleep_time.tv_nsec = 200000000; //5Hz
                while (!publish() && ros::ok())
                    nanosleep(&sleep_time, NULL);
            }
        }
        pthread_mutex_unlock(&mutex);

    }
    return NULL;
}

int main(int argc, char **argv) {
    /* init */
    buf_flag = false;
    image_obj_tracked_flag = false;
    ros::init(argc, argv, "sync_tracking");
    ros::NodeHandle nh;

    /* create server thread */
    pthread_t th;
    pthread_create(&th, NULL, thread, (void *)NULL );

    ros::Subscriber image_obj_ranged_sub = nh.subscribe("/image_obj_ranged", 1, image_obj_ranged_callback);
    ros::Subscriber image_raw_sub = nh.subscribe("/sync_drivers/image_raw", 1, image_raw_callback);
    image_obj_ranged__pub = nh.advertise<cv_tracker::image_obj_ranged>("/sync_tracking/image_obj_ranged", 5);
    image_raw__pub = nh.advertise<sensor_msgs::Image>("/sync_tracking/image_raw", 5);

    while ((!buf_flag) && ros::ok()) {
        ros::spinOnce();
        usleep(100000);
    }
    pthread_mutex_lock(&mutex);
    if(!publish()) {
        /* when to publish is failure, republish */
        struct timespec sleep_time;
        sleep_time.tv_sec = 0;
        sleep_time.tv_nsec = 200000000; //5Hz
        while (!publish() || ros::ok())
            nanosleep(&sleep_time, NULL);
    }
    pthread_mutex_unlock(&mutex);

    ros::spin();
    pthread_mutex_unlock(&mutex);

    /* shutdown server thread */
    ROS_INFO("wait until shutdown a thread");
    pthread_kill(th, SIGINT);
    pthread_join(th, NULL);

    return 0;
}



#if 0
cv_tracker::image_obj_ranged image_obj_ranged_buf;
sensor_msgs::Image image_raw_buf;

void image_obj_ranged_callback(const cv_tracker::image_obj_ranged::ConstPtr& image_obj_ranged_msg) {
    pthread_mutex_lock(&mutex);
    image_obj_ranged_ringbuf.push_front(*image_obj_ranged_msg);

    //image_raw is empty
    if (image_raw_ringbuf.begin() == image_raw_ringbuf.end()) {
        pthread_mutex_unlock(&mutex);
        ROS_INFO("image_raw ring buffer is empty");
        return;
    }

    buf_flag = true;

    // image_obj_ranged > image_raw
    if (get_time(&(image_obj_ranged_ringbuf.front().header)) >= get_time(&(image_raw_ringbuf.front().header))) {
        image_raw_buf = image_raw_ringbuf.front();
        boost::circular_buffer<cv_tracker::image_obj_ranged>::iterator it = image_obj_ranged_ringbuf.begin();
        if (image_obj_ranged_ringbuf.size() == 1) {
            image_obj_ranged_buf = *it;
            pthread_mutex_unlock(&mutex);
            return;
        } else {
            for (it++; it != image_obj_ranged_ringbuf.end(); it++) {
                if (fabs_time_diff(&(image_raw_ringbuf.front().header), &((it-1)->header))
                    < fabs_time_diff(&(image_raw_ringbuf.front().header), &(it->header))) {
                    image_obj_ranged_buf = *(it-1);
                    break;
                }
            }
            if (it == image_obj_ranged_ringbuf.end()) {
                image_obj_ranged_buf = image_obj_ranged_ringbuf.back();
            }
        }

    } else {
        image_obj_ranged_buf = image_obj_ranged_ringbuf.front();
        boost::circular_buffer<sensor_msgs::Image>::iterator it = image_raw_ringbuf.begin();
        if (image_raw_ringbuf.size() == 1) {
            image_raw_buf = *it;
            pthread_mutex_unlock(&mutex);
            return;
        }

        for (it++; it != image_raw_ringbuf.end(); it++) {
            if (fabs_time_diff(&(image_obj_ranged_ringbuf.front().header), &((it-1)->header))
                < fabs_time_diff(&(image_obj_ranged_ringbuf.front().header), &(it->header))) {
                image_raw_buf = *(it-1);
                break;
            }
        }

        if (it == image_raw_ringbuf.end()) {
            image_raw_buf = image_raw_ringbuf.back();
        }
    }
    pthread_mutex_unlock(&mutex);
}

void image_raw_callback(const sensor_msgs::Image::ConstPtr& image_raw_msg) {
    pthread_mutex_lock(&mutex);
    image_raw_ringbuf.push_front(*image_raw_msg);
    //image_obj_ranged is empty
    if (image_obj_ranged_ringbuf.begin() == image_obj_ranged_ringbuf.end()) {
        ROS_INFO("image_obj_ranged ring buffer is empty");
        pthread_mutex_unlock(&mutex);
        return;
    }

    buf_flag = true;

    // image_obj_ranged > image_raw
    if (get_time(&(image_obj_ranged_ringbuf.front().header)) >= get_time(&(image_raw_ringbuf.front().header))) {
        image_raw_buf = image_raw_ringbuf.front();
        boost::circular_buffer<cv_tracker::image_obj_ranged>::iterator it = image_obj_ranged_ringbuf.begin();
        if (image_obj_ranged_ringbuf.size() == 1) {
            image_obj_ranged_buf = *it;
            pthread_mutex_unlock(&mutex);
            return;
        } else {
            for (it++; it != image_obj_ranged_ringbuf.end(); it++) {
                if (fabs_time_diff(&(image_raw_ringbuf.front().header), &((it-1)->header))
                    < fabs_time_diff(&(image_raw_ringbuf.front().header), &(it->header))) {
                    image_obj_ranged_buf = *(it-1);
                    break;
                }
            }
            if (it == image_obj_ranged_ringbuf.end()) {
                image_obj_ranged_buf = image_obj_ranged_ringbuf.back();
            }
        }

    } else {
        image_obj_ranged_buf = image_obj_ranged_ringbuf.front();
        boost::circular_buffer<sensor_msgs::Image>::iterator it = image_raw_ringbuf.begin();
        if (image_raw_ringbuf.size() == 1) {
            image_raw_buf = *it;
            pthread_mutex_unlock(&mutex);
            return;
        }

        for (it++; it != image_raw_ringbuf.end(); it++) {
            if (fabs_time_diff(&(image_obj_ranged_ringbuf.front().header), &((it-1)->header))
                < fabs_time_diff(&(image_obj_ranged_ringbuf.front().header), &(it->header))) {
                image_raw_buf = *(it-1);
                break;
            }
        }

        if (it == image_raw_ringbuf.end()) {
            image_raw_buf = image_raw_ringbuf.back();
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
        image_obj_ranged__pub.publish(image_obj_ranged_buf);
        image_raw__pub.publish(image_raw_buf);
        pthread_mutex_unlock(&mutex);
        return true;
    } else {
        ROS_INFO("publish failed");
        return false;
    }
}

#endif
