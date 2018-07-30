#include "ros/ros.h"
#include "autoware_msgs/ImageObj.h"
#include "autoware_msgs/PointsImage.h"
#include "autoware_msgs/ImageObjRanged.h"
#include "sync.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "sync_ranging");
    std::string ns(ros::this_node::getNamespace());
    std::string sub1("/image_obj");
    std::string sub2("/vscan_image");
    std::string req("/image_obj_ranged");
    std::string pub1("/image_obj");
    std::string pub2("/vscan_image");

    Synchronizer<autoware_msgs::ImageObj, autoware_msgs::PointsImage, autoware_msgs::ImageObjRanged> synchronizer(sub1, sub2, pub1, pub2, req, ns);
    synchronizer.run();

    return 0;
}

#if 0
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
#include "autoware_msgs/ImageObj.h"
#include "autoware_msgs/PointsImage.h"
#include "autoware_msgs/ImageObjRanged.h"

/* ----mode---- */
#define _REQ_PUB 1

/* ----var---- */
/* common var */
bool buf_flag;
pthread_mutex_t mutex;
/* user var */
boost::circular_buffer<autoware_msgs::ImageObj> image_obj_ringbuf(10);
boost::circular_buffer<autoware_msgs::PointsImage> vscan_image_ringbuf(10);
ros::Publisher image_obj__pub;
ros::Publisher vscan_image__pub;
bool image_obj_ranged_flag;

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
autoware_msgs::ImageObj* p_image_obj_buf;
autoware_msgs::PointsImage* p_vscan_image_buf;

void publish_msg(autoware_msgs::ImageObj* p_image_obj_buf, autoware_msgs::PointsImage* p_vscan_image_buf) {
    ROS_INFO("publish");
    image_obj__pub.publish(*p_image_obj_buf);
    vscan_image__pub.publish(*p_vscan_image_buf);
}

bool publish() {
    if (buf_flag) {
        //image_obj is empty
        if (image_obj_ringbuf.begin() == image_obj_ringbuf.end()) {
            ROS_INFO("image_obj ring buffer is empty");
            return false;
        }

        //vscan_image is empty
        if (vscan_image_ringbuf.begin() == vscan_image_ringbuf.end()) {
            ROS_INFO("vscan_image ring buffer is empty");
            return false;
        }

        // image_obj > vscan_image
        if (get_time(&(image_obj_ringbuf.front().header)) >= get_time(&(vscan_image_ringbuf.front().header))) {
            p_vscan_image_buf = &(vscan_image_ringbuf.front());
            boost::circular_buffer<autoware_msgs::ImageObj>::iterator it = image_obj_ringbuf.begin();
            if (image_obj_ringbuf.size() == 1) {
                p_image_obj_buf = &*it;
                publish_msg(p_image_obj_buf, p_vscan_image_buf);
                if (image_obj_ranged_flag == true){
                    buf_flag = false;
                    image_obj_ranged_flag = false;
                    image_obj_ringbuf.clear();
                    vscan_image_ringbuf.clear();
                }
                return true;
            } else {
                for (it++; it != image_obj_ringbuf.end(); it++) {
                    if (fabs_time_diff(&(vscan_image_ringbuf.front().header), &((it-1)->header))
                        < fabs_time_diff(&(vscan_image_ringbuf.front().header), &(it->header))) {
                        p_image_obj_buf = &*(it-1);
                        break;
                    }
                }
                if (it == image_obj_ringbuf.end()) {
                    p_image_obj_buf = &(image_obj_ringbuf.back());
                }
            }
        }
        // image_obj < vscan_image
        else {
            p_image_obj_buf = &(image_obj_ringbuf.front());
            boost::circular_buffer<autoware_msgs::PointsImage>::iterator it = vscan_image_ringbuf.begin();
            if (vscan_image_ringbuf.size() == 1) {
                p_vscan_image_buf = &*it;
                publish_msg(p_image_obj_buf, p_vscan_image_buf);
                if (image_obj_ranged_flag == true){
                    buf_flag = false;
                    image_obj_ranged_flag = false;
                    image_obj_ringbuf.clear();
                    vscan_image_ringbuf.clear();
                }
                return true;
            }

            for (it++; it != vscan_image_ringbuf.end(); it++) {
                if (fabs_time_diff(&(image_obj_ringbuf.front().header), &((it-1)->header))
                    < fabs_time_diff(&(image_obj_ringbuf.front().header), &(it->header))) {
                    p_vscan_image_buf = &*(it-1);
                    break;
                }
            }

            if (it == vscan_image_ringbuf.end()) {
                p_vscan_image_buf = &(vscan_image_ringbuf.back());
            }
        }
        publish_msg(p_image_obj_buf, p_vscan_image_buf);
        if (image_obj_ranged_flag == true){
            buf_flag = false;
            image_obj_ranged_flag = false;
            image_obj_ringbuf.clear();
            vscan_image_ringbuf.clear();
        }

        return true;
    } else {
        return false;
    }
}

void image_obj_callback(const autoware_msgs::ImageObj::ConstPtr& image_obj_msg) {
    pthread_mutex_lock(&mutex);
    image_obj_ringbuf.push_front(*image_obj_msg);
    //vscan_image is empty
    if (vscan_image_ringbuf.begin() == vscan_image_ringbuf.end()) {
        ROS_INFO("vscan_image ring buffer is empty");
        buf_flag = false;
        pthread_mutex_unlock(&mutex);
        return;
    }
    buf_flag = true;
    pthread_mutex_unlock(&mutex);
    pthread_mutex_lock(&mutex);
    if (image_obj_ranged_flag == true) {
        publish();
    }
    pthread_mutex_unlock(&mutex);
}

void vscan_image_callback(const autoware_msgs::PointsImage::ConstPtr& vscan_image_msg) {
    pthread_mutex_lock(&mutex);
    vscan_image_ringbuf.push_front(*vscan_image_msg);
    //image_obj is empty
    if (image_obj_ringbuf.begin() == image_obj_ringbuf.end()) {
        ROS_INFO("image_obj ring buffer is empty");
        buf_flag = false;
        pthread_mutex_unlock(&mutex);
        return;
    }
    buf_flag = true;
    pthread_mutex_unlock(&mutex);
    pthread_mutex_lock(&mutex);
    if (image_obj_ranged_flag == true) {
        publish();
    }
    pthread_mutex_unlock(&mutex);
}
#else
#endif

void image_obj_ranged_callback(const autoware_msgs::ImageObjRanged::ConstPtr& image_obj_ranged_msg) {
    pthread_mutex_lock(&mutex);
    image_obj_ranged_flag = true;
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
    ros::Subscriber image_obj_ranged_sub = nh_rcv.subscribe("/image_obj_ranged", 1, image_obj_ranged_callback);
    while (nh_rcv.ok()) {
        rcv_callbackqueue.callAvailable(ros::WallDuration(3.0f));
        pthread_mutex_lock(&mutex);
        bool flag = (image_obj_ranged_flag == false && buf_flag == true);
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
    image_obj_ranged_flag = false;
    ros::init(argc, argv, "sync_ranging");
    ros::NodeHandle nh;

    /* create server thread */
    pthread_t th;
    pthread_create(&th, NULL, thread, (void *)NULL );

    ros::Subscriber image_obj_sub = nh.subscribe("/image_obj", 1, image_obj_callback);
    ros::Subscriber vscan_image_sub = nh.subscribe("/vscan_image", 1, vscan_image_callback);
    image_obj__pub = nh.advertise<autoware_msgs::ImageObj>("/sync_ranging/image_obj", 5);
    vscan_image__pub = nh.advertise<autoware_msgs::PointsImage>("/sync_ranging/vscan_image", 5);

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
        while (!publish() && ros::ok())
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
autoware_msgs::ImageObj image_obj_buf;
autoware_msgs::PointsImage vscan_image_buf;

void image_obj_callback(const autoware_msgs::ImageObj::ConstPtr& image_obj_msg) {
    pthread_mutex_lock(&mutex);
    image_obj_ringbuf.push_front(*image_obj_msg);

    //vscan_image is empty
    if (vscan_image_ringbuf.begin() == vscan_image_ringbuf.end()) {
        pthread_mutex_unlock(&mutex);
        ROS_INFO("vscan_image ring buffer is empty");
        return;
    }

    buf_flag = true;

    // image_obj > vscan_image
    if (get_time(&(image_obj_ringbuf.front().header)) >= get_time(&(vscan_image_ringbuf.front().header))) {
        vscan_image_buf = vscan_image_ringbuf.front();
        boost::circular_buffer<autoware_msgs::ImageObj>::iterator it = image_obj_ringbuf.begin();
        if (image_obj_ringbuf.size() == 1) {
            image_obj_buf = *it;
            pthread_mutex_unlock(&mutex);
            return;
        } else {
            for (it++; it != image_obj_ringbuf.end(); it++) {
                if (fabs_time_diff(&(vscan_image_ringbuf.front().header), &((it-1)->header))
                    < fabs_time_diff(&(vscan_image_ringbuf.front().header), &(it->header))) {
                    image_obj_buf = *(it-1);
                    break;
                }
            }
            if (it == image_obj_ringbuf.end()) {
                image_obj_buf = image_obj_ringbuf.back();
            }
        }

    } else {
        image_obj_buf = image_obj_ringbuf.front();
        boost::circular_buffer<autoware_msgs::PointsImage>::iterator it = vscan_image_ringbuf.begin();
        if (vscan_image_ringbuf.size() == 1) {
            vscan_image_buf = *it;
            pthread_mutex_unlock(&mutex);
            return;
        }

        for (it++; it != vscan_image_ringbuf.end(); it++) {
            if (fabs_time_diff(&(image_obj_ringbuf.front().header), &((it-1)->header))
                < fabs_time_diff(&(image_obj_ringbuf.front().header), &(it->header))) {
                vscan_image_buf = *(it-1);
                break;
            }
        }

        if (it == vscan_image_ringbuf.end()) {
            vscan_image_buf = vscan_image_ringbuf.back();
        }
    }
    pthread_mutex_unlock(&mutex);
}

void vscan_image_callback(const autoware_msgs::PointsImage::ConstPtr& vscan_image_msg) {
    pthread_mutex_lock(&mutex);
    vscan_image_ringbuf.push_front(*vscan_image_msg);
    //image_obj is empty
    if (image_obj_ringbuf.begin() == image_obj_ringbuf.end()) {
        ROS_INFO("image_obj ring buffer is empty");
        pthread_mutex_unlock(&mutex);
        return;
    }

    buf_flag = true;

    // image_obj > vscan_image
    if (get_time(&(image_obj_ringbuf.front().header)) >= get_time(&(vscan_image_ringbuf.front().header))) {
        vscan_image_buf = vscan_image_ringbuf.front();
        boost::circular_buffer<autoware_msgs::ImageObj>::iterator it = image_obj_ringbuf.begin();
        if (image_obj_ringbuf.size() == 1) {
            image_obj_buf = *it;
            pthread_mutex_unlock(&mutex);
            return;
        } else {
            for (it++; it != image_obj_ringbuf.end(); it++) {
                if (fabs_time_diff(&(vscan_image_ringbuf.front().header), &((it-1)->header))
                    < fabs_time_diff(&(vscan_image_ringbuf.front().header), &(it->header))) {
                    image_obj_buf = *(it-1);
                    break;
                }
            }
            if (it == image_obj_ringbuf.end()) {
                image_obj_buf = image_obj_ringbuf.back();
            }
        }

    } else {
        image_obj_buf = image_obj_ringbuf.front();
        boost::circular_buffer<autoware_msgs::PointsImage>::iterator it = vscan_image_ringbuf.begin();
        if (vscan_image_ringbuf.size() == 1) {
            vscan_image_buf = *it;
            pthread_mutex_unlock(&mutex);
            return;
        }

        for (it++; it != vscan_image_ringbuf.end(); it++) {
            if (fabs_time_diff(&(image_obj_ringbuf.front().header), &((it-1)->header))
                < fabs_time_diff(&(image_obj_ringbuf.front().header), &(it->header))) {
                vscan_image_buf = *(it-1);
                break;
            }
        }

        if (it == vscan_image_ringbuf.end()) {
            vscan_image_buf = vscan_image_ringbuf.back();
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
        image_obj__pub.publish(image_obj_buf);
        vscan_image__pub.publish(vscan_image_buf);
        pthread_mutex_unlock(&mutex);
        return true;
    } else {
        ROS_INFO("publish failed");
        return false;
    }
}

#endif
#endif
