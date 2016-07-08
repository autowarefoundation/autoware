/* ----header---- */
/* common header */
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include <std_msgs/Float64.h>
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

#define PRINT

extern "C" double fabs_time_diff(std_msgs::Header *timespec1, std_msgs::Header *timespec2) {
    double time1 = (double)timespec1->stamp.sec + (double)timespec1->stamp.nsec/1000000000L;
    double time2 = (double)timespec2->stamp.sec + (double)timespec2->stamp.nsec/1000000000L;

    return fabs(time1 - time2);
}

extern "C" double get_time(const std_msgs::Header *timespec) {
    return (double)timespec->stamp.sec + (double)timespec->stamp.nsec/1000000000L;
}


template<typename T1, typename T2, typename T3>
Synchronizer<T1, T2, T3>::Synchronizer(const std::string sub1_topic, const std::string sub2_topic, const std::string pub1_topic, const std::string pub2_topic, const std::string req_topic, const std::string ns) :
    type1_ringbuf_(10), type2_ringbuf_(10)
{
    /* init */
    pthread_mutex_init(&mutex_, NULL);

    ros::NodeHandle nh;
    buf_flag_ = false;
    is_req_ = false;

    type1_sub_ = nh.subscribe(sub1_topic, 1, &Synchronizer::type1_callback, this);
    type2_sub_ = nh.subscribe(sub2_topic, 1, &Synchronizer::type2_callback, this);
#if 0
    ros::NodeHandle nh_rcv;
    nh_rcv.setCallbackQueue(&rcv_callbackqueue_);
    req_sub_ = nh_rcv.subscribe(req_topic, 1, &Synchronizer::req_callback, this);
#endif
    req_topic_ = req_topic;
    type1_pub_ = nh.advertise<T1>(ns+pub1_topic, 5);
    type2_pub_ = nh.advertise<T2>(ns+pub2_topic, 5);
    sync_time_diff_pub_ = nh.advertise<std_msgs::Float64>("/"+ns+"/time_diff", 5);
}

template<typename T1, typename T2, typename T3>
void Synchronizer<T1, T2, T3>::run() {
    /* create server thread */
    pthread_t th;
    pthread_create(&th, NULL, launchThread, this);

    while (!buf_flag_) {
        ros::spinOnce();
        usleep(100000);
        if(!ros::ok())
            return;
    }

    pthread_mutex_lock(&mutex_);
    if(!publish()) {
        /* when to publish is failure, republish */
        struct timespec sleep_time;
        sleep_time.tv_sec = 0;
        sleep_time.tv_nsec = 200000000; //5Hz
        while (!publish() && ros::ok())
            nanosleep(&sleep_time, NULL);
    }
    pthread_mutex_unlock(&mutex_);

    ros::spin();

    /* shutdown server thread */
    ROS_DEBUG("wait until shutdown a thread");
    pthread_mutex_trylock(&mutex_);
    pthread_mutex_unlock(&mutex_);
    pthread_kill(th, SIGINT);
    pthread_join(th, NULL);

}

template<typename T1, typename T2, typename T3>
void Synchronizer<T1, T2, T3>::thread() {
#if 1
    ros::NodeHandle nh_rcv;
    ros::CallbackQueue rcv_callbackqueue_;
    nh_rcv.setCallbackQueue(&rcv_callbackqueue_);
    ros::Subscriber req_sub = nh_rcv.subscribe(req_topic_, 1, &Synchronizer::req_callback, this);
    while (nh_rcv.ok()) {
#else
    while (ros::ok()) {
#endif
        rcv_callbackqueue_.callAvailable(ros::WallDuration(3.0f));
        pthread_mutex_lock(&mutex_);
        bool flag = (is_req_ == false && buf_flag_ == true);
        if (flag) {
            ROS_DEBUG("timeout");
            if(!publish()) {
                /* when to publish is failure, republish */
                struct timespec sleep_time;
                sleep_time.tv_sec = 0;
                sleep_time.tv_nsec = 200000000; //5Hz
                while (!publish() && ros::ok() && nh_rcv.ok())
                    nanosleep(&sleep_time, NULL);
            }
        }
        pthread_mutex_unlock(&mutex_);
    }
}

template<typename T1, typename T2, typename T3>
void Synchronizer<T1, T2, T3>::publish_msg(T1* p_type1_buf, T2* p_type2_buf) {
    ROS_DEBUG("publish");
    type1_pub_.publish(*p_type1_buf);
    type2_pub_.publish(*p_type2_buf);

    std_msgs::Float64 time_diff;
    time_diff.data = fabs_time_diff(&p_type1_buf->header, &p_type2_buf->header);
    sync_time_diff_pub_.publish(time_diff);
}

template<typename T1, typename T2, typename T3>
bool Synchronizer<T1, T2, T3>::publish() {
    if (buf_flag_) {
        //image_obj_ranged is empty
        if (type1_ringbuf_.begin() == type1_ringbuf_.end()) {
            ROS_DEBUG("type1 ring buffer is empty");
            return false;
        }

        //image_raw is empty
        if (type2_ringbuf_.begin() == type2_ringbuf_.end()) {
            ROS_DEBUG("type2 ring buffer is empty");
            return false;
        }

        // image_obj_ranged > image_raw
        if (get_time(&(type1_ringbuf_.front().header)) >= get_time(&(type2_ringbuf_.front().header))) {
            p_type2_buf_ = &(type2_ringbuf_.front());
            typename boost::circular_buffer<T1>::iterator it = type1_ringbuf_.begin();
            if (type1_ringbuf_.size() == 1) {
                p_type1_buf_ = &*it;
                publish_msg(p_type1_buf_, p_type2_buf_);
                if (is_req_ == true){
                    buf_flag_ = false;
                    is_req_ = false;
                    if (fabs_time_diff(&p_type1_buf_->header, &p_type2_buf_->header) == 0.0) {
                        type1_ringbuf_.clear();
                        type2_ringbuf_.clear();
                    }
                }
                return true;
            } else {
                for (it++; it != type1_ringbuf_.end(); it++) {
                    if (fabs_time_diff(&(type2_ringbuf_.front().header), &((it-1)->header))
                        < fabs_time_diff(&(type2_ringbuf_.front().header), &(it->header))) {
                        p_type1_buf_ = &*(it-1);
                        break;
                    }
                }
                if (it == type1_ringbuf_.end()) {
                    p_type1_buf_ = &(type1_ringbuf_.back());
                }
            }
        }
        // image_obj_ranged < image_raw
        else {
            p_type1_buf_ = &(type1_ringbuf_.front());
            typename boost::circular_buffer<T2>::iterator it = type2_ringbuf_.begin();
            if (type2_ringbuf_.size() == 1) {
                p_type2_buf_ = &*it;
                publish_msg(p_type1_buf_, p_type2_buf_);
                if (is_req_ == true){
                    buf_flag_ = false;
                    is_req_ = false;
                    if (fabs_time_diff(&p_type1_buf_->header, &p_type2_buf_->header) == 0.0) {
                        type1_ringbuf_.clear();
                        type2_ringbuf_.clear();
                    }
                }
                return true;
            }

            for (it++; it != type2_ringbuf_.end(); it++) {
                if (fabs_time_diff(&(type1_ringbuf_.front().header), &((it-1)->header))
                    < fabs_time_diff(&(type1_ringbuf_.front().header), &(it->header))) {
                    p_type2_buf_ = &*(it-1);
                    break;
                }
            }

            if (it == type2_ringbuf_.end()) {
                p_type2_buf_ = &(type2_ringbuf_.back());
            }
        }
        publish_msg(p_type1_buf_, p_type2_buf_);
        if (is_req_ == true){
            buf_flag_ = false;
            is_req_ = false;
            if (fabs_time_diff(&p_type1_buf_->header, &p_type2_buf_->header) == 0.0) {
                type1_ringbuf_.clear();
                type2_ringbuf_.clear();
            }
        }

        return true;
    } else {
        return false;
    }
}

template<typename T1, typename T2, typename T3>
void Synchronizer<T1, T2, T3>::type1_callback(const typename T1::ConstPtr& type1_msg) {
    ROS_DEBUG("catch type1 topic");

    pthread_mutex_lock(&mutex_);
    type1_ringbuf_.push_front(*type1_msg);
    //image_raw is empty
    if (type2_ringbuf_.begin() == type2_ringbuf_.end()) {
        ROS_DEBUG("image_raw ring buffer is empty");
        buf_flag_ = false;
        pthread_mutex_unlock(&mutex_);
        return;
    }
    buf_flag_ = true;
    pthread_mutex_unlock(&mutex_);
    pthread_mutex_lock(&mutex_);
    if (is_req_ == true) {
        publish();
    }
    pthread_mutex_unlock(&mutex_);
}

template<typename T1, typename T2, typename T3>
void Synchronizer<T1, T2, T3>::type2_callback(const typename T2::ConstPtr& type2_msg) {
    ROS_DEBUG("catch type2 topic");

    pthread_mutex_lock(&mutex_);
    type2_ringbuf_.push_front(*type2_msg);
    //image_obj_ranged is empty
    if (type1_ringbuf_.begin() == type1_ringbuf_.end()) {
        ROS_DEBUG("image_obj_ranged ring buffer is empty");
        buf_flag_ = false;
        pthread_mutex_unlock(&mutex_);
        return;
    }
    buf_flag_ = true;
    pthread_mutex_unlock(&mutex_);
    pthread_mutex_lock(&mutex_);
    if (is_req_ == true) {
        publish();
    }
    pthread_mutex_unlock(&mutex_);
}

template<typename T1, typename T2, typename T3>
void Synchronizer<T1, T2, T3>::req_callback(const typename T3::ConstPtr& req_msg) {
    pthread_mutex_lock(&mutex_);
    is_req_ = true;
    ROS_DEBUG("catch publish request");
    if (publish() == false) {
        ROS_DEBUG("waitting...");
    }
    pthread_mutex_unlock(&mutex_);
}
