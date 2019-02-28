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
#include "cv_tracker/obj_label.h"
#include "autoware_msgs/Centroids.h"
#include "visualization_msgs/MarkerArray.h"

/* ----mode---- */
#define _REQ_PUB 1

/* ----var---- */
/* common var */
bool buf_flag;
pthread_mutex_t mutex;
/* user var */
boost::circular_buffer<cv_tracker::obj_label> obj_label_ringbuf(10);
boost::circular_buffer<autoware_msgs::Centroids> cluster_centroids_ringbuf(10);
ros::Publisher obj_label__pub;
ros::Publisher cluster_centroids__pub;
bool obj_pose_flag;

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
cv_tracker::obj_label* p_obj_label_buf;
autoware_msgs::Centroids* p_cluster_centroids_buf;

void publish_msg(cv_tracker::obj_label* p_obj_label_buf, autoware_msgs::Centroids* p_cluster_centroids_buf)
{
    ROS_INFO("publish");
    obj_label__pub.publish(*p_obj_label_buf);
    cluster_centroids__pub.publish(*p_cluster_centroids_buf);
}

bool publish() {
    if (buf_flag) {
        //obj_label is empty
        if (obj_label_ringbuf.begin() == obj_label_ringbuf.end()) {
            ROS_INFO("obj_label ring buffer is empty");
            return false;
        }

        //cluster_centroids is empty
        if (cluster_centroids_ringbuf.begin() == cluster_centroids_ringbuf.end()) {
            ROS_INFO("cluster_centroids ring buffer is empty");
            return false;
        }

        // obj_label > cluster_centroids
        if (get_time(&(obj_label_ringbuf.front().header)) >= get_time(&(cluster_centroids_ringbuf.front().header))) {
            p_cluster_centroids_buf = &(cluster_centroids_ringbuf.front());
            boost::circular_buffer<cv_tracker::obj_label>::iterator it = obj_label_ringbuf.begin();
            if (obj_label_ringbuf.size() == 1) {
                p_obj_label_buf = &*it;
                publish_msg(p_obj_label_buf, p_cluster_centroids_buf);
                if (obj_pose_flag == true){
                    buf_flag = false;
                    obj_pose_flag = false;
                    obj_label_ringbuf.clear();
                    cluster_centroids_ringbuf.clear();
                }
                return true;
            } else {
                for (it++; it != obj_label_ringbuf.end(); it++) {
                    if (fabs_time_diff(&(cluster_centroids_ringbuf.front().header), &((it-1)->header))
                        < fabs_time_diff(&(cluster_centroids_ringbuf.front().header), &(it->header))) {
                        p_obj_label_buf = &*(it-1);
                        break;
                    }
                }
                if (it == obj_label_ringbuf.end()) {
                    p_obj_label_buf = &(obj_label_ringbuf.back());
                }
            }
        }
        // obj_label < cluster_centroids
        else {
            p_obj_label_buf = &(obj_label_ringbuf.front());
            boost::circular_buffer<autoware_msgs::Centroids>::iterator it = cluster_centroids_ringbuf.begin();
            if (cluster_centroids_ringbuf.size() == 1) {
                p_cluster_centroids_buf = &*it;
                publish_msg(p_obj_label_buf, p_cluster_centroids_buf);
                if (obj_pose_flag == true){
                    buf_flag = false;
                    obj_pose_flag = false;
                    obj_label_ringbuf.clear();
                    cluster_centroids_ringbuf.clear();
                }
                return true;
            }

            for (it++; it != cluster_centroids_ringbuf.end(); it++) {
                if (fabs_time_diff(&(obj_label_ringbuf.front().header), &((it-1)->header))
                    < fabs_time_diff(&(obj_label_ringbuf.front().header), &(it->header))) {
                    p_cluster_centroids_buf = &*(it-1);
                    break;
                }
            }

            if (it == cluster_centroids_ringbuf.end()) {
                p_cluster_centroids_buf = &(cluster_centroids_ringbuf.back());
            }
        }
        publish_msg(p_obj_label_buf, p_cluster_centroids_buf);
        if (obj_pose_flag == true){
            buf_flag = false;
            obj_pose_flag = false;
            obj_label_ringbuf.clear();
            cluster_centroids_ringbuf.clear();
        }
        return true;
    } else {
        return false;
    }
}

void obj_label_callback(const cv_tracker::obj_label::ConstPtr& obj_label_msg) {
    pthread_mutex_lock(&mutex);
    obj_label_ringbuf.push_front(*obj_label_msg);
    //cluster_centroids is empty
    if (cluster_centroids_ringbuf.begin() == cluster_centroids_ringbuf.end()) {
        pthread_mutex_unlock(&mutex);
        ROS_INFO("cluster_centroids ring buffer is empty");
        return;
    }
    buf_flag = true;
    pthread_mutex_unlock(&mutex);
    pthread_mutex_lock(&mutex);
    if (obj_pose_flag == true) {
        publish();
    }
    pthread_mutex_unlock(&mutex);
}

void cluster_centroids_callback(const autoware_msgs::Centroids::ConstPtr& cluster_centroids_msg) {
    pthread_mutex_lock(&mutex);
    cluster_centroids_ringbuf.push_front(*cluster_centroids_msg);
    //obj_label is empty
    if (obj_label_ringbuf.begin() == obj_label_ringbuf.end()) {
        ROS_INFO("obj_label ring buffer is empty");
        pthread_mutex_unlock(&mutex);
        return;
    }

    buf_flag = true;
    pthread_mutex_unlock(&mutex);
    pthread_mutex_lock(&mutex);
    if (obj_pose_flag == true) {
        publish();
    }
    pthread_mutex_unlock(&mutex);
}

#else
cv_tracker::obj_label obj_label_buf;
autoware_msgs::Centroids cluster_centroids_buf;

bool publish() {
    if (buf_flag) {
        ROS_INFO("publish");
        obj_label__pub.publish(obj_label_buf);
        cluster_centroids__pub.publish(cluster_centroids_buf);
        if (obj_pose_flag == true){
            buf_flag = false;
            obj_pose_flag = false;
            obj_label_ringbuf.clear();
            cluster_centroids_ringbuf.clear();
        }
        return true;
    } else {
        ROS_INFO("publish failed");
        return false;
    }
}

void obj_label_callback(const cv_tracker::obj_label::ConstPtr& obj_label_msg) {
    pthread_mutex_lock(&mutex);
    obj_label_ringbuf.push_front(*obj_label_msg);

    //cluster_centroids is empty
    if (cluster_centroids_ringbuf.begin() == cluster_centroids_ringbuf.end()) {
        pthread_mutex_unlock(&mutex);
        ROS_INFO("cluster_centroids ring buffer is empty");
        return;
    }

    buf_flag = true;

    // obj_label > cluster_centroids
    if (get_time(&(obj_label_ringbuf.front().header)) >= get_time(&(cluster_centroids_ringbuf.front().header))) {
        cluster_centroids_buf = cluster_centroids_ringbuf.front();
        boost::circular_buffer<cv_tracker::obj_label>::iterator it = obj_label_ringbuf.begin();
        if (obj_label_ringbuf.size() == 1) {
            obj_label_buf = *it;
            pthread_mutex_unlock(&mutex);
            pthread_mutex_lock(&mutex);
            if (obj_pose_flag == true) {
                publish();
            }
            pthread_mutex_unlock(&mutex);
            return;
        } else {
            for (it++; it != obj_label_ringbuf.end(); it++) {
                if (fabs_time_diff(&(cluster_centroids_ringbuf.front().header), &((it-1)->header))
                    < fabs_time_diff(&(cluster_centroids_ringbuf.front().header), &(it->header))) {
                    obj_label_buf = *(it-1);
                    break;
                }
            }
            if (it == obj_label_ringbuf.end()) {
                obj_label_buf = obj_label_ringbuf.back();
            }
        }

    } else {
        obj_label_buf = obj_label_ringbuf.front();
        boost::circular_buffer<autoware_msgs::Centroids>::iterator it = cluster_centroids_ringbuf.begin();
        if (cluster_centroids_ringbuf.size() == 1) {
            cluster_centroids_buf = *it;
            pthread_mutex_unlock(&mutex);
            pthread_mutex_lock(&mutex);
            if (obj_pose_flag == true) {
                publish();
            }
            pthread_mutex_unlock(&mutex);
            return;
        }

        for (it++; it != cluster_centroids_ringbuf.end(); it++) {
            if (fabs_time_diff(&(obj_label_ringbuf.front().header), &((it-1)->header))
                < fabs_time_diff(&(obj_label_ringbuf.front().header), &(it->header))) {
                cluster_centroids_buf = *(it-1);
                break;
            }
        }

        if (it == cluster_centroids_ringbuf.end()) {
            cluster_centroids_buf = cluster_centroids_ringbuf.back();
        }
    }
    pthread_mutex_unlock(&mutex);
}

void cluster_centroids_callback(const autoware_msgs::Centroids::ConstPtr& cluster_centroids_msg) {
    pthread_mutex_lock(&mutex);
    cluster_centroids_ringbuf.push_front(*cluster_centroids_msg);
    //obj_label is empty
    if (obj_label_ringbuf.begin() == obj_label_ringbuf.end()) {
        ROS_INFO("obj_label ring buffer is empty");
        pthread_mutex_unlock(&mutex);
        return;
    }

    buf_flag = true;

    // obj_label > cluster_centroids
    if (get_time(&(obj_label_ringbuf.front().header)) >= get_time(&(cluster_centroids_ringbuf.front().header))) {
        cluster_centroids_buf = cluster_centroids_ringbuf.front();
        boost::circular_buffer<cv_tracker::obj_label>::iterator it = obj_label_ringbuf.begin();
        if (obj_label_ringbuf.size() == 1) {
            obj_label_buf = *it;
            pthread_mutex_unlock(&mutex);
            pthread_mutex_lock(&mutex);
            if (obj_pose_flag == true) {
                publish();
            }
            pthread_mutex_unlock(&mutex);
            return;
        } else {
            for (it++; it != obj_label_ringbuf.end(); it++) {
                if (fabs_time_diff(&(cluster_centroids_ringbuf.front().header), &((it-1)->header))
                    < fabs_time_diff(&(cluster_centroids_ringbuf.front().header), &(it->header))) {
                    obj_label_buf = *(it-1);
                    break;
                }
            }
            if (it == obj_label_ringbuf.end()) {
                obj_label_buf = obj_label_ringbuf.back();
            }
        }

    } else {
        obj_label_buf = obj_label_ringbuf.front();
        boost::circular_buffer<autoware_msgs::Centroids>::iterator it = cluster_centroids_ringbuf.begin();
        if (cluster_centroids_ringbuf.size() == 1) {
            cluster_centroids_buf = *it;
            pthread_mutex_unlock(&mutex);
            pthread_mutex_lock(&mutex);
            if (obj_pose_flag == true) {
                publish();
            }
            pthread_mutex_unlock(&mutex);
            return;
        }

        for (it++; it != cluster_centroids_ringbuf.end(); it++) {
            if (fabs_time_diff(&(obj_label_ringbuf.front().header), &((it-1)->header))
                < fabs_time_diff(&(obj_label_ringbuf.front().header), &(it->header))) {
                cluster_centroids_buf = *(it-1);
                break;
            }
        }

        if (it == cluster_centroids_ringbuf.end()) {
            cluster_centroids_buf = cluster_centroids_ringbuf.back();
        }
    }
    pthread_mutex_unlock(&mutex);
}

#endif

void obj_pose_callback(const visualization_msgs::MarkerArray::ConstPtr& obj_pose_msg) {
    pthread_mutex_lock(&mutex);
    obj_pose_flag = true;
    ROS_INFO("catch publish request");
    if (publish() == false) {
        ROS_INFO("waitting...");
    }
    pthread_mutex_unlock(&mutex);}

void* thread(void* args)
{
    ros::NodeHandle nh_rcv;
    ros::CallbackQueue rcv_callbackqueue;
    nh_rcv.setCallbackQueue(&rcv_callbackqueue);
    ros::Subscriber obj_pose_sub = nh_rcv.subscribe("obj_pose", 5, obj_pose_callback);
    while (nh_rcv.ok()) {
        rcv_callbackqueue.callAvailable(ros::WallDuration(1.0f));
        pthread_mutex_lock(&mutex);
        bool flag = (obj_pose_flag == false && buf_flag == true);
        if (flag) {
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
        pthread_mutex_unlock(&mutex);
    }
    return NULL;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sync_car_obj_fusion");
    ros::NodeHandle nh;

    /* create server thread */
    pthread_t th;
    pthread_create(&th, NULL, thread, (void *)NULL );

    ros::Subscriber obj_label_sub = nh.subscribe("/obj_car/obj_label", 1, obj_label_callback);
    ros::Subscriber cluster_centroids_sub = nh.subscribe("/cluster_centroids", 1, cluster_centroids_callback);
    obj_label__pub = nh.advertise<cv_tracker::obj_label>("/obj_car/obj_label_", 5);
    cluster_centroids__pub = nh.advertise<autoware_msgs::Centroids>("/cluster_centroids_", 5);
    while (!buf_flag) {
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
    pthread_mutex_lock(&mutex);

    ros::spin();

    /* shutdown server thread */
    ROS_INFO("wait until shutdown a thread");
    pthread_kill(th, SIGINT);
    pthread_join(th, NULL);

    return 0;
}
