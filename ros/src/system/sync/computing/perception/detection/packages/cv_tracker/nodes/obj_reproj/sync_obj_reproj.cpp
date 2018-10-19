#include "ros/ros.h"
#include "autoware_msgs/ImageObjTracked.h"
#include "geometry_msgs/PoseStamped.h"
#include "autoware_msgs/ObjLabel.h"
#include "sync.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "sync_reprojection");
    std::string ns(ros::this_node::getNamespace());
    std::string sub1("/image_obj_tracked");
    std::string sub2("/current_pose");
    std::string req("/obj_label");
    std::string pub1("/image_obj_tracked");
    std::string pub2("/current_pose");

    Synchronizer<autoware_msgs::ImageObjTracked, geometry_msgs::PoseStamped, autoware_msgs::ObjLabel> synchronizer(sub1, sub2, pub1, pub2, req, ns);
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
#include "autoware_msgs/ImageObjTracked.h"
#include "geometry_msgs/PoseStamped.h"
#include "autoware_msgs/ObjLabel.h"

/* ----mode---- */
#define _REQ_PUB 1

/* ----var---- */
/* common var */
bool buf_flag;
pthread_mutex_t mutex;
/* user var */
boost::circular_buffer<autoware_msgs::ImageObjTracked> image_obj_tracked_ringbuf(10);
boost::circular_buffer<geometry_msgs::PoseStamped> current_pose_ringbuf(10);
ros::Publisher image_obj_tracked__pub;
ros::Publisher current_pose__pub;
bool obj_label_flag;

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
autoware_msgs::ImageObjTracked* p_image_obj_tracked_buf;
geometry_msgs::PoseStamped* p_current_pose_buf;

void publish_msg(autoware_msgs::ImageObjTracked* p_image_obj_tracked_buf, geometry_msgs::PoseStamped* p_current_pose_buf) {
    ROS_INFO("publish");
    image_obj_tracked__pub.publish(*p_image_obj_tracked_buf);
    current_pose__pub.publish(*p_current_pose_buf);
}

bool publish() {
    if (buf_flag) {
        //image_obj_tracked is empty
        if (image_obj_tracked_ringbuf.begin() == image_obj_tracked_ringbuf.end()) {
            ROS_INFO("image_obj_tracked ring buffer is empty");
            return false;
        }

        //current_pose is empty
        if (current_pose_ringbuf.begin() == current_pose_ringbuf.end()) {
            ROS_INFO("current_pose ring buffer is empty");
            return false;
        }

        // image_obj_tracked > current_pose
        if (get_time(&(image_obj_tracked_ringbuf.front().header)) >= get_time(&(current_pose_ringbuf.front().header))) {
            p_current_pose_buf = &(current_pose_ringbuf.front());
            boost::circular_buffer<autoware_msgs::ImageObjTracked>::iterator it = image_obj_tracked_ringbuf.begin();
            if (image_obj_tracked_ringbuf.size() == 1) {
                p_image_obj_tracked_buf = &*it;
                publish_msg(p_image_obj_tracked_buf, p_current_pose_buf);
                if (obj_label_flag == true){
                    buf_flag = false;
                    obj_label_flag = false;
                    image_obj_tracked_ringbuf.clear();
                    current_pose_ringbuf.clear();
                }
                return true;
            } else {
                for (it++; it != image_obj_tracked_ringbuf.end(); it++) {
                    if (fabs_time_diff(&(current_pose_ringbuf.front().header), &((it-1)->header))
                        < fabs_time_diff(&(current_pose_ringbuf.front().header), &(it->header))) {
                        p_image_obj_tracked_buf = &*(it-1);
                        break;
                    }
                }
                if (it == image_obj_tracked_ringbuf.end()) {
                    p_image_obj_tracked_buf = &(image_obj_tracked_ringbuf.back());
                }
            }
        }
        // image_obj_tracked < current_pose
        else {
            p_image_obj_tracked_buf = &(image_obj_tracked_ringbuf.front());
            boost::circular_buffer<geometry_msgs::PoseStamped>::iterator it = current_pose_ringbuf.begin();
            if (current_pose_ringbuf.size() == 1) {
                p_current_pose_buf = &*it;
                publish_msg(p_image_obj_tracked_buf, p_current_pose_buf);
                if (obj_label_flag == true){
                    buf_flag = false;
                    obj_label_flag = false;
                    image_obj_tracked_ringbuf.clear();
                    current_pose_ringbuf.clear();
                }
                return true;
            }

            for (it++; it != current_pose_ringbuf.end(); it++) {
                if (fabs_time_diff(&(image_obj_tracked_ringbuf.front().header), &((it-1)->header))
                    < fabs_time_diff(&(image_obj_tracked_ringbuf.front().header), &(it->header))) {
                    p_current_pose_buf = &*(it-1);
                    break;
                }
            }

            if (it == current_pose_ringbuf.end()) {
                p_current_pose_buf = &(current_pose_ringbuf.back());
            }
        }
        publish_msg(p_image_obj_tracked_buf, p_current_pose_buf);
        if (obj_label_flag == true){
            buf_flag = false;
            obj_label_flag = false;
            image_obj_tracked_ringbuf.clear();
            current_pose_ringbuf.clear();
        }

        return true;
    } else {
        return false;
    }
}

void image_obj_tracked_callback(const autoware_msgs::ImageObjTracked::ConstPtr& image_obj_tracked_msg) {
    pthread_mutex_lock(&mutex);
    image_obj_tracked_ringbuf.push_front(*image_obj_tracked_msg);
    //current_pose is empty
    if (current_pose_ringbuf.begin() == current_pose_ringbuf.end()) {
        buf_flag = false;
        pthread_mutex_unlock(&mutex);
        ROS_INFO("current_pose ring buffer is empty");
        return;
    }
    buf_flag = true;
    pthread_mutex_unlock(&mutex);
    pthread_mutex_lock(&mutex);
    if (obj_label_flag == true) {
        publish();
    }
    pthread_mutex_unlock(&mutex);
}

void current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& current_pose_msg) {
    pthread_mutex_lock(&mutex);
    current_pose_ringbuf.push_front(*current_pose_msg);
    //image_obj_tracked is empty
    if (image_obj_tracked_ringbuf.begin() == image_obj_tracked_ringbuf.end()) {
        ROS_INFO("image_obj_tracked ring buffer is empty");
        buf_flag = false;
        pthread_mutex_unlock(&mutex);
        return;
    }
    buf_flag = true;
    pthread_mutex_unlock(&mutex);
    pthread_mutex_lock(&mutex);
    if (obj_label_flag == true) {
        publish();
    }
    pthread_mutex_unlock(&mutex);
}

#else
#endif

void obj_label_callback(const autoware_msgs::ObjLabel::ConstPtr& obj_label_msg) {
    pthread_mutex_lock(&mutex);
    obj_label_flag = true;
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
    ros::Subscriber obj_label_sub = nh_rcv.subscribe("/obj_label", 5, obj_label_callback);
    while (nh_rcv.ok()) {
        rcv_callbackqueue.callAvailable(ros::WallDuration(1.0f));
        pthread_mutex_lock(&mutex);
        bool flag = (obj_label_flag == false && buf_flag == true);
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
    obj_label_flag = false;
    ros::init(argc, argv, "sync_reprojection");
    ros::NodeHandle nh;

    /* create server thread */
    pthread_t th;
    pthread_create(&th, NULL, thread, (void *)NULL );

    ros::Subscriber image_obj_tracked_sub = nh.subscribe("/image_obj_tracked", 1, image_obj_tracked_callback);
    ros::Subscriber current_pose_sub = nh.subscribe("/current_pose", 1, current_pose_callback);
    image_obj_tracked__pub = nh.advertise<autoware_msgs::ImageObjTracked>("/sync_reprojection/image_obj_tracked", 5);
    current_pose__pub = nh.advertise<geometry_msgs::PoseStamped>("/sync_reprojection/current_pose", 5);

    while (!buf_flag && ros::ok()) {
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

    /* shutdown server thread */
    ROS_INFO("wait until shutdown a thread");
    pthread_kill(th, SIGINT);
    pthread_join(th, NULL);

    return 0;
}

#if 0
autoware_msgs::ImageObjTracked image_obj_tracked_buf;
geometry_msgs::PoseStamped current_pose_buf;

void image_obj_tracked_callback(const autoware_msgs::ImageObjTracked::ConstPtr& image_obj_tracked_msg) {
    pthread_mutex_lock(&mutex);
    image_obj_tracked_ringbuf.push_front(*image_obj_tracked_msg);

    //current_pose is empty
    if (current_pose_ringbuf.begin() == current_pose_ringbuf.end()) {
        pthread_mutex_unlock(&mutex);
        ROS_INFO("current_pose ring buffer is empty");
        return;
    }

    buf_flag = true;

    // image_obj_tracked > current_pose
    if (get_time(&(image_obj_tracked_ringbuf.front().header)) >= get_time(&(current_pose_ringbuf.front().header))) {
        current_pose_buf = current_pose_ringbuf.front();
        boost::circular_buffer<autoware_msgs::ImageObjTracked>::iterator it = image_obj_tracked_ringbuf.begin();
        if (image_obj_tracked_ringbuf.size() == 1) {
            image_obj_tracked_buf = *it;
            pthread_mutex_unlock(&mutex);
            pthread_mutex_lock(&mutex);
            if (obj_label_flag == true) {
                publish();
            }
            pthread_mutex_unlock(&mutex);
            return;
        } else {
            for (it++; it != image_obj_tracked_ringbuf.end(); it++) {
                if (fabs_time_diff(&(current_pose_ringbuf.front().header), &((it-1)->header))
                    < fabs_time_diff(&(current_pose_ringbuf.front().header), &(it->header))) {
                    image_obj_tracked_buf = *(it-1);
                    break;
                }
            }
            if (it == image_obj_tracked_ringbuf.end()) {
                image_obj_tracked_buf = image_obj_tracked_ringbuf.back();
            }
        }

    } else {
        image_obj_tracked_buf = image_obj_tracked_ringbuf.front();
        boost::circular_buffer<geometry_msgs::PoseStamped>::iterator it = current_pose_ringbuf.begin();
        if (current_pose_ringbuf.size() == 1) {
            current_pose_buf = *it;
            pthread_mutex_unlock(&mutex);
            pthread_mutex_lock(&mutex);
            if (obj_label_flag == true) {
                publish();
            }
            pthread_mutex_unlock(&mutex);
            return;
        }

        for (it++; it != current_pose_ringbuf.end(); it++) {
            if (fabs_time_diff(&(image_obj_tracked_ringbuf.front().header), &((it-1)->header))
                < fabs_time_diff(&(image_obj_tracked_ringbuf.front().header), &(it->header))) {
                current_pose_buf = *(it-1);
                break;
            }
        }

        if (it == current_pose_ringbuf.end()) {
            current_pose_buf = current_pose_ringbuf.back();
        }
    }
    pthread_mutex_unlock(&mutex);
    pthread_mutex_lock(&mutex);
    if (obj_label_flag == true) {
        publish();
    }
    pthread_mutex_unlock(&mutex);
}

void current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& current_pose_msg) {
    pthread_mutex_lock(&mutex);
    current_pose_ringbuf.push_front(*current_pose_msg);
    //image_obj_tracked is empty
    if (image_obj_tracked_ringbuf.begin() == image_obj_tracked_ringbuf.end()) {
        ROS_INFO("image_obj_tracked ring buffer is empty");
        pthread_mutex_unlock(&mutex);
        return;
    }

    buf_flag = true;

    // image_obj_tracked > current_pose
    if (get_time(&(image_obj_tracked_ringbuf.front().header)) >= get_time(&(current_pose_ringbuf.front().header))) {
        current_pose_buf = current_pose_ringbuf.front();
        boost::circular_buffer<autoware_msgs::ImageObjTracked>::iterator it = image_obj_tracked_ringbuf.begin();
        if (image_obj_tracked_ringbuf.size() == 1) {
            image_obj_tracked_buf = *it;
            pthread_mutex_unlock(&mutex);
            pthread_mutex_lock(&mutex);
            if (obj_label_flag == true) {
                publish();
            }
            pthread_mutex_unlock(&mutex);
            return;
        } else {
            for (it++; it != image_obj_tracked_ringbuf.end(); it++) {
                if (fabs_time_diff(&(current_pose_ringbuf.front().header), &((it-1)->header))
                    < fabs_time_diff(&(current_pose_ringbuf.front().header), &(it->header))) {
                    image_obj_tracked_buf = *(it-1);
                    break;
                }
            }
            if (it == image_obj_tracked_ringbuf.end()) {
                image_obj_tracked_buf = image_obj_tracked_ringbuf.back();
            }
        }

    } else {
        image_obj_tracked_buf = image_obj_tracked_ringbuf.front();
        boost::circular_buffer<geometry_msgs::PoseStamped>::iterator it = current_pose_ringbuf.begin();
        if (current_pose_ringbuf.size() == 1) {
            current_pose_buf = *it;
            pthread_mutex_unlock(&mutex);
            pthread_mutex_lock(&mutex);
            if (obj_label_flag == true) {
                publish();
            }
            pthread_mutex_unlock(&mutex);
            return;
        }

        for (it++; it != current_pose_ringbuf.end(); it++) {
            if (fabs_time_diff(&(image_obj_tracked_ringbuf.front().header), &((it-1)->header))
                < fabs_time_diff(&(image_obj_tracked_ringbuf.front().header), &(it->header))) {
                current_pose_buf = *(it-1);
                break;
            }
        }

        if (it == current_pose_ringbuf.end()) {
            current_pose_buf = current_pose_ringbuf.back();
        }
    }
    pthread_mutex_unlock(&mutex);
    pthread_mutex_lock(&mutex);
    if (obj_label_flag == true) {
        publish();
    }
    pthread_mutex_unlock(&mutex);
}

bool publish() {
    if (buf_flag) {
        ROS_INFO("publish");
        image_obj_tracked__pub.publish(image_obj_tracked_buf);
        current_pose__pub.publish(current_pose_buf);
        if (obj_label_flag == true){
            buf_flag = false;
            obj_label_flag = false;
            image_obj_tracked_ringbuf.clear();
            current_pose_ringbuf.clear();
        }
        return true;
    } else {
        ROS_INFO("publish failed");
        return false;
    }
}
#endif
#endif
