#ifndef _SYNC_HEADER_
#define _SYNC_HEADER_
#include "ros/ros.h"
#include <boost/circular_buffer.hpp>
#include <ros/callback_queue.h>
#include <pthread.h>

template<typename T1, typename T2, typename T3>
class Synchronizer
{
public:
    Synchronizer(const std::string sub1_topic, const std::string sub2_topic, const std::string pub1_topic, const std::string pub2_topic, const std::string req_topic, const std::string ns);
    void run();

private:
    bool buf_flag_;
    pthread_mutex_t mutex_;
    /* user var */
    boost::circular_buffer<T1> type1_ringbuf_;
    boost::circular_buffer<T2> type2_ringbuf_;
    ros::Publisher type1_pub_;
    ros::Publisher type2_pub_;
    ros::Publisher sync_time_diff_pub_;
    bool is_req_;
    T1* p_type1_buf_;
    T2* p_type2_buf_;
    ros::Subscriber type1_sub_;
    ros::Subscriber type2_sub_;
#if 0
    ros::Subscriber req_sub_;
    ros::CallbackQueue rcv_callbackqueue_;
#endif
    std::string req_topic_;

    void publish_msg(T1* p_type1_buf_, T2* p_type2_buf_);
    bool publish();
    void type1_callback(const typename T1::ConstPtr& type1_msg);
    void type2_callback(const typename T2::ConstPtr& type2_msg);
    void req_callback(const typename T3::ConstPtr& req_msg);
    static void* launchThread(void *args) {
        reinterpret_cast<Synchronizer<T1,T2,T3>*>(args)->thread();
        pthread_exit(NULL);
    }
    void thread();
};

#include "impl/sync_impl.hpp"

#endif
