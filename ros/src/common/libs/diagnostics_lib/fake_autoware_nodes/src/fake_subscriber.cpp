#include <fake_autoware_nodes/fake_subscriber.h>

FakeSubscriber::FakeSubscriber()
{
    diag_sub_ = nh_.subscribe("watchdog_node/diag/all", 1, &FakeSubscriber::diag_callback_, this);
    fake_sub_ = nh_.subscribe(ros::this_node::getName()+"/data", 1, &FakeSubscriber::callback_, this);
}

FakeSubscriber::~FakeSubscriber()
{

}

void FakeSubscriber::callback_(const std_msgs::Float64ConstPtr msg)
{
    diag_manager_.DIAG_ASSERT_VALUE_MIN(0.0, msg->data, 0);
    return;
}

void FakeSubscriber::diag_callback_(const diag_msgs::diagConstPtr msg)
{
    boost::optional<diag_msgs::diag_node_errors> error = diag_filter_.filter(*msg, ros::this_node::getName());
    if(error)
    {
        ROS_ERROR_STREAM(error.get());
    }
    return;
}