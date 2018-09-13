#include <fake_autoware_nodes/fake_subscriber.h>

fake_subscriber::fake_subscriber()
{
    fake_sub_ = nh_.subscribe(ros::this_node::getName()+"/data", 1, &fake_subscriber::callback_, this);
}

fake_subscriber::~fake_subscriber()
{

}

void fake_subscriber::callback_(const std_msgs::Float64ConstPtr msg)
{
    diag_manager_.DIAG_ASSERT_VALUE_MIN(0.0, msg->data, 0);
    return;
}