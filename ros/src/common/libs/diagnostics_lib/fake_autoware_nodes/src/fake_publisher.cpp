#include <fake_autoware_nodes/fake_publisher.h>

#include <std_msgs/Float64.h>

fake_publisher::fake_publisher()
{
    fake_pub_ = nh_.advertise<std_msgs::Float64>(ros::this_node::getName()+"/data", 1);
}

fake_publisher::~fake_publisher()
{

}

void fake_publisher::run()
{
    ros::Rate rate(10);
    std_msgs::Float64 msg;
    msg.data = -50;
    while(ros::ok())
    {
        diag_manager_.DIAG_RATE_CHECK(3);
        fake_pub_.publish(msg);
        msg.data = msg.data + 1.0;
        diag_manager_.DIAG_ASSERT_VALUE_RANGE(-20.0, 20.0, msg.data, 0);
        if(msg.data == 50)
        {
            diag_manager_.DIAG_LOW_RELIABILITY(4);
            msg.data = -50;
        }
        try
        {
            divide(1,msg.data);
        }
        catch(std::range_error& exception)
        {
            diag_manager_.DIAG_ASSERT_EXCEPTION(exception,1);
        }
        rate.sleep();
    }
    diag_manager_.WRITE_LOG();
    return;
}

double fake_publisher::divide(double a, double b)
{
    if (b == 0)
    {
        throw std::range_error("Divided by zero.");
    }
    return a / b;
}
