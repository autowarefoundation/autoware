//headers in ROS
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <ros/message_traits.h>
#include <std_msgs/String.h>

//headers in Autoware
#include <autoware_health_checker/rate_checker.h>
#include <autoware_health_checker/node_status_publisher.h>

//headers in boost
#include <boost/shared_ptr.hpp>

//headers in STL
#include <functional>

namespace ros
{
    class ProtectedSubscriber
    {
    public:
        template <typename T>
        ProtectedSubscriber(std::vector<std::string> publisher_nodes,std::string topic_name,int buffer_length,double warn_rate,double error_rate,double fatal_rate,
            boost::shared_ptr<autoware_health_checker::NodeStatusPublisher> node_status_publisher_ptr,std::function<void(T)> callback);
        ~ProtectedSubscriber();
        const std::string topic_name;
    private:
        int buffer_length_;
        volatile bool is_advertised_;
        ros::NodeHandle nh_;
        ros::Subscriber message_sub_;
        topic_tools::ShapeShifter shape_shifter_;
        boost::shared_ptr<autoware_health_checker::NodeStatusPublisher> node_status_publisher_ptr_;
        double warn_rate_;
        double error_rate_;
        double fatal_rate_;
        void topicCallback(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_event);
        std::vector<std::string> publisher_nodes_;
    };
}