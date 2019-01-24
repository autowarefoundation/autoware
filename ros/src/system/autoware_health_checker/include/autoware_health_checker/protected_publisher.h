//headers in ROS
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <ros/message_traits.h>

//headers in Autoware
#include <autoware_health_checker/rate_checker.h>
#include <autoware_health_checker/node_status_publisher.h>

//headers in boost
#include <boost/shared_ptr.hpp>

//headers in STL
#include <functional>

namespace ros
{
    class ProtectedPublisher
    {
    public:
        ProtectedPublisher(std::string topic_name,int buffer_length,double warn_rate,double error_rate,double fatal_rate,
            boost::shared_ptr<autoware_health_checker::NodeStatusPublisher> node_status_publisher_ptr);
        ~ProtectedPublisher();
        template<typename T>
        void publish(T message);
        const std::string topic_name;
    private:
        int buffer_length_;
        volatile bool is_advertised_;
        ros::NodeHandle nh_;
        ros::Publisher msg_pub_;
        topic_tools::ShapeShifter shape_shifter_;
        boost::shared_ptr<autoware_health_checker::NodeStatusPublisher> node_status_publisher_ptr_;
        double warn_rate_;
        double error_rate_;
        double fatal_rate_;
        template<typename T>
        void serializeToByteArray(const T& msg, std::vector<uint8_t>& destination_buffer);
    };
}