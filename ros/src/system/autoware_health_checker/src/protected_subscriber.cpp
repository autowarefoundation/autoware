#include <autoware_health_checker/protected_subscriber.h>

namespace ros
{
    template <typename T>
    ProtectedSubscriber::ProtectedSubscriber(std::vector<std::string> publisher_nodes,std::string topic_name,int buffer_length,double warn_rate,double error_rate,double fatal_rate,
        boost::shared_ptr<autoware_health_checker::NodeStatusPublisher> node_status_publisher_ptr, boost::function<void(T)> callback) : topic_name(topic_name)
    {
        is_advertised_ = false;
        buffer_length_ = buffer_length;
        node_status_publisher_ptr_ = node_status_publisher_ptr;
        warn_rate_ = warn_rate;
        error_rate_ = error_rate;
        fatal_rate_ = fatal_rate;
        publisher_nodes_ = publisher_nodes;
        //callback_ = callback;
        message_sub_ = nh_.subscribe(topic_name, buffer_length_, &ProtectedSubscriber::topicCallback, this);
    }

    ProtectedSubscriber::~ProtectedSubscriber()
    {

    }

    void ProtectedSubscriber::topicCallback(const ros::MessageEvent<topic_tools::ShapeShifter>& msg_event)
    {
        std::string publisher_name = msg_event.getPublisherName();
        try
        {
            node_status_publisher_ptr_->CHECK_SUBSCRIBED_TOPICS("invalid_topic:"+topic_name,publisher_name,publisher_nodes_,"publisher of topic " + topic_name + " is invalid.");
            node_status_publisher_ptr_->CHECK_RATE("slow_publish_rate:"+topic_name,warn_rate_,error_rate_,fatal_rate_,topic_name + " topic subscribe rate is low");
        }
        catch(...)
        {
            ROS_ERROR_STREAM("failed to initialize node_status_publisher in checking " << topic_name << " topic.");
            return;
        }
    }
}