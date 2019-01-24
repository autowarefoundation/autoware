#include <autoware_health_checker/protected_publisher.h>

namespace ros
{
    ProtectedPublisher::ProtectedPublisher(std::string topic_name,int buffer_length,double warn_rate,double error_rate,double fatal_rate,
        boost::shared_ptr<autoware_health_checker::NodeStatusPublisher> node_status_publisher_ptr) : topic_name(topic_name)
    {
        is_advertised_ = false;
        buffer_length_ = buffer_length;
        node_status_publisher_ptr_ = node_status_publisher_ptr;
        warn_rate_ = warn_rate;
        error_rate_ = error_rate;
        fatal_rate_ = fatal_rate;
    }

    ProtectedPublisher::~ProtectedPublisher()
    {

    }

    template<typename T>
    void ProtectedPublisher::publish(T message)
    {
        if(!is_advertised_)
        {
            is_advertised_ = true;
            shape_shifter_.morph(ros::message_traits::MD5Sum<T>::value(),ros::message_traits::DataType<T>::value(),ros::message_traits::Definition<T>::value(),"" );
            msg_pub_ = shape_shifter_.advertise(nh_,topic_name,buffer_length_);
        }
        try
        {
            node_status_publisher_ptr_->CHECK_RATE("slow_publish_rate:"+topic_name,warn_rate_,error_rate_,fatal_rate_,topic_name + " topic publish rate is low");
        }
        catch(...)
        {
            ROS_ERROR_STREAM("failed to initialize node_status_publisher in checking " << topic_name << " topic.");
            return;
        }
        return;
    }

    template <typename T>
    void ProtectedPublisher::serializeToByteArray(const T& msg, std::vector<uint8_t>& destination_buffer)
    {
        const uint32_t length = ros::serialization::serializationLength(msg);
        destination_buffer.resize( length );
        //copy into your own buffer
        ros::serialization::OStream stream(destination_buffer.data(), length);
        ros::serialization::serialize(stream, msg);
    }
}