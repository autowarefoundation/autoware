#include <cstdio>
#include <vector>
#include <list>
#include <boost/lambda/lambda.hpp>
#include "ros/ros.h"
#include "ros/header.h"
#include "ros/console.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "jsk_topic_tools/List.h"
#include "jsk_topic_tools/Update.h"
#include "topic_tools/shape_shifter.h"
#include "topic_tools/parse.h"

using std::string;
using std::vector;
using std::list;
using namespace topic_tools;
static bool debug;

class pub_info_t
{
public:
    std::string topic_name;
    ros::Publisher pub;
    ros::Subscriber *sub;
    bool advertised;
    boost::shared_ptr<ShapeShifter > msg;
    ros::Timer timer;
    ros::Duration rate;
    ros::Time last_time_received;
    bool topic_with_header;
    bool latched;
    uint32_t last_seq_received;
    uint32_t topic_received;

    void publish(const ros::TimerEvent &event)
    {
        if ( advertised == false ) return;

        topic_received++;
        if ( topic_with_header == true ) {
            std_msgs::Header header;
            uint8_t buf[msg->size()];
            ros::serialization::OStream ostream(buf, msg->size());
            ros::serialization::IStream istream(buf, msg->size());
            msg->write(ostream);
            ((uint32_t *)buf)[0] = last_seq_received + topic_received;
            ros::Time tmp(last_time_received.toSec() + topic_received * rate.toSec());
            ((uint32_t *)buf)[1] = tmp.sec;
            ((uint32_t *)buf)[2] = tmp.nsec;
            msg->read(istream);
        }
	if(debug){
	  ROS_INFO_STREAM("publishing " << topic_name);
	}
        pub.publish(msg);
    }
};

typedef boost::shared_ptr<pub_info_t> pub_info_ref;

static list<pub_info_ref> g_pubs;

static ros::NodeHandle *g_node = NULL;

static bool use_fixed_rate;
static bool use_periodic_rate = false;



void in_cb(const boost::shared_ptr<ShapeShifter const>& msg,
           boost::shared_ptr<pub_info_t> s)
{
    using namespace boost::lambda;

    s->msg = boost::const_pointer_cast<ShapeShifter>(msg);
    s->topic_received = 0; // reset topic_received
    if ( s->advertised == false ) {
        s->pub = msg->advertise(*g_node, s->topic_name+string("_buffered"), 10, s->latched);
        s->advertised = true;
    }
    if(debug){
      ROS_INFO_STREAM("advertised as " << s->topic_name+string("_buffered") << " running at " << 1/(s->rate.toSec()) << "Hz");
    }

    // check if msg has header
    {
        std_msgs::Header header;
        uint8_t buf[msg->size()];
        ros::serialization::OStream stream(buf, msg->size());
        msg->write(stream);
        header.seq = ((uint32_t *)buf)[0];
        header.stamp.sec = ((uint32_t *)buf)[1];
        header.stamp.nsec = ((uint32_t *)buf)[2];

        if ( abs((header.stamp - ros::Time::now()).toSec()) < 5.0 ) {
	  if(debug){
	    ROS_INFO_STREAM(" this message contains headers.. seq =" <<  header.seq << " stamp = " << header.stamp);
	  }
            s->topic_with_header = true;
            s->last_seq_received = header.seq;
            s->last_time_received = header.stamp;
        }
    }
    //g_node->createTimer(ros::Duration(0.1), [](const ros::TimerEvent event) { std::cerr << "hoge" << std::endl; });
    {// at first publish once
      ros::TimerEvent ev;
      s->publish (ev);
    }
    s->timer = g_node->createTimer(s->rate, &pub_info_t::publish, s);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "topic_buffer_client", ros::init_options::AnonymousName);

    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    double fixed_rate = 0.1; // 10Hz
    if (nh.hasParam("fixed_rate")) {
      use_fixed_rate = true;
      nh.param ("fixed_rate", fixed_rate, 0.1);
      ROS_INFO("use fixed rate = %f", fixed_rate);
    }

    double update_rate = 10; // 0.1Hz
    if (nh.hasParam("update_rate")) {
      nh.param ("update_rate", update_rate, 10.0);
      ROS_INFO("use update rate = %f", update_rate);
    }

    double periodic_rate = 0.1; // 10Hz
    if (nh.hasParam("periodic_rate")) {
      use_periodic_rate = true;
      nh.param ("periodic_rate", periodic_rate, 0.1);
      ROS_INFO("use periodic rate = %f", periodic_rate);
    }

    bool latched;
    if (nh.hasParam("latched")) {
      nh.param ("latched", latched, false);
      if(latched) {
        ROS_INFO("use latched");
      }
    }

    g_node = &n;
    bool use_service_p = false;
    std::vector<std::string> target_topics;
    // use service or parameter
    // check the parameter first
    XmlRpc::XmlRpcValue topics;
    if (!nh.getParam ("topics", topics)) {
      ROS_WARN("no ~topics is available, use service interface");
      use_service_p = true;
    }
    else {
      switch (topics.getType ()) {
      case XmlRpc::XmlRpcValue::TypeArray: {
        for (int d = 0; d < topics.size(); ++d) {
          target_topics.push_back((std::string)(topics[d]));
        }
        break;
      }
      default: {
        ROS_WARN("~topics mismatches type, it should be a list, use service interface");
        use_service_p = true;
      }
      }
    }
    
    if (use_service_p) {
      target_topics.clear();
      // New service
      ros::service::waitForService(string("/list"), -1);
      ros::ServiceClient sc_list = n.serviceClient<jsk_topic_tools::List>(string("/list"), true);
      jsk_topic_tools::List::Request req;
      jsk_topic_tools::List::Response res;
      ROS_INFO_STREAM("calling "  << sc_list.getService());
      while ( sc_list.call(req, res) == false) {
        ROS_WARN_STREAM("calling " << sc_list.getService() << " fails, retry...");
        ros::Duration(1).sleep();
      }
      ROS_WARN_STREAM("calling /list success!!!");
      for(vector<std::string>::iterator it = res.topic_names.begin(); it != res.topic_names.end(); ++it) {
        target_topics.push_back(*it);
      }
      ROS_INFO_STREAM("calling /list has done.. found " << res.topic_names.size() << " topics to publish");
    }
    for(size_t i = 0; i < target_topics.size(); i++) {
        boost::shared_ptr<pub_info_t> pub_info(new pub_info_t);
        pub_info->topic_name = target_topics[i];
        if (use_fixed_rate) {
          pub_info->rate = ros::Duration(fixed_rate);
        }
        pub_info->latched = latched;
        pub_info->advertised = false;
        pub_info->topic_with_header = false;
        ROS_INFO_STREAM("subscribe " << pub_info->topic_name+string("_update"));
        pub_info->sub = new ros::Subscriber(n.subscribe<ShapeShifter>(pub_info->topic_name+string("_update"), 10, boost::bind(in_cb, _1, pub_info)));

        g_pubs.push_back(pub_info);
    }

    ros::Rate rate_loop(100);
    ros::Time last_updated;

    bool use_service = true;
    nh.param("use_service", use_service, true);

    nh.param("debug", debug, false);
    
    ros::ServiceClient sc_update = n.serviceClient<jsk_topic_tools::Update>(string("/update"), true);
    ros::Publisher pub_update;
    if (!use_service) {
        pub_update = n.advertise<std_msgs::String>("/update", 1);
    }

    if (use_periodic_rate) {
      for (list<pub_info_ref>::iterator it = g_pubs.begin();
           it != g_pubs.end();
           ++it) {
        jsk_topic_tools::Update::Request req;
        jsk_topic_tools::Update::Response res;
        req.topic_name = (*it)->topic_name;
        req.periodic = true;
        req.periodic_rate = ros::Duration(periodic_rate);
        sc_update.call(req, res);
        ROS_INFO_STREAM("sending request for periodic rate publish  topic:" << req.topic_name << " rate:" << req.periodic_rate);
      }
    }
    while ( ros::ok() ) {

        if ( update_rate >= 0 && (ros::Time::now() - last_updated) > ros::Duration(update_rate) ) {
            for (list<pub_info_ref>::iterator it = g_pubs.begin();
                 it != g_pubs.end();
                 ++it) {
                
                if (use_service) {
                  jsk_topic_tools::Update::Request req;
                  jsk_topic_tools::Update::Response res;
                  req.topic_name = (*it)->topic_name;
                  if ( sc_update.call(req, res) == false ) {
                    ROS_ERROR_STREAM("calling /update (" << req.topic_name << ") fails, retry...");
                    sc_update = n.serviceClient<jsk_topic_tools::Update>(string("/update"), true);
                    continue;
                  }
                  (*it)->rate = ros::Duration(res.rate);
                  if(debug){
		    ROS_INFO_STREAM("calling /update " << req.topic_name << " .. " << res.rate);
		  }
                }
                else {
                  std_msgs::String msg;
                  msg.data = (*it)->topic_name;
                  pub_update.publish(msg);
		  if(debug){
		    ROS_INFO_STREAM("publishing /update " << msg.data);
		  }
                }
            }
            last_updated = ros::Time::now();
        }

        ros::spinOnce();
        rate_loop.sleep();
    }

    return 0;
}
