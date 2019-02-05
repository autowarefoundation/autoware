#ifndef WATCHDOG_H_INCLUDED
#define WATCHDOG_H_INCLUDED

//headers in diag_lib
#include <diag_lib/diag_info.h>
#include <diag_lib/diag_manager.h>
#include <diag_lib/diag_subscriber.h>

//headers in diag_msgs
#include <diag_msgs/diag.h>
#include <diag_msgs/diag_error.h>
#include <diag_msgs/diag_node_errors.h>

//headers in STL
#include <vector>
#include <map>

//headers in YAML-CPP
#include <yaml-cpp/yaml.h>

//headers in Boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>

class watchdog
{
public:
    watchdog();
    ~watchdog();
    void run();
private:
    ros::NodeHandle nh_;
    // parameters
    std::string config_filepath_;
    double publish_rate_;
    // diagnostic manager
    diag_manager diag_;
    std::map<std::string,boost::shared_ptr<std::vector<diag_info> > > watchdog_diag_info_;
    std::vector<std::string> diag_target_nodes_;
    std::vector<std::string> watchdog_target_nodes_;
    std::map<std::string,bool> connection_status_;
    std::map<std::string,boost::shared_ptr<diag_subscriber> > diag_sub_;
    ros::Publisher diag_pub_;
    void update_connection_status_();
    void publish_diag_();
    void write_error_code_csv_(YAML::Node config);
};
#endif  //WATCHDOG_H_INCLUDED