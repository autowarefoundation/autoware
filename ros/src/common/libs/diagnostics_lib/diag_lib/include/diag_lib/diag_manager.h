#ifndef DIAG_MANAGER_H_INCLUDED
#define DIAG_MANAGER_H_INCLUDED

//headers in diag_lib
#include <diag_lib/diag_macros.h>
#include <diag_lib/diag_manager.h>
#include <diag_lib/diag_info.h>
#include <diag_lib/rate_checker.h>
#include <diag_lib/error_category.h>
#include <diag_lib/diag_manager_config.h>

//headers in ROS
#include <ros/ros.h>

//headers in STL
#include <vector>
#include <map>
#include <thread>
#include<fstream>

//headers in diag_msgs
#include <diag_msgs/diag_error.h>

//headers in boost
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

// headers in YAML-CPP
#include <yaml-cpp/yaml.h>

class diag_manager
{
public:
    diag_manager();
    ~diag_manager();
    template<typename T>
    void DIAG_ASSERT_VALUE_RANGE(T min, T max, T value, int num)
    {
        if(enable_diag_ == false)
            return;
        std::vector<int> required_error_code = {INVALID_INITIAL_VALUE, INVALID_VALUE};
        if(check_error_code(num, required_error_code))
        {
            if(value < min)
            {
                ADD_DIAG_LOG_WARN(query_diag_info(num)->description);
                publish_diag_(query_diag_info(num).get());
                return;
            }
            if(value > max)
            {
                ADD_DIAG_LOG_WARN(query_diag_info(num)->description);
                publish_diag_(query_diag_info(num).get());
                return;
            }
        }
        return;
    }
    template<typename T>
    void DIAG_ASSERT_VALUE_MIN(T min, T value, int num)
    {
        if(enable_diag_ == false)
            return;
        std::vector<int> required_error_code = {INVALID_INITIAL_VALUE, INVALID_VALUE};
        if(check_error_code(num, required_error_code))
        {
            if(value < min)
            {
                ADD_DIAG_LOG_WARN(query_diag_info(num)->description);
                publish_diag_(query_diag_info(num).get());
                return;
            }
        }
        return;
    }
    template<typename T>
    void DIAG_ASSERT_VALUE_MAX(T max, T value, int num)
    {
        if(enable_diag_ == false)
            return;
        std::vector<int> required_error_code = {INVALID_INITIAL_VALUE, INVALID_VALUE};
        if(check_error_code(num, required_error_code))
        {
            if(value > max)
            {
                ADD_DIAG_LOG_WARN(query_diag_info(num)->description);
                publish_diag_(query_diag_info(num).get());
                return;
            }
        }
        return;
    }
    template<class T>
    void DIAG_ASSERT_EXCEPTION(T exception,int num)
    {
        if(enable_diag_ == false)
            return;
        std::vector<int> required_error_code = {EXCEPTION};
        if(check_error_code(num, required_error_code))
        {
            ADD_DIAG_LOG_WARN(query_diag_info(num)->description);
            ADD_DIAG_LOG_WARN(exception.what());
            publish_diag_(query_diag_info(num).get());
        }
        return;
    }
    void DIAG_RESOURCE(std::string target_resource_path, int num);
    void DIAG_RATE_CHECK(int num);
    void DIAG_LOW_RELIABILITY(int num);
    std::vector<diag_info> get_diag_info(){return diag_info_;}
    boost::optional<diag_info> query_diag_info(int num);
    void WRITE_LOG();
private:
    void ADD_DIAG_LOG_WARN(std::string log_text);
    void ADD_DIAG_LOG_ERROR(std::string log_text);
    void check_rate_();
    bool check_error_code(int requested_error_code, std::vector<int> right_categories);
    void publish_diag_(diag_info info);
    // check resource for diag_manager
    bool diag_resource(std::string target_resource_path);
    volatile bool enable_diag_;
    std::vector<diag_info> diag_info_;
    std::vector<std::string> diag_log_;
    ros::Publisher diag_pub_;
    ros::NodeHandle nh_;
    ros::Time timer_;
    //config
    std::string error_code_config_path_;
    YAML::Node error_code_config_;
    //rate checker
    std::map<int,boost::shared_ptr<rate_checker> > checkers_;
};
#endif //DIAG_MANAGER_H_INCLUDED