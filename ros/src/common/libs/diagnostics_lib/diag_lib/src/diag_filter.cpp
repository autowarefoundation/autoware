// headers in diag_lib
#include <diag_lib/diag_filter.h>

// headers in YAML-CPP
#include <yaml-cpp/yaml.h>

//headers in boost
#include <boost/filesystem.hpp>

diag_filter::diag_filter()
{
    nh_.param<std::string>("/error_code_config_path", error_code_config_path_, std::string(""));
    if(check_resource_(error_code_config_path_))
    {
        enable_ = true;
    }
    else
    {
        enable_ = false;
    }
}

diag_filter::~diag_filter()
{

}

bool diag_filter::check_resource_(std::string target_resource_path)
{
    namespace fs = boost::filesystem;
    fs::path path(target_resource_path);
    boost::system::error_code error;
    const bool result = fs::exists(path, error);
    if (!result || error)
        return false;
    return true;
}

boost::optional<diag_msgs::diag_node_errors> diag_filter::filter(diag_msgs::diag diag, std::string target_node)
{
    boost::optional<diag_msgs::diag_node_errors> ret;
    return ret;
}

boost::optional<diag_msgs::diag_node_errors> diag_filter::filter(diag_msgs::diag diag, int target_node_number)
{
    for(int i=0; i<diag.nodes.size() ; i++)
    {
        if(diag.nodes[i].node_number == target_node_number)
            return diag.nodes[i];
    }
    return boost::none;
}