//headers in diag_lib
#include <diag_lib/watchdog.h>

//headers in ROS
#include <ros/ros.h>

watchdog::watchdog()
{
    nh_.param<std::string>("/error_code_config_path", config_filepath_, "");
    nh_.param<double>(ros::this_node::getName() + "/publish_rate", publish_rate_, 10);
    diag_.DIAG_RESOURCE(config_filepath_, 0);
    YAML::Node config = YAML::LoadFile(config_filepath_.c_str());
    for(YAML::const_iterator it=config.begin();it != config.end();++it)
    {
        if(it->first.as<std::string>() != ros::this_node::getName())
        {
            std::string itr_node_name = it->first.as<std::string>();
            diag_target_nodes_.push_back(itr_node_name);
            YAML::Node target_node = config[itr_node_name];
            diag_sub_[itr_node_name] = boost::make_shared<diag_subscriber>(itr_node_name, target_node["node_number"].as<int>());
            boost::shared_ptr<std::vector<diag_info> > target_node_watchdog_diag_info = boost::make_shared<std::vector<diag_info> >();
            for(const YAML::Node &error : target_node["errors"])
            {
                if(error["category"].as<int>() == NODE_IS_DEAD)
                {
                    diag_info info(error["num"].as<int>(), error["name"].as<std::string>(), error["category"].as<int>(), error["description"].as<std::string>());
                    target_node_watchdog_diag_info->push_back(info);
                }
            }
            if(target_node_watchdog_diag_info->size() != 0)
            {
                watchdog_diag_info_[itr_node_name] = target_node_watchdog_diag_info;
                watchdog_target_nodes_.push_back(itr_node_name);
            }
        }
    }
    write_error_code_csv_(config);
    diag_pub_ = nh_.advertise<diag_msgs::diag>(ros::this_node::getName()+"/diag/all", 1);
}

watchdog::~watchdog()
{
    
}

void watchdog::write_error_code_csv_(YAML::Node config)
{
    namespace fs = boost::filesystem;
    const fs::path path("/tmp/Autoware/Diag/");
    boost::system::error_code error;
    const bool result = fs::create_directories(path, error);
    std::string write_string = "node_name,node_number,num,name,category,description,threshold,level\n";
    for(YAML::const_iterator it=config.begin();it != config.end();++it)
    {
        std::string node_name = it->first.as<std::string>();
        YAML::Node target_node = config[node_name];
        int node_number = target_node["node_number"].as<int>();
        for(const YAML::Node &error : target_node["errors"])
        {
            std::string line = "";
            int num = error["num"].as<int>();
            std::string name = error["name"].as<std::string>();
            int category = error["category"].as<int>();
            std::string description = error["description"].as<std::string>();
            if((category == LOW_SUBSCRIBE_RATE) || (category == LOW_PUBLISH_RATE) || (category == LOW_OPERATION_CYCLE))
            {
                double threashold = error["threshold"].as<double>();
                std::string level = error["level"].as<std::string>();
                line = node_name + "," + std::to_string(node_number) + "," + std::to_string(num) + "," + name + "," + std::to_string(category) + "," + description + "," + std::to_string(threashold) + "," + level + "\n";
            }
            else
            {
                line = node_name + "," + std::to_string(node_number) + "," + std::to_string(num) + "," + name + "," + std::to_string(category) + "," + description + ",,\n";
            }
            write_string = write_string + line;
        }
    }
    std::ofstream outputfile(std::string("/tmp/Autoware/Diag/error_list.csv").c_str());
    outputfile << write_string;
    outputfile.close();
    return;
}

void watchdog::publish_diag_()
{
    ros::Rate rate(publish_rate_);
    while(ros::ok())
    {
        update_connection_status_();
        diag_msgs::diag diag_msg;
        for(auto itr = watchdog_target_nodes_.begin(); itr != watchdog_target_nodes_.end(); ++itr)
        {
            diag_msgs::diag_node_errors errors = diag_sub_[*itr]->get_diag_node_errors();
            if(connection_status_[*itr] == false)
            {
                std::vector<diag_info> target_node_watchdog_diag_info = *watchdog_diag_info_[*itr];
                for(int i=0; i<target_node_watchdog_diag_info.size(); i++)
                {
                    diag_msgs::diag_error error;
                    error.name = target_node_watchdog_diag_info[i].name;
                    error.num = target_node_watchdog_diag_info[i].num;
                    error.category = target_node_watchdog_diag_info[i].category;
                    error.description = target_node_watchdog_diag_info[i].description;
                    errors.errors.push_back(error);
                }
            }
            if(errors.errors.size() != 0)
            {
                diag_msg.nodes.push_back(errors);
            }
        }
        diag_msg.header.stamp = ros::Time::now();
        diag_pub_.publish(diag_msg);
        rate.sleep();
    }
    diag_.WRITE_LOG();
    return;
}

void watchdog::run()
{
    boost::thread publish_thread(boost::bind(&watchdog::publish_diag_, this));
    ros::spin();
    publish_thread.join();
    return;
}

void watchdog::update_connection_status_()
{
    std::vector<std::string> detected_nodes;
    ros::master::getNodes(detected_nodes);
    for(auto itr = watchdog_target_nodes_.begin(); itr != watchdog_target_nodes_.end(); ++itr)
    {
        if(std::find(detected_nodes.begin(), detected_nodes.end(), *itr) != detected_nodes.end())
        {
            connection_status_[*itr] = true;
        }
        else
        {
            connection_status_[*itr] = false;
        }
    }
    return;
}