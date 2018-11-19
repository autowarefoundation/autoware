#include <diag_lib/diag_manager.h>

diag_manager::diag_manager()
{
    enable_diag_ = false;
    nh_.param<std::string>("/error_code_config_path", error_code_config_path_, std::string(""));
    if(!diag_resource(error_code_config_path_))
    {
        return;
    }
    YAML::Node config = YAML::LoadFile(error_code_config_path_.c_str());
    error_code_config_ = config[ros::this_node::getName()];
    try
    {
        for(const YAML::Node &error : error_code_config_["errors"])
        {
            diag_info info(error["num"].as<int>(), error["name"].as<std::string>(), error["category"].as<int>(), error["description"].as<std::string>());
            if((info.category == LOW_SUBSCRIBE_RATE) || (info.category == LOW_PUBLISH_RATE) || (info.category == LOW_OPERATION_CYCLE))
            {
                if(error["level"].as<std::string>() == "warn")
                {
                    diag_info detail_info(error["num"].as<int>(), error["name"].as<std::string>(), error["category"].as<int>(), error["description"].as<std::string>() ,error["threshold"].as<double>(), LEVEL_WARN);
                    boost::shared_ptr<rate_checker> rate_checker_ptr = boost::make_shared<rate_checker>(RATE_CHECKER_BUFFER_LENGTH);
                    checkers_[info.num] = rate_checker_ptr;
                    diag_info_.push_back(detail_info);
                }
                if(error["level"].as<std::string>() == "error")
                {
                    diag_info detail_info(error["num"].as<int>(), error["name"].as<std::string>(), error["category"].as<int>(), error["description"].as<std::string>() ,error["threshold"].as<double>(), LEVEL_ERROR);
                    boost::shared_ptr<rate_checker> rate_checker_ptr = boost::make_shared<rate_checker>(RATE_CHECKER_BUFFER_LENGTH);
                    checkers_[info.num] = rate_checker_ptr;
                    diag_info_.push_back(detail_info);
                }
                continue;
            }
            diag_info_.push_back(info);
        }
    }
    catch(...)
    {
#ifdef STRICT_ERROR_CODE_CHECK
        ROS_ERROR_STREAM("diag config file format was wrong. kill " + ros::this_node::getName() + ". Please check " << error_code_config_path_);
        WRITE_LOG();
        std::exit(-1);
#else
        ROS_WARN_STREAM("diag config file format was wrong in " + ros::this_node::getName() + ". Please check " << error_code_config_path_);
#endif
    }
    diag_pub_ = nh_.advertise<diag_msgs::diag_error>(ros::this_node::getName() + "/diag", 10);
    boost::thread rate_check_thread(boost::bind(&diag_manager::check_rate_, this));;
    enable_diag_ = true;
}

diag_manager::~diag_manager()
{

}

void diag_manager::check_rate_()
{
    ros::Rate rate(RATE_CHECK_FREQUENCY);
    while(ros::ok())
    {
        for (auto const& checker : checkers_)
        {
            boost::optional<double> rate = checker.second->get_rate();
            boost::optional<diag_info> info = query_diag_info(checker.first);
            if(rate && info && info.get().threshold)
            {
                if(*rate <info.get().threshold.get())
                {
                    if(info.get().level.get() == LEVEL_WARN)
                    {
                        ADD_DIAG_LOG_WARN(info.get().description);
                        publish_diag_(query_diag_info(info.get().num).get());
                    }
                    if(info.get().level.get() == LEVEL_ERROR)
                    {
                        ADD_DIAG_LOG_ERROR(info.get().description);
                        publish_diag_(query_diag_info(info.get().num).get());
                    }
                }
            }
        }
        rate.sleep();
    }
    return;
}

void diag_manager::DIAG_LOW_RELIABILITY(int num)
{
    if(enable_diag_ == false)
        return;    
    std::vector<int> required_error_code = {LOW_RELIABILITY};
    if(check_error_code(num, required_error_code))
    {
        boost::optional<diag_info> info = query_diag_info(num);
        ADD_DIAG_LOG_WARN(info.get().description);
        publish_diag_(query_diag_info(info.get().num).get());
    }
    return;
}

void diag_manager::DIAG_RATE_CHECK(int num)
{
    if(enable_diag_ == false)
        return;
    std::vector<int> required_error_code = {LOW_SUBSCRIBE_RATE, LOW_PUBLISH_RATE, LOW_OPERATION_CYCLE};
    if(check_error_code(num, required_error_code))
    {
        try
        {
            checkers_.at(num)->check();
        }
        catch(std::out_of_range&)
        {
#ifdef STRICT_ERROR_CODE_CHECK
        ROS_ERROR_STREAM("rate checker object cannot found. Please check " << error_code_config_path_);
        WRITE_LOG();
        std::exit(-1);
#else
        ROS_WARN_STREAM("rate checker object cannot found. Please check " << error_code_config_path_);
#endif
        }
    }
    return;
}

boost::optional<diag_info> diag_manager::query_diag_info(int num)
{
    for(auto diag_info_itr = diag_info_.begin(); diag_info_itr != diag_info_.end(); diag_info_itr++)
    {
        if(diag_info_itr->num == num)
        {
            if(diag_info_itr->threshold && diag_info_itr->level)
            {
                diag_info ret(diag_info_itr->num, diag_info_itr->name, diag_info_itr->category, diag_info_itr->description, diag_info_itr->threshold.get(), diag_info_itr->level.get());
                return ret;
            }
            else
            {
                diag_info ret(diag_info_itr->num, diag_info_itr->name, diag_info_itr->category, diag_info_itr->description);
                return ret;  
            }
        }
    }
    return boost::none;
}

void diag_manager::WRITE_LOG()
{
    namespace fs = boost::filesystem;
    const fs::path path("/tmp/Autoware/Diag/Log/" + ros::this_node::getName());
    boost::system::error_code error;
    const bool result = fs::create_directories(path, error);
    std::vector<std::string>::iterator it = diag_log_.begin();
    std::string write_string = "";
    while( it != diag_log_.end() ) {
        write_string = write_string + *it + "\n";
        ++it;
    }
    std::ofstream outputfile(std::string("/tmp/Autoware/Diag/Log/" + ros::this_node::getName() + "/log.txt").c_str());
    outputfile << write_string;
    outputfile.close();
    return;
}

void diag_manager::ADD_DIAG_LOG_ERROR(std::string log_text)
{
    log_text = "in " + ros::this_node::getName() + ": " + log_text;
    boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
    std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
    std::string text =  "[" + iso_time_str + "] : " + log_text;
    ROS_ERROR_STREAM(log_text);
    diag_log_.push_back(text);
    return;
}

void diag_manager::ADD_DIAG_LOG_WARN(std::string log_text)
{
    log_text = "in " + ros::this_node::getName() + ": " + log_text;
    boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
    std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
    std::string text =  "[" + iso_time_str + "] : " + log_text;
    ROS_WARN_STREAM(log_text);
    diag_log_.push_back(text);
    return;
}

void diag_manager::DIAG_RESOURCE(std::string target_resource_path, int num)
{
    if(enable_diag_ == false)
        return;
    std::vector<int> required_error_code = {RESOURCE_NOT_FOUND};
    if(check_error_code(num, required_error_code))
    {
        if(query_diag_info(num))
        {
            namespace fs = boost::filesystem;
            fs::path path(target_resource_path);
            boost::system::error_code error;
            const bool result = fs::exists(path, error);
            if (!result || error)
            {
                //ROS_ERROR_STREAM("required resource " << path << " does not found.");
                ADD_DIAG_LOG_ERROR("required resource " + target_resource_path + " does not found.");
                publish_diag_(query_diag_info(num).get());
                ROS_ERROR_STREAM("shutdonw " << ros::this_node::getName());
                WRITE_LOG();
                std::exit(-1);
            }
            return;
        }
        else
        {
            ADD_DIAG_LOG_WARN(query_diag_info(num)->description);
            publish_diag_(query_diag_info(num).get());
            return;
        }
    }
}

bool diag_manager::diag_resource(std::string target_resource_path)
{
    namespace fs = boost::filesystem;
    fs::path path(target_resource_path);
    boost::system::error_code error;
    const bool result = fs::exists(path, error);
    if (!result || error)
    {
        ADD_DIAG_LOG_ERROR("required resource " + target_resource_path + " does not found.");
        ADD_DIAG_LOG_ERROR("disable diag module in " + ros::this_node::getName());
        return false;
    }
    return true;
}

bool diag_manager::check_error_code(int requested_error_number, std::vector<int> right_categories)
{
    if(query_diag_info(requested_error_number))
    {
        for(auto category_itr = right_categories.begin(); category_itr != right_categories.end(); category_itr++)
        {
            if(*category_itr == query_diag_info(requested_error_number)->category)
            {
                return true;
            }
        }
#ifdef STRICT_ERROR_CODE_CHECK
        ROS_ERROR_STREAM("error category : " << query_diag_info(requested_error_number)->category << " in " << ros::this_node::getName() << " does not match. Please check " << error_code_config_path_);
        WRITE_LOG();
        std::exit(-1);
#else
        ROS_WARN_STREAM("error category : " << query_diag_info(requested_error_number)->category << " in " << ros::this_node::getName() << " does not match. Please check " << error_code_config_path_);
#endif
        return false;
    }
    else
    {
        ROS_WARN_STREAM("error number : " << requested_error_number << " in " << ros::this_node::getName() << " does not exist. Check " << error_code_config_path_);
        return false;
    }
}

void diag_manager::publish_diag_(diag_info info)
{
    diag_msgs::diag_error msg;
    msg.num = info.num;
    msg.name = info.name;
    msg.category = info.category;
    msg.description = info.description;
    diag_pub_.publish(msg);
    return;
}