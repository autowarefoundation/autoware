/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * v1.0 Masaya Kataoka
 */

#include <autoware_health_checker/value_manager.h>

ValueManager::ValueManager(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    nh_.getParam("health_checker", diag_params_);
    ros_ok_ = true;
    default_value_exist_ = false;
}

ValueManager::~ValueManager()
{
    ros_ok_ = false;
}

void ValueManager::run()
{
    boost::thread update_thread = boost::thread(boost::bind(&ValueManager::updateParams,this));
}

void ValueManager::setDefaultValue(std::string key,double warn_value,double error_value, double fatal_value)
{
    if(default_value_exist_)
    {
        return;
    }
    data_[{key,autoware_health_checker::LEVEL_WARN}] = warn_value;
    data_[{key,autoware_health_checker::LEVEL_ERROR}] = error_value;
    data_[{key,autoware_health_checker::LEVEL_FATAL}] = fatal_value;
    default_value_exist_ = true;
    return;
}

bool ValueManager::foundParamKey(std::string key,uint8_t level,std::string& key_str)
{
    key_str == "";
    if(level == autoware_health_checker::LEVEL_WARN)
    {
        key_str = key + "/warn";
    }
    if(level == autoware_health_checker::LEVEL_ERROR)
    {
        key_str = key + "/error";
    }
    if(level == autoware_health_checker::LEVEL_FATAL)
    {
        key_str = key + "/fatal";
    }
    for(auto itr = diag_params_.begin(); itr != diag_params_.end(); itr++)
    {
        if(itr->first == key_str)
        {
            return true;
        }
    }
    return false;
}

double ValueManager::getValue(std::string key,uint8_t level)
{
    double ret;
    mtx_.lock();
    if(level == autoware_health_checker::LEVEL_WARN)
    {
        std::string key_str;
        if(foundParamKey(key,level,key_str))
        {
            ret = diag_params_[key_str];
        }
        else
        {
            ret = data_[{key,level}];
        }
    }
    mtx_.unlock();
    return ret;
}

void ValueManager::updateParams()
{
    ros::Rate rate(1);
    while(ros_ok_)
    {
        mtx_.lock();
        nh_.getParam("health_checker", diag_params_);
        mtx_.unlock();
        rate.sleep();
    }
    return;
}