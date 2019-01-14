#ifndef DIAG_INFO_H_INCLUDED
#define DIAG_INFO_H_INCLUDED

//headers in diag_lib
#include <diag_lib/diag_manager_config.h>

//headers in STL
#include <string>

//headers in boost
#include <boost/optional.hpp>

struct diag_info
{
    const int num;
    const std::string name;
    const int category;
    const std::string description;
    const boost::optional<double> threshold;
    const boost::optional<int> level;
    diag_info(int num_, std::string name_, int category_, std::string description_) 
    : num(num_), name(name_), category(category_) ,description(description_) , threshold(boost::none), level(boost::none)
    {};
    diag_info(int num_, std::string name_, int category_, std::string description_, double threshold_, int level_)
    : num(num_), name(name_), category(category_) ,description(description_) , threshold(threshold_), level(level_)
    {

    }
};
#endif  //DIAG_INFO_H_INCLUDED