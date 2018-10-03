#ifndef LGSVL_SIMULATOR_LAUNCHER_H_INCLUDED
#define LGSVL_SIMULATOR_LAUNCHER_H_INCLUDED

//headers in ROS
#include <ros/package.h>

//headers in STL
#include <stdlib.h>

class lgsvl_simulator_launcher{
public:
    lgsvl_simulator_launcher();
    ~lgsvl_simulator_launcher();
    void launch();
private:
    std::string exec_path_;
};

#endif  //LGSVL_SIMULATOR_LAUNCHER_H_INCLUDED