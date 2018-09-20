#ifndef MONITOR_DRAWER_H_INCLUDED
#define MONITOR_DRAWER_H_INCLUDED

#define RAD 0
#define DEG 1

// headers for opencv
#include <opencv2/core/core.hpp>

class monitor_drawer{
    public:
        monitor_drawer(int mode) : _mode(mode)
        {

        };
        ~monitor_drawer()
        {

        };
        cv::Mat draw(double steering_angle){

        }
    private:
        const int _mode;
};

#endif  //MONITOR_DRAWER_H_INCLUDED