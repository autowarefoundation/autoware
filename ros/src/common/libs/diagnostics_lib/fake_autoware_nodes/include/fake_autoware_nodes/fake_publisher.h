#ifndef FAKE_PUBLISHER_H_INCLUDED
#define FAKE_PUBLISHER_H_INCLUDED

//headers in diag_lib
#include <diag_lib/diag_manager.h>

class fake_publisher
{
    public:
        fake_publisher();
        ~fake_publisher();
        void run();
        double divide(double a, double b);
    private:
        ros::NodeHandle nh_;
        ros::Publisher fake_pub_;
        diag_manager diag_manager_;
};

#endif  //FAKE_PUBLISHER_H_INCLUDED