#ifndef FAKE_PUBLISHER_H_INCLUDED
#define FAKE_PUBLISHER_H_INCLUDED

//headers in diag_lib
#include <diag_lib/diag_manager.h>

class FakePublisher
{
    public:
        FakePublisher();
        ~FakePublisher();
        void run();
        double divide(double a, double b);
    private:
        ros::NodeHandle nh_;
        ros::Publisher fake_pub_;
        DiagManager diag_manager_;
};

#endif  //FAKE_PUBLISHER_H_INCLUDED