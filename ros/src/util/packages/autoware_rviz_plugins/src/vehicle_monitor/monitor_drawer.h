#ifndef MONITOR_DRAWER_H_INCLUDED
#define MONITOR_DRAWER_H_INCLUDED

//headers in Qt
#include <QImage>
#include <QString>

//headers in ROS
#include <ros/package.h>
#include <ros/ros.h>

#define RAD 0
#define DEG 1

class monitor_drawer
{
public:
    monitor_drawer();
    ~monitor_drawer();
    QImage draw();
private:
    QImage handle_image_;
};
#endif  //MONITOR_DRAWER_H_INCLUDED