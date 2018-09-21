#ifndef MONITOR_DRAWER_H_INCLUDED
#define MONITOR_DRAWER_H_INCLUDED

//headers in Qt
#include <QImage>
#include <QString>
#include <QColor>

//headers in ROS
#include <ros/package.h>
#include <ros/ros.h>


//headers for opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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