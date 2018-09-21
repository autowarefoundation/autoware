#include "monitor_drawer.h"

monitor_drawer::monitor_drawer(){
    std::string handle_image_path = ros::package::getPath("autoware_rviz_plugins") + "/media/handle.png";
    handle_image_.load(QString(handle_image_path.c_str()));
}

monitor_drawer::~monitor_drawer(){

}

QImage monitor_drawer::draw(){
    return handle_image_;
}