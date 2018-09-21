#include "monitor_drawer.h"

monitor_drawer::monitor_drawer(){
    std::string handle_image_path = ros::package::getPath("autoware_rviz_plugins") + "/media/handle.jpg";
    cv::Mat handle_image_cv = cv::imread(handle_image_path);
    handle_image_ = QImage(handle_image_cv.rows, handle_image_cv.cols, QImage::Format_RGBA8888);
    for (int i = 0; i < handle_image_cv.cols; i++)
    {
        for (int j = 0; j < handle_image_cv.rows; j++)
        {
            //ROS_ERROR_STREAM(handle_image_cv.data[j * handle_image_cv.step + i * handle_image_cv.elemSize() + 2]);
            QColor color(handle_image_cv.data[j * handle_image_cv.step + i * handle_image_cv.elemSize() + 2],
            handle_image_cv.data[j * handle_image_cv.step + i * handle_image_cv.elemSize() + 1],
            handle_image_cv.data[j * handle_image_cv.step + i * handle_image_cv.elemSize() + 0],
            255);
            handle_image_.setPixel(i, j, color.rgba());
        }
    }
    //cv::imshow("test",handle_image_cv);
    //cv::waitKey(0);
    //handle_image_ = QImage(640, 640, QImage::Format_RGBA8888);
    //handle_image_.load(QString(handle_image_path.c_str()));
}

monitor_drawer::~monitor_drawer(){

}

QImage monitor_drawer::draw(){
    return handle_image_;
}