#ifndef TUNER_BODY_H
#define TUNER_BODY_H

#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#define DEFAULT_SAT_LOWER ((double)0.37 * 255)
#define DEFAULT_SAT_UPPER (255)
#define DEFAULT_VAL_LOWER (90)
#define DEFAULT_VAL_UPPER (255)

typedef struct {
    int center;
    int range;
} value_set;

typedef struct {
    value_set hue;
    value_set sat;
    value_set val;
    bool isUpdated;
} thresholds_set;

class TunerBody
{

private:
    static cv::Point Clicked_point;
    static int Signal_color;
    int H_slider_val;
    int S_slider_val;
    int V_slider_val;
    static cv::Mat src_img;
    cv::Mat base;
    static cv::Mat mask;
    static std::string windowName;
    static thresholds_set Red_set;
    static thresholds_set Yellow_set;
    static thresholds_set Green_set;
    static thresholds_set* Selected_set;
    static bool updateImage;

public:
    enum signal_state {
        GREEN = 0,
        YELLOW = 1,
        RED = 2,
    };

    TunerBody();
    ~TunerBody();
    void launch(void);
    static void setColor(signal_state state);
    static void setClickedPoint(cv::Point pt);
    static void saveResult(std::string fileName);
    static void openSetting(std::string fileName);
    static void setUpdateImage(void);
    static void image_raw_callBack(const sensor_msgs::Image& image_msg);

};

#endif // TUNER_BODY_H
