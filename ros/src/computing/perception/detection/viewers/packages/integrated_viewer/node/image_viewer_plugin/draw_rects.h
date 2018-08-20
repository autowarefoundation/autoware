#ifndef DRAW_RECTS_H
#define DRAW_RECTS_H

#include <opencv/cv.h>
#include <autoware_msgs/DetectedObjectArray.h>

#define XSTR(x) #x
#define STR(x) XSTR(x)

const std::string DEFAULT_PATH                                      = STR(IMAGE_VIEWER_DEFAULT_PATH);

namespace integrated_viewer
{
    // helper class to draw detection result rectangle
    class DrawRects
    {
    public:
        explicit DrawRects(void);

        void DrawImageRect(const autoware_msgs::DetectedObjectArray::ConstPtr &in_objects, cv::Mat &image);
        void DrawImageBox(const autoware_msgs::DetectedObjectArray::ConstPtr &in_objects, cv::Mat &image);

    protected:
        static const int kRectangleThickness;

    private:
        void DrawLabel(const autoware_msgs::DetectedObject& in_object, cv::Mat &image);

        void OverlayImage(const cv::Mat &in_background, const cv::Mat &in_foreground,
                          cv::Mat &output, cv::Point2i in_location);

        std::vector<cv::Scalar> color_map_;
        static const cv::Scalar kBlue;
        static const cv::Scalar kGreen;

        cv::Mat car_image_;
        cv::Mat pedestrian_image_;
    };
}
#endif // DRAW_RECTS_H
