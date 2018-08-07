#include "draw_rects.h"
#include <string>
#include <vector>


#include <opencv2/core/version.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "draw_rects.h"

#if (CV_MAJOR_VERSION == 3)

#include "gencolors.cpp"

#else
#include <opencv2/contrib/contrib.hpp>
#include <autoware_msgs/DetectedObjectArray.h>

#endif

namespace integrated_viewer
{
    const int        DrawRects::kRectangleThickness = 3;

    DrawRects::DrawRects(void)
    {
        // Generate color map to represent tracked object
#if (CV_MAJOR_VERSION == 3)
        generateColors(color_map_, 25);
#else
        cv::generateColors(color_map_, 25);
#endif
        car_image_ = cv::imread(DEFAULT_PATH + "car.png", cv::IMREAD_UNCHANGED);
        pedestrian_image_ = cv::imread(DEFAULT_PATH + "pedestrian.png", cv::IMREAD_UNCHANGED);

    } // DrawRects::DrawRects()

    void DrawRects::OverlayImage(const cv::Mat &in_background, const cv::Mat &in_foreground,
                                      cv::Mat &output, cv::Point2i in_location)
    {
        in_background.copyTo(output);

        for(int y = std::max(in_location.y , 0); y < in_background.rows; ++y)
        {
            int fY = y - in_location.y;

            if(fY >= in_foreground.rows)
                break;
            for(int x = std::max(in_location.x, 0); x < in_background.cols; ++x)
            {
                int fX = x - in_location.x;

                if(fX >= in_foreground.cols)
                    break;

                double opacity =
                        ((double)in_foreground.data[fY * in_foreground.step + fX * in_foreground.channels() + 3])/ 255.;
                for(int c = 0; opacity > 0 && c < output.channels(); ++c)
                {
                    unsigned char foregroundPx =
                            in_foreground.data[fY * in_foreground.step + fX * in_foreground.channels() + c];
                    unsigned char in_backgroundPx =
                            in_background.data[y * in_background.step + x * in_background.channels() + c];
                    output.data[y*output.step + output.channels()*x + c] =
                            in_backgroundPx * (1.-opacity) + foregroundPx * opacity;
                }
            }
        }
    }

    void DrawRects::DrawImageRect(const autoware_msgs::DetectedObjectArray::ConstPtr &detected_objects,
                                  cv::Mat &image)
    {
        if (detected_objects == NULL)
        {
            return;
        }

        // Draw rectangles for each object
        for (const auto &detected_object : detected_objects->objects)
        {
            // Draw object information label
            DrawLabel(detected_object, image);

            // Draw rectangle
            cv::rectangle(image,
                          cv::Point(detected_object.x, detected_object.y),
                          cv::Point(detected_object.x + detected_object.width,
                                    detected_object.y + detected_object.height),
                          cv::Scalar(detected_object.color.r, detected_object.color.g, detected_object.color.b),
                          kRectangleThickness,
                          CV_AA,
                          0);
        }
    } // DrawRects::DrawImageRect()

    void DrawRects::DrawImageBox(const autoware_msgs::DetectedObjectArray::ConstPtr &detected_objects,
                                 cv::Mat &image)
    {
        if (detected_objects == NULL)
        {
            return;
        }

        // Draw rectangles for each object
        for (const auto &detected_object : detected_objects->objects)
        {
            DrawLabel(detected_object, image);

            // Draw rectangle
            cv::rectangle(image,
                          cv::Point(detected_object.x, detected_object.y),
                          cv::Point(detected_object.x + detected_object.width,
                                    detected_object.y + detected_object.height),
                          cv::Scalar(detected_object.color.r, detected_object.color.g, detected_object.color.b),
                          kRectangleThickness,
                          CV_AA,
                          0);
        }
    } // DrawRects::DrawImageRect()

    void DrawRects::DrawLabel(const autoware_msgs::DetectedObject &in_detected_object,
                              cv::Mat &image)
    {

        cv::Point rectangle_origin(in_detected_object.x, in_detected_object.y);
        // label's property
        const int font_face = cv::FONT_HERSHEY_DUPLEX;
        const double font_scale = 0.8;
        const int font_thickness = 1;
        int font_baseline = 0;
        int icon_width = 40;
        int icon_height = 40;
        std::ostringstream label_one;
        std::ostringstream label_two;

        cv::Size label_size = cv::getTextSize("0123456789",
                                              font_face,
                                              font_scale,
                                              font_thickness,
                                              &font_baseline);

        cv::Point label_origin = cv::Point(rectangle_origin.x,
                                           rectangle_origin.y - font_baseline - kRectangleThickness*2 - icon_height);

        double distance = sqrt(in_detected_object.pose.position.x*in_detected_object.pose.position.x +
                                       in_detected_object.pose.position.y*in_detected_object.pose.position.y);

        label_one << in_detected_object.label;
        if (distance > 0.1)
        {
            label_two << std::setprecision(2) << distance << "meters";
        }

        if (in_detected_object.label == "car" || in_detected_object.label == "truck")
        {
            OverlayImage(image, car_image_, image, label_origin);
        }
        else if (in_detected_object.label == "person")
        {
            OverlayImage(image, pedestrian_image_, image, label_origin);
        }
        else
        {
            icon_width = 0;
        }

        if(label_origin.x < 0)
            label_origin.x = 0;
        if(label_origin.y < 0)
            label_origin.y = 0;

        cv::Rect text_holder_rect;
        text_holder_rect.x = label_origin.x;
        text_holder_rect.y = label_origin.y;
        text_holder_rect.width = label_size.width + icon_width;
        if (text_holder_rect.x + text_holder_rect.width > image.cols)
            text_holder_rect.width = image.cols - text_holder_rect.x - 1;
        text_holder_rect.height = label_size.height + icon_height;
        if (text_holder_rect.y + text_holder_rect.height > image.rows)
            text_holder_rect.height = image.rows - text_holder_rect.y - 1;

        cv::Mat roi = image(text_holder_rect);

        cv::Mat text_holder (roi.size(), CV_8UC3, cv::Scalar(0,0,0));

        double alpha = 0.3;
        cv::addWeighted(text_holder, alpha, roi, 1.0 - alpha, 0.0, roi);
        label_origin.x+= icon_width;
        label_origin.y+= text_holder_rect.height / 3;
        cv::putText(image,
                    label_one.str(),
                    label_origin,
                    font_face,
                    font_scale,
                    CV_RGB(255, 255, 255));
        label_origin.y+= text_holder_rect.height / 3;
        cv::putText(image,
                    label_two.str(),
                    label_origin,
                    font_face,
                    font_scale,
                    CV_RGB(255, 255, 255));

    } // DrawRects::DrawLabel()
} // end namespace integrated_viewer
