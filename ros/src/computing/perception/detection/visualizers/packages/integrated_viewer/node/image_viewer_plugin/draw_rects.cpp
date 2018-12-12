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
        generateColors(color_map_, 10);
#else
        cv::generateColors(color_map_, 10);
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
            if (detected_object.x >= 0
                && detected_object.y >= 0
                && detected_object.width > 0
                && detected_object.height > 0)
            {
                // Draw object information label
                DrawLabel(detected_object, image);

                int x2 = detected_object.x + detected_object.width;
                if (x2 >= image.cols)
                    x2 = image.cols - 1;
                int y2 = detected_object.y + detected_object.height;
                if (y2 >= image.rows)
                    y2 = image.rows - 1;

                if (detected_object.dimensions.x > 0
                    && detected_object.dimensions.y > 0
                    && detected_object.dimensions.z > 0)
                {
                    cv::Mat image_roi = image(cv::Rect(cv::Point(detected_object.x, detected_object.y),
                                                       cv::Point(x2,
                                                                 y2)));
                    cv::Mat color_fill(image_roi.size(), CV_8UC3,
                                       color_map_[0]);
                    double alpha = 0.3;
                    cv::addWeighted(color_fill, alpha, image_roi, 1.0 - alpha , 0.0, image_roi);
                }

                // Draw rectangle
                cv::rectangle(image,
                              cv::Point(detected_object.x, detected_object.y),
                              cv::Point(x2,
                                        y2),
                              color_map_[0],
                              kRectangleThickness,
                              CV_AA,
                              0);
            }
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
                          color_map_[0],
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
        const double font_scale = 0.7;
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
        if (in_detected_object.id > 0)
        {
            label_one << " " << std::to_string(in_detected_object.id);
        }
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
                    CV_RGB(255, 255, 255),
                    1,
                    CV_AA);
        label_origin.y+= text_holder_rect.height / 3;
        cv::putText(image,
                    label_two.str(),
                    label_origin,
                    font_face,
                    font_scale,
                    CV_RGB(255, 255, 255),
                    1,
                    CV_AA);

    } // DrawRects::DrawLabel()
} // end namespace integrated_viewer
