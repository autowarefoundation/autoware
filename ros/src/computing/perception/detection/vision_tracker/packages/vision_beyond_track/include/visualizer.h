#ifndef COLOR_H
#define COLOR_H

#include "detection.h"
#include <opencv2/opencv.hpp>

static cv::Scalar color_list[26] = {
  cv::Scalar(255, 255, 255),
  cv::Scalar(255, 0, 0),
  cv::Scalar(0, 0, 255),
  cv::Scalar(0, 255, 0),
  cv::Scalar(44, 0, 0),
  cv::Scalar(184, 26, 255),
  cv::Scalar(0, 211, 255),
  cv::Scalar(0, 88, 0),
  cv::Scalar(255, 132, 132),
  cv::Scalar(70, 79, 158),
  cv::Scalar(193, 255, 0),
  cv::Scalar(149, 132, 0),
  cv::Scalar(123, 0, 0),
  cv::Scalar(79, 211, 149),
  cv::Scalar(220, 158, 246),
  cv::Scalar(255, 18, 211),
  cv::Scalar(105, 26, 123),
  cv::Scalar(97, 18, 246),
  cv::Scalar(132, 193, 255),
  cv::Scalar(9, 35, 35),
  cv::Scalar(123, 167, 141),
  cv::Scalar(9, 132, 246),
  cv::Scalar(0, 114, 132),
  cv::Scalar(255, 246, 114),
  cv::Scalar(255, 193, 158),
  cv::Scalar(123, 97, 114),
};

void visualize_results(string img_path, vector<Detection> outputs){
  cv::Mat src_img = cv::imread(img_path, 1);
  for (const auto& e: outputs) {
    cv::Rect rec = cv::Rect(e.bbox[0], e.bbox[1], e.bbox[2]-e.bbox[0], e.bbox[3]-e.bbox[1]);
    cv::Scalar c = color_list[e.dno % 26]; //cv::Scalar(0, 125, 125);
    cv::Mat roi = src_img(rec);
    cv::Mat color(roi.size(), CV_8UC3, c);
    double alpha = 0.4;
    cv::addWeighted(color, alpha, roi, 1.0 - alpha , 0.0, roi);
    cv::rectangle(src_img, rec, c, 2, 1);
    if (e.dno > 100) {
      rec = cv::Rect(e.bbox[0], e.bbox[1]-15, 27, 15);
    } else {
      rec = cv::Rect(e.bbox[0], e.bbox[1]-15, 20, 15);
    }
    c = color_list[0]; //cv::Scalar(0, 125, 125);
    roi = src_img(rec);
    cv::Mat white(roi.size(), CV_8UC3, c);
    alpha = 1.0;
    cv::addWeighted(white, alpha, roi, 1.0 - alpha , 0.0, roi);
    cv::Point txt_p;
    if (e.dno >= 10) {
      txt_p = cv::Point(e.bbox[0], e.bbox[1]-3);
    } else {
      txt_p = cv::Point(e.bbox[0]+5, e.bbox[1]-3);
    }

    int fontface = cv::FONT_HERSHEY_SIMPLEX;
    cv::putText(src_img, std::to_string(e.dno), txt_p,
                fontface, 0.4, cvScalar(0, 0, 0), 0, CV_AA);

  }
  cv::imshow("Image", src_img);
  cv::waitKey(200);
}

#endif
