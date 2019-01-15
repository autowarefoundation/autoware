#include "jsk_recognition_utils/cv_utils.h"
#include <opencv2/opencv.hpp>
#include <gtest/gtest.h>


TEST(CvUtils, testLabelToRGB){
  cv::Mat label_image = cv::Mat::zeros(1, 2, CV_32SC1);
  label_image.at<int>(0, 1) = 1;
  cv::Mat rgb_image;
  jsk_recognition_utils::labelToRGB(label_image, rgb_image);
  // background label 0 -> [0, 0, 0]
  EXPECT_EQ(0, cv::norm(rgb_image.at<cv::Vec3b>(0, 0)));
  // label 1 -> not [0, 0, 0]
  EXPECT_NE(0, cv::norm(rgb_image.at<cv::Vec3b>(0, 1)));
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
