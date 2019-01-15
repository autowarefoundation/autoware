#include "jsk_recognition_utils/rgb_colors.h"
#include <opencv2/opencv.hpp>
#include <gtest/gtest.h>


TEST(RGBColors, testGetRGBColor){
  cv::Vec3d color;
  // red
  color = jsk_recognition_utils::getRGBColor(jsk_recognition_utils::RED);
  EXPECT_EQ(1, color[0]);
  EXPECT_EQ(0, color[1]);
  EXPECT_EQ(0, color[2]);
  // gray
  color = jsk_recognition_utils::getRGBColor(jsk_recognition_utils::GRAY);
  EXPECT_EQ(0.502, color[0]);
  EXPECT_EQ(0.502, color[1]);
  EXPECT_EQ(0.502, color[2]);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
