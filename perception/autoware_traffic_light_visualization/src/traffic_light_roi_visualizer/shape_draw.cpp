// Copyright 2024 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "shape_draw.hpp"

#include "opencv2/core/types.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace autoware::traffic_light::visualization
{
void drawShape(
  const DrawFunctionParams & params, const std::string & filename, bool flipHorizontally,
  bool flipVertically, int x_offset, int y_offset, double scale_factor)
{
  std::string filepath =
    ament_index_cpp::get_package_share_directory("autoware_traffic_light_visualization") +
    "/images/" + filename;
  cv::Mat shapeImg = cv::imread(filepath, cv::IMREAD_UNCHANGED);
  if (shapeImg.empty()) {
    std::cerr << "Failed to load image: " << filepath << std::endl;
    return;
  }

  if (flipHorizontally) {
    cv::flip(shapeImg, shapeImg, 1);  // Flip horizontally
  }

  if (flipVertically) {
    cv::flip(shapeImg, shapeImg, 0);  // Flip vertically
  }

  cv::resize(
    shapeImg, shapeImg, cv::Size(params.size, params.size), scale_factor, scale_factor,
    cv::INTER_AREA);

  // Calculate the center position including offsets
  cv::Point position(
    params.position.x + x_offset, params.position.y - shapeImg.rows / 2 + y_offset);

  // Check for image boundaries
  if (
    position.x < 0 || position.y < 0 || position.x + shapeImg.cols > params.image.cols ||
    position.y + shapeImg.rows > params.image.rows) {
    // TODO(KhalilSelyan): This error message may flood the terminal logs, so commented out
    // temporarily. Need to consider a better way.

    // std::cerr << "Adjusted position is out of image bounds." << std::endl;
    return;
  }

  // Calculate the width of the text
  std::string probabilityText =
    std::to_string(static_cast<int>(round(params.probability * 100))) + "%";
  int baseline = 0;
  cv::Size textSize =
    cv::getTextSize(probabilityText, cv::FONT_HERSHEY_SIMPLEX, 0.75, 2, &baseline);

  // Adjust the filled rectangle to be at the top edge and the correct width
  int filledRectWidth =
    shapeImg.cols + (filename != "unknown.png" ? textSize.width + 10 : 5);  // Add some padding
  int filledRectHeight = shapeImg.rows + 10;                                // Add some padding

  cv::rectangle(
    params.image, cv::Rect(position.x - 2, position.y - 5, filledRectWidth, filledRectHeight),
    params.color,
    -1);  // Filled rectangle

  // Create ROI on the destination image
  cv::Mat destinationROI = params.image(cv::Rect(position, cv::Size(shapeImg.cols, shapeImg.rows)));

  // Overlay the image onto the main image
  for (int y = 0; y < shapeImg.rows; ++y) {
    for (int x = 0; x < shapeImg.cols; ++x) {
      cv::Vec4b & pixel = shapeImg.at<cv::Vec4b>(y, x);
      if (pixel[3] != 0) {  // Only non-transparent pixels
        destinationROI.at<cv::Vec3b>(y, x) = cv::Vec3b(pixel[0], pixel[1], pixel[2]);
      }
    }
  }

  // Position the probability text right next to the shape
  if (filename != "unknown.png") {
    cv::putText(
      params.image, probabilityText,
      cv::Point(
        position.x + shapeImg.cols + 5, position.y + shapeImg.rows / 2 + textSize.height / 2),
      cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
  }
}

void drawCircle(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 5;
  drawShape(params, "circle.png", false, false, 0, -y_offset);
}

void drawLeftArrow(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 5;
  drawShape(params, "left_arrow.png", false, false, 0, -y_offset);
}

void drawRightArrow(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 5;
  drawShape(params, "left_arrow.png", true, false, 0, -y_offset);
}

void drawStraightArrow(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 5;  // This adjusts the base position upwards

  drawShape(params, "straight_arrow.png", false, false, 0, -y_offset);
}
void drawDownArrow(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 5;  // This adjusts the base position upwards
  drawShape(params, "straight_arrow.png", false, true, 0, -y_offset);
}

void drawDownLeftArrow(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 5;
  drawShape(params, "down_left_arrow.png", false, false, 0, -y_offset);
}

void drawDownRightArrow(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 5;
  drawShape(params, "down_left_arrow.png", true, false, 0, -y_offset);
}

void drawCross(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 5;

  drawShape(params, "cross.png", false, false, 0, -y_offset);
}

void drawUnknown(const DrawFunctionParams & params)
{
  int y_offset = params.size / 2 + 5;
  drawShape(params, "unknown.png", false, false, 0, -y_offset);
}

void drawTrafficLightShape(
  cv::Mat & image, const std::string & shape, const cv::Point & position, const cv::Scalar & color,
  int size, float probability)
{
  static std::map<std::string, DrawFunction> shapeToFunction = {
    {"circle", drawCircle},
    {"left", drawLeftArrow},
    {"right", drawRightArrow},
    {"straight", drawStraightArrow},
    {"down", drawDownArrow},
    {"down_left", drawDownLeftArrow},
    {"down_right", drawDownRightArrow},
    {"cross", drawCross},
    {"unknown", drawUnknown}};
  auto it = shapeToFunction.find(shape);
  if (it != shapeToFunction.end()) {
    DrawFunctionParams params{image, position, color, size, probability};
    it->second(params);
  } else {
    std::cerr << "Unknown shape: " << shape << std::endl;
  }
}

}  // namespace autoware::traffic_light::visualization
