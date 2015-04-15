#include "TrafficLight.h"
#include "TrafficLightDetector.h"

/*
  ref:
  http://imagingsolution.net/program/opencv/unsharpmasking-2/
 */
static void UnsharpMasking(Mat* src, Mat* dst, float k)
{
  float kernelData[] = {
    -k/9.0f, -k/9.0f,          -k/9.0f,
    -k/9.0f, 1 + (8 * k)/9.0f, -k/9.0f,
    -k/9.0f, -k/9.0f         , -k/9.0f,
  };

  Mat kernel = Mat(3, 3, CV_32F, kernelData);
  filter2D(*src, *dst, src->depth(), kernel, Point(-1, -1), 0, BORDER_CONSTANT);
}


/*
  ref:
  http://seesaawiki.jp/image_processing/d/%A5%B4%A5%DE%B1%F6%A5%CE%A5%A4%A5%BA%BD%FC%B5%EE
*/
static void remove_SoltPepperNoise(Mat *input, int iterations)
{
  for (int i=0; i<iterations; i++)
    dilate(*input, *input, Mat(), Point(-1, -1), 1);

  for (int i=0; i<iterations; i++)
    erode(*input, *input, Mat(), Point(-1, -1), 1);
}

TrafficLightDetector::TrafficLightDetector() {}

LightState TrafficLightDetector::brightnessDetect(const Mat &input) {

	Mat tmpImage;
	input.copyTo(tmpImage);

    //    blur(tmpImage, tmpImage, Size(5, 5), Point(-1, -1), BORDER_DEFAULT);
	for (int i = 0; i < contexts.size(); i++) {
      //		Context context = contexts[i];
      Context context = contexts.at(i);
#if 0
		// Mat roi = tmpImage(Rect(context.topLeft, context.botRight));
#endif
        Mat roi;
        tmpImage.copyTo(roi);

        UnsharpMasking(&roi, &roi, 3.0f);

        imshow("unsharped", roi);

		threshold(roi, roi, 0, 255, THRESH_BINARY | THRESH_OTSU);

        imshow("binalized", roi);

        remove_SoltPepperNoise(&roi, 1);

        imshow("remove_noize", roi);
        //        waitKey(5);

        // std::cout << "red:" << std::endl;
        // std::cout << "center = (" << context.redCenter.x << ", " << context.redCenter.y << ")" << std::endl;
        //		bool display_red = (getBrightnessRatioInCircle(tmpImage, context.redCenter, context.lampRadius) > 0.5);
        bool display_red = (getBrightnessRatioInCircle(roi, context.redCenter, context.lampRadius) > 0.5);
        // std::cout << "yellow:" << std::endl;
        // std::cout << "center = (" << context.yellowCenter.x << ", " << context.yellowCenter.y << ")" << std::endl;
        //		bool display_yellow = (getBrightnessRatioInCircle(tmpImage, context.yellowCenter, context.lampRadius) > 0.5);
		bool display_yellow = (getBrightnessRatioInCircle(roi, context.yellowCenter, context.lampRadius) > 0.5);
        // std::cout << "green:" << std::endl;
        // std::cout << "center = (" << context.greenCenter.x << ", " << context.greenCenter.y << ")" << std::endl;
        //		bool display_green = (getBrightnessRatioInCircle(tmpImage, context.greenCenter, context.lampRadius) > 0.5);
        bool display_green = (getBrightnessRatioInCircle(roi, context.greenCenter, context.lampRadius) > 0.5);

#if 0
        std::cout << "display_red: " << display_red << std::endl;
        std::cout << "display_yellow: " << display_yellow << std::endl;
        std::cout << "display_green: " << display_green << std::endl;
#endif

		int currentLightsCode = getCurrentLightsCode(display_red, display_yellow, display_green);
        //		contexts[i].lightState = determineState(contexts[i].lightState, currentLightsCode);
        contexts.at(i).lightState = determineState(contexts.at(i).lightState, currentLightsCode);

		// Make ROI black
		roi.setTo(Scalar(0));
	}
    //	return contexts[0].lightState;
    return contexts.at(0).lightState;
}

double getBrightnessRatioInCircle(const Mat &input, const Point center, const int radius) {

	int whitePoints = 0;
	int blackPoints = 0;

	for (int i = center.x - radius; i < center.x + radius; i++) {
		for (int j = center.y - radius; j < center.y + radius; j++) {
          if ((i - center.x)*(i - center.x) + (j - center.y)*(j - center.y) <= radius*radius) {
				(input.at<uchar>(j,i) == 0) ? blackPoints++ : whitePoints++;
          }
		}
	}
	//printf("Ratio: %f\n", ((double)whitePoints) / (whitePoints + blackPoints));
    std::cout << "  white:" << whitePoints << " black: " << blackPoints << std::endl;
	return ((double)whitePoints) / (whitePoints + blackPoints);
}

int getCurrentLightsCode(bool display_red, bool display_yellow, bool display_green) {
	return (int)display_red + 2 * ((int) display_yellow) + 4 * ((int) display_green);
}

LightState determineState(LightState previousState, int currentLightsCode) {
	//printf("Previous state: %d, currentCode: %d, Switched state to %d\n", previousState, currentLightsCode, STATE_TRANSITION_MATRIX[previousState][currentLightsCode]);
	return STATE_TRANSITION_MATRIX[previousState][currentLightsCode];
}



/*
 *  Attempt to recognize by color tracking in HSV. Detects good only green, but need to
 *  play also with S and V parameters range.
 */
void TrafficLightDetector::colorDetect(const Mat &input, Mat &output, const Rect coords, int Hmin, int Hmax) {

	if (input.channels() != 3) {
		return;
	}

	Mat hsv, thresholded;
	cvtColor(input, hsv, CV_RGB2HSV, 0);
	inRange(hsv, Scalar(Hmin,0,0), Scalar(Hmax,255,255), thresholded);

	cvtColor(thresholded, thresholded, CV_GRAY2RGB);
	thresholded.copyTo(output);

	rectangle(output, coords, MY_COLOR_RED);
}
