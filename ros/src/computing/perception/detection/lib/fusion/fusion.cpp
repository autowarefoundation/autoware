/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

#include "fusion_func.h"
#include "search_distance.h"

#if _DEBUG //debug
static const char *window_name = "CAR_TRACK";
//for imageCallback
static cv_bridge::CvImagePtr cv_image;
static IplImage temp;
static IplImage *image;
static double ratio = 1;	//resize ratio
#endif

/*for obstacle_detectionCallback */
static std::string obj_type;
static std::vector<int> g_corner_points;
static std::vector<float> g_scores;
static std::vector<int> filtered_corner_points;
static std::vector<float> filtered_scores;
static int g_objects_num;
static int filtered_objects_num;
/*for distance_measurementCallback */
static Scan_image g_scan_image;
/* for common Callback */
static std::vector<float> g_distances;
static std::vector<float> filtered_distances;

static std::vector<float> filtered_min_heights;//stores the min height of the object
static std::vector<float> filtered_max_heights;//stores the max height of the object

static points2image::PointsImage points_msg;

static bool objectsStored = false, pointsStored = false;

float Min_low_height = -1.5;
float Max_low_height = -1.0;
float Max_height = 2.0;
int Min_points = 2;
float Dispersion = 1.0;

//checks if a float is close to zero
static inline bool isAlmostZero(float x)
{
	float abs_x  = (float)fabs(x);
	const int rangeScale = 100;
	return(abs_x < FLT_MIN*rangeScale);
}

//receives and sets params from the publisher node
void setParams(float minLowHeight, float maxLowHeight, float maxHeight, int minPoints, float disp)
{
	Min_low_height = minLowHeight;
	Max_low_height = maxLowHeight;
	Max_height = maxHeight;
	Min_points = minPoints;
	Dispersion = disp;
}

//Check wheter vscanpoints are contained in the detected object bounding box(rect) or not, store the vscanpoints indices in outIndices
bool rectangleContainsPoints(cv::Rect rect, std::vector<Point5> &vScanPoints, float object_distance, std::vector<int> &outIndices)
{
	int numPoints = vScanPoints.size();

	if (numPoints <= 0)
		return false;

	int pointsFound = 0;
	for (int i = 0; i < numPoints; i++)
	{
		if (vScanPoints[i].x >= rect.x && vScanPoints[i].y >= rect.y &&
				(vScanPoints[i].min_h > Min_low_height && vScanPoints[i].min_h < Max_low_height) &&
				(vScanPoints[i].max_h < Max_height))
		{
			outIndices.push_back(i);//store indices of points inside the bounding box
			pointsFound++;
		}
	}
	if ( pointsFound >= Min_points)
		return true;
	else
		return false;
}

//get the average of the minimum heights for only those vscan points in theindices
float getMinAverage(std::vector<Point5> &vScanPoints, std::vector<int> &indices)
{
	float average = 0.0;
	int num = indices.size();
	if (num < 0)
		return 0.0;
	for (int i = 0 ; i < num ; i++)
	{
		average+=vScanPoints[indices[i]].min_h;
	}
	return average/num;
}

//get the standard deviation of only those min heightsa in the indices vector
float getStdDev(std::vector<Point5> &vScanPoints, std::vector<int> &indices, float avg)
{
	float N = indices.size();
	if(N==0.0)
		return 0.0;
	float stddev = 0.0;
	for (int i = 0 ; i < N ; i++)
	{
		stddev+=(vScanPoints[indices[i]].min_h-avg)*(vScanPoints[indices[i]].min_h-avg);
	}
	stddev/=N;

	return sqrt(stddev);
}

//obtain the coefficient of dispersion of the min height to check for uneven heights
bool dispersed(std::vector<Point5> &vScanPoints, std::vector<int> &indices)
{
	float avg = getMinAverage(vScanPoints, indices);
	float stddev = getStdDev(vScanPoints, indices, avg);

	if(abs(stddev/avg>=Dispersion))
		return true;

	return false;
}

//returns the vscanpoints in the pointcloud
void getVScanPoints(std::vector<Point5> &vScanPoints)
{
	int w = points_msg.image_width;
	int h = points_msg.image_height;
	for(int y=0; y<h; y++)
	{
		for(int x=0; x<w; x++)
		{
			int i = y * w + x;
			double distance = points_msg.distance[i];
			float min_h = points_msg.min_height[i];
			float max_h = points_msg.max_height[i];

			if(distance == 0)
				continue;

			vScanPoints.push_back({x, y, distance, min_h, max_h});//add Real Points so they can be later checked against the detection bounding box
		}
	}
}

void getMinMaxHeight(std::vector<Point5>& vScanPoints, std::vector<int> indices, float& outMinHeight, float& outMaxHeight)
{
	float N = indices.size();
	if(N==0.0)
		return;
	float tmpMinH = 0, tmpMaxH = 0;
	for (int i = 0 ; i < N ; i++)
	{
		if (vScanPoints[indices[i]].min_h < tmpMinH)
			tmpMinH = vScanPoints[indices[i]].min_h;
		if (vScanPoints[indices[i]].max_h > tmpMaxH)
			tmpMaxH = vScanPoints[indices[i]].max_h;
	}
	outMinHeight = tmpMinH;
	outMaxHeight = tmpMaxH;
}

void fuseFilterDetections(std::vector<Point5>& vScanPoints)
{
	std::vector<int> pointsInBoundingBox;
	//reset
	filtered_objects_num = 0;
	filtered_corner_points.clear();
    filtered_scores.clear();
	filtered_distances.clear();
	filtered_max_heights.clear();
	filtered_min_heights.clear();
	for(int i = 0; i < g_objects_num; i++)
	{
		//corner_point[0]=>X1		corner_point[1]=>Y1
		//corner_point[2]=>width	corner_point[3]=>height
		cv::Rect detection = cv::Rect(g_corner_points[0+i*4], g_corner_points[1+i*4], g_corner_points[2+i*4], g_corner_points[3+i*4]);
		if (!isAlmostZero(g_distances.at(i)) &&
			rectangleContainsPoints(detection, vScanPoints, g_distances.at(i), pointsInBoundingBox) &&
			!dispersed(vScanPoints, pointsInBoundingBox)
		    )
		{
			//if all the conditions above are true -> store this detection
			//objects_num
			filtered_objects_num++;
			//corner_points
			filtered_corner_points.push_back(detection.x);
			filtered_corner_points.push_back(detection.y);
			filtered_corner_points.push_back(detection.width);
			filtered_corner_points.push_back(detection.height);
			//detection score
			filtered_scores.push_back(g_scores.at(i));
			//distance
			filtered_distances.push_back(g_distances.at(i));
			//calculate min and max height of the points in the bounding box
			float minHeight = 0.0, maxHeight = 0.0;
			getMinMaxHeight(vScanPoints, pointsInBoundingBox, minHeight, maxHeight);
			filtered_max_heights.push_back(maxHeight);
			filtered_min_heights.push_back(minHeight);
		}
	}
}

void fuse()
{
	if (!pointsStored || !objectsStored)
		return;

	calcDistance();//obtain distance for each object

	//Draw VScan Points and get them before displaying detections
	std::vector<Point5> vScanPoints;

	getVScanPoints(vScanPoints);//store all the vscanpoints and its data

	fuseFilterDetections(vScanPoints);//filter and store fused objects

}

#if _DEBUG
static constexpr CvScalar COLOR_CYAN = {255, 255, 0};

static void showRects(IplImage *image, int object_num, const std::vector<int>& corner_point)
{
	for(int i = 0; i < object_num; i++) {
		CvPoint p1 = cvPoint(corner_point[0+i*4], corner_point[1+i*4]);
		CvPoint p2 = cvPoint(corner_point[0+i*4] + corner_point[2+i*4], corner_point[1+i*4] + corner_point[3+i*4]);
		cvRectangle(image, p1, p2, COLOR_CYAN, 3);
	}
}
#endif

void setDetectedObjects(const cv_tracker::image_obj& detected_objects)
{
	objectsStored = false;
	obj_type = detected_objects.type;
	g_corner_points.resize(4*detected_objects.obj.size());
	g_scores.resize(detected_objects.obj.size());

	g_objects_num = detected_objects.obj.size();
	for (int i = 0 ;i < g_objects_num; i++) {
		g_corner_points[0+i*4] = detected_objects.obj.at(i).x;
		g_corner_points[1+i*4] = detected_objects.obj.at(i).y;
		g_corner_points[2+i*4] = detected_objects.obj.at(i).width;
		g_corner_points[3+i*4] = detected_objects.obj.at(i).height;
		g_scores[i]            = detected_objects.obj.at(i).score;
	}
	objectsStored = true;
}

/*void setScanImage(const scan2image::ScanImage& scan_image)
{
#if _DEBUG
	if(image == nullptr){
		return;
	}
#endif
	//
	// Assign distance and intensity to scan_image
	//
	for(int i = 0; i < (int)scan_image.distance.size(); i++) {
		int height = (int)(i % IMAGE_HEIGHT);
		int width = (int)(i / IMAGE_HEIGHT);
		g_scan_image.distance[width][height] = scan_image.distance.at(i); //unit of length is centimeter
		g_scan_image.intensity[width][height] = scan_image.intensity.at(i);
	}
	g_scan_image.max_y = scan_image.max_y;
	g_scan_image.min_y = scan_image.min_y;
}*/

void setPointsImage(const points2image::PointsImage& points_image)
{
#if _DEBUG
	if(image == nullptr){
		return;
	}
#endif
	points_msg = points_image;//store vscan pointcloud
	pointsStored = false;

	/*
	 * Reset 2D vector
	 */
	for (auto& distance : g_scan_image.distance) {
	    distance.clear();
	}
	for (auto& intensity : g_scan_image.intensity) {
	    intensity.clear();
	}
	g_scan_image.distance.clear();
	g_scan_image.intensity.clear();

	g_scan_image.distance.resize(points_msg.image_width);
	g_scan_image.intensity.resize(points_msg.image_width);
	for (auto i=0; i<points_msg.image_width; i++) {
		g_scan_image.distance[i].resize(points_msg.image_height);
		g_scan_image.intensity[i].resize(points_msg.image_height);
	}

	/*
	* Assign distance and intensity to scan_image
	*/
	for(int i = 0; i < (int)points_image.distance.size(); i++) {
		int height = (int)(i / points_msg.image_width);
		int width = (int)(i % points_msg.image_width);
		if (height < points_msg.image_height && width < points_msg.image_width) {
			g_scan_image.distance[width][height] = points_image.distance.at(i); //unit of length is centimeter
			g_scan_image.intensity[width][height] = points_image.intensity.at(i);
		}
	}
	g_scan_image.max_y = points_image.max_y;
	g_scan_image.min_y = points_image.min_y;
	pointsStored=true;
}

void calcDistance()
{
	g_distances.clear();
	for(int i = 0; i < g_objects_num; i++)
	{
		float obstacle_distance = NO_DATA;
		int search_scope_max_y;
		int search_scope_min_y;

		if (g_scan_image.max_y > g_corner_points[1+i*4] + g_corner_points[3+i*4]) {
			search_scope_max_y = g_corner_points[1+i*4] + g_corner_points[3+i*4];
		} else {
			search_scope_max_y = g_scan_image.max_y;
		}

		if (g_scan_image.min_y < g_corner_points[1+i*4]) {
			search_scope_min_y = g_corner_points[1+i*4];
		} else {
			search_scope_min_y = g_scan_image.min_y;
		}

		std::vector<float> distance_candidates;
		for(int j = g_corner_points[0+i*4]; j <= g_corner_points[0+i*4] + g_corner_points[2+i*4]; j++) {
		    for(int k = search_scope_min_y; k <= search_scope_max_y; k++) {
			if(g_scan_image.distance[j][k] != NO_DATA) {
			    distance_candidates.push_back(g_scan_image.distance[j][k]);
			}
		    }
		}

                /* calculate mode (most common) value in candidates */
                obstacle_distance = getMode(distance_candidates);

		g_distances.push_back(obstacle_distance); //unit of length is centimeter
#if _DEBUG //debug
		if(obstacle_distance != NO_DATA) {
			printf("%f\n", obstacle_distance);
		} else {
			printf("no data\n");
		}
		char distance_string[32];
		CvFont dfont;
		float hscale      = 1.0f;
		float vscale      = 1.0f;
		float italicscale = 0.0f;
		int  thickness    = 2;

		/*
		 * Plot distances on an image
		 */
		if(obstacle_distance != NO_DATA) {
			cvInitFont (&dfont, CV_FONT_HERSHEY_SIMPLEX , hscale, vscale, italicscale, thickness, CV_AA);
			sprintf(distance_string, "%.2f m", obstacle_distance / 100);
			cvPutText(image, distance_string, cvPoint(g_corner_points[0+i*4] , g_corner_points[1+i*4] + g_corner_points[3+i*4]), &dfont, CV_RGB(255, 0, 0));
		} else {
			cvInitFont (&dfont, CV_FONT_HERSHEY_SIMPLEX , hscale, vscale, italicscale, thickness, CV_AA);
			sprintf(distance_string, "No data");
			cvPutText(image, distance_string, cvPoint(g_corner_points[0+i*4] , g_corner_points[1+i*4] + g_corner_points[3+i*4]), &dfont, CV_RGB(255, 0, 0));
		}
#endif
	}

#if _DEBUG //debug
	/*
	 * Plot depth points on an image
	 */
	CvPoint pt;
	for(int i = 0; i < points_msg.image_height; i++) {
		for(int j = 0; j < points_msg.image_width; j++) {
			if (g_scan_image.distance[j][i] != 0.0) {
				pt.x = j;
				pt.y = i;
				cvCircle(image, pt, 2, CV_RGB (0, 255, 0), CV_FILLED, 8, 0);
			}
		}
	}

	showRects(image, g_objects_num, g_corner_points);

	/*
	 * Show image
	 */
	cvShowImage(window_name, image);
	cvWaitKey(2);
#endif
}

std::vector<float> getMinHeights()
{
	return filtered_min_heights;
}

std::vector<float> getMaxHeights()
{
	return filtered_max_heights;
}

std::vector<cv_tracker::image_rect_ranged> getObjectsRectRanged()
{
	std::vector<cv_tracker::image_rect_ranged> fused_objects;
	for (int i=0; i<filtered_objects_num; i++)
	{
		int base = i * 4;
		cv_tracker::image_rect_ranged obj_ranged;
		obj_ranged.rect.x      = filtered_corner_points.at(base);
		obj_ranged.rect.y      = filtered_corner_points.at(base + 1);
		obj_ranged.rect.width  = filtered_corner_points.at(base + 2);
		obj_ranged.rect.height = filtered_corner_points.at(base + 3);
		obj_ranged.rect.score  = filtered_scores.at(i);
		obj_ranged.range       = filtered_distances.at(i);
		obj_ranged.min_height  = filtered_min_heights.at(i);
		obj_ranged.max_height  = filtered_max_heights.at(i);
		fused_objects.push_back(obj_ranged);
	}

	return fused_objects;
}

std::string getObjectsType()
{
	return obj_type;
}

#if _DEBUG //debug
void imageCallback(const sensor_msgs::Image& image_source)
{
	cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
	temp = cv_image->image;
	image = &temp;
}
#endif

void init()
{
	g_objects_num = 0;
	filtered_objects_num = 0;
#if _DEBUG //debug
	cvNamedWindow(window_name, 2);
	image = nullptr;
#endif
}

void destroy()
{
#if _DEBUG //debug
	cvDestroyWindow(window_name);
#endif
}
