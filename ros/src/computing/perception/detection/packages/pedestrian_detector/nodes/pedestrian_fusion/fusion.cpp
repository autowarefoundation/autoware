#include "pedestrian_fusion_func.h"

#if _DEBUG //debug
char window_name[] = "CAR_TRACK";
//for imageCallback
cv_bridge::CvImagePtr cv_image;
IplImage temp;
IplImage *image;
std::vector<int> depth_point_x_on_image;
std::vector<int> depth_point_y_on_image;
double ratio = 1;	//resize ratio
#endif

/*for obstacle_detectionCallback */
std::vector<int> g_corner_point;
int g_object_num;
/*for distance_measurementCallback */
Scan_image g_scan_image;
/* for common Callback */
std::vector<float> g_distance;


void showRects(IplImage *Image,int object_num, std::vector<int> corner_point, double ratio);

void setImageObjects(const dpm::ImageObjects& image_objects)
{
    int i;

    g_corner_point.resize(image_objects.corner_point.size());

    g_object_num = image_objects.car_num;
    for (i = 0 ;i < image_objects.car_num; i++) {
        g_corner_point[0+i*4] = image_objects.corner_point[0+i*4];
        g_corner_point[1+i*4] = image_objects.corner_point[1+i*4];
        g_corner_point[2+i*4] = image_objects.corner_point[2+i*4];
        g_corner_point[3+i*4] = image_objects.corner_point[3+i*4];
    }
}

void setScanImage(const scan_to_image::ScanImage& scan_image)
{
    int i;
#if _DEBUG
    if(image == NULL){
      return;
    }
#endif

    /*
     * Initialize. Assign 0 to scan_image.distance[][]
     */
    {
        int height, width;
        for(i = 0; i < (int)scan_image.distance.size(); i++) {
            height = (int)(i % IMAGE_HEIGHT);
            width = (int)(i / IMAGE_HEIGHT);
            g_scan_image.distance[width][height] = scan_image.distance.at(i);
            g_scan_image.intensity[width][height] = scan_image.intensity.at(i);
        }
    }
    g_scan_image.max_y = scan_image.max_y;
    g_scan_image.min_y = scan_image.min_y;
}

void setPointsImage(const points_to_image::PointsImage& points_image)
{
    int i;
#if _DEBUG
    if(image == NULL){
      return;
    }
#endif

    /*
     * Initialize. Assign 0 to scan_image.distance[][]
     */
    {
        int height, width;
        for(i = 0; i < (int)points_image.distance.size(); i++) {
            height = (int)(i % IMAGE_HEIGHT);
            width = (int)(i / IMAGE_HEIGHT);
            if (height < IMAGE_HEIGHT && width < IMAGE_WIDTH) {
                g_scan_image.distance[width][height] = points_image.distance.at(i)/100;
                g_scan_image.intensity[width][height] = points_image.intensity.at(i);
            }
        }
    }
    g_scan_image.max_y = points_image.max_y;
    g_scan_image.min_y = points_image.min_y;
}


void calcDistance()
{
    int i, j, k;
    g_distance.clear();
	for(i = 0; i < g_object_num; i++) {
        float obstacle_distance = NO_DATA;
        int search_scope_max_y;
        int search_scope_min_y;

        if (g_scan_image.max_y > g_corner_point[1+i*4] + g_corner_point[3+i*4]) {
            search_scope_max_y = g_corner_point[1+i*4] + g_corner_point[3+i*4];
        } else {
            search_scope_max_y = g_scan_image.max_y;
        }

        if (g_scan_image.min_y < g_corner_point[1+i*4]) {
            search_scope_min_y = g_corner_point[1+i*4];
        } else {
            search_scope_min_y = g_scan_image.min_y;
        }

        /*
         * Search shortest distance in obstacle boxes
         */
        for(j = g_corner_point[0+i*4]; j <= g_corner_point[0+i*4] + g_corner_point[2+i*4]; j++) {
            for(k = search_scope_min_y; k <= search_scope_max_y; k++) {
                if(g_scan_image.distance[j][k] != NO_DATA) {
                    if(g_scan_image.distance[j][k] < obstacle_distance || obstacle_distance == NO_DATA){
                        obstacle_distance = g_scan_image.distance[j][k];
                    }
                }
            }
        }

        g_distance.push_back(obstacle_distance);
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
            sprintf(distance_string, "%.2f m", obstacle_distance);
            cvPutText(image, distance_string, cvPoint(g_corner_point[0+i*4] , g_corner_point[1+i*4] + g_corner_point[3+i*4]), &dfont, CV_RGB(255, 0, 0));
        } else {
            cvInitFont (&dfont, CV_FONT_HERSHEY_SIMPLEX , hscale, vscale, italicscale, thickness, CV_AA);
            sprintf(distance_string, "No data");
            cvPutText(image, distance_string, cvPoint(g_corner_point[0+i*4] , g_corner_point[1+i*4] + g_corner_point[3+i*4]), &dfont, CV_RGB(255, 0, 0));
        }
#endif
     }

#if _DEBUG //debug
    /*
     * Plot depth points on an image
     */
    CvPoint pt;
    for(i = 0; i < IMAGE_HEIGHT; i++) {
          for(j = 0; j < IMAGE_WIDTH; j++) {
              if (g_scan_image.distance[j][i] != 0.0) {
                  pt.x = j;
                  pt.y = i;
                  cvCircle(image, pt, 2, CV_RGB (0, 255, 0), CV_FILLED, 8, 0);
              }
          }
    }

    showRects(image, g_object_num, g_corner_point, ratio);

    /*
     * Show image
     */
    cvShowImage(window_name, image);
    cvWaitKey(2);
#endif
}

int getObjectNum()
{
    return g_object_num;
}

std::vector<int> getCornerPoint()
{
    return g_corner_point;
}

std::vector<float> getDistance()
{
    return g_distance;
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
    g_object_num = 0;
#if _DEBUG //debug
    cvNamedWindow(window_name, 2);
    image = NULL;
#endif
}

void destroy()
{
#if _DEBUG //debug
  cvDestroyWindow(window_name);
#endif
}

void showRects(IplImage *Image,int object_num, std::vector<int> corner_point, double ratio)
{
	for(int i = 0; i < object_num; i++)
	{
		CvScalar col = cvScalar(255.0,255.0,0.0);
		CvPoint p1=cvPoint(corner_point[0+i*4], corner_point[1+i*4]);
		CvPoint p2=cvPoint(corner_point[0+i*4] + corner_point[2+i*4], corner_point[1+i*4] + corner_point[3+i*4]);
		cvRectangle(Image,p1,p2,col,3);
	}
}
