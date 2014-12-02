#include <iostream>
#if defined(ROS) // AXE
#include "objdetect.hpp"
#else
#include "objdetect/objdetect.hpp"
#endif
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"

#include <stdio.h>

#if defined(WIN32) || defined(_WIN32)
#include <io.h>
#else
#include <dirent.h>
#endif

#ifdef HAVE_CVCONFIG_H
#include <cvconfig.h>
#endif

#ifdef HAVE_TBB
#include "tbb/task_scheduler_init.h"
#endif

#if defined(ROS) // AXE
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "dpm/ImageObjects.h"
#endif

using namespace std;
using namespace cv;

ros::Publisher image_objects;

#if !defined(ROS)
static void help()
{
    cout << "This program demonstrated the use of the latentSVM detector." << endl <<
            "It reads in a trained object models and then uses them to detect the objects in an images." << endl <<
             endl <<
            "Call:" << endl <<
            "./latentsvm_multidetect <imagesFolder> <modelsFolder> [<overlapThreshold>][<threadsNumber>]" << endl <<
            "<overlapThreshold> - threshold for the non-maximum suppression algorithm." << endl <<
            "Example of <modelsFolder> is opencv_extra/testdata/cv/latentsvmdetector/models_VOC2007" << endl <<
             endl <<
            "Keys:" << endl <<
            "'n' - to go to the next image;" << endl <<
            "'esc' - to quit." << endl <<
            endl;
}
#endif

static void detectAndDrawObjects( Mat& image, LatentSvmDetector& detector,
			const vector<Scalar>& colors, float overlapThreshold, int numThreads, FILE *f )
{
    vector<LatentSvmDetector::ObjectDetection> detections;

    TickMeter tm;
    tm.start();
    cout << endl << "START" <<endl;
    detector.detect( image, detections, overlapThreshold, numThreads);
    tm.stop();

    cout << "END Total time = " << tm.getTimeSec() << " sec" << endl;

    const vector<string> classNames = detector.getClassNames();
    CV_Assert( colors.size() == classNames.size() );

#if defined(ROS) // AXE
    int num = detections.size();
    std::vector<int> corner_point_array(num * 4,0);
    std::vector<int> car_type_array(num, 0);
#endif

    for( size_t i = 0; i < detections.size(); i++ )
    {
        const LatentSvmDetector::ObjectDetection& od = detections[i];
        //od.rect contains x,y, width, height
        rectangle( image, od.rect, colors[od.classID], 3 );
        //write to output file
        if (NULL != f)
        {
        	//KITTI layout: frame, tracklet(-1), type (Car), truncation(-1), occlusion(-1), alpha (angle 1), bbox(left), bbox (top), bbox (right), bbox (bottom), -1, -1, -1, -1000, -1000, -10, SCORE

        	fprintf(f, "%d -1 Car -1 -1 1 %d %d %d %d -1 -1 -1 -1000 -1000 -1000 -10 %.4f\n",
        				0, od.rect.x, od.rect.y, od.rect.x+od.rect.width, od.rect.y + od.rect.height, od.score );
        }
#if defined(ROS) // AXE
        car_type_array[i] = od.classID; // ?
        corner_point_array[0+i*4] = od.rect.x;
        corner_point_array[1+i*4] = od.rect.y;
        corner_point_array[2+i*4] = od.rect.x + od.rect.width;
        corner_point_array[3+i*4] = od.rect.y + od.rect.height;
#endif
    }
    // put text over the all rectangles
    for( size_t i = 0; i < detections.size(); i++ )
    {
        const LatentSvmDetector::ObjectDetection& od = detections[i];
        putText( image, classNames[od.classID], Point(od.rect.x+4,od.rect.y+13), FONT_HERSHEY_SIMPLEX, 0.55, colors[od.classID], 2 );
    }

#if defined(ROS) // AXE
    dpm::ImageObjects image_objects_msg;
    image_objects_msg.car_num = num;
    image_objects_msg.corner_point = corner_point_array;
    image_objects_msg.car_type = car_type_array;

    image_objects.publish(image_objects_msg);
#endif

}

#if defined(ROS) // AXE

LatentSvmDetector* pDetector = NULL;
vector<Scalar>* pColors = NULL;
float* pOverlapThreshold = NULL;
int* pNumThreads = NULL;
FILE** pF = NULL;

void image_objectsCallback(const sensor_msgs::Image& image_source)
{
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
  Mat image = cv_image->image;

  if(pDetector && pColors && pOverlapThreshold && pNumThreads && pF){
    detectAndDrawObjects( image, *pDetector, *pColors, *pOverlapThreshold, *pNumThreads, *pF );
  }

  imshow( "result", image );
  cvWaitKey(2);
}
#endif

#if defined(ROS)
#define XSTR(x) #x
#define STR(x) XSTR(x)
#endif

// XXX FIXME
#if defined(CAR_DETECTOR)
#define THRESHOLD_PARAM "/car_detector/threshold"
#define THREADS_PARAM "/car_detector/threads"
#elif defined(PEDESTRIAN_DETECTOR)
#define THRESHOLD_PARAM "/pedestrian_detector/threshold"
#define THREADS_PARAM "/pedestrian_detector/threads"
#else
#define THRESHOLD_PARAM ""
#define THREADS_PARAM ""
#endif

#if defined(ROS) // AXE
int dpm_ocv_main(int argc, char* argv[], const std::string& detection_type)
#else
int main(int argc, char* argv[])
#endif
{
#if defined(ROS) // AXE
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/image_hog", 1, image_objectsCallback);

    std::string published_node;
    std::string model_file(STR(OPENCV_MODEL_DIR));
    if (detection_type == "car") {
        published_node = "car_pos_xy";
	model_file.append("car.xml");
    } else if (detection_type == "pedestrian") {
	published_node = "pedestrian_pos_xy";
	model_file.append("person.xml");
    } else {
        std::cerr << "Invalid detection type: " << detection_type
                  << std::endl;
        std::exit(1);
    }
    image_objects = n.advertise<dpm::ImageObjects>("image_objects", 1);
#else
    help();
#endif

    FILE *f = fopen("output.txt", "w");
    if (f == NULL)
    {
        cout << "Error opening detection output file"<<endl;
        fclose(f);
        exit(-1);
    }

    string images_folder, models_folder;
    float overlapThreshold = 0.5f;
    int numThreads = 6;
#if defined(ROS) // AXE
    if(n.hasParam(THRESHOLD_PARAM)){
      double threshold = 0.5;
      n.getParam(THRESHOLD_PARAM, threshold);
      overlapThreshold = (float)threshold;
    }
    if(n.hasParam(THREADS_PARAM)){
      n.getParam(THREADS_PARAM, numThreads);
    }
#else
    if( argc < 3 )
    {
        cout << "Must provide imagename and modelname, ex. $exe /image.jpg /model.xml" << endl;
        fclose(f);
        exit(-1);

    }
    if (argc>3)
    {
    	overlapThreshold=atof(argv[3]);
    	numThreads=atoi(argv[4]);
    }
#endif

    vector<string> images_filenames, models_filenames;
    //readDirectory( images_folder, images_filenames );
    //readDirectory( models_folder, models_filenames );

    images_filenames.clear();models_filenames.clear();
#if ROS //AXE
    models_filenames.push_back( model_file );
#else
    images_filenames.push_back( std::string(argv[1]) );
    models_filenames.push_back( std::string(argv[2]) );
#endif

    LatentSvmDetector detector( models_filenames );
    if( detector.empty() )
    {
        cout << "Models can't be loaded: " << model_file << endl;
        fclose(f);
        exit(-1);
    }

    const vector<string>& classNames = detector.getClassNames();
    cout << "Loaded " << classNames.size() << " models:" << endl;
    for( size_t i = 0; i < classNames.size(); i++ )
    {
        cout << i << ") " << classNames[i] << "; ";
    }
    cout << endl;

    cout << "overlapThreshold = " << overlapThreshold << endl;

    vector<Scalar> colors;
    generateColors( colors, detector.getClassNames().size() );

#if defined(ROS) // AXE
#else
    for( size_t i = 0; i < images_filenames.size(); i++ )
    {
        Mat image = imread( images_filenames[i] );
        if( image.empty() )  continue;

        cout << "Process image " << images_filenames[i] << endl;
        detectAndDrawObjects( image, detector, colors, overlapThreshold, numThreads, f );

        imshow( "result", image );

        for(;;)
        {
            int c = waitKey();
            if( (char)c == 'n')
                break;
            else if( (char)c == '\x1b' )
                exit(0);
        }
    }
#endif

#if defined(ROS) // AXE
    pF = &f;
    pOverlapThreshold = &overlapThreshold;
    pNumThreads = &numThreads;
    pDetector = &detector;
    pColors = &colors;

    ros::spin();
#endif

    fclose(f);

    return 0;
}
