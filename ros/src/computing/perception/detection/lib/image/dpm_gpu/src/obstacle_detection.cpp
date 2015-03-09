/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#if !defined(ROS)
#ifdef _DEBUG
// case of Debug mode
#pragma comment(lib,"cv200d.lib")
#pragma comment(lib,"cxcore200d.lib")
#pragma comment(lib,"cvaux200d.lib")
#pragma comment(lib,"highgui200d.lib")
#else
// case of Release mode
#pragma comment(lib,"cv200.lib")
#pragma comment(lib,"cxcore200.lib")
#pragma comment(lib,"cvaux200.lib")
#pragma comment(lib,"highgui200.lib")
#endif
#endif
//C++ library
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/resource.h>
#include <math.h>

//ORIGINAL header files
#include "Laser_func.h"
#include "car_det_func.h"
#include "Common.h"
#include "Depth_points_func.h"

#include "std_msgs/String.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>//C++ library
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#if 1 // AXE
#include "dpm/ImageObjects.h"
#include "for_use_GPU.h"
#else
#include "sensors_fusion/ObstaclePosition.h"
#endif
#include <boost/array.hpp>

#include <runtime_manager/ConfigCarDpm.h>
#include <runtime_manager/ConfigPedestrianDpm.h>

#if 1 // AXE
#else
char ldata_name[]="2010_2_3.txt";
char WINDOW_NAME[] = "CAR_TRACK";
#endif
double ratio = 1;	//resize ratio
MODEL *MO;
double overlap = 0.4;    // threshold overlap parameter (default :0.4)
double thresh = -0.5;    // threshold score of detection (default :0.0)
#if 1 // AXE
static ros::Publisher image_objects;
#else
ros::Publisher image_and_obstacle_position;
#endif

struct timeval tv_memcpy_start, tv_memcpy_end;
float time_memcpy;
struct timeval tv_kernel_start, tv_kernel_end;
float time_kernel;

int device_num;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
double get_processing_time(struct timespec start, struct timespec end)
{
    long sec, nsec;
    sec =  end.tv_sec - start.tv_sec;
    nsec =  end.tv_nsec - start.tv_nsec;
    if(nsec < 0) {
        sec--;
        nsec += 1000000000L;
    }

    return (double)sec * 1000 + (double)nsec/1000000;
}
int k=1;
void obstacle_detectionCallback(const sensor_msgs::Image& image_source)
{
    char buf[32];
    struct timespec start, end, s_start, s_end;
    clock_gettime(CLOCK_REALTIME, &s_start);
    cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
    int D_NUMS=0;    //# of detected-Object
    IplImage temp = cv_image->image;
    IplImage *IM_D= &temp;
    /* start processing time */
    clock_gettime(CLOCK_REALTIME, &start);
    IplImage *R_I = IM_D;
    FLOAT *A_SCORE = ini_ac_score(R_I);	//alloc accum
    RESULT *CUR = car_detection(IM_D,MO,thresh,&D_NUMS,A_SCORE,overlap);	//detect car
    clock_gettime(CLOCK_REALTIME, &end);
#if 1 // AXE
    dpm::ImageObjects image_objects_msg;
#else
    sensors_fusion::ObstaclePosition image_and_obstacle_position_msg;
#endif
    std::vector<int> corner_point_array(CUR->num * 4,0);
    std::vector<int> car_type_array(CUR->num,0);
    int i;

    for (i = 0 ;i < CUR->num; i++) {
        car_type_array[i] = CUR->type[i];
        corner_point_array[0+i*4] = *(CUR->OR_point + (i*4));
        corner_point_array[1+i*4] = *(CUR->OR_point + (1+i*4));
        corner_point_array[2+i*4] = *(CUR->OR_point + (2+i*4)) - *(CUR->OR_point + (i*4));//updated
        corner_point_array[3+i*4] = *(CUR->OR_point + (3+i*4)) - *(CUR->OR_point + (1+i*4));//updated
    }

    /* store data which will be published */
#if 1 // AXE
    image_objects_msg.header = image_source.header;
    image_objects_msg.car_num = CUR->num;
    image_objects_msg.corner_point = corner_point_array;
    image_objects_msg.car_type = car_type_array;
    image_objects_msg.header.stamp = image_source.header.stamp;
#else
//    image_and_obstacle_position_msg.image_raw = image_source;
    image_and_obstacle_position_msg.header = image_source.header;
    image_and_obstacle_position_msg.car_num = CUR->num;
    image_and_obstacle_position_msg.corner_point = corner_point_array;
    image_and_obstacle_position_msg.car_type = car_type_array;
#endif
    /* publish data */
#if 1 // AXE
    image_objects.publish(image_objects_msg);
#else
    image_and_obstacle_position.publish(image_and_obstacle_position_msg);
#endif

    /* end processing time */
    clock_gettime(CLOCK_REALTIME, &s_end);
    printf("time:%f msec\n", get_processing_time(start, end));
    fprintf(stderr,"%f, %f\n", get_processing_time(start, end), get_processing_time(s_start, s_end));
    /* save image */
#if 1 // AXE
    sprintf(buf, "num_png/%d.png", k);
#else
    sprintf(buf, "%d.png", k);
#endif
    cvSaveImage(buf, IM_D);
    k++;

    s_free(CUR);
    s_free(A_SCORE);
}

void car_configParamCallback(const runtime_manager::ConfigCarDpm::ConstPtr& param)
{
  float score_threshold = param->score_threshold;
  float group_threshold = param->group_threshold;
  int lambda = param->Lambda;
  int numCells = param->num_cells;

  thresh = score_threshold;
  overlap = group_threshold;
  MO->MI->interval = lambda;
  MO->MI->sbin = numCells;
}

void pedestrian_configParamCallback(const runtime_manager::ConfigPedestrianDpm::ConstPtr& param)
{
  float score_threshold = param->score_threshold;
  float group_threshold = param->group_threshold;
  int lambda = param->Lambda;
  int numCells = param->num_cells;

  thresh = score_threshold;
  overlap = group_threshold;
  MO->MI->interval = lambda;
  MO->MI->sbin = numCells;
}
// %EndTag(CALLBACK)%

#if 1 // AXE
#define XSTR(x) #x
#define STR(x) XSTR(x)

std::string com_name;
std::string root_name;
std::string part_name;

int dpm_ttic_main(int argc, char* argv[], const char *cubin_path,
		  const std::string& detection_type)
#else
int main(int argc, char **argv)
#endif
{
#if 1 // AXE
	std::string published_topic;
	if (detection_type == "car") {
		published_topic = "car_pixel_xy";
		com_name = STR(DPM_GPU_ROOT) "car_comp.csv";
		root_name = STR(DPM_GPU_ROOT) "car_root.csv";
		part_name = STR(DPM_GPU_ROOT) "car_part.csv";
	} else if (detection_type == "pedestrian") {
		published_topic = "pedestrian_pixel_xy";
		com_name = STR(DPM_GPU_ROOT)  "person_comp.csv";
		root_name = STR(DPM_GPU_ROOT) "person_root.csv";
		part_name = STR(DPM_GPU_ROOT) "person_part.csv";
	} else {
		std::cerr << "Invalid detection type: "
			  << detection_type
			  << std::endl;
	}

	init_cuda_with_cubin(cubin_path);
#else
	FILE* fp;					//file pointer
	CvCapture *capt;			//movie file capture
	fpos_t curpos,fsize;		//file size
	int ss=0;					//image number
	int fnum=0;					//frame number
	bool FLAG = true;			//file end FLAG

	int TH_length = 80;			//tracking length threshold

	//get laser_save data pass (file should be in savedata folder)
	char *FPASS= get_file_pass(ldata_name);

	int i;
	printf("FPASS:");
	for (i=0;*(FPASS+i) != '\0';i++){
		printf("%c",*(FPASS+i));
	}
	printf("\n");

	//open save file
	if ((fp = fopen(FPASS,"rb")) == NULL) { printf("file open error!!\n"); exit(EXIT_FAILURE);}
#endif

	//get car-detector model
	MO=load_model(ratio);

	//create lesult information
	RESULT *LR = create_result(0);

#if 1 // AXE
#else
	//get file size and current file position
	get_f_size(fp,&curpos,&fsize);
	skip_data_2(fp,1,&ss);   

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "obstacle_detection");
#endif

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;


  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  
subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("/image_raw", 1, obstacle_detectionCallback);
#if 1 // AXE
  image_objects = n.advertise<dpm::ImageObjects>(published_topic, 1);
#else
  image_and_obstacle_position = n.advertise<sensors_fusion::ObstaclePosition>("obstacle_position", 1);
#endif

  /* configuration parameter subscribing */
  ros::Subscriber configParam_sub;
  if (detection_type == "car") {
    configParam_sub = n.subscribe("/config/car_dpm", 1, car_configParamCallback);
  } else if (detection_type == "pedestrian") {
    configParam_sub = n.subscribe("/config/pedestrian_dpm", 1, pedestrian_configParamCallback);
  } else {
    std::cerr << "Invalid detection type: "
              <<detection_type
              << std::endl;
  }

//  image_transport::ImageTransport it(n);
//  image_transport::Subscriber sub = it.subscribe("imageraw", 1, chatterCallback);
// %EndTag(SUBSCRIBER)%

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  ros::spin();
  clean_cuda();
// %EndTag(SPIN)%

  //release car-detector-model
  free_model(MO);

  //release detection result
  release_result(LR);

#if 1 // AXE
#else
  //close and release file information
  fclose(fp);			//close laser_file
  //cvReleaseCapture(&capt);

  s_free(FPASS);		//release laser_file pass
#endif
  return 0;
}
// %EndTag(FULLTEXT)%
