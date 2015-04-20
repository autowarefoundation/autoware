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
#if defined(ROS) // AXE
#include "dpm/ImageObjects.h"
#include "for_use_GPU.h"
#else
#include "sensors_fusion/ObstaclePosition.h"
#endif
#include <boost/array.hpp>

#include <runtime_manager/ConfigCarDpm.h>
#include <runtime_manager/ConfigPedestrianDpm.h>

#include <dpm_gpu.hpp>

#if !defined(ROS) // AXE
char ldata_name[]="2010_2_3.txt";
char WINDOW_NAME[] = "CAR_TRACK";
#endif
double ratio = 1;	//resize ratio
MODEL *MO;
double overlap = 0.4;    // threshold overlap parameter (default :0.4)
double thresh = -0.5;    // threshold score of detection (default :0.0)
#if !defined(ROS) // AXE
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
#if !defined(ROS)
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
    sensors_fusion::ObstaclePosition image_and_obstacle_position_msg;
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
//    image_and_obstacle_position_msg.image_raw = image_source;
    image_and_obstacle_position_msg.header = image_source.header;
    image_and_obstacle_position_msg.car_num = CUR->num;
    image_and_obstacle_position_msg.corner_point = corner_point_array;
    image_and_obstacle_position_msg.car_type = car_type_array;

    /* publish data */
    image_and_obstacle_position.publish(image_and_obstacle_position_msg);

    /* end processing time */
    clock_gettime(CLOCK_REALTIME, &s_end);
    printf("time:%f msec\n", get_processing_time(start, end));
    fprintf(stderr,"%f, %f\n", get_processing_time(start, end), get_processing_time(s_start, s_end));
    /* save image */
    sprintf(buf, "%d.png", k);
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
#endif

#if defined(ROS)

std::string com_name;
std::string root_name;
std::string part_name;

void dpm_gpu_init_cuda(const std::string& cubin_path)
{
	init_cuda_with_cubin(cubin_path.c_str());
}

void dpm_gpu_load_models(const std::string& com_csv,
			 const std::string& root_csv,
			 const std::string& part_csv)
{
	com_name = com_csv;
	root_name = root_csv;
	part_name = part_csv;
	MO = load_model(ratio);
}

void dpm_gpu_cleanup_cuda()
{
	clean_cuda();
	free_model(MO);
}

DPMGPUResult dpm_gpu_detect_objects(IplImage *image, double threshold,
				    double overlap, int lambda, int num_cells)
{
	MO->MI->interval = lambda;
	MO->MI->sbin     = num_cells;

	int detected_objects;
	FLOAT *ac_score = ini_ac_score(image);
	RESULT *cars = car_detection(image, MO, threshold,
				     &detected_objects, ac_score,
				     overlap);	//detect car
	s_free(ac_score);

	DPMGPUResult result;
	result.num = cars->num;
	for (int i = 0; i < cars->num; ++i) {
		result.type.push_back(cars->type[i]);
	}

	for (int i = 0; i < cars->num; ++i) {
		int base = i * 4;
		int *data = &(cars->OR_point[base]);

		result.corner_points.push_back(data[0]);
		result.corner_points.push_back(data[1]);
		result.corner_points.push_back(data[2] - data[0]);
		result.corner_points.push_back(data[3] - data[1]);
	}

	s_free(cars->point);
	s_free(cars->type);
	s_free(cars->scale);
	s_free(cars->score);
	s_free(cars->IM);
	return result;
}
#else
int main(int argc, char **argv)
{
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

	//get car-detector model
	MO=load_model(ratio);

	//create lesult information
	RESULT *LR = create_result(0);

	//get file size and current file position
	get_f_size(fp,&curpos,&fsize);
	skip_data_2(fp,1,&ss);

  ros::init(argc, argv, "obstacle_detection");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/image_raw", 1, obstacle_detectionCallback);
  image_and_obstacle_position = n.advertise<sensors_fusion::ObstaclePosition>("obstacle_position", 1);

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

  ros::spin();
  clean_cuda();

  //release car-detector-model
  free_model(MO);

  //release detection result
  release_result(LR);

  //close and release file information
  fclose(fp);			//close laser_file
  //cvReleaseCapture(&capt);

  s_free(FPASS);		//release laser_file pass
  return 0;
}
// %EndTag(FULLTEXT)%
#endif
