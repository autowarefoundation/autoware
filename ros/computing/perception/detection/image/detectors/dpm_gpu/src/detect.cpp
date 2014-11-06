///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////detect.cpp   Detect car from an image  ///////////////////////////////////////////////////////////////////////


//OpenCV library
//#include "cv.h"			
//#include "cxcore.h"
//#include "highgui.h"	
/*
#include "C:\OpenCV2.0\include\opencv\cv.h"
#include "C:\OpenCV2.0\include\opencv\highgui.h"
#include "C:\OpenCV2.0\include\opencv\cxcore.h"
*/
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#ifdef _DEBUG
    //Debugモードの場合
    #pragma comment(lib,"cv200d.lib") 
    #pragma comment(lib,"cxcore200d.lib") 
    #pragma comment(lib,"cvaux200d.lib") 
    #pragma comment(lib,"highgui200d.lib") 
#else
    //Releaseモードの場合
    #pragma comment(lib,"cv200.lib") 
    #pragma comment(lib,"cxcore200.lib") 
    #pragma comment(lib,"cvaux200.lib") 
    #pragma comment(lib,"highgui200.lib") 
#endif
//C++ library
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <opencv2/legacy/legacy.hpp>

//ORIGINAL header files
#include "MODEL_info.h"		//File information
#include "detect_func.h"	//functions
#include "Common.h"

#include "switch_float.h"
#include "switch_release.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//definiton of functions//

//resize Image (IplImage)
IplImage *ipl_resize(IplImage *IM,FLOAT ratio);

//create and resize Iplimage 
IplImage *ipl_cre_resize(IplImage *IM,int width,int height);

//initialize accumulated score
FLOAT *ini_ac_score(IplImage *IM);

//detect object and return rectangle-box coorinate (extended to main.cpp)
FLOAT *detect(IplImage *IM,MODEL *MO,FLOAT thresh,int *D_NUMS,FLOAT *A_SCORE);

//detect car-boundary-boxes
RESULT *car_detection(IplImage *IM,MODEL *MO,FLOAT thresh,int *D_NUMS,FLOAT *A_SCORE,FLOAT overlap);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////




//resize Image (IplImage)
IplImage *ipl_resize(IplImage *IM,FLOAT ratio)
{
	IplImage *R_I;	//Output (Resized Image)

	//parameters 
	const int height = IM->height;
	const int width = IM->width;

	const int UpY = height/10;
	const int NEW_Y = height-UpY-height/10;
	
	const int depth = IM->depth;
	const int nChannels = IM->nChannels;

	//set ROI
	CvRect REC = cvRect(0,UpY,width,NEW_Y);
	cvSetImageROI(IM,REC);			//change ROI of Image 

	if((int)((FLOAT)IM->height*ratio)==IM->height)
	{
		R_I =cvCreateImage(cvSize(width,NEW_Y),depth,nChannels);
		cvCopy(IM,R_I);		//copy
	}
	else
	{
		R_I = cvCreateImage(cvSize((int)((FLOAT)width*ratio),(int)((FLOAT)NEW_Y*ratio)),depth,nChannels);
		cvResize(IM,R_I);	//resize
	}
	cvResetImageROI(IM);
	//printf("ORIGINAL Image size [%d %d]\n",R_I->height,R_I->width);  

	return(R_I);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
IplImage *ipl_cre_resize(IplImage *IM,int width,int height)
{
	IplImage *R_I = cvCreateImage(cvSize(width,height),IM->depth,IM->nChannels);
	cvResize(IM,R_I);	//resize
	return(R_I);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

size_t size_A_SCORE;

//initialize accumulated score
FLOAT *ini_ac_score(IplImage *IM)
{
  int L = IM->height*IM->width;
  
  size_A_SCORE = L*sizeof(FLOAT);
    
  FLOAT *A_SCORE = (FLOAT *)calloc(L,sizeof(FLOAT));
  for(int ii=0;ii<L;ii++) *(A_SCORE+ii)=-100.0;
  return (A_SCORE);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//detect and save detected boxes
FLOAT *detect(IplImage *IM,MODEL *MO,FLOAT thresh,int *D_NUMS,FLOAT *A_SCORE)
{
  /* for measurement */
  struct timeval tv;
  struct timeval tv_ini_scales_start, tv_ini_scales_end;
  float time_ini_scales;
  struct timeval tv_ini_feat_size_start, tv_ini_feat_size_end;
  float time_ini_feat_size;
  struct timeval tv_get_boxes_start, tv_get_boxes_end;
  float time_get_boxes;
  struct timeval tv_calc_f_pyramid_start, tv_calc_f_pyramid_end;
  float time_calc_f_pyramid = 0;
  
  
  //for time measurement
  clock_t t1,t2,t3;
  
  //initialize scale information for hierachical detection
  gettimeofday(&tv_ini_scales_start, NULL);
  FLOAT *scales=ini_scales(MO->MI,IM,IM->width,IM->height);
  gettimeofday(&tv_ini_scales_end, NULL);

  //initialize feature-size matrix
  gettimeofday(&tv_ini_feat_size_start, NULL);
  int *featsize=ini_featsize(MO->MI);
  gettimeofday(&tv_ini_feat_size_end, NULL);

  //calculate feature pyramid
  t1=clock();
  gettimeofday(&tv_calc_f_pyramid_start, NULL);
  FLOAT **feature=calc_f_pyramid(IM,MO->MI,featsize,scales);		
  gettimeofday(&tv_calc_f_pyramid_end, NULL);
  tvsub(&tv_calc_f_pyramid_end, &tv_calc_f_pyramid_start, &tv);

  time_kernel += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
  time_calc_f_pyramid += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;
#ifdef PRINT_INFO
  printf("\n");
  printf("calc_f_pyramid %f[ms]\n", time_calc_f_pyramid);
#endif  // ifdef PRINT_INFO

  t2=clock();

  //detect boundary boxes
  gettimeofday(&tv_get_boxes_start, NULL);
  FLOAT *boxes = get_boxes(feature,scales,featsize,MO,D_NUMS,A_SCORE,thresh);
  gettimeofday(&tv_get_boxes_end, NULL);
  t3=clock();


#if 1
  // tvsub(&tv_ini_scales_end, &tv_ini_scales_start, &tv);
  // time_ini_scales = tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

  // tvsub(&tv_ini_feat_size_end, &tv_ini_feat_size_start, &tv);
  // time_ini_feat_size = tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

  // tvsub(&tv_calc_f_pyramid_end, &tv_calc_f_pyramid_start, &tv);
  // time_calc_f_pyramid = tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

  // tvsub(&tv_get_boxes_end, &tv_get_boxes_start, &tv);
  // time_get_boxes = tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

  // printf("ini_scales : %f\n", time_ini_scales);
  // printf("ini_feat_size : %f\n", time_ini_feat_size);
  //  printf("calc_f_pyramid : %f\n", time_calc_f_pyramid);
  // printf("get_boxes : %f\n", time_get_boxes);

  //  printf("\n");
#endif

  
  s_free(scales);						//release scale-information
  s_free(featsize);					//release feat size information
  free_features(feature,MO->MI);
  
  return boxes;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//detect car-boundary-boxes
RESULT *car_detection(IplImage *IM,MODEL *MO,FLOAT thresh,int *D_NUMS,FLOAT *A_SCORE,FLOAT overlap)
{
  /* for measurement */
  struct timeval tv;
  struct timeval tv_detect_start, tv_detect_end;
  float time_detect;
  struct timeval tv_nms_start, tv_nms_end;
  float time_nms;
  struct timeval tv_get_new_rects_start, tv_get_new_rects_end;
  float time_get_new_rects;
  
  gettimeofday(&tv_detect_start, NULL);
  FLOAT *boxes = detect(IM,MO,thresh,D_NUMS,A_SCORE);	//detect high-score region
  gettimeofday(&tv_detect_end, NULL);

  gettimeofday(&tv_nms_start, NULL);
  FLOAT *rects = nms(boxes,overlap,D_NUMS,MO);			//get boundary-rectangles of car
  gettimeofday(&tv_nms_end, NULL);

  gettimeofday(&tv_get_new_rects_start, NULL);
  RESULT *CUR = get_new_rects(IM,MO,rects,D_NUMS);		//get current result
  gettimeofday(&tv_get_new_rects_end, NULL);


#if 0
  tvsub(&tv_detect_end, &tv_detect_start, &tv);
  time_detect = tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

  tvsub(&tv_nms_end, &tv_nms_start, &tv);
  time_nms = tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

  tvsub(&tv_get_new_rects_end, &tv_get_new_rects_start, &tv);
  time_get_new_rects = tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

  printf("detect : %f\n", time_detect);
  printf("nms : %f\n", time_nms);
  printf("get_new_rects : %f\n", time_get_new_rects);
#endif

    
  s_free(boxes);
  s_free(rects);
  
  return CUR;
}
