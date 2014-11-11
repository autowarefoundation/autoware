///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////detect_func.h functions about car-detection (to extend main.cc) //////////////////////////////////////////////

#include <stdio.h>
#include "MODEL_info.h"		//File information

#ifndef INCLUDED_MFunctions_
#define INCLUDED_MFunctions_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//load_model.cpp

extern MODEL *load_model(double ratio);						//load MODEL(filter)
extern void free_model(MODEL *MO);							//release model

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//detect.cpp

extern RESULT *car_detection(IplImage *IM,MODEL *MO,double thresh,int *D_NUMS,double *A_SCORE,double overlap);//car detection
extern IplImage *ipl_cre_resize(IplImage *IM,int width,int height);									//create and resize Iplimage
extern IplImage *ipl_resize(IplImage *IM,double ratio);												//resize image (IplImage)
extern double *ini_ac_score(IplImage *IM);															//initialize accumulated score
extern IplImage *load_suc_image(int fnum);															//load image_pictures

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//showboxes.cpp
//extern void showboxes(IplImage *Image,MODEL *MO,double *boxes,int *NUM);
//yukky
extern void show_rects(IplImage *Image, int car_num, int *corner_point, int *type, double ratio);
//extern void show_rects(IplImage *Image,RESULT *CUR,double ratio);									//show rectangle-boxes
extern void show_depth_points(IplImage *Image, double *u, double *v);									//show rlaser_range_points yukky
//extern void show_array(IplImage *Image,RESULT *LR,int *PP);										//show integer array(for debug)
//extern int *show_particles(IplImage *Image,RESULT *CUR,PINFO *P_I);								//show particles (for debug)
//extern void show_det_score(IplImage *Image,double *ac_score,RESULT *CUR);							//show detector score (for debug)
extern void show_vector(IplImage *Image,IplImage *TMAP,RESULT *CUR,PINFO *P_I,double ratio);;		//show vector of velocity
extern void print_information(void);
extern void save_result(IplImage *Image,int fnum);													//save result imagee
//extern void ovw_det_result(IplImage *OR,IplImage *DE, double ratio);								//over-write detection result


// tracking.cpp
extern RESULT *create_result(int num);
extern void release_result(RESULT *LR);																//release last_result data space
extern void update_result(RESULT *LR,RESULT *CUR);													//update result
extern RESULT *get_new_rects(IplImage *Image,MODEL *MO,double *boxes,int *NUM);
extern int finalization(RESULT *CUR,RESULT *LR,PINFO *P_I,double *A_SCORE,IplImage* Image,int THL);	//finalize tracking

#endif
