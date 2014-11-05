///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////main.cpp   main function of car tracking /////////////////////////////////////////////////////////////////////

//OpenCV library
/*#include "cv.h"			
#include "cxcore.h"
#include "highgui.h"*/	
#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#ifdef _DEBUG
    //Debugモードの場合
    #pragma comment(lib,"cv200d.lib") 
    #pragma comment(lib,"cxcore200d.lib") 
    #pragma comment(lib,"\cvaux200d.lib") 
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
#include "Laser_info.h"
#include "MODEL_info.h"

#include "switch_float.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef INCLUDED_L_FUNC_
#define INCLUDED_L_FUNC_

extern char *get_file_pass(char *lname);														//get laser-data pass
extern SCANDATA *save_sdata(FLOAT *fdata,int SNUM,IplImage *IM);								//save scan-point data
extern void Release_sdata(SCANDATA *sdata);														//release scan-point data
extern IplImage *ini_Image(int WID,int HEI);													//initialize zero image
//extern void draw_point_on_image(SCANDATA *sdata,IplImage* IM,char *WNAME,RESULT *RE);			//draw scan-point on image
//extern void draw_point_MAP2D(SCANDATA *sdata,char *WNAMES);									//draw sacn-point 2D map
extern void radar_data_fusion(SCANDATA *sdata,IplImage* IM,RESULT *RES,PINFO *PI);				//draw scan point on image 
extern IplImage *draw_sdata(SCANDATA *sdata,IplImage* IM,RESULT *RES);							//visualize scan data
extern void skip_laser_frame(FILE* fp,int sk_num,int *fnum);									//skip frames (for synchronization & debug)
extern void skip_image_frame(CvCapture *capt,int sk_num);										//skip frames (for synchronization % debug)
extern void get_f_size(FILE* fp,fpos_t *curpos,fpos_t *fsize);									//get file size and current file position
extern SCANDATA *get_s_data(FILE *fp,IplImage *IMG,fpos_t *curpos);								//get scan-point data	
extern void skip_data(FILE* fp,CvCapture *capt,int sk_num,int *fnum);							//skip data (for debug)
extern void skip_data_2(FILE* fp,int sk_num,int *ss);
extern IplImage *combine_image (int num, IplImage ** tmp);										//combine image

#endif
