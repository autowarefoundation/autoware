/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

/////tracking.cpp   calculate_time-variation information //////////////////////////////////////////////////////////

//OpenCV library
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
//C++ library
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <time.h>

#include "MODEL_info.h"		//Model-structure definition
#include "common.hpp"

#include "switch_float.h"
#include "tracking.hpp"

//create new_result data
static RESULT *create_result(int num)
{
	RESULT *RES = (RESULT *)malloc(sizeof(RESULT));
	RES->num=num;
	if(num==0)
	{
		RES->point = NULL;
		RES->type = NULL;
		RES->score = NULL;
		RES->scale = NULL;
		RES->IM = NULL;
		RES->OR_point = NULL;
	}
	else
	{
		RES->point = (int *)calloc(num*4,sizeof(int));
		RES->OR_point = (int *)calloc(num*4,sizeof(int));
		RES->type = (int *)calloc(num,sizeof(int));
		RES->score = (FLOAT *)calloc(num,sizeof(FLOAT));
		RES->scale = (FLOAT *)calloc(num,sizeof(FLOAT));
		RES->IM = (IplImage **)malloc(sizeof(IplImage *)*num);
	}
	return(RES);
}

//get new_rectangle information
RESULT *dpm_ttic_cpu_get_new_rects(IplImage *Image,MODEL *MO,FLOAT *boxes,int *NUM)
{
	const int *numpart = MO->MI->numpart;
	const int GL = (numpart[0]+1)*4+3;
	const FLOAT ratio = MO->MI->ratio;

	int LL = GL-3;
	FLOAT **x1 = MO->MI->x1; FLOAT **x2 = MO->MI->x2;
	FLOAT **y1 = MO->MI->y1; FLOAT **y2 = MO->MI->y2;
	int ML = 1+2*(1+numpart[0]);

	RESULT *CUR =create_result(*NUM);

	//no_rectangle was detected
	if(*NUM==0) return(CUR);

	FLOAT *Avec = (FLOAT *)calloc(ML,sizeof(FLOAT));
	for(int ii=0;ii<*NUM;ii++)
	{
		FLOAT *P = boxes+GL*ii;
		FLOAT *Avec_T = Avec;
		int CNUM = (int)(*(P+GL-3));
		int PP[4];

		*(Avec_T++)=P[3]-P[1];

		for(int kk=0;kk<LL;kk+=4)
		{
			*(Avec_T++)=*(P+kk+1);
			*(Avec_T++)=*(P+kk);
		}

		FLOAT XP1=0,XP2=0,YP1=0,YP2=0;
		Avec_T = Avec;
		FLOAT *x1_T = x1[CNUM]; FLOAT *x2_T = x2[CNUM];
		FLOAT *y1_T = y1[CNUM]; FLOAT *y2_T = y2[CNUM];

		//get rectangle coodinate (by linear-method)
		for(int kk=0;kk<ML;kk++)
		{
			YP1+=*Avec_T*(*(y1_T++)); YP2+=*Avec_T*(*(y2_T++));
			XP1+=*Avec_T*(*(x1_T++)); XP2+=*Avec_T*(*(x2_T++));
			Avec_T++;
		}

		//save result
		if(XP1>0)			{PP[0]=(int)XP1;}
		else				{PP[0]=0;}
		if(YP1>0)			{PP[1]=(int)YP1;}
		else				{PP[1]=0;}
		if(XP2<Image->width){PP[2]=(int)XP2;}
		else				{PP[2]=Image->width;}
		if(YP2<Image->height)	{PP[3]=(int)YP2;}
		else					{PP[3]=Image->height;}
		//memcpy_s(CUR->point+ii*4,4*sizeof(int),PP,4*sizeof(int));
		memcpy(CUR->point+ii*4, PP,4*sizeof(int));
		CUR->scale[ii]=*(P+GL-1); CUR->score[ii]=*(P+GL-2); CUR->type[ii] = CNUM;

		//calculate image coodinate for ORIGINAL-scale-image[640x480]
		int *OPP = CUR->OR_point+ii*4;
		OPP[0] = (int)((FLOAT)PP[0]/ratio);
		OPP[1] = (int)((FLOAT)PP[1]/ratio);
		OPP[2] = (int)((FLOAT)PP[2]/ratio);
		OPP[3] = (int)((FLOAT)PP[3]/ratio);

		//for debug
		printf("scale:%f score:%f type:%d\n",CUR->scale[ii],CUR->score[ii],CUR->type[ii]);
	}
	s_free(Avec);
	return(CUR);
}

