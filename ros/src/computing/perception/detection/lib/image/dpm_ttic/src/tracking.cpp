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

#include <opencv2/legacy/legacy.hpp>
//Header files
#include "MODEL_info.h"		//Model-structure definition
#include "Common.h"

#include "switch_float.h"
#include "tracking.hpp"

//definition of functions

//get object_rectangles
int* elm_rect(RESULT *CUR,int *partner);						//eliminate rectangle(bad score)

//functions about tracking(higher level)
void get_texture(RESULT *RES,IplImage *IM);	//get texture

#define n_particle 1000		//number of particles
#define max_hist 5

extern FILE *resFP;

//create new_result data
static RESULT *create_result(int num)
{
	RESULT *RES = (RESULT *)malloc(sizeof(RESULT));
	RES->num=num;
	if(num==0)
	{
		RES->point = NULL; RES->type = NULL; RES->score = NULL;
		RES->scale = NULL; RES->IM = NULL;	 RES->OR_point = NULL;
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
RESULT *get_new_rects(IplImage *Image,MODEL *MO,FLOAT *boxes,int *NUM)
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
		//printf("x1:%d y1:%d x2:%d y2:%d\n",PP[0],PP[1],PP[2],PP[3]);
		printf("scale:%f score:%f type:%d\n",CUR->scale[ii],CUR->score[ii],CUR->type[ii]);
		fprintf(resFP, "scale:%f score:%f type:%d\n",CUR->scale[ii],CUR->score[ii],CUR->type[ii]);
	}
	s_free(Avec);
	return(CUR);
}

//eliminate rectangle for
int* elm_rect(RESULT *CUR,int *partner)
{
	int *NEW_PARTNER;		//output
	int N_NUM=CUR->num;
	int *CHECK	=(int *)calloc(N_NUM,sizeof(int));
	//check bad-score & independent object
	for(int ii=0;ii<CUR->num;ii++)
	{
		if(partner[ii]<0 && CUR->score[ii]<0.0){ CHECK[ii]=1; N_NUM--;}
		else									 CHECK[ii]=0;
	}

	//check socore and independence
	if(N_NUM<CUR->num && N_NUM>0)		//bad matching (have to reduce object)
	{
		int *N_P=(int *)calloc(N_NUM*4,sizeof(int));
		int *N_T=(int *)calloc(N_NUM,sizeof(int));
		FLOAT *N_SCORE=(FLOAT *)calloc(N_NUM,sizeof(FLOAT));
		FLOAT *N_SCALE=(FLOAT *)calloc(N_NUM,sizeof(FLOAT));
		IplImage **N_IM	=(IplImage**)calloc(N_NUM,sizeof(IplImage*));
		NEW_PARTNER = (int *)calloc(N_NUM,sizeof(int));

		int nc=0;
		for(int ii=0;ii<CUR->num;ii++)
		{
			if(CHECK[ii]==0)
			{
				int *PP = CUR->point+ii*4;
				//memcpy_s(N_P+nc*4,sizeof(int)*4,PP,sizeof(int)*4);
				memcpy(N_P+nc*4, PP,sizeof(int)*4);
				N_T[nc]=CUR->type[ii];
				N_SCORE[nc]=CUR->score[ii];
				N_SCALE[nc]=CUR->scale[ii];
				NEW_PARTNER[nc]=partner[ii];
				nc++;
			}
		}
		//release old data
		s_free(CUR->point); s_free(CUR->scale); s_free(CUR->score);
		s_free(CUR->type);	s_free(CUR->IM);
		//rewrite new data
		CUR->num=N_NUM;		CUR->type=N_T;	CUR->scale=N_SCALE;
		CUR->score=N_SCORE;	CUR->IM=N_IM;	CUR->point=N_P;
		NEW_PARTNER = (int *)calloc(CUR->num,sizeof(int));
		//memcpy_s(NEW_PARTNER,CUR->num*sizeof(int),partner,CUR->num*sizeof(int));
		memcpy(NEW_PARTNER, partner,CUR->num*sizeof(int));
	}
	else if(N_NUM==0)		//zero matching (all rectangles are eliminated)
	{
		s_free(CUR->point); s_free(CUR->scale); s_free(CUR->score);
		s_free(CUR->type);	s_free(CUR->IM);
		CUR->num=0;
		NEW_PARTNER=NULL;
	}
	else
	{
		NEW_PARTNER = (int *)calloc(CUR->num,sizeof(int));
		//memcpy_s(NEW_PARTNER,CUR->num*sizeof(int),partner,CUR->num*sizeof(int));
		memcpy(NEW_PARTNER, partner,CUR->num*sizeof(int));
	}

	s_free(CHECK);
	return NEW_PARTNER;
}

//get texture of object_rectangle
void get_texture(RESULT *RES,IplImage *IM)
{
	for(int ii=0;ii<RES->num;ii++)
	{
		int *PP = RES->point+ii*4;
		int WID = *(PP+2)-*PP;
		int HEI = *(PP+3)-*(PP+1);
		//printf("POINTS %d %d %d %d\n",*PP,*(PP+1),*(PP+2),*(PP+3));
		CvRect REC = cvRect(*PP,*(PP+1),WID,HEI);
		cvSetImageROI(IM,REC);								//change ROI of Image
		IplImage * COLIM=cvCreateImage(cvSize(WID,HEI),IM->depth,IM->nChannels);	//color image (temporary)
		cvCopy(IM,COLIM);
		RES->IM[ii]=cvCreateImage(cvSize(WID,HEI),IM->depth,1);				//create gray_scale image
		cvCvtColor(COLIM,RES->IM[ii],CV_BGR2GRAY);					//get gray_scale image
		cvReleaseImage(&COLIM);								//release temporary image
		cvResetImageROI(IM);								//reset ROI of Image
	}
}
