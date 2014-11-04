///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////tracking.cpp   calculate_time-variation information //////////////////////////////////////////////////////////

//OpenCV library
//#include "cv.h"			
//#include "cxcore.h"
//#include "highgui.h"
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
#include <math.h>
#include <time.h>

//added by messi
#include <legacy.hpp>
//Header files
#include "MODEL_info.h"		//Model-structure definition
#include "Common.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//definition of functions 

extern int *show_particles(IplImage *Image,RESULT *CUR,PINFO *P_I);

//get object_rectangles
RESULT *get_new_rects(IplImage *Image,MODEL *MO,double *boxes,int *NUM);			//get new_rectangle pixel_point
int* elm_rect(RESULT *CUR,int *partner);											//eliminate rectangle(bad score)

//functions about result
void reset_result(RESULT *LR,int num);												//reset last result
RESULT *create_result(int num);														//create new result
void release_result(RESULT *LR);													//release last_result data space
void update_result(RESULT *LR,RESULT *CUR);											//update result

//function about Condensation(Particle_filter tracking lower_level functions)
CvConDensation *initialize_cond(IplImage *Image,int TYPE);							//initialize particle filter parameters
CvMat *match_likelihood(IplImage *TEMP,IplImage *Img,int *COORD);					//match template 
int calc_part_likelihood(CvConDensation *COND,CvMat *LIKE,int *COORD,double *A_SCORE,IplImage *RTEMP,IplImage *Im,RESULT *CUR); //calculate likelihood
void init_particle_position(RESULT *CUR,CvConDensation *COND,IplImage *IM,int NUM,int L);//initialize position of particles
void object_tracking(RESULT *CUR,RESULT *LR,CvConDensation *COND,double *A_SCORE,IplImage *Im,int Lnum,int NUM); //tracking function

//functions about tracking(higher level)
void reset_pinfo(PINFO *PI,RESULT *LR);												//reset particle filter information
int *get_partner_rectangle(RESULT *LR,RESULT *CUR,PINFO *PI,int THL);				//get partner rectangle information
void update_h_position(PINFO *P_I,int LNUM,RESULT *CUR,int CCNUM,int **NP,int *NSNUM);			//update historical positon
void update_h_velocity(PINFO *P_I,int LNUM,RESULT *CUR,int CCNUM,double **NVX,double **NVY,int *NSNUM);//update historical velocity
void update_ave_p(PINFO *P_I,int LNUM,RESULT *CUR,int CCNUM,double **avep,int *NSNUM);			//update average position (Laser RADER)

void object_prediction(RESULT *CUR,CvConDensation **COND);							//prediction of object_position
void update_pinfo(PINFO *P_I,RESULT *LR,RESULT *CUR,double *A_SCORE,IplImage *Image,int THL);//update particle filter information

//finalization_function
void get_texture(RESULT *RES,IplImage *IM);											//get texture 
int finalization(RESULT *CUR,RESULT *LR,PINFO *P_I,double *A_SCORE,IplImage* Image,int THL);//finalize object_information(tracking)

#define n_particle 1000																//number of particles
#define max_hist 5

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//get new_rectangle information
RESULT *get_new_rects(IplImage *Image,MODEL *MO,double *boxes,int *NUM)
{
  const int *numpart = MO->MI->numpart;	
  const int GL = (numpart[0]+1)*4+3;
  const double ratio = MO->MI->ratio;
  const int height = 480;
  const int width =  640;
  const int UpY = height/10;
  const int NEW_Y = height-UpY-height/10;
  
  int LL = GL-3;
  double **x1 = MO->MI->x1; double **x2 = MO->MI->x2;
  double **y1 = MO->MI->y1; double **y2 = MO->MI->y2;
  int ML = 1+2*(1+numpart[0]);
  
  RESULT *CUR =create_result(*NUM);
  
  //no_rectangle was detected 
  if(*NUM==0) return(CUR);
  
  double *Avec = (double *)calloc(ML,sizeof(double));
  for(int ii=0;ii<*NUM;ii++)
    {
      double *P = boxes+GL*ii;
      double *Avec_T = Avec;
      int CNUM = (int)(*(P+GL-3));
      int PP[4];
      
      *(Avec_T++)=P[3]-P[1];
      
      for(int kk=0;kk<LL;kk+=4)
        {
          *(Avec_T++)=*(P+kk+1);
          *(Avec_T++)=*(P+kk);
        }
      
      double XP1=0,XP2=0,YP1=0,YP2=0;
      Avec_T = Avec;
      double *x1_T = x1[CNUM]; double *x2_T = x2[CNUM];
      double *y1_T = y1[CNUM]; double *y2_T = y2[CNUM];		
      
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
      OPP[0] = (int)((double)PP[0]/ratio);
      OPP[1] = (int)((double)PP[1]/ratio);
      OPP[2] = (int)((double)PP[2]/ratio);
      OPP[3] = (int)((double)PP[3]/ratio);
      
      //for debug 
      //printf("x1:%d y1:%d x2:%d y2:%d\n",PP[0],PP[1],PP[2],PP[3]);
      printf("scale:%f score:%f type:%d\n",CUR->scale[ii],CUR->score[ii],CUR->type[ii]);
    }
  s_free(Avec);	
  return(CUR);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//eliminate rectangle for 
int* elm_rect(RESULT *CUR,int *partner)
{
  int *NEW_PARTNER;		//output
  int N_NUM=CUR->num;
  int *CHECK	=(int *)calloc(N_NUM,sizeof(int));
  int n_shift=0;
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
      double *N_SCORE=(double *)calloc(N_NUM,sizeof(double));
      double *N_SCALE=(double *)calloc(N_NUM,sizeof(double));
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//function about result(create and release and reset)
//reset result data (for creation of new_data) 
void reset_result(RESULT *RES,int num)
{
	//release previous data
	if(RES->num>0)
	{
		for(int ii=0;ii<RES->num;ii++) if(RES->IM[ii]!=NULL) cvReleaseImage(&RES->IM[ii]);
		s_free(RES->point);
		s_free(RES->type);
		s_free(RES->scale);
		s_free(RES->score);
	}

	//alloc new data
	RES->num=num;
	RES->point = (int *)calloc(num*4,sizeof(int));
	RES->type = (int *)calloc(num,sizeof(int));
	RES->score = (double *)calloc(num,sizeof(double));
	RES->scale = (double *)calloc(num,sizeof(double));
	RES->IM = (IplImage **)malloc(sizeof(IplImage *)*num);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//create new_result data
RESULT *create_result(int num)
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
		RES->score = (double *)calloc(num,sizeof(double));
		RES->scale = (double *)calloc(num,sizeof(double));
		RES->IM = (IplImage **)malloc(sizeof(IplImage *)*num);
	}
	return(RES);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//release last_result data space
void release_result(RESULT *LR)
{
	for(int ii=0;ii<LR->num;ii++) cvReleaseImage(&LR->IM[ii]);	
	s_free(LR->point);
	s_free(LR->OR_point);
	s_free(LR->type);
	s_free(LR->scale);
	s_free(LR->score);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//update result
void update_result(RESULT *LR,RESULT *CUR)
{
	release_result(LR);
	LR->num=CUR->num;
	LR->IM=CUR->IM;
	LR->point=CUR->point;
	LR->OR_point=CUR->OR_point;
	LR->scale=CUR->scale;
	LR->score=CUR->score;
	LR->type=CUR->type;
	s_free(CUR);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//tracking function (lower_level)
//functions about condensation
//initialize particle filter parameters
CvConDensation *initialize_cond(IplImage *Image,int TYPE)
{
	double w = Image->width;				//position bound
	double h = Image->height;
	double w_ve = 60.0; double h_ve = 30.0;	//velocity bound
	double w_no = 10.0; double h_no = 10.0;	//noise parameter
	
	int n_stat;
	if(TYPE==0)			n_stat=4; 	
	else if(TYPE==1)	n_stat=2;
	CvConDensation *cond = cvCreateConDensation(n_stat,n_stat,n_particle);

	//matrix for boundary information
	CvMat *lowerBound,*upperBound;
	lowerBound=cvCreateMat(n_stat,1,CV_32FC1);
	upperBound=cvCreateMat(n_stat,1,CV_32FC1);
	cvmSet(lowerBound,0,0,0.0); cvmSet(lowerBound,1,0,0.0);
	cvmSet(upperBound,0,0,w); cvmSet(upperBound,1,0,h);
	if(TYPE==0) //if linear uniform motion
	{
		cvmSet(lowerBound,2,0,-w_ve); cvmSet(lowerBound,3,0,-h_ve);
		cvmSet(upperBound,2,0,w_ve); cvmSet(upperBound,3,0,h_ve);
	}

	//initialize condensation
	cvConDensInitSampleSet(cond,lowerBound,upperBound);

	//Set state-vector matrix and noise parameter
	if(TYPE==0) //if linear uniform motion
	{
		cond->DynamMatr[0] =  1.0; cond->DynamMatr[1]  = 0.0; cond->DynamMatr[2]  = 1.0; cond->DynamMatr[3]  = 0.0; 
		cond->DynamMatr[4] =  0.0; cond->DynamMatr[5]  = 1.0; cond->DynamMatr[6]  = 0.0; cond->DynamMatr[7]  = 1.0;
		cond->DynamMatr[8] =  0.0; cond->DynamMatr[9]  = 0.0; cond->DynamMatr[10] = 1.0; cond->DynamMatr[11] = 0.0;
		cond->DynamMatr[12] = 0.0; cond->DynamMatr[13] = 0.0; cond->DynamMatr[14] = 0.0; cond->DynamMatr[15] = 1.0;
		cvRandInit(&(cond->RandS[0]),-w_no,w_no,0); cvRandInit(&(cond->RandS[1]),-h_no,h_no,1);
		cvRandInit(&(cond->RandS[2]),-5,5,2);   cvRandInit(&(cond->RandS[3]),-5,5,3);
	}
	if(TYPE==1)
	{
		cond->DynamMatr[0] =  1.0; cond->DynamMatr[1]  = 0.0; cond->DynamMatr[2]  = 0.0; cond->DynamMatr[3]  = 1.0;
		cvRandInit(&(cond->RandS[0]),-w_no,w_no,0); cvRandInit(&(cond->RandS[1]),-h_no,h_no,1);
	}

	return(cond);
}

//match template (get likelihood matrix)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
CvMat *match_likelihood(IplImage *TEMP,IplImage *Img,int *COORD)
{
	int WID = COORD[2]-COORD[0];
	int HEI = COORD[3]-COORD[1];
	CvRect REC = cvRect(COORD[0],COORD[1],WID,HEI);
	cvSetImageROI(Img,REC);														//change ROI of Image
	IplImage *RANGE=cvCreateImage(cvSize(WID,HEI),Img->depth,Img->nChannels);	//cut Image(searcnig range)
	cvCopy(Img,RANGE);
	cvResetImageROI(Img);														//Reset ROI of Image
	IplImage *GRAYIM = cvCreateImage(cvSize(WID,HEI),RANGE->depth,1);			//Image for gray-scale
	cvCvtColor(RANGE,GRAYIM,CV_BGR2GRAY);										//change color (gray_scale)
	cvReleaseImage(&RANGE);
	int dst_WID = GRAYIM->width-TEMP->width+1;									//Output width
	int dst_HEI = GRAYIM->height-TEMP->height+1;								//Output height
	CvMat *MRES = cvCreateMat(dst_HEI,dst_WID,CV_32FC1);						//Output matrix

	//debug
	//cvNamedWindow("TEST1",CV_WINDOW_AUTOSIZE);
	//cvShowImage("TEST1",GRAYIM);	//show image
	//cvNamedWindow("TEST2",CV_WINDOW_AUTOSIZE);
	//cvShowImage("TEST2",TEMP);	//show image
	//cvWaitKey(0);
	//cvDestroyWindow("TEST1");	//destroy window
	//cvDestroyWindow("TEST2");	//destroy window

	cvMatchTemplate(GRAYIM,TEMP,MRES,CV_TM_CCORR_NORMED);						//Matching
	cvReleaseImage(&GRAYIM);
	return(MRES);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//calculate likelihood for each particle
int calc_part_likelihood(CvConDensation *COND,CvMat *LIKE,int *COORD,double *A_SCORE,IplImage *RTEMP,IplImage *Im,RESULT *CUR)
{
	double alpha = -8.0;
	int L = 20;
	int IM_HEI = Im->height;
	int WID = RTEMP->width/2;		//width of template 
	int HEI = RTEMP->height/2;	//height of template 
	int X1 = COORD[0]+WID;		int Y1 = COORD[1]+HEI;			//valid range of tracking
	int X2 = X1+LIKE->width;	int Y2 = Y1+LIKE->height;

	int count =0;
	srand((unsigned)time(NULL));
	int *PP = CUR->point;
	float XC = ((float)*PP+(float)*(PP+2)-(float)L)/2;
	float YC = ((float)*(PP+1)+(float)*(PP+3)-(float)L)/2;
	//debug	
	IplImage *TESTIMAGE = cvCreateImage(cvSize(448,268),RTEMP->depth,RTEMP->nChannels);
	//for(int ss=0;ss<TESTIMAGE->height;ss++) for(int mm=0;mm<TESTIMAGE->width;mm++) cvSetReal2D(TESTIMAGE,ss,mm,0.0);

	//caluculate likelihood for each particle
	for(int ii=0;ii<COND->SamplesNum;ii++)
	{
		int X = (int)COND->flSamples[ii][0];
		int Y = (int)COND->flSamples[ii][1];
		if(X>=X1 && X<X2 && Y>=Y1 && Y<Y2)	//check valid range
		{
			//get likelihood
			float pro = (float)(exp(alpha*(1-cvmGet(LIKE,Y-Y1,X-X1))));	//likelihood of image correlation
			float det = (float)A_SCORE[Y+X*IM_HEI];
			//get new_confidence
			COND->flConfidence[ii]=det+pro;
			//for debug
			//printf("probability %f\n",pro);
			//cvSetReal2D(TESTIMAGE,Y,X,255.0);
			count++;
		}
		else 
		{
			COND->flConfidence[ii]=0.0;
			COND->flSamples[ii][0]=XC+rand()%L;	
			COND->flSamples[ii][1]=YC+rand()%L;
			//float XN = XC+rand()%L;	
			//float YN = YC+rand()%L;
			//cvSetReal2D(TESTIMAGE,(int)YN,(int)XN,255.0);
		}							
	}
	//debug
	//cvShowImage("TEST2",TESTIMAGE);	//show image
	cvReleaseImage(&TESTIMAGE);
	return(count);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//initialize position of particles
void init_particle_position(RESULT *CUR,CvConDensation *COND,IplImage *IM,int NUM,int L)
{
	srand((unsigned)time(NULL));
	int *PP = CUR->point+NUM*4;
	float X = ((float)*PP+(float)*(PP+2))/2;
	float Y = ((float)*(PP+1)+(float)*(PP+3))/2;
	int LX=L,LY=L;
	int TYPE = CUR->type[NUM];

	X-=(LX/2);	Y-=(LY/2);
	
	//Original filter position
	for(int ii=0;ii<COND->SamplesNum;ii++) 
	{
		float XN = X+rand()%LX;	
		float YN = Y+rand()%LY;

		COND->flSamples[ii][0]=XN;
		COND->flSamples[ii][1]=YN;
		COND->flConfidence[ii]=1.0;
	}
	COND->State[0]=X;
	COND->State[1]=Y;
	if(COND->DP>2)
	{
		COND->State[2]=0.0;
		COND->State[3]=0.0;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//tracking function
void object_tracking(RESULT *CUR,RESULT *LR,CvConDensation *COND,double *A_SCORE,IplImage *Im,int Lnum,int NUM)
{
	double ASP_TH = 0.2;
	int R_flag=0,F_flag=0;
	IplImage *RTEMP;
	int BTH =15;
	
	//get searching range information
	int *PP = CUR->point+NUM*4;
	int WID1 = *(PP+2)-*PP;		int HEI1 = *(PP+3)-*(PP+1);
	int OrX = (*(PP+2)+*PP)/2;	int OrY = (*(PP+3)+*(PP+1))/2;
	int L_BORD = PP[0];			int R_BORD = PP[2];
	PP = LR->point+Lnum*4;
	int LaX = (*(PP+2)+*PP)/2;	int LaY = (*(PP+3)+*(PP+1))/2;
	double XMOVE = (double)(OrX-LaX); 
	double YMOVE = (double)(OrY-LaY);
	double LENGTH =sqrt(XMOVE*XMOVE+YMOVE*YMOVE);

	//check static object
	if(LENGTH<5 && CUR->type[NUM]==1)
	{
		init_particle_position(CUR,COND,Im,NUM,20);	//initialize particle position
		return;
	}

	//occlusion check
	IplImage *Limg;
	IplImage *LTEMP = LR->IM[Lnum];
	CvRect REC;
	//left boader
	if(L_BORD<BTH)				 { F_flag =1; REC = cvRect(BTH,0,LTEMP->width,LTEMP->height);}
	//right boader
	else if(R_BORD>Im->width-BTH){ F_flag =1; REC = cvRect(0,0,LTEMP->width-BTH,LTEMP->height);}
	else Limg = LR->IM[Lnum];
	if(F_flag==1)
	{
		cvSetImageROI(LTEMP,REC);
		Limg = cvCreateImage(cvSize(LTEMP->width-BTH,LTEMP->height),LTEMP->depth,LTEMP->nChannels);
		cvCopy(LTEMP,Limg);
		cvResetImageROI(LTEMP);
	}

	//calculate aspect-ratio
	double ASP_L = (double)WID1/(double)HEI1;
	double ASP_C = (double)Limg->width/(double)Limg->height;
	//resize template(fit scale of matching)
	if(CUR->scale[NUM]==LR->scale[Lnum]) { RTEMP = Limg;}
	else
	{
		R_flag=1;
		double ratio = CUR->scale[NUM]/LR->scale[Lnum];
		double RW = (double)WID1/(double)Limg->width;
		double RH = (double)HEI1/(double)Limg->height;
		double LRAT = (RW+RH)*0.5;
		//get matched-scale
		if(abs(ASP_L-ASP_C)>ASP_TH) {ratio = LRAT;}
		else						{ratio = (LRAT+ratio)*0.5;}
		int N_WID = (int)(Limg->width*ratio);
		int N_HEI = (int)(Limg->height*ratio);
		RTEMP=cvCreateImage(cvSize(N_WID,N_HEI),Limg->depth,Limg->nChannels);
		cvResize(Limg,RTEMP);
	}
	
	int WID = RTEMP->width; int HEI = RTEMP->height;
	if(WID<WID1) WID = WID1;
	if(HEI<HEI1) HEI = HEI1;
	WID = (int)((double)WID*0.7);
	HEI = (int)((double)HEI*0.7);
	//set searching range
	int COORD[4] = {OrX-WID,OrY-HEI,OrX+WID,OrY+HEI};
	if(COORD[0]<0) COORD[0]=0;					
	if(COORD[1]<0) COORD[1]=0;
	if(COORD[2]>Im->width) COORD[2]=Im->width;	
	if(COORD[3]>Im->height) COORD[3]=Im->height;

	CvMat *Like = match_likelihood(RTEMP,Im,COORD);						//template matching
	int val_num=calc_part_likelihood(COND,Like,COORD,A_SCORE,RTEMP,Im,CUR);		//calculate likelihood

	//if tracking was failed => initialize particle and calculate likelihood again
	if(val_num<100)
	{
		cvReleaseConDensation(&COND);
		COND = initialize_cond(Im,0);
		init_particle_position(CUR,COND,Im,NUM,20);		//initialize particle_position
		val_num = calc_part_likelihood(COND,Like,COORD,A_SCORE,RTEMP,Im,CUR);
	}

	//debug
	//CvScalar COL = cvScalar(255.0,200.0,150.0);	
	//IplImage *tesIM = cvCloneImage(Im);
	//for(int ss=0;ss<COND->SamplesNum;ss++)
	//{
	//	CvPoint PP = cvPoint((int)COND->flSamples[ss][0],(int)COND->flSamples[ss][1]);
	//	cvCircle(tesIM,PP,2,COL);			//draw each particles
	//}
	//CvPoint p1=cvPoint(COORD[0],COORD[1]);		
	//CvPoint p2=cvPoint(COORD[2],COORD[3]);
	//cvRectangle(tesIM,p1,p2,COL,3);
	//cvNamedWindow("TEST",CV_WINDOW_AUTOSIZE);
	//cvShowImage("TEST",tesIM);	//show image
	//cvWaitKey(0);
	//cvShowImage("TEST",RTEMP);	//show image
	//cvWaitKey(0);
	//cvDestroyWindow("TEST");	//destroy window
	//cvReleaseImage(&tesIM);

	//debug
	cvReleaseMat(&Like);
	if(F_flag==1) cvReleaseImage(&Limg);
	if(R_flag==1) cvReleaseImage(&RTEMP);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//function about particle_filter information
//reset_P_info
void reset_pinfo(PINFO *PI,RESULT *LR)
{
	if(LR->num>0)
	{
		for(int ii=0;ii<LR->num;ii++)
		{
			cvReleaseConDensation(&PI->condens[ii]);
			s_free(PI->L_P[ii]);
			s_free(PI->L_VX[ii]);
			s_free(PI->L_VY[ii]);
			s_free(PI->ave_p[ii]);
		}
		s_free(PI->partner);
		s_free(PI->condens);
		s_free(PI->L_P);
		s_free(PI->L_VX);
		s_free(PI->L_VY);
		s_free(PI->ave_p);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//get partner rectangle
//link last-frame object
int *get_partner_rectangle(RESULT *LR,RESULT *CUR,PINFO *PI,int THL)
{
	int C_num = CUR->num;	//# of current frame object
	int L_num = LR->num;	//# of current frame object
	int *partner=(int *)calloc(C_num,sizeof(int));	//partner objects
	int *Lcheck=(int *)calloc(L_num,sizeof(int));	
	double *VEL = (double *)calloc(L_num,sizeof(double));	//average velocity

	//get average velocity
	for(int ss=0;ss<L_num;ss++)
	{
		double tempV=0.0;
		int SNUM = PI->se_num[ss];
		if(SNUM<1){ VEL[ss]=-1.0; continue;}
		if(SNUM>5) SNUM=5;
		for(int tt=0;tt<SNUM;tt++){ tempV+=PI->L_VX[ss][tt];}
		VEL[ss]=tempV/(double)SNUM;
	}

	//get partner information (compare length between detected rectangle)
	for(int ss=0;ss<C_num;ss++)
	{
		partner[ss]=-1;

		int *PP = CUR->point+ss*4;
		int Cx = (*(PP+2)+*(PP))/2;			//position of current object(X)
		int Cy = (*(PP+3)+*(PP+1))/2;		//position of current object(Y)
		int Shtest = THL;								//shortest length
		int tempP=-1;
		for(int kk=0;kk<L_num;kk++)	
		{
			if(Lcheck[kk]==0)
			{
				int *PP = LR->point+kk*4;
				int Lx = (*(PP+2)+*(PP))/2;		//position of last frame object(X)		
				int Ly = (*(PP+3)+*(PP+1))/2;	//position of last frame object(Y)
				int Length = (int)sqrt((double)((Lx-Cx)*(Lx-Cx)+(Ly-Cy)*(Ly-Cy)));	//lenght of current and last object
				//find near and same-type object
				if(Length<Shtest && CUR->type[ss]==LR->type[kk])
				{
					int VX_CUR = Cx-Lx;
					double C_A = abs((double)VX_CUR-VEL[kk]);
					double C_B = VEL[kk]*VX_CUR;
					//debug
					//printf("LENGTH %d  ABS %f\n",Length,C_A);
					if		(C_A<40)									{ tempP=kk;Shtest=Length;}
					else if	(abs(VEL[kk])>3 && abs(VX_CUR)>3 && C_B>0)	{ tempP=kk;Shtest=Length;}
				}
			}
		}
		if(tempP>=0){ partner[ss]=tempP;Lcheck[tempP]=1;}
		//debug
		//printf("CUR%d-LR%d Shtest%d\n",ss,tempP,Shtest);
	}
	s_free(VEL);
	return(partner);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//update historical-position
void update_h_position(PINFO *P_I,int LNUM,RESULT *CUR,int CCNUM,int **NP,int *NSNUM)
{
	NP[CCNUM]=(int *)calloc(max_hist*4,sizeof(int));

	int *LP = P_I->L_P[LNUM];
	int *PP = CUR->point+4*CCNUM;

	NP[CCNUM][0]=*PP;
	NP[CCNUM][1]=*(PP+1);
	NP[CCNUM][2]=*(PP+2);
	NP[CCNUM][3]=*(PP+3);

	NSNUM[CCNUM]=P_I->se_num[LNUM]+1;
	int P_COUNT = NSNUM[CCNUM];
	if(P_COUNT>max_hist-1) P_COUNT=max_hist-1;
	for(int ii=1;ii<=P_COUNT;ii++)
	{
		NP[CCNUM][4*ii]=LP[4*(ii-1)];
		NP[CCNUM][4*ii+1]=LP[4*(ii-1)+1];
		NP[CCNUM][4*ii+2]=LP[4*(ii-1)+2];
		NP[CCNUM][4*ii+3]=LP[4*(ii-1)+3];
	}
	s_free(P_I->L_P[LNUM]);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//update historical-position
void update_h_velocity(PINFO *P_I,int LNUM,RESULT *CUR,int CCNUM,double **NVX,double **NVY,int *NSNUM)
{
	NVX[CCNUM]=(double *)calloc(max_hist,sizeof(double));
	NVY[CCNUM]=(double *)calloc(max_hist,sizeof(double));

	double *LVX = P_I->L_VX[LNUM];
	double *LVY = P_I->L_VX[LNUM];
	int P_COUNT = NSNUM[CCNUM];
	if(P_COUNT>max_hist-1) P_COUNT=max_hist-1;

	float *s_vector=P_I->condens[LNUM]->State;
	//間違え？？NVY[CCNUM][1]→NVY[CCNUM][0]
	if(P_I->condens[LNUM]->DP>2) {NVX[CCNUM][0]=*(s_vector+2);NVY[CCNUM][1]=*(s_vector+3);}
	else						 {NVX[CCNUM][0]=0;NVY[CCNUM][1]=0;}	

	for(int ii=1;ii<=P_COUNT;ii++)
	{
		NVX[CCNUM][ii]=LVX[ii-1]; NVY[CCNUM][ii]=LVY[ii-1];
	}
	s_free(P_I->L_VX[LNUM]); s_free(P_I->L_VY[LNUM]);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//update historical-position
void update_ave_p(PINFO *P_I,int LNUM,RESULT *CUR,int CCNUM,double **avep,int *NSNUM)
{
	avep[CCNUM]=(double *)calloc(max_hist*2,sizeof(double));

	double *LAP = P_I->ave_p[LNUM];
	int P_COUNT = NSNUM[CCNUM];
	if(P_COUNT>max_hist-1) P_COUNT=max_hist-1;

	for(int ii=1;ii<=P_COUNT;ii++)
	{
		avep[CCNUM][2*ii]=LAP[2*(ii-1)]; 
		avep[CCNUM][2*ii+1]=LAP[2*(ii-1)+1];
	}
	s_free(P_I->ave_p[LNUM]);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//object prediction
void object_prediction(RESULT *CUR,CvConDensation **COND)
{
	for(int ii=0;ii<CUR->num;ii++)
	{
		CvConDensation *cond = COND[ii];
		cvConDensUpdateByTime(cond);		//Update object_tracking info by time
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//update P_info
void update_pinfo(PINFO *P_I,RESULT *LR,RESULT *CUR,double *A_SCORE,IplImage *Image,int THL)
{
	//check partner-rectangle for all rectangles detected in current-frame
	int *ptemp = get_partner_rectangle(LR,CUR,P_I,THL);
	int *c_partner = elm_rect(CUR,ptemp);	//eliminate bad matching (consider time information)
	s_free(ptemp);
	if(CUR->num==0) {return;}

	int *CHECK = (int *)calloc(LR->num,sizeof(int));	//for take-over check
	CvConDensation ** new_cond= (CvConDensation **)calloc(CUR->num,sizeof(CvConDensation*));
	int *new_seq_num = (int *)calloc(CUR->num,sizeof(int));
	int **new_L_P= (int **)calloc(CUR->num,sizeof(int*));
	double **new_L_VX= (double **)calloc(CUR->num,sizeof(double*));
	double **new_L_VY= (double **)calloc(CUR->num,sizeof(double*));
	double **new_ave_P= (double **)calloc(CUR->num,sizeof(double*));

	//update particle filter tracking information
	for(int ii=0;ii<CUR->num;ii++)
	{	
		if(c_partner[ii]>=0)						//continus_object
		{
			CHECK[c_partner[ii]]=1;					//filter information take-over check
			new_cond[ii]=P_I->condens[c_partner[ii]];	//take over condensation from last_result
			update_h_position(P_I,c_partner[ii],CUR,ii,new_L_P,new_seq_num);	//update historical positon
			update_h_velocity(P_I,c_partner[ii],CUR,ii,new_L_VX,new_L_VY,new_seq_num);	//update historical velocity
			update_ave_p(P_I,c_partner[ii],CUR,ii,new_ave_P,new_seq_num);		//update historical velocity
			int Lnum = c_partner[ii];
			object_tracking(CUR,LR,new_cond[ii],A_SCORE,Image,Lnum,ii);	//object_tracking
		}
		else							//new object
		{
			new_cond[ii]=initialize_cond(Image,0);				//initialize condensation(for particle filter)
			new_seq_num[ii]=0;
			new_L_P[ii]=(int *)calloc(max_hist*4,sizeof(int));
			new_L_VX[ii]=(double *)calloc(max_hist,sizeof(double));
			new_L_VY[ii]=(double *)calloc(max_hist,sizeof(double));
			new_ave_P[ii]=(double *)calloc(max_hist*2,sizeof(double));
			init_particle_position(CUR,new_cond[ii],Image,ii,20);		//initialize particle_position
		}
	}

	//release unnecessary condensation
	for(int ii=0;ii<LR->num;ii++)
	{		
		if(CHECK[ii]==0) 
		{
			cvReleaseConDensation(&P_I->condens[ii]);	//release unnecessary condensation
			s_free(P_I->L_P[ii]);  
			s_free(P_I->L_VX[ii]); s_free(P_I->L_VY[ii]);
			s_free(P_I->ave_p[ii]);
		}
	}

	//prediction update by time
	object_prediction(CUR,new_cond);

	//release old data
	s_free(P_I->condens);
	s_free(P_I->partner);
	s_free(P_I->se_num);
	s_free(P_I->L_P); 
	s_free(P_I->L_VX); s_free(P_I->L_VY);
	s_free(P_I->ave_p);
	s_free(CHECK);
	
	//return
	P_I->partner = c_partner;
	P_I->condens = new_cond;
	P_I->se_num = new_seq_num;
	P_I->L_P=new_L_P;
	P_I->L_VX=new_L_VX; P_I->L_VY=new_L_VY;
	P_I->ave_p=new_ave_P;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
		cvSetImageROI(IM,REC);														//change ROI of Image
		IplImage * COLIM=cvCreateImage(cvSize(WID,HEI),IM->depth,IM->nChannels);	//color image (temporary)
		cvCopy(IM,COLIM);
		RES->IM[ii]=cvCreateImage(cvSize(WID,HEI),IM->depth,1);						//create gray_scale image
		cvCvtColor(COLIM,RES->IM[ii],CV_BGR2GRAY);									//get gray_scale image
		cvReleaseImage(&COLIM);														//release temporary image
		cvResetImageROI(IM);														//reset ROI of Image
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//finalize object regions
int finalization(RESULT *CUR,RESULT *LR,PINFO *P_I,double *A_SCORE,IplImage* Image,int THL)
{
	//no-object
	if(CUR->num==0)
	{
		reset_pinfo(P_I,LR);				//reset particle_filter information
		reset_result(CUR,0);				//reset current_result
	}
	else
	{
		//update particle_filter tracking information
		update_pinfo(P_I,LR,CUR,A_SCORE,Image,THL);	//update  particle filter information
		
		//all rectangles are bad-score
		if(CUR->num==0)
		{
			reset_pinfo(P_I,LR);
			reset_result(CUR,0);
		}
		get_texture(CUR,Image);				//get texture
	}
	return(0);
}
