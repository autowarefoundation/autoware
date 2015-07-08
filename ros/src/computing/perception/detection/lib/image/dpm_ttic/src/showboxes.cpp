///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

/////showboxes.cpp   show object_rectangle_box (write to IplImage) ////////////////////////////////////////////////

//OpenCV library
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "MODEL_info.h"		//File information
#include "Laser_info.h"
#include "Common.h"

#include "switch_float.h"

#define MAXLINE 256

//definiton of functions//
void showboxes(IplImage *Image,MODEL *MO,FLOAT *boxes,int *NUM);				//show root-rectangle-boxes (extended to main.cpp)
void show_rects(IplImage *Image,RESULT *CUR,FLOAT ratio);					//show rectangle-boxes (extended to main.cpp)
void show_array(IplImage *Image,RESULT *LR,int *PP);						//show integer array(for debug)
int *show_particles(IplImage *Image,RESULT *CUR,PINFO *P_I);					//show particles (extended to main.cpp)
int *show_vector_im(IplImage *Image,RESULT *CUR,PINFO *P_I,FLOAT ratio);			//show velocity vector on image
void show_vector_2D(IplImage *MAP,IplImage *IM,RESULT *CUR,PINFO *P_I,int *I_VEC,FLOAT ratio);	//show velocity vector on 2DMAP
void show_vector(IplImage *Image,IplImage *TMAP,RESULT *CUR,PINFO *P_I,FLOAT ratio);		//show vector of velocity
void show_likelihood(IplImage *Image,CvMat *LIKE,int *COORD);					//show likelihood (for debug)
void show_det_score(IplImage *Image,FLOAT *ac_score,RESULT *CUR);				//show detector accumulated score (debug)
void save_result(IplImage *Image,int fnum);							//save result image
void ovw_det_result(IplImage *OR,IplImage *DE, FLOAT ratio);					//over-write detection result
IplImage *load_suc_image(int fnum);								//load successive images (jpeg data)

static CvScalar get_color(int coltype)
{
	CvScalar COL;
	switch(coltype%3)
	{
	case 0:
		COL = cvScalar(255.0,255.0,0.0);
		break;
	case 1:
		COL = cvScalar(255.0,0.0,0.0);
		break;
	case 2:
		COL = cvScalar(255.0,255.0,0.0);
		break;
	}
	return(COL);
}

//show rectangle boxes(with object_tracking result)
void show_rects(IplImage *Image,RESULT *CUR,FLOAT ratio)
{
	for(int ii=0;ii<CUR->num;ii++)
	{
		//int *P = CUR->point+4*ii;
		int *P = CUR->OR_point+4*ii;
		CvScalar col = get_color(CUR->type[ii]);
		CvPoint p1=cvPoint(P[0],P[1]);
		CvPoint p2=cvPoint(P[2],P[3]);
		cvRectangle(Image,p1,p2,col,3);			//draw current-object rectangle
		cvLine(Image,p1,p2,col,2);
		p1 = cvPoint(P[0],P[3]);
		p2 = cvPoint(P[2],P[1]);
		cvLine(Image,p1,p2,col,2);
	}
}

//show rectangle boxes(with object_tracking result) for debug
void show_array(IplImage *Image,RESULT *LR,int *PP)
{
	CvScalar COL = cvScalar(255.0,200.0,150.0);	//color of particles
	for(int ii=0;ii<LR->num;ii++)
	{
		int *P=PP+ii*4;
		CvPoint p1=cvPoint(P[0],P[1]);
		CvPoint p2=cvPoint(P[2],P[3]);
		cvRectangle(Image,p1,p2,COL,3);
		cvLine(Image,p1,p2,COL,2);
		p1 = cvPoint(P[0],P[3]);
		p2 = cvPoint(P[2],P[1]);
		cvLine(Image,p1,p2,COL,2);
	}
	s_free(PP);
}

//show particle positions (for debug)
int *show_particles(IplImage *Image,RESULT *CUR,PINFO *P_I)
{
	int *NEXT = (int *)calloc(CUR->num*4,sizeof(int));
	if(CUR->num>0)
	{
		CvScalar COL = cvScalar(255.0,255.0,255.0);	//color of particles
		CvScalar COL2 = cvScalar(150.0,200.0,0.0);	//color of predicted position
		for(int ii=0;ii<CUR->num;ii++)
		{
			CvConDensation *cond = P_I->condens[ii];
			int NUM = cond->SamplesNum;
			if(cond->flCumulative[NUM-1]>0.0)
			{
				FLOAT NEXT_X =0,NEXT_Y =0;					//predicted position
				FLOAT RATIO = 1/cond->flCumulative[NUM-1];	//total weight
				for(int jj=0;jj<NUM;jj++)
				{
					FLOAT conf = cond->flConfidence[jj];	//confidence
					if(conf>0.0)
					{
						float X = cond->flSamples[jj][0];	//position
						float Y = cond->flSamples[jj][1];
						NEXT_X+=conf*X*RATIO;				//calculate new position (consider weight)
						NEXT_Y+=conf*Y*RATIO;				//calculate new position (consider weight)
						CvPoint PP = cvPoint((int)X,(int)Y);
						cvCircle(Image,PP,2,COL);			//draw each particles
					}
				}

				//caluculate rectangle coordinate
				int *PP = CUR->point+ii*4;
				int WID = (*(PP+2)-*PP)/2;
				int HEI = (*(PP+3)-*(PP+1))/2;
				int *NPP = NEXT+ii*4;
				NPP[0]=(int)NEXT_X-WID; NPP[1]=(int)NEXT_Y-HEI;
				NPP[2]=(int)NEXT_X+WID;	NPP[3]=(int)NEXT_Y+HEI;
				CvPoint pp = cvPoint(NPP[0],NPP[1]);	//draw average particle position
				CvPoint pp2=cvPoint(NPP[2],NPP[3]);		//show rectangle predected for next frame
				cvRectangle(Image,pp,pp2,COL2,3);
				cvLine(Image,pp,pp2,COL2,2);
				pp = cvPoint(NPP[0],NPP[3]);
				pp2= cvPoint(NPP[2],NPP[1]);
				cvLine(Image,pp,pp2,COL2,2);
			}
		}
	}
	return(NEXT);
}

//show velocity-vector on image
int *show_vector_im(IplImage *Image,RESULT *CUR,PINFO *P_I,FLOAT ratio)
{
	//parameters
	const int height = Image->height;
	const int UpY = height/10;
	int *IM_V = (int *)calloc(CUR->num*4,sizeof(int));

	CvScalar COL = cvScalar(0.0,0.0,255.0);	//color of particles
	CvScalar GCOL = cvScalar(0.0,255.0,0.0);	//white
	for(int ii=0;ii<CUR->num;ii++)
	{
		if(P_I->se_num[ii]<1) continue;					//check "first" frame
		CvConDensation *cond = P_I->condens[ii];

		FLOAT NEXT_X = cond->State[0];	//predected position
		FLOAT NEXT_Y = cond->State[1];

		//caluculate rectangle coordinate
		int *PP = CUR->point+ii*4;
		int WID = (*(PP+2)-*PP)/2;
		int HEI = (*(PP+3)-*(PP+1))/2;

		//draw vector
		int LAS_X = (PP[2]+PP[0])/2;
		int LAS_Y = (PP[3]+PP[1])/2;

		int X_MOVE = (int)NEXT_X-LAS_X;
		int Y_MOVE = (int)NEXT_Y-LAS_Y;
		FLOAT M_LENGTH = sqrt((FLOAT)(X_MOVE*X_MOVE+Y_MOVE*Y_MOVE));


		//save vector coordinate
		IM_V[4*ii]=(int)((FLOAT)LAS_X/ratio);
		IM_V[4*ii+1]=(int)((FLOAT)LAS_Y/ratio)+UpY;
		IM_V[4*ii+2]=(int)(NEXT_X/ratio);
		IM_V[4*ii+3]=(int)(NEXT_Y/ratio)+UpY;

		CvPoint PLAS = cvPoint(IM_V[4*ii],IM_V[4*ii+1]);	//draw current rectangle position
		CvPoint PNEX = cvPoint(IM_V[4*ii+2],IM_V[4*ii+3]);		//show rectangle predected for next frame

		//int TEMP_X1 = P_I->L_P[ii][4];
		//int TEMP_Y1 = P_I->L_P[ii][5];
		//int TEMP_X2 = P_I->L_P[ii][6];
		//int TEMP_Y2 = P_I->L_P[ii][7];
		//printf("last [%d %d %d %d]\n ",TEMP_X1,TEMP_X2,TEMP_Y1,TEMP_Y2);
		//CvPoint p1=cvPoint(TEMP_X1,TEMP_Y1);
		//CvPoint p2=cvPoint(TEMP_X2,TEMP_Y2);
		//cvRectangle(Image,p1,p2,GCOL,3);			//draw current-object rectangle

		if(M_LENGTH<3 && CUR->type[ii]==1)	//static object
		{
			//for debug
			if(WID>50 && HEI>50)
			{
				cvCircle(Image,PLAS,10,GCOL,-1);
				IM_V[4*ii]=-1; IM_V[4*ii+1]=-1; IM_V[4*ii+2]=-1; IM_V[4*ii+3]=-1;
				continue;
			}
		}

		if(abs(X_MOVE)>0)
		{
			FLOAT theata = -atan((FLOAT)Y_MOVE/(FLOAT)X_MOVE);
			FLOAT XA1,ARL;

			if(M_LENGTH>10){XA1 =M_LENGTH-10; ARL=10.0;}
			else if(M_LENGTH>5){XA1 =M_LENGTH-5; ARL=5.0;}
			else {XA1 = M_LENGTH;ARL = 2.0;}

			FLOAT COS_T = cos(theata);
			FLOAT SIN_T = sin(theata);
			int XAA1,XAA2,YAA1,YAA2;
			int xmflag = 1;
			if(X_MOVE<0) xmflag =-1;

			if(CUR->type[ii]==0)	//side-object
			{
				LAS_X+=WID*xmflag;
				NEXT_X+=(FLOAT)(WID*xmflag);
				PLAS = cvPoint((int)((FLOAT)LAS_X/ratio),(int)((FLOAT)LAS_Y/ratio)+UpY);
				PNEX = cvPoint((int)(NEXT_X/ratio),(int)(NEXT_Y/ratio)+UpY);
			}
			//translate coodinate for array
			XAA1 =xmflag*(int)(COS_T*XA1+SIN_T*ARL)+LAS_X;
			YAA1 =xmflag*(int)(-SIN_T*XA1+COS_T*ARL)+LAS_Y;
			XAA2 =xmflag*(int)(COS_T*XA1+SIN_T*(-ARL))+LAS_X;
			YAA2 =xmflag*(int)(-SIN_T*XA1+COS_T*(-ARL))+LAS_Y;
			//draw line and array
			cvLine(Image,PLAS,PNEX,COL,5);
			//CvPoint PP = cvPoint(XAA1,YAA1);
			CvPoint PP = cvPoint((int)((FLOAT)XAA1/ratio),(int)((FLOAT)YAA1/ratio+UpY));
			cvLine(Image,PNEX,PP,COL,5);
			PP = cvPoint((int)((FLOAT)XAA2/ratio),(int)((FLOAT)YAA2/ratio+UpY));
			cvLine(Image,PNEX,PP,COL,5);
		}
		else if(abs(Y_MOVE)>0)
		{
			FLOAT ARL2;
			if(M_LENGTH>10) ARL2=10.0;
			else if(M_LENGTH>5) ARL2=5.0;
			else ARL2 = 2.0;
			if(Y_MOVE<0) ARL2*=-1;

			CvPoint PP2 = cvPoint((int)((NEXT_X+5.0)/ratio),(int)((NEXT_Y-ARL2)/ratio)+UpY);
			cvLine(Image,PNEX,PP2,COL,5);

			PP2 = cvPoint((int)((NEXT_X-5.0)/ratio),(int)((NEXT_Y-ARL2)/ratio)+UpY);
			cvLine(Image,PNEX,PP2,COL,5);
			cvLine(Image,PLAS,PNEX,COL,5);
		}
	}
	return IM_V;
}

//show velocity-vector on 2D MAP
void show_vector_2D(IplImage *MAP,IplImage *IM,RESULT *CUR,PINFO *P_I,int *I_VEC,FLOAT ratio)
{
	//Image information
	const int width = IM->width;

	FLOAT TAN_ANG = tan(VA/2/180*m_PI);

	//2D MAP information
	FLOAT X_RANGE = 2.0;
	FLOAT Y_RANGE = 4.0;
	int Y_OFFS = 20;
	int Y_OF_IM = MAP->height-Y_OFFS;
	int X_HAL = MAP->width/2;
	FLOAT Xratio=(FLOAT)X_HAL/X_RANGE;
	FLOAT Yratio=(FLOAT)(Y_OF_IM)/Y_RANGE;
	CvScalar COL = cvScalar(0.0,0.0,255.0);
	CvScalar COL2 = cvScalar(0.0,255.0,255.0);

	for(int kk=0;kk<CUR->num;kk++)
	{
		if(P_I->se_num[kk]<1) continue;
		//draw average point
		FLOAT X = P_I->ave_p[kk][1];
		FLOAT Y = P_I->ave_p[kk][0];
		int Ximg = X_HAL  -(int)(Xratio*X);
		int Yimg = Y_OF_IM-(int)(Yratio*Y);
		FLOAT D_RANGE = Y*TAN_ANG*2;
		FLOAT Pi_AX=(FLOAT)I_VEC[4*kk];
		FLOAT Pi_BX=(FLOAT)I_VEC[4*kk+2];

		FLOAT A_RATIO = (Pi_BX-Pi_AX)/(FLOAT)width;

		FLOAT XD = A_RATIO*D_RANGE;
		printf("A_RATIO %f D_RANGE %f XD %f\n",A_RATIO,D_RANGE,XD);
		FLOAT YD = P_I->ave_p[kk][2]-Y;

		CvPoint PC=cvPoint(Ximg,Yimg);
		cvCircle(MAP,PC,5,COL,-1);		//draw average vehicle point

		int XimgA = X_HAL  -(int)(Xratio*(X-XD));
		int YimgA = Y_OF_IM-(int)(Yratio*(Y-YD));

		CvPoint PC2=cvPoint(XimgA,YimgA);
		//cvCircle(MAP,PC2,2,COL2,-1);		//draw average vehicle point
		cvLine(MAP,PC,PC2,COL2,3);

		int X_MOVE = XimgA-Ximg;
		int Y_MOVE = YimgA-Yimg;
		int M_LENGTH = (int)(sqrt((FLOAT)(X_MOVE*X_MOVE+Y_MOVE*Y_MOVE)));

		if(abs(X_MOVE)>0)
		{
			int XA1;
			FLOAT ARL=5.0;
			FLOAT theata = -atan((FLOAT)Y_MOVE/(FLOAT)X_MOVE);
			FLOAT COS_T = cos(theata);
			FLOAT SIN_T = sin(theata);
			int xmflag = 1;
			if(X_MOVE<0) xmflag =-1;
			if(M_LENGTH>10)	{XA1 =M_LENGTH-10; ARL=10.0;}
			else			{XA1 = M_LENGTH-2;ARL = 5.0;}

			int XAA1 =xmflag*(int)(COS_T*XA1+SIN_T*ARL)+Ximg;
			int YAA1 =xmflag*(int)(-SIN_T*XA1+COS_T*ARL)+Yimg;
			int XAA2 =xmflag*(int)(COS_T*XA1+SIN_T*(-ARL))+Ximg;
			int YAA2 =xmflag*(int)(-SIN_T*XA1+COS_T*(-ARL))+Yimg;
			CvPoint PP = cvPoint(XAA1,YAA1);

			printf("%d %d %d %d\n",XAA1,YAA1,XimgA,YimgA);
			cvLine(MAP,PC2,PP,COL2,3);
			PP = cvPoint(XAA2,YAA2);
			cvLine(MAP,PC2,PP,COL2,3);
		}
	}
}

//show velocity-vector on image
void show_vector(IplImage *Image,IplImage *TMAP,RESULT *CUR,PINFO *P_I,FLOAT ratio)
{
	if(CUR->num>0)
	{
		int *I_VEC=show_vector_im(Image,CUR,P_I,ratio);
		show_vector_2D(TMAP,Image,CUR,P_I,I_VEC,ratio);
		s_free(I_VEC);
	}
}

//show likelihood (for debug)
void show_likelihood(IplImage *Image,CvMat *LIKE,int *COORD)
{
	IplImage *OIM = cvCreateImage(cvSize(Image->width,Image->height),Image->depth,1);

	int X = COORD[0]; int Y = COORD[1];
	FLOAT min_val=0,max_val=0;

	cvMinMaxLoc(LIKE,(double *)&min_val,(double *)&max_val);

	for(int ii=0;ii<LIKE->width;ii++)
	{
		for(int jj=0;jj<LIKE->height;jj++)
		{
			cvSetReal2D(OIM,Y+jj,X+ii,cvmGet(LIKE,jj,ii));
		}
	}

	cvReleaseImage(&OIM);
}

void show_det_score(IplImage *Image,FLOAT *ac_score,RESULT *CUR)
{
	IplImage *D_score = cvCreateImage(cvSize(Image->width,Image->height),Image->depth,3);
	int Step = D_score->widthStep;
	int chs = D_score->nChannels;
	FLOAT *TP = ac_score;

	if(CUR->num>0)
	{
		//draw detector score
		TP= ac_score;
		for(int ii=0;ii<Image->width;ii++)
		{
			for(int jj=0;jj<Image->height;jj++)
			{
				*(D_score->imageData+jj*Step+ii*chs)  =(int)(*(TP)*255.0);
				*(D_score->imageData+jj*Step+ii*chs+1)=(int)(*(TP)*255.0);
				*(D_score->imageData+jj*Step+ii*chs+2)=(int)(*(TP)*255.0);
				TP++;
			}
		}

		//draw rectangle
		for(int ii=0;ii<CUR->num;ii++)
		{
			int *P = CUR->point+4*ii;
			CvScalar col = get_color(CUR->type[ii]);
			CvPoint p1=cvPoint(P[0],P[1]);
			CvPoint p2=cvPoint(P[2],P[3]);
			cvRectangle(D_score,p1,p2,col,3);			//draw current-object rectangle
			cvLine(D_score,p1,p2,col,2);
			p1 = cvPoint(P[0],P[3]);
			p2 = cvPoint(P[2],P[1]);
			cvLine(D_score,p1,p2,col,2);
		}
		cvShowImage("Detector Score",D_score);	//show image
	}
	cvReleaseImage(&D_score );
}

//save_result
void save_result(IplImage *Image,int fnum)
{
	char pass[MAXLINE];
	char num[8];

	strcpy(pass, OUT_NAME);
	sprintf(num, "%d",fnum);

	strcat(pass, num);
	strcat(pass, EX_NAME);
	cvSaveImage(pass,Image);
}

//load_successive_image
IplImage *load_suc_image(int fnum)
{
	char pass[MAXLINE];
	char num[8];

	strcpy(pass, IN_S_NAME);
	sprintf(num, "%d",fnum);
	strcat(pass, num);
	strcat(pass, EX_NAME);
	printf("%s\n",pass);
	return(cvLoadImage(pass,CV_LOAD_IMAGE_COLOR));
}

//over-write detection result
void ovw_det_result(IplImage *OR,IplImage *DE, FLOAT ratio)
{
	//parameters
	const int height = OR->height;
	const int width = OR->width;
	const int UpY = height/10;
	const int NEW_Y = height-UpY-height/10;
	const int step = OR->widthStep;

	for(int ii=UpY;ii<NEW_Y+UpY-1;ii++)
	{
		int SY = (int)((FLOAT)(ii-UpY)*ratio);
		for(int jj=0;jj<width;jj++)
		{
			int SX = (int)((FLOAT)jj*ratio);
			for(int cc=0;cc<3;cc++)
			{
				unsigned char *col = (unsigned char *)(DE->imageData+SY*DE->widthStep+SX*3+cc);
				*(OR->imageData+ii*step+jj*3+cc)= *col;
			}
		}
	}
}
