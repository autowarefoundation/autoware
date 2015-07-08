/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

/////featurepyramid.cpp   calculate HOG-feature pyramid ///////////////////////////////////////////////////////////

//OpenCV library
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

//C++ library
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <time.h>
#include <iostream>
using namespace std;

//Header files
#include "MODEL_info.h"		//File information
#include "Common.h"

#include <pthread.h>

#include "switch_float.h"

#ifndef WIN32
#define __stdcall void*
typedef void *HANDLE;
typedef long LONG_PTR;
#define INVALID_HANDLE_VALUE ((HANDLE)(LONG_PTR)-1)
#endif

//definition of constant
#define eps 0.0001

//definition of sin and cos
const FLOAT Hcos[9]={1.0000,0.9397,0.7660,0.5000,0.1736,-0.1736,-0.5000,-0.7660,-0.9397};
const FLOAT Hsin[9]={0.0000,0.3420,0.6428,0.8660,0.9848,0.9848,0.8660,0.6428,0.3420};

//definition of structure
struct thread_data {
	FLOAT *IM;
	int ISIZE[3];
	int FSIZE[2];
	int F_C;
	int sbin;
	FLOAT *Out;
};

//inline functions

static inline int max_i(int x,int y);									//return maximum number (integer)
static inline int min_i(int x,int y);									//return minimum number (integer)
static inline FLOAT min_2(FLOAT x);									//compare FLOAT with 0.2

//initialization functions
FLOAT *ini_scales(Model_info *MI,IplImage *IM,int X,int Y);		//initialize scales (extended to main)
int *ini_featsize(Model_info *MI);					//initialize feature size information matrix (extended to main)

//subfunction
FLOAT *Ipl_to_FLOAT(IplImage *Input);						//get intensity data (FLOAT) of input
void free_features(FLOAT **features,Model_info *MI);				//release features
FLOAT *calc_feature(FLOAT *SRC,int *ISIZE,int *FTSIZE,int sbin);		//calculate HOG features
void ini_thread_data(thread_data *TD,FLOAT *IM,int *INSIZE,int sbin,int level);	//for thread-initialization
void* feat_calc(void *thread_arg); //for thread_process

//main function to calculate feature pyramid
FLOAT **calc_f_pyramid(IplImage *Image,Model_info *MI,int *FTSIZE,FLOAT *scale);		//calculate feature pyramid (extended to detect.c)

//external function

//resize.cpp
extern FLOAT *resize(FLOAT *src,int *sdims,int *odims,FLOAT scale); //resize image

//inline functions

//return maximum number (integer)
static inline int max_i(int x,int y) {return (x >= y ? x : y); }

//return minimum number (integer)
static inline int min_i(int x,int y) {return (x <= y ? x : y); }

//return minimum number (FLOAT)
static inline FLOAT min_2(FLOAT x) {return (x <= 0.2 ? x :0.2); }


//initialization functions

//initialize scales
FLOAT *ini_scales(Model_info *MI,IplImage *IM,int X,int Y) //X,Y length of image
{
	int interval,max_scale;

	if(MI->ini)
	{
		//calculate max scale
		//MI->interval/=2;	//reduce calculation time
		const int sbin = MI->sbin;
		interval = MI->interval;
		const FLOAT sc = pow(2.0,(1/(double)interval));//縮小比を表している。
		const int numcomponent = MI->numcomponent;
		max_scale = 36;
		const int L_NUM = interval+max_scale;

		FLOAT MRY =(FLOAT)MI->rsize[0];
		FLOAT MRX =(FLOAT)MI->rsize[1];

		for(int kk=1;kk<numcomponent;kk++)
		{
			if(MI->rsize[kk*2]<MRY) MRY=MI->rsize[kk*2];
			if(MI->rsize[kk*2+1]<MRX) MRX=MI->rsize[kk*2+1];
		}

		MRY/=2;
		MRX/=2;

		FLOAT height =(FLOAT)IM->height/(FLOAT)sbin;
		FLOAT width = (FLOAT)IM->width/(FLOAT)sbin;
		FLOAT sc_step =1/sc;   //縮小率

		for(int kk=0;kk<L_NUM;kk++)
		{
			height*=sc_step;
			width*=sc_step;
			if(height<MRY || width<MRX)
			{
				max_scale = kk-interval-1;
				break;
			}
		}

		if(max_scale<interval) max_scale = interval;
		MI->max_scale=max_scale;
		printf("max_scale:%d\n",max_scale);
		MI->IM_HEIGHT=IM->height;
		/*printf("高さ%d\n",MI->IM_HEIGHT);*/
		MI->IM_WIDTH=IM->width;
		/*printf("横%d\n",MI->IM_WIDTH);*/
		MI->ini=false;
	}
	else
	{
		interval = MI->interval;
		max_scale = MI->max_scale;
		MI->IM_HEIGHT = IM->height;
		MI->IM_WIDTH = IM->width;
	}

	//return
	FLOAT *scales = (FLOAT*)calloc((max_scale+interval),sizeof(FLOAT));		//Model information
	return(scales);
}

//initialize feature size matrix

int *ini_featsize(Model_info *MI)
{
	const int LofFeat=MI->max_scale+MI->interval;
	int *featsize = (int*)calloc(LofFeat*2,sizeof(FLOAT)); // feature size information matrix
	return(featsize);
}

//calculate HOG features from Image
//HOG features are calculated for each block(BSL*BSL pixels)
FLOAT *calc_feature(FLOAT *SRC,int *ISIZE,int *FTSIZE,int sbin)
{
	//input size
	const int height=ISIZE[0]; //{268,268,134,67,233,117,203,203,177,154,89,203,154,77}
	const int width=ISIZE[1];  //{448,112,224,390,195,340,170,296,257,148,340,257,129}
	const int dims[2]={height,width};

	//size of Histgrams and Norm calculation space size
	const int blocks[2] = {(int)floor(double(height)/double(sbin)+0.5),(int)floor(double(width)/double(sbin)+0.5)};//{67,112}....sbine=4
	const int BLOCK_SQ = blocks[0]*blocks[1];//{7504}...
	const int BX = blocks[0]+1;//68...

	//Output features size(Output)
	const int OUT_SIZE[3]={max_i(blocks[0]-2,0),max_i(blocks[1]-2,0),27+4};//{65,110,31}.....
	const int O_DIM=OUT_SIZE[0]*OUT_SIZE[1];//{7150}.....
	const int DIM_N =9*BLOCK_SQ;//{67536}

	//Visible range (eliminate border blocks)
	const int visible[2]={blocks[0]*sbin,blocks[1]*sbin};
	const int vis_R[2] ={visible[0]-1,visible[1]-1};
	const int vp0=dims[0]-2;
	const int vp1=dims[1]-2;
	const int SQUARE =dims[0]*dims[1];
	const FLOAT SBIN = FLOAT(sbin);


	//HOG Histgram and Norm
	FLOAT *HHist = (FLOAT*)calloc(BLOCK_SQ*18,sizeof(FLOAT));	// HOG histgram
	FLOAT *Norm = (FLOAT*)calloc(BLOCK_SQ,sizeof(FLOAT));		// Norm

	//feature(Output)
	FLOAT *feat=(FLOAT*)calloc(OUT_SIZE[0]*OUT_SIZE[1]*OUT_SIZE[2],sizeof(FLOAT));

	//calculate HOG histgram
	for(int x=1;x<vis_R[1];x++)
	{
		//parameters for interpolation
		FLOAT xp=((FLOAT)x+0.5)/SBIN-0.5;
		int ixp=(int)floor(xp);
		int ixpp=ixp+1;
		int ixp_b  = ixp * blocks[0];
		int ixpp_b = ixp_b + blocks[0];
		FLOAT vx0=xp-(FLOAT)ixp;
		FLOAT vx1=1.0-vx0;
		bool flag1=true,flag2=true,flagX=true;
		if(ixp<0) {flag1=false;flagX=false;}
		if(ixpp>=blocks[1]) {flag2=false;flagX=false;}
		int YC=min_i(x,vp1)*dims[0];
		FLOAT *SRC_YC = SRC+YC;

		for(int y=1;y<vis_R[0];y++)
		{
			//first color channel
			FLOAT *s=SRC_YC+min_i(y,vp0);
			FLOAT dy=*(s+1)-*(s-1);
			FLOAT dx=*(s+dims[0])-*(s-dims[0]);
			FLOAT v=dx*dx+dy*dy;

			//second color channel
			s+=SQUARE;
			FLOAT dy2=*(s+1)-*(s-1);
			FLOAT dx2=*(s+dims[0])-*(s-dims[0]);
			FLOAT v2=dx2*dx2+dy2*dy2;

			//third color channel
			s+=SQUARE;
			FLOAT dy3=*(s+1)-*(s-1);
			FLOAT dx3=*(s+dims[0])-*(s-dims[0]);
			FLOAT v3=dx3*dx3+dy3*dy3;

			//pick channel with strongest gradient
			if(v2>v){v=v2;dx=dx2;dy=dy2;}
			if(v3>v){v=v3;dx=dx3;dy=dy3;}

			FLOAT best_dot=0.0;
			int best_o=0;

			//snap to one of 18 orientations
			for(int o=0;o<9;o++)
			{
				FLOAT dot=Hcos[o]*dx+Hsin[o]*dy;
				if(dot>best_dot)		{best_dot=dot;best_o=o;}
				else if (-dot>best_dot)	{best_dot=-dot;best_o=o+9;}
			}

			//Add to 4 histgrams around pixel using linear interpolation
			FLOAT yp=((FLOAT)y+0.5)/SBIN-0.5;
			int iyp=(int)floor(yp);
			int iypp=iyp+1;
			FLOAT vy0=yp-(FLOAT)iyp;
			FLOAT vy1=1.0-vy0;
			v=sqrt(v);
			int ODim=best_o*BLOCK_SQ;
			FLOAT *Htemp = HHist+ODim;
			FLOAT vx1Xv =vx1*v;
			FLOAT vx0Xv = vx0*v;

			if(flagX)
			{
				if(iyp>=0)
				{
					*(Htemp+ ixp_b+iyp)+=vy1*vx1Xv; //1-少数をxyでかけたものにエッジ強度の2乗をかけたもの
					*(Htemp+ ixpp_b+iyp)+=vy1*vx0Xv;
				}
				if (iypp<blocks[0])
				{
					*(Htemp+ ixp_b+iypp)+=vy0*vx1Xv;
					*(Htemp+ ixpp_b+iypp)+=vy0*vx0Xv;
				}
			}
			else if(flag1)
			{
				if (iyp>=0) *(Htemp+ixp_b+iyp)+=vy1*vx1Xv;
				if (iypp<blocks[0]) *(Htemp+ixp_b+iypp)+=vy0*vx1Xv;
			}
			else if(flag2)
			{
				if(iyp>=0) *(Htemp+ixpp_b+iyp)+=vy1*vx0Xv;
				if(iypp<blocks[0]) *(Htemp+ixpp_b+iypp)+=vy0*vx0Xv;
			}
		}
	}

	//compute energy in each block by summing over orientations
	for(int kk=0;kk<9;kk++)
	{
		FLOAT *src1=HHist+kk*BLOCK_SQ;
		FLOAT *src2=src1+DIM_N;
		FLOAT *dst=Norm;
		FLOAT *end=Norm+BLOCK_SQ;
		while(dst<end)
		{
			FLOAT sss=*src1+*src2;
			*(dst++)+=sss*sss;
			src1++;src2++;
		}
	}

	//compute features
	for(int x=0;x<OUT_SIZE[1];x++)
	{
		FLOAT *dst_X = feat+x*OUT_SIZE[0];
		int BB = x*blocks[0];
		int BA = BB+blocks[0];

		FLOAT *pt = Norm+BA;
		FLOAT nc1 = 1.0/sqrt(*pt+*(pt+1)+*(pt+blocks[0])+*(pt+BX)+eps);
		pt = Norm+BB;
		FLOAT nc3 = 1.0/sqrt(*pt+*(pt+1)+*(pt+blocks[0])+*(pt+BX)+eps);

		for(int y=0;y<OUT_SIZE[0];y++)
		{
			FLOAT *dst=dst_X+y;
			FLOAT *src,*p,n1,n2,n3,n4;
			//calculate normarize factor
			int yp = y+1;

			p=Norm+BA+yp;
			n1 = 1.0/sqrt(*p+*(p+1)+*(p+blocks[0])+*(p+BX)+eps);
			n2 = nc1;
			nc1 = n1;

			p=Norm+BB+yp;
			n3 = 1.0/sqrt(*p+*(p+1)+*(p+blocks[0])+*(p+BX)+eps);
			n4 = nc3;
			nc3 = n3;

			//features for each orientations
			FLOAT t1=0,t2=0,t3=0,t4=0;

			//contrast-sensitive features(18)
			src=HHist+BA+yp;
			for(int kk=0;kk<18;kk++)
			{
				FLOAT h1=min_2(*src*n1);
				FLOAT h2=min_2(*src*n2);
				FLOAT h3=min_2(*src*n3);
				FLOAT h4=min_2(*src*n4);
				*dst=0.5*(h1+h2+h3+h4);
				t1+=h1;
				t2+=h2;
				t3+=h3;
				t4+=h4;
				dst+=O_DIM;
				src+=BLOCK_SQ;
			}

			//contrast-insensitive features(9)
			src=HHist+BA+yp;
			for(int kk=0;kk<9;kk++)
			{
				FLOAT sum = *src+*(src+DIM_N);
				FLOAT h1=min_2(sum*n1);
				FLOAT h2=min_2(sum*n2);
				FLOAT h3=min_2(sum*n3);
				FLOAT h4=min_2(sum*n4);
				*dst=0.5*(h1+h2+h3+h4);
				dst+=O_DIM;
				src+=BLOCK_SQ;
			}

			//texture gradient
			*dst=0.2357*t1;
			dst+=O_DIM;
			*dst=0.2357*t2;
			dst+=O_DIM;
			*dst=0.2357*t3;
			dst+=O_DIM;
			*dst=0.2357*t4;
		}
	}

	//Release
	s_free(HHist);
	s_free(Norm);

	//size of feature(output)
	*FTSIZE=OUT_SIZE[0];
	*(FTSIZE+1)=OUT_SIZE[1];
	//    printf("feat%f\n",*(feat));
	return(feat);
}

//sub functions

// get pixel-intensity(FLOAT)  of image(IplImage)

FLOAT *Ipl_to_FLOAT(IplImage *Input)	//get intensity data (FLOAT) of input
{
	const int width = Input->width;
	printf("%d\n",width);
	const int height = Input->height;
	printf("%d\n",height);
	const int nChannels = Input->nChannels;
	printf("%d\n",nChannels);
	const int SQ = height*width;
	const int WS = Input->widthStep;

	FLOAT *Output = (FLOAT *)malloc(sizeof(FLOAT)*height*width*nChannels);
	printf("%d",height*width*nChannels);

	FLOAT *R= Output;
	FLOAT *G= Output+SQ;
	FLOAT *B= Output+2*SQ;
	char *IDATA = Input->imageData;

	//pick intensity of pixel (color)
	for(int x=0;x<width;x++)
	{
		int XT = x*3;
		for(int y=0;y<height;y++)
		{
			int pp = WS*y+XT;
			*(B++)=(FLOAT)(unsigned char)IDATA[pp];	//B
			pp++;
			*(G++)=(FLOAT)(unsigned char)IDATA[pp];	//G
			pp++;
			*(R++)=(FLOAT)(unsigned char)IDATA[pp];	//R
		}
	}
	return(Output);
}

// feature calculation
//unsigned __stdcall feat_calc(void *thread_arg)
void* feat_calc(void *thread_arg)
{
	thread_data *args = (thread_data *)thread_arg;
	FLOAT *Out =calc_feature(args->IM,args->ISIZE,args->FSIZE,args->sbin);
	args->Out =Out;
	//_endthreadex(0);
	//return(0);
	pthread_exit((void*)thread_arg);
}

//void initialize thread data
void ini_thread_data(thread_data *TD,FLOAT *IM,int *INSIZE,int sbin,int level)
{
	TD->IM=IM;
	//memcpy_s(TD->ISIZE,sizeof(int)*3,INSIZE,sizeof(int)*3);
	memcpy(TD->ISIZE, INSIZE,sizeof(int)*3);
	TD->FSIZE[0]=0;
	TD->FSIZE[1]=0;
	TD->sbin=sbin;
	TD->F_C=level;
}

//calculate feature pyramid (extended to main.cpp)
FLOAT **calc_f_pyramid(IplImage *Image,Model_info *MI,int *FTSIZE,FLOAT *scale)	//calculate feature pyramid
{
	//constant parameters
	const int max_scale = MI->max_scale;
	const int interval = MI->interval;
	const int sbin = MI->sbin;
	const int sbin2 = (int)floor((double)sbin/2.0);
	const int LEN = max_scale+interval;
	const FLOAT sc = pow(2,(1.0/(double)interval));
	int INSIZE[3]={Image->height,Image->width,Image->nChannels};
	int RISIZE[3]={0,0,0},OUTSIZE[3] ={0,0,0};

	//Original image (FLOAT)
	FLOAT *D_I = Ipl_to_FLOAT(Image);

	//features
	FLOAT **feat=(FLOAT**)malloc(sizeof(FLOAT*)*LEN);		//Model information

	//thread for feature calculation
	thread_data *td = (thread_data *)calloc(LEN, sizeof(thread_data));
	//HANDLE *ts = (HANDLE *)calloc(LEN, sizeof(HANDLE));
	pthread_t *ts = (pthread_t *)calloc(LEN, sizeof(HANDLE));

	FLOAT **RIM_S =(FLOAT**)calloc(LEN,sizeof(FLOAT*));

	int *RI_S = (int*)calloc(interval*3,sizeof(int));
	FLOAT *RIM_T;
	int t_count=0;

	//calculate resized image
	for(int ii=0;ii<interval;ii++)
	{
		FLOAT st = 1.0/pow(sc,ii);
		RIM_S[ii] = resize(D_I,INSIZE,RISIZE,st);
		memcpy(RI_S+ii*3, RISIZE,sizeof(int)*3);
	}

	for(int ii=0;ii<interval;ii++)
	{
		FLOAT st = 1.0/pow(sc,ii);
		memcpy(RISIZE, RI_S+ii*3,sizeof(int)*3);

		//"first" 2x interval
		ini_thread_data(&td[t_count],RIM_S[ii],RISIZE,sbin2,ii);  //initialize thread
		if( pthread_create(&ts[t_count], NULL, feat_calc, (void*)&td[t_count]))
		{printf("Error thread\n");exit(0);}
		*(scale+ii)=st*2;									//save scale
		t_count++;

		//"second" 1x interval
		RIM_S[ii+interval]=RIM_S[ii];

		ini_thread_data(&td[t_count],RIM_S[ii+interval],RISIZE,sbin,ii+interval);	//initialize thread
		//ts[t_count]=(HANDLE)_beginthreadex(NULL,0,feat_calc,(void*)&td[t_count],0,&threadID);
		if(pthread_create(&ts[t_count], NULL, feat_calc, (void*)&td[t_count]))
			//if (ts[t_count]==INVALID_HANDLE_VALUE)
		{printf("Error thread\n");exit(0);}
		*(scale+ii+interval)=st;							//save scale
		t_count++;

		//remained resolutions (for root_only)
		RIM_T = RIM_S[ii];		//get original image (just a copy)
		for(int jj=ii+interval;jj<max_scale;jj+=interval)
		{
			RIM_S[jj+interval] = resize(RIM_T,RISIZE,OUTSIZE,0.5);			//resize image (FLOAT)
			//memcpy_s(RISIZE,sizeof(int)*3,OUTSIZE,sizeof(int)*3);
			memcpy(RISIZE, OUTSIZE,sizeof(int)*3);
			ini_thread_data(&td[t_count],RIM_S[jj+interval],RISIZE,sbin,jj+interval); //initialize thread
			//ts[t_count]=(HANDLE)_beginthreadex(NULL,0,feat_calc,(void*)&td[t_count],0,&threadID);
			if(pthread_create(&ts[t_count], NULL, feat_calc, (void*)&td[t_count]))
				//if (ts[t_count]==INVALID_HANDLE_VALUE)
			{printf("Error thread\n");exit(0);}
			*(scale+jj+interval)=0.5*(*(scale+jj));			//save scale
			RIM_T = RIM_S[jj+interval];
			t_count++;
		}
	}


	//get thread data
	for(int ss=0;ss<LEN;ss++)
	{
		pthread_join(ts[ss], NULL);
		feat[td[ss].F_C]=td[ss].Out;
		memcpy(&FTSIZE[td[ss].F_C*2], td[ss].FSIZE,sizeof(int)*2);
	}

	//release original image
	s_free(D_I);

	//release resized image
	for(int ss=0;ss<interval;ss++) s_free(RIM_S[ss]);
	for(int ss=interval*2;ss<LEN;ss++) s_free(RIM_S[ss]);
	s_free(RI_S);
	s_free(RIM_S);

	//release thread information
	s_free(td);
	s_free(ts);

	return(feat);
}

//release feature pyramid
void free_features(FLOAT **features,Model_info *MI)
{
	int LofFeat=MI->max_scale+MI->interval;
	if(features!=NULL)
	{
		for (int ii=0;ii<LofFeat;ii++)
		{
			s_free(features[ii]);
		}
		s_free(features);
	}
}

FLOAT *calc_feature1(FLOAT *SRC,int *ISIZE,int *FTSIZE,int sbin)
{
	//input size
	const int height=ISIZE[0];
	const int width=ISIZE[1];
	const int dims[2]={height,width};

	//size of Histgrams and Norm calculation space size
	const int blocks[2] = {(int)floor(double(height)/double(sbin)+0.5),(int)floor(double(width)/double(sbin)+0.5)};
	const int BLOCK_SQ = blocks[0]*blocks[1];

	//Output features size(Output)
	const int OUT_SIZE[3]={max_i(blocks[0]-2,0),max_i(blocks[1]-2,0),27+4};
	const int O_DIM=OUT_SIZE[0]*OUT_SIZE[1];
	const int DIM_N =9*BLOCK_SQ;

	//Visible range (eliminate border blocks)
	const int visible[2]={blocks[0]*sbin,blocks[1]*sbin};
	const int vis_R[2] ={visible[0]-1,visible[1]-1};
	const int BLOCK_SQ2 = vis_R[0]*vis_R[1];
	const int vp0=dims[0]-2;
	const int vp1=dims[1]-2;
	const int SQUARE =dims[0]*dims[1];
	const FLOAT SBIN = FLOAT(sbin);

	//HOG Histgram and Norm
	FLOAT *HHist = (FLOAT*)calloc(BLOCK_SQ*18,sizeof(FLOAT));	// HOG histgram
	FLOAT *Norm = (FLOAT*)calloc(BLOCK_SQ,sizeof(FLOAT));		// Norm
	int X;int Y,A,B;

	int BBQ =2* visible[0] * visible[1];
	FLOAT *hozonn=(FLOAT*)calloc(BBQ,sizeof(FLOAT));
	int *locate=(int*)calloc(BBQ/2,sizeof(int));
	int *locate2=(int*)calloc(BLOCK_SQ ,sizeof(int));

	int iii;
	for (iii=0;iii<BBQ;iii++){
		*(hozonn+iii)=0;
	}

	//feature(Output)
	FLOAT *feat=(FLOAT*)calloc(OUT_SIZE[0]*OUT_SIZE[1]*OUT_SIZE[2],sizeof(FLOAT));

	//calculate HOG histgram
	for (X=1;(X+1)<blocks[1];X++)
	{
		for(Y=1;(Y+1)<blocks[0];Y++)
		{
			int a1 = X-1;
			int a = (X-1) * sbin - sbin/2;
			int aa1 = X+2;
			int aa = (X+2) * sbin + sbin/2;
			if (X == 1){a = 1;aa1 = 0;}
			if ((X+2)==blocks[1]){aa = vis_R[1]; aa1 =blocks[1];}
			int b1 = Y-1;
			int b = (Y-1) * sbin - sbin/2;
			int bb1 = Y+2;
			int bb = (Y+2) * sbin + sbin/2;
			if (Y == 1){b = 1;bb1 = 0;}
			if ((Y+2)==blocks[0]){bb =vis_R[0]; bb1 =blocks[0];}

			for(int x=a;x<aa;x++)
			{
				//parameters for interpolation
				FLOAT xp=((FLOAT)x+0.5)/SBIN-0.5;
				int ixp=(int)floor(xp);
				int ixpp=ixp+1;
				int ixp_b  = ixp * blocks[0];
				int ixpp_b = ixp_b + blocks[0];
				FLOAT vx0=xp-(FLOAT)ixp;
				FLOAT vx1=1.0-vx0;
				bool flag1=true,flag2=true,flagX=true;
				if(ixp<0) {flag1=false;flagX=false;}
				if(ixpp>=blocks[1]) {flag2=false;flagX=false;}
				int YC=min_i(x,vp1)*dims[0];
				FLOAT *SRC_YC = SRC+YC;

				for(int y=b;y<bb;y++)
				{
					if (*(locate+x*(vis_R[0])+y) == 1)
						break;

					//first color channel
					FLOAT *s=SRC_YC+min_i(y,vp0);
					FLOAT dy=*(s+1)-*(s-1);
					FLOAT dx=*(s+dims[0])-*(s-dims[0]);
					FLOAT v=dx*dx+dy*dy;

					//second color channel
					s+=SQUARE;
					FLOAT dy2=*(s+1)-*(s-1);
					FLOAT dx2=*(s+dims[0])-*(s-dims[0]);
					FLOAT v2=dx2*dx2+dy2*dy2;

					//third color channel
					s+=SQUARE;
					FLOAT dy3=*(s+1)-*(s-1);
					FLOAT dx3=*(s+dims[0])-*(s-dims[0]);
					FLOAT v3=dx3*dx3+dy3*dy3;

					//pick channel with strongest gradient
					if(v2>v){v=v2;dx=dx2;dy=dy2;}
					if(v3>v){v=v3;dx=dx3;dy=dy3;}

					FLOAT best_dot=0.0;
					int best_o=0;

					//snap to one of 18 orientations
					for(int o=0;o<9;o++)
					{
						FLOAT dot=Hcos[o]*dx+Hsin[o]*dy;
						if(dot>best_dot)		{best_dot=dot;best_o=o;}
						else if (-dot>best_dot)	{best_dot=-dot;best_o=o+9;}
					}

					*(hozonn+x*(vis_R[0])+y) = v;
					*(hozonn+x*(vis_R[0])+y+BLOCK_SQ2) = best_o;
					*(locate+x*(vis_R[0])+y) = 1;

					//Add to 4 histgrams around pixel using linear interpolation
					FLOAT yp=((FLOAT)y+0.5)/SBIN-0.5;
					int iyp=(int)floor(yp);
					int iypp=iyp+1;
					FLOAT vy0=yp-(FLOAT)iyp;
					FLOAT vy1=1.0-vy0;
					v=sqrt(v);
					int ODim=best_o*BLOCK_SQ;
					FLOAT *Htemp = HHist+ODim;
					FLOAT vx1Xv =vx1*v;
					FLOAT vx0Xv = vx0*v;

					if(flagX)
					{
						if(iyp>=0)
						{
							*(Htemp+ ixp_b+iyp)+=vy1*vx1Xv;
							*(Htemp+ ixpp_b+iyp)+=vy1*vx0Xv;
						}
						if (iypp<blocks[0])
						{
							*(Htemp+ ixp_b+iypp)+=vy0*vx1Xv;
							*(Htemp+ ixpp_b+iypp)+=vy0*vx0Xv;
						}
					}
					else if(flag1)
					{
						if (iyp>=0) *(Htemp+ixp_b+iyp)+=vy1*vx1Xv;
						if (iypp<blocks[0]) *(Htemp+ixp_b+iypp)+=vy0*vx1Xv;
					}
					else if(flag2)
					{
						if(iyp>=0) *(Htemp+ixpp_b+iyp)+=vy1*vx0Xv;
						if(iypp<blocks[0]) *(Htemp+ixpp_b+iypp)+=vy0*vx0Xv;
					}
				}
			}

			//compute energy in each block by summing over orientations
			FLOAT *ttt;
			for(A= a1;A<aa1;A++)
			{
				for(B=b1;B<bb1;B++)
				{
					for(int kk=0;kk<9;kk++)
					{
						if (*(locate2+A*blocks[0]+B) == 1){break;}
						ttt = Norm+blocks[0]*A+B;
						FLOAT *src1=HHist+blocks[0]*A+B+kk*BLOCK_SQ;
						FLOAT *src2=src1+DIM_N;
						FLOAT sss=*src1+*src2;
						*ttt+=sss*sss;
					}
					*(locate2+A*blocks[0]+B) = 1;
				}
			}
		}
	}

	//compute features
	FLOAT n1,n2,n3,n4;
	for(int X=0;X<OUT_SIZE[1];X++)
	{
		for(int Y=0;Y<OUT_SIZE[0];Y++)
		{
			FLOAT *dst = feat+X*OUT_SIZE[0]+Y;
			FLOAT *pt = Norm+(X+1)*blocks[0]+(Y+1);
			n4 = 1.0/sqrt(*pt+*(pt-1)+*(pt-blocks[0])+*(pt-blocks[0]-1)+eps);
			n2 = 1.0/sqrt(*pt+*(pt-1)+*(pt+blocks[0])+*(pt+blocks[0]-1)+eps);
			n3 = 1.0/sqrt(*pt+*(pt+1)+*(pt-blocks[0])+*(pt+blocks[0]+1)+eps);
			n1 = 1.0/sqrt(*pt+*(pt+1)+*(pt+blocks[0])+*(pt+blocks[0]+1)+eps);

			//features for each orientations
			FLOAT t1=0,t2=0,t3=0,t4=0;

			//contrast-sensitive features(18)
			FLOAT *src=HHist+(X+1)*blocks[0]+Y+1;
			for(int kk=0;kk<18;kk++)
			{
				FLOAT h1=min_2(*src*n1);
				FLOAT h2=min_2(*src*n2);
				FLOAT h3=min_2(*src*n3);
				FLOAT h4=min_2(*src*n4);
				*dst=0.5*(h1+h2+h3+h4);
				t1+=h1;
				t2+=h2;
				t3+=h3;
				t4+=h4;
				dst+=O_DIM;
				src+=BLOCK_SQ;
			}

			//contrast-insensitive features(9)
			src=HHist+(X+1)*blocks[0]+Y+1;
			for(int kk=0;kk<9;kk++)
			{
				FLOAT sum = *src+*(src+DIM_N);
				FLOAT h1=min_2(sum*n1);
				FLOAT h2=min_2(sum*n2);
				FLOAT h3=min_2(sum*n3);
				FLOAT h4=min_2(sum*n4);
				*dst=0.5*(h1+h2+h3+h4);
				dst+=O_DIM;
				src+=BLOCK_SQ;
			}

			//texture gradient
			*dst=0.2357*t1;
			dst+=O_DIM;
			*dst=0.2357*t2;
			dst+=O_DIM;
			*dst=0.2357*t3;
			dst+=O_DIM;
			*dst=0.2357*t4;
		}
	}

	//Release
	s_free(HHist);
	s_free(Norm);
	s_free(hozonn);
	s_free(locate);
	s_free(locate2);

	//size of feature(output)
	*FTSIZE=OUT_SIZE[0];
	*(FTSIZE+1)=OUT_SIZE[1];

	//	printf("feat%f\n",*(feat));
	return(feat);
}
