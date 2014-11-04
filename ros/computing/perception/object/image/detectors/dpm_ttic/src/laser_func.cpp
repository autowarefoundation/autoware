///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////Laser_sub.cpp  functions about laser_rader data processing  ///////////////////////////////////////

//C++ library
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

//original headers
#include "Common.h"
#include "F_info.h"
#include "MODEL_info.h"
#include "Laser_info.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//definition of functions

char *get_file_pass(char *lname);													//get file pass
SCANDATA *save_sdata(double *fdata,int SNUM,IplImage *IM);							//save file information
void Release_sdata(SCANDATA *sdata);												//release scan point data
CvScalar get_c_color(int coltype);													//get color for each channel
CvScalar get_c_mono(int coltype);
IplImage *ini_Image(int WID,int HEI);												//initialize zero-image
void radar_data_fusion(SCANDATA *sdata,IplImage* IM,RESULT *RES,PINFO *PI);			//draw scan point on image 
void draw_point_on_image(SCANDATA *sdata,IplImage* IM,RESULT *RES);					//draw scan-point on image
IplImage *draw_point_MAP2D(SCANDATA *sdata,char *WNAME);							//draw sacn-point 2D map
void skip_laser_frame(FILE* fp,int sk_num,int *fnum);								//skip_laser_frame
void skip_image_frame(CvCapture *capt,int sk_num);									//skip_image_frame
void get_f_size(FILE* fp,fpos_t *curpos,fpos_t *fsize);								//get file size 
SCANDATA *get_s_data(FILE *fp,IplImage *IMG,fpos_t *curpos);						//get scan-point data	
IplImage *draw_sdata(SCANDATA *sdata,IplImage* IM,RESULT *RES);						//visualize scan data
void skip_data(FILE* fp,CvCapture *capt,int sk_num,int *fnum);						//skip data (for debug)
void skip_data_2(FILE* fp,int sk_num,int *ss);										
IplImage *combine_image (int num, IplImage ** tmp);									//combine image

char b_cmp[]={0xAF,0xFE,0xC0,0xC0,0xAF,0xFE,0xC0,0xC0};								//magic words of laser-lader data
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//get file pass
char *get_file_pass(char lname[])
{
  int LDP_LENGTH=strlen(lname)+strlen(L_DATA_PASS)+200;
  char *ldpass=(char *)calloc(LDP_LENGTH,sizeof(char));
  //strcpy_s(ldpass,sizeof(L_DATA_PASS),L_DATA_PASS);
  strcpy(ldpass, L_DATA_PASS);
  //strcat_s(ldpass,LDP_LENGTH-strlen(L_DATA_PASS)-1,lname);
  strcat(ldpass, lname);
  return ldpass;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//save file information
SCANDATA *save_sdata(double *fdata,int SNUM,IplImage *IM)
{
	SCANDATA *s_data = (SCANDATA *)calloc(1,sizeof(SCANDATA));
	int		*ctype	= (int *)calloc(SNUM,sizeof(int));			//channel type of each scan point
	double *xyzdata	= (double *)calloc(SNUM*3,sizeof(double));	//(X,Y,Z) data of each scan point
	int		*imxy	= (int *)calloc(SNUM*2,sizeof(int));		//(X,Y) image-coordinate of each scan point
	int		*Cclass = (int *)calloc(SNUM,sizeof(int));			//car_class information

	//get scan point data (channel and xyz data)
	for(int ii=0;ii<SNUM;ii++)
	{
		ctype[ii]		=(int)fdata[ii*4];		//save channel type
		xyzdata[ii*3]	=fdata[ii*4+1];	//save X
		xyzdata[ii*3+1]	=fdata[ii*4+2];	//save Y
		xyzdata[ii*3+2]	=fdata[ii*4+3];	//save Y
	}

	const double iw = (double)IM->width;
	const double ih = (double)IM->height;
	const double cha_fact = cha*m_PI/180.0;
	const double cva_fact = cva*m_PI/180.0;
	const double va_fact	= tan(m_PI*VA/360.0);

	double IW_2 = iw/2.0 - 0.5;
	double IH_2 = ih/2.0 - 0.5;

	//get (X,Y) image coordinate data
	for(int ii=0;ii<SNUM;ii++)
	{
		double X = xyzdata[ii*3];
		double Y = xyzdata[ii*3+1];
		double Z = xyzdata[ii*3+2];

		double YmCYP = Y-cyp;
		double XmCXP = X-cxp;
		double ZmCZP = Z-czp;

		//calculate image coordinate (X,Y)
		double ix = IW_2 - (IW_2)*tan(asin(YmCYP/sqrt(YmCYP*YmCYP+XmCXP*XmCXP))-cha_fact)/va_fact;
		double iy = IH_2 - (IW_2)*tan(asin(ZmCZP/sqrt(ZmCZP*ZmCZP+XmCXP*XmCXP))-cva_fact)/va_fact;
		imxy[ii*2]=(int)ix;
		imxy[ii*2+1]=(int)iy;
	}

	//substitution
	s_data->SNUM=SNUM;
	s_data->Ctype=ctype;
	s_data->XYZdata=xyzdata;
	s_data->XYIM=imxy;
	s_data->CCLASS=Cclass;

	return(s_data);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//release scan point data
void Release_sdata(SCANDATA *sdata)
{
	s_free(sdata->Ctype);
	s_free(sdata->XYZdata);
	s_free(sdata->XYIM);
	s_free(sdata->CCLASS);
	s_free(sdata);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//get color for each channel(select for each channel-type 1to4)
CvScalar get_c_color(int coltype)
{
	CvScalar COL;
	switch(coltype)
	{
		case 0: //Orange
			COL = cvScalar(0.0,140.0,255.0);
			break;
		case 1:	//Water-Blue
			COL = cvScalar(255.0,160.0,0.0);
			break;
		case 2: //Purple
			COL = cvScalar(255.0,80.0,255.0);
			break;
		case 3: //Yellow
			COL = cvScalar(0.0,255.0,255.0);
			break;
	}
	return(COL);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//get mono-color level for each channel(select for each channel-type 1to4)
CvScalar get_c_mono(int coltype)
{
	CvScalar COL;
	switch(coltype)
	{
		case 0: 
			COL = cvScalar(25.0,50.0,25.0);
			break;
		case 1:	
			COL = cvScalar(50.0,100.0,50.0);
			break;
		case 2:
			COL = cvScalar(75.0,150.0,75.0);
			break;
		case 3:
			COL = cvScalar(125.0,255.0,125.0);
			break;
	}
	return(COL);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//initialize image 
IplImage *ini_Image(int WID,int HEI)
{
	IplImage *IMG = cvCreateImage(cvSize(WID,HEI),IPL_DEPTH_8U,3);
	int STEP =IMG->widthStep;

	for(int ii=0;ii<HEI;ii++)
	{
		for(int jj=0;jj<WID;jj++)
		{
			IMG->imageData[ii*STEP+jj*3]=0;
			IMG->imageData[ii*STEP+jj*3+1]=0;
			IMG->imageData[ii*STEP+jj*3+2]=0;
		}
	}
	return(IMG);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//draw scan point on image 
void radar_data_fusion(SCANDATA *sdata,IplImage* IM,RESULT *RES,PINFO *PI)
{
	for(int ii=0;ii<sdata->SNUM;ii++) sdata->CCLASS[ii]=-1;
	//clasification of data
	for(int kk=0;kk<RES->num;kk++)
	{
		int *PP = RES->OR_point+kk*4;
		int X1 = PP[0]; int Y1 = PP[1];
		int X2 = PP[2]; int Y2 = PP[3];

		for(int ii=0;ii<sdata->SNUM;ii++)
		{
			int X= sdata->XYIM[2*ii];
			int Y= sdata->XYIM[2*ii+1];
			if(X>X1 && X<X2 && Y>Y1 && Y<Y2)
			{
				CvScalar COL = get_c_color(sdata->Ctype[ii]);
				sdata->CCLASS[ii]=kk;
			}
		}
	}

	//get average position
	for(int kk=0;kk<RES->num;kk++)
	{
		double ave_XN =0.0; double ave_YN =0.0;				//average (X,Y) of each vehicle (include noise)
		double AX = 0.0,AY=0.0;								//average (X,Y) of each vehicle (without noise)
		int c_count =0;

		for(int ii=0;ii<sdata->SNUM;ii++)
		{
			if(sdata->CCLASS[ii]==kk)
			{
				int SS = 3*ii;
				c_count++;
				ave_XN+=sdata->XYZdata[SS];
				ave_YN+=sdata->XYZdata[SS+1];
			}
		}

		//eliminate noise
		if(c_count>5)
		{
			ave_XN/=(double)c_count; ave_YN/=(double)c_count;	//average (X,Y) of each vehicle (TEMP)
			int CC = 0;
			
			for(int ii=0;ii<sdata->SNUM;ii++)
			{
				if(sdata->CCLASS[ii]==kk)
				{
					int SS = 3*ii;
					double CuX = sdata->XYZdata[3*ii];
					double CuY = sdata->XYZdata[3*ii+1];
					if(CuX<(ave_XN-0.6) || CuX>(ave_XN+0.6) || CuY<(ave_YN-0.6) || CuY>(ave_YN+0.6)) {sdata->CCLASS[ii]=-1;}
					else {AX+=sdata->XYZdata[SS];AY+=sdata->XYZdata[SS+1];CC++;}
				}
			}
			if(CC>0) {AX/=(double)CC; AY/=(double)CC;}
			else     {AX = -1.0;AY=-1.0;}
		}
		else{AX = -1.0;AY=-1.0;}

		PI->ave_p[kk][0]=AX;
		PI->ave_p[kk][1]=AY;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//draw scan point on image 
void draw_point_on_image(SCANDATA *sdata,IplImage* IM,RESULT *RES)
{

	//draw scan point data
	for(int ii=0;ii<sdata->SNUM;ii++)
	{
		int X= sdata->XYIM[2*ii];
		int Y= sdata->XYIM[2*ii+1];

		//get color for each channel
		if(sdata->CCLASS[ii]==-1)
		{
			if(X>=0 && X<IM->width && Y>=0 && Y<IM->height)
			{
				CvScalar COL = get_c_mono(sdata->Ctype[ii]);
				CvPoint PP=cvPoint(X,Y);
				cvCircle(IM,PP,2,COL,-1);	//draw circle
			}
			else {sdata->CCLASS[ii]=-2;}
		}
		else
		{
			CvScalar COL = get_c_color(sdata->Ctype[ii]);
			CvPoint PP=cvPoint(X,Y);
			cvCircle(IM,PP,2,COL,-1);	//draw circle	
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//draw scan point MAP on 2D
IplImage *draw_point_MAP2D(SCANDATA *sdata)
{
	CvScalar COL;
	IplImage *IM = ini_Image(480,480);

	//drawing-range of (X,Y) coordinate
	double X_RANGE = 2.0;
	double Y_RANGE = 4.0;
	//drawing-offset of Y
	int Y_OFFS = 20;
	int Y_OF_IM = IM->height-Y_OFFS;

	int X_HAL = IM->width/2;
	double Xratio=(double)X_HAL/X_RANGE;	
	double Yratio=(double)(Y_OF_IM)/Y_RANGE;

	//draw camera position
	CvPoint PP=cvPoint(X_HAL,IM->height-Y_OFFS);
	cvCircle(IM,PP,4,cvScalar(255.0,255.0,255.0),-1);

	//draw angle-grid
	CvPoint PP2=cvPoint(0,50);
	cvLine(IM,PP,PP2,cvScalar(100.0,100.0,100.0),1);
	PP2=cvPoint(480,50);
	cvLine(IM,PP,PP2,cvScalar(100.0,100.0,100.0),1);

	//draw grid line (X)
	for(int ii=0;ii<X_RANGE*4;ii++)
	{
		CvPoint PPA=cvPoint((int)Xratio*ii/2,IM->height);
		CvPoint PPB=cvPoint((int)Xratio*ii/2,0);
		cvLine(IM,PPA,PPB,cvScalar(100.0,100.0,100.0),1);
	}
	//draw grid line (Y)
	for(int ii=0;ii<Y_RANGE*2;ii++)
	{
		CvPoint PPA=cvPoint(0,Y_OF_IM-(int)Yratio*ii/2);
		CvPoint PPB=cvPoint(IM->width,Y_OF_IM-(int)Yratio*ii/2);
		cvLine(IM,PPA,PPB,cvScalar(100.0,100.0,100.0),1);
	}

	//draw each scan point
	for(int ii=0;ii<sdata->SNUM;ii++)
	{
		double Y= sdata->XYZdata[3*ii];
		double X= sdata->XYZdata[3*ii+1];
		int CC = sdata->CCLASS[ii];

		if(abs(X)<X_RANGE && Y<Y_RANGE && CC>=-1)
		{
			int Ximg = X_HAL  -(int)(Xratio*X);
			int Yimg = Y_OF_IM-(int)(Yratio*Y);
			if		(CC==-1) COL = get_c_mono(sdata->Ctype[ii]);
			//else if	(CC>=0)	 //COL = get_c_color(sdata->Ctype[ii]);
			else if	(CC>=0)	 COL = cvScalar(255.0,0.0,255.0);

			CvPoint PC=cvPoint(Ximg,Yimg);
			cvCircle(IM,PC,1,COL,-1);
		}
	}
	return IM;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//skip laser pointer data (for synchronization and debug)
void skip_laser_frame(FILE* fp,int sk_num,int *fnum)
{
	const char b_cmp[]={0xAF,0xFE,0xC0,0xC0,0xAF,0xFE,0xC0,0xC0};	//comparing array 
	char mword[]={0xAF,0xFE,0xC0,0xC0,0xAF,0xFE,0xC0,0xC0};	//magic words of laser-lader data
	int b_locate;	//get number of all elements ([Channel,X,Y,Z]*scanpoint)
	double temp;
	for(int ii=0;ii<sk_num;ii++)
	{
		//get magic-word of file
		fread(&mword,sizeof(double),1,fp);
		//check magic-word
		if(strncmp(&mword[0],&b_cmp[0],8)!=0){printf("magic word error\n");return;}
		//get size of laser-points int time T
		fread(&temp,sizeof(double),1,fp);
		b_locate = (int)temp;		//get scan-data size 
		//access scan-point data 
		double *spoint = (double *)calloc(b_locate,sizeof(double));
		fread(spoint,sizeof(double),b_locate,fp);
		s_free(spoint);
		*(fnum)+=1;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//skip laser pointer data (for synchronization and debug)
void skip_image_frame(CvCapture *capt,int sk_num)
{
	for(int ii=0;ii<sk_num;ii++)
	{
		IplImage *IMG;
		if((IMG=cvQueryFrame(capt))==NULL){printf("end of movie\n");break;}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ファイルの位置の設定
//get file size
void get_f_size(FILE* fp,fpos_t *curpos,fpos_t *fsize)
{
	fseek(fp,0,SEEK_END);   //ファイルの終端位置へ移動
	fgetpos(fp,fsize);		//get file size 
	fseek(fp,0,SEEK_SET);   //ファイルの先端位置を指定
	fgetpos(fp,curpos);		//get current file position
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//get scan-point data(includes Magic word checking)
SCANDATA *get_s_data(FILE *fp,IplImage *IMG,fpos_t *curpos)
{
	//char mword[]={0xAF,0xFE,0xC0,0xC0,0xAF,0xFE,0xC0,0xC0};	//magic words of laser-lader data
	int b_locate;	//get number of all elements ([Channel,X,Y,Z]*scanpoint)
	int SNUM;		//number of scan point
	//double temp;

	//get magic-word of file
	fread(&SNUM,sizeof(int),1,fp);
	//check magic word
	//if(strncmp(&mword[0],&b_cmp[0],8)!=0){printf("magic word error\n");exit(EXIT_FAILURE);}

	//get number of all-elements	
	//fread(&temp,sizeof(double),1,fp);
	//b_locate = (int)temp;				//get scan-data size 	
	b_locate = 4*SNUM;					//get number of scan point
	//access scan-point data 	
	double *spoint = (double *)calloc(b_locate,sizeof(double));		//scanpoint data
	fread(spoint,sizeof(double),b_locate,fp);						//read scanpoint data
	//get scan point data (# of data, (X,Y,Z)data, (X,Y)data on image)
	SCANDATA *Sdata =save_sdata(spoint,SNUM,IMG);
	s_free(spoint);
	//get current file-access-position (for termination)
	fgetpos(fp,curpos);

	return Sdata;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//scan-data visualization (draw on image and 2D mapping)
IplImage *draw_sdata(SCANDATA *sdata,IplImage* IM,RESULT *RES)
{
	//Scan-point visualization
	draw_point_on_image(sdata,IM,RES);						//draw scan-points on Image 	
	IplImage *TDMAP = draw_point_MAP2D(sdata);		//draw scan-points on 2D MAP 
	return TDMAP;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//skip data and image (for debug)
void skip_data(FILE* fp,CvCapture *capt,int sk_num,int *fnum)
{
	const char b_cmp[]={0xAF,0xFE,0xC0,0xC0,0xAF,0xFE,0xC0,0xC0};	//comparing array 
	char mword[]={0xAF,0xFE,0xC0,0xC0,0xAF,0xFE,0xC0,0xC0};	//magic words of laser-lader data
	int b_locate;	//get number of all elements ([Channel,X,Y,Z]*scanpoint)
	double temp;
	for(int ii=0;ii<sk_num;ii++)
	{
		IplImage *IMG;
		//get magic-word of file
		fread(&mword,sizeof(double),1,fp);
		if(strncmp(&mword[0],&b_cmp[0],8)!=0){printf("magic word error\n");return;}
		fread(&temp,sizeof(double),1,fp);
		b_locate = (int)temp;		//get scan-data size 
		//access scan-point data 
		double *spoint = (double *)calloc(b_locate,sizeof(double));
		fread(spoint,sizeof(double),b_locate,fp);
		s_free(spoint);
		if((IMG=cvQueryFrame(capt))==NULL){printf("end of movie\n");break;}
		*(fnum)+=1;
	}
}

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
void skip_data_2(FILE* fp,int sk_num,int *ss)
{
	int b_locate;	//get number of all elements ([Channel,X,Y,Z]*scanpoint)

	
	for(int ii=0;ii<sk_num;ii++)
	{
		fread(&b_locate,sizeof(int),1,fp);
		//printf("b_locate:%d\n",b_locate);
		b_locate = b_locate*4;
		//printf("b_locate:%d\n",b_locate);
		//access scan-point data 
		double *spoint = (double *)calloc(b_locate,sizeof(double));
		fread(spoint,sizeof(double),b_locate,fp);
		//printf("spoint0:%f spoint1:%f spoint2:%f spoint3:%f spoint4:%f\n",*spoint,*(spoint+1),*(spoint+2),*(spoint+3),*(spoint+4));
		s_free(spoint);
	}

	*ss = sk_num;
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////

/* 画像を連結する関数 */
IplImage *combine_image (int num, IplImage ** tmp)
{   int i;
	int width = 0, height = 0;
	IplImage *cimg;
	CvRect roi = cvRect (0, 0, 0, 0);  // (3)与えられた各画像から，連結後の幅と高さを求める
	for (i = 0; i < num; i++) {
		width += tmp[i]->width;
		height = height < tmp[i]->height ? tmp[i]->height : height;
	}
	cimg = cvCreateImage (cvSize (width, height), IPL_DEPTH_8U, 3);
	cvZero (cimg);  // (4)ROIを利用して各画像をコピーする
	for (i = 0; i < num; i++) {
		roi.width = tmp[i]->width;
		roi.height = tmp[i]->height;
		cvSetImageROI (cimg, roi);
		cvCopy (tmp[i], cimg);
		roi.x += roi.width;
	}
	cvResetImageROI (cimg);
	return cimg;
}



