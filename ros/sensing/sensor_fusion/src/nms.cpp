///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////nms.cpp   non_maximum suppression of detected box ////////////////////////////////////////////////////////////

//C++ include header
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "MODEL_info.h"		//File information
#include "Common.h"


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//definiton of functions//

//inline function
static inline double max_d(double x,double y);
static inline double min_d(double x,double y);

//sub functions
inline void exchangeOfValues(double *x , double *y);		//sub function for quick-sort
inline void exchangeOfOrders(int *x , int *y);				//sub function for quick-sort
void  quickSort(double *ary , int *Order, int first_index , int last_index);		//quick sort function

//Non_maximum suppression function (extended to detect.cc)
double *nms(double *boxes,double overlap,int *num,MODEL *MO);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//inline function
static inline double max_d(double x,double y) {return (x >= y ? x : y); }
static inline double min_d(double x,double y) {return (x <= y ? x : y); }


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//sub functions

inline void exchangeOfValues(double *x , double *y) {
	double tmp;
	tmp = *x; *x = *y ; *y = tmp;
	return;
}

inline void exchangeOfOrders(int *x , int *y) {
	int TMP;
	TMP = *x; *x = *y ; *y = TMP;
	return;
}

//Quick sort function
void  quickSort(double *ary , int *Order, int first_index , int last_index) {
	int i = first_index, j = last_index;
	double key = *(ary + (first_index + last_index) / 2);

	while(1) {
		while (*(ary + i ) > key) i++;
		while (*(ary + j ) < key) j--;
		if (i >= j) break;
		exchangeOfValues(ary + i , ary + j);
		exchangeOfOrders(&Order[i],&Order[j]);
		i++; j--;
	}
	if (first_index < i - 1) quickSort(ary , Order,first_index , i - 1);
	if (last_index > j + 1) quickSort(ary , Order,j + 1 , last_index);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double *nms(double *boxes,double overlap,int *num,MODEL *MO)
{
  int NUM = *num;
  if(NUM<=0) return(NULL);
  else
    {
      const int *numpart = MO->MI->numpart;
      const int GL = (numpart[0]+1)*4+3;
      double *area = (double*)calloc(NUM,sizeof(double));
      double *scores = (double*)calloc(NUM,sizeof(double));
      int *sorted_orders = (int *)calloc(NUM,sizeof(int));
      double *P=boxes;
      //calculate area of each boundary-box
      
      double *score_t = scores;
      int *so_t =sorted_orders; 
      for(int ii=0;ii<NUM;ii++)
        {
          area[ii]=(P[3]-P[1]+1.0)*(P[2]-P[0]+1.0);
          *(score_t++) = P[GL-2];
          //??printf("amari %f\n",P[GL-1]);
          *(so_t++)=ii;
          P+=GL;
        }
      
      quickSort(scores,sorted_orders,0,NUM-1);	//sort score of detected rectangle
      
      int *checked = (int *)calloc(NUM,sizeof(int));		//cheked flags (picked =1,non-checked =0,suppressed =-1)
      int cnum =NUM-1;
      int cur=0;
      int pi_num=0;
      
      while(cnum>0)
        {
          int A=sorted_orders[cur];
          P=boxes+GL*A;
          double Ay1 = P[0];
          double Ax1 = P[1];
          double Ay2 = P[2];
          double Ax2 = P[3];
          checked[A]=1;
          cnum--;
          cur++;
          pi_num++;
          
          for(int kk=cur;kk<NUM;kk++)
            {
              int B = sorted_orders[kk];
              if(checked[B]!=0) continue;
              
              P=boxes+GL*B;
              double yy1 = max_d(Ay1,P[0]);
              double xx1 = max_d(Ax1,P[1]);
              double yy2 = min_d(Ay2,P[2]);
              double xx2 = min_d(Ax2,P[3]);
              double w = xx2-xx1+1.0;
              double h = yy2-yy1+1.0;
              double R_AREA = min_d(area[A],area[B]);
              
              //eliminate over-lap rectangles
              //over-lap(normal)
              if(w>0&&h>0)
                {
                  //double o = w*h/area[A];		//compute overlap
                  double o = w*h/R_AREA;			//compute overlap
                  if(o>overlap)
                    {
                      checked[B]=-1;	//suppress
                      cnum--;
                    }
                }
              //full over-lap
              else if(Ay1<P[0]&&Ax1<P[1]&&Ay2>P[2]&&Ax2>P[3])
                {
                  checked[B]=-1;	//suppress	
                  cnum--;
                }
              else if(Ay1>P[0]&&Ax1>P[1]&&Ay2<P[2]&&Ax2<P[3])
                {
                  checked[B]=-1;	//suppress	
                  cnum--;
                }
            }
          
          //decide next current
          for(int kk=cur;kk<NUM;kk++)
            {
              if(checked[sorted_orders[kk]]==0)
                {
                  cur=kk;
                  break;
                }
            }
          
        }
      
      //get result (rectangle-box coordinate)
      double *Out = (double*)calloc(pi_num*GL,sizeof(double));
      int count =0,ii=0;
      double *Pout =Out;
      
      while(count<pi_num)
        {
          if(checked[ii]==1)
            {
              P = boxes+GL*ii;
              //memcpy_s(Pout,sizeof(double)*GL,P,sizeof(double)*GL);
              memcpy(Pout, P,sizeof(double)*GL);
              Pout+=GL;
              count++;
            }
          ii++;
        }
      //release
      s_free(area);
      s_free(scores);
      s_free(sorted_orders);
      s_free(checked);
      
      //Output 
      *num = pi_num;
      return(Out);
    }
}
