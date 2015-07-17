/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

/////nms.cpp   non_maximum suppression of detected box ////////////////////////////////////////////////////////////

//C++ include header
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "MODEL_info.h"		//File information
#include "switch_float.h"
#include "nms.hpp"

//inline function(Why does not use standard library ?)
static inline FLOAT max_d(FLOAT x,FLOAT y)
{
	return (x >= y ? x : y);
}

static inline FLOAT min_d(FLOAT x,FLOAT y)
{
	return (x <= y ? x : y);
}

//sub functions
static inline void exchangeOfValues(FLOAT *x , FLOAT *y)
{
	FLOAT tmp;
	tmp = *x; *x = *y ; *y = tmp;
	return;
}

static inline void exchangeOfOrders(int *x , int *y)
{
	int tmp;
	tmp = *x; *x = *y ; *y = tmp;
	return;
}

static void quickSort(FLOAT *ary , int *Order, int first_index , int last_index)
{
	int i = first_index, j = last_index;
	FLOAT key = *(ary + (first_index + last_index) / 2);

	while(1) {
		while (*(ary + i ) > key) i++;
		while (*(ary + j ) < key) j--;
		if (i >= j)
			break;
		exchangeOfValues(ary + i , ary + j);
		exchangeOfOrders(&Order[i],&Order[j]);
		i++; j--;
	}
	if (first_index < i - 1)
		quickSort(ary , Order,first_index , i - 1);
	if (last_index > j + 1)
		quickSort(ary , Order,j + 1 , last_index);
}

FLOAT *dpm_ttic_cpu_nms(FLOAT *boxes,FLOAT overlap,int *num,MODEL *MO)
{
	int NUM = *num;
	if (NUM <= 0)
		return nullptr;

	const int *numpart = MO->MI->numpart;
	const int GL = (numpart[0]+1)*4+3;
	FLOAT *area = (FLOAT*)calloc(NUM,sizeof(FLOAT));
	FLOAT *scores = (FLOAT*)calloc(NUM,sizeof(FLOAT));
	int *sorted_orders = (int *)calloc(NUM,sizeof(int));
	FLOAT *P=boxes;
	//calculate area of each boundary-box

	FLOAT *score_t = scores;
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
		FLOAT Ay1 = P[0];
		FLOAT Ax1 = P[1];
		FLOAT Ay2 = P[2];
		FLOAT Ax2 = P[3];
		checked[A]=1;
		cnum--;
		cur++;
		pi_num++;

		for(int kk=cur;kk<NUM;kk++)
		{
			int B = sorted_orders[kk];
			if(checked[B]!=0) continue;

			P=boxes+GL*B;
			FLOAT yy1 = max_d(Ay1,P[0]);
			FLOAT xx1 = max_d(Ax1,P[1]);
			FLOAT yy2 = min_d(Ay2,P[2]);
			FLOAT xx2 = min_d(Ax2,P[3]);
			FLOAT w = xx2-xx1+1.0;
			FLOAT h = yy2-yy1+1.0;
			FLOAT R_AREA = min_d(area[A],area[B]);

			//eliminate over-lap rectangles
			//over-lap(normal)
			if(w>0&&h>0)
			{
				FLOAT o = w*h/R_AREA;			//compute overlap
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
	FLOAT *Out = (FLOAT*)calloc(pi_num*GL,sizeof(FLOAT));
	int count =0,ii=0;
	FLOAT *Pout =Out;

	while(count<pi_num)
	{
		if(checked[ii]==1)
		{
			P = boxes+GL*ii;
			memcpy(Pout, P,sizeof(FLOAT)*GL);
			Pout+=GL;
			count++;
		}
		ii++;
	}
	//release
	free(area);
	free(scores);
	free(sorted_orders);
	free(checked);

	//Output
	*num = pi_num;
	return(Out);
}
