// Car tracking project with laser_radar_data_fusion
// Copyright 2009-10 Akihiro Takeuchi

/////dt.cpp   Decide best filter position by dynamic programing 

//C++ library
#include <cstdio>
#include <cstdlib>

#include "switch_float.h"
#include "dt.hpp"

static inline void dt_helper(FLOAT *src, FLOAT *dst, int *ptr, int step, int s1, int s2, int d1, int d2, FLOAT a, FLOAT b)
{
	if (d2 >= d1)
	{
		int d = (d1+d2) >> 1;
		int ds =d*step;
		int s = s1;
		FLOAT src_ss = *(src+s*step);
		for (int p = s1+1; p <= s2; p++)
		{
			int t1 = d-s;
			int t2 = d-p;
			if (src_ss + a*t1*t1 + b*t1 > *(src+p*step) + a*t2*t2 + b*t2)
			{
				s = p;
				src_ss = *(src+s*step);
			}
		}
		int D = d-s;
		dst[ds] = *(src+s*step) + a*D*D + b*D;
		ptr[ds] = s;
		dt_helper(src, dst, ptr, step, s1, s, d1, d-1, a, b);
		dt_helper(src, dst, ptr, step, s, s2, d+1, d2, a, b);
	}
}

//sub function of dt
static void dt1d(FLOAT *src, FLOAT *dst, int *ptr, int step, int n, FLOAT a, FLOAT b)
{
	dt_helper(src, dst, ptr, step, 0, n-1, 0, n-1, a, b);
}

//Decide best filter position by dynamic programing
FLOAT *dpm_ttic_cpu_dt(FLOAT *vals,FLOAT ax,FLOAT bx,FLOAT ay,FLOAT by,int *dims,int *Ix,int *Iy)
{
	const int SQ = dims[0]*dims[1];
	FLOAT *M = (FLOAT*)malloc(sizeof(FLOAT)*SQ);
	FLOAT *tmpM = (FLOAT*)malloc(sizeof(FLOAT)*SQ);
	int *tmpIx = (int*)malloc(sizeof(int)*SQ);
	int *tmpIy = (int*)malloc(sizeof(int)*SQ);
	int XD=0;

	for (int x = 0; x < dims[1]; x++)
	{
		dt1d(vals+XD, tmpM+XD, tmpIy+XD, 1, dims[0], ay, by);
		XD+=dims[0];
	}
	for (int y = 0; y < dims[0]; y++)
	{
		dt1d(tmpM+y, M+y, tmpIx+y, dims[0], dims[1], ax, bx);
	}

	int *IX_P = Ix;
	int *IY_P = Iy;
	int *tmpIx_P=tmpIx;
	for (int x = 0; x < dims[1]; x++)
	{
		for (int y = 0; y < dims[0]; y++)
		{
			*(IX_P++) = *tmpIx_P;
			*(IY_P++) = tmpIy[*tmpIx_P*dims[0]+y];
			tmpIx_P++;
		}
	}

	free(tmpM);
	free(tmpIx);
	free(tmpIy);
	return(M);
}
