/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

/////dt.cpp   Decide best filter position by dynamic programing  //////////////////////////////////////////////////

#include <cstdio>
#include <cstdlib>

#include "switch_float.h"
#include "dt.hpp"

void add_part_calculation(FLOAT *score, FLOAT*M,int *rootsize,int *partsize,int ax,int ay)
{
	FLOAT *S = score;
	int jj_L = ax+2*(rootsize[1]-1)-1;
	int ii_L = ay+2*(rootsize[0]-1);
	int axm = ax-1;

	//add part score(resolution of part is 2x of root)
	for(int jj=axm;jj<=jj_L;jj+=2)
	{
		int L = jj*partsize[0];
		for(int ii=ay;ii<=ii_L;ii+=2)
		{
			*S -= M[ii+L-1];
			S++;
		}
	}
}
