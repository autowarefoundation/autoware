///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

/////resize.cpp   resize image (Input and Output must be FLOAT-array)

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "common.hpp"
#include "switch_float.h"
#include "resize.hpp"

// struct used for caching interpolation values
struct alphainfo {
	int si, di;
	FLOAT alpha;
};

//sub functions
// copy src into dst using precomputed interpolation values
static inline void alphacopy(FLOAT *src, FLOAT *dst, struct alphainfo *ofs, int n)
{
	struct alphainfo *end = ofs + n;
	while (ofs != end)
	{
		dst[ofs->di] += ofs->alpha * src[ofs->si];
		ofs++;
	}
}

// resize along each column
// result is transposed, so we can apply it twice for a complete resize
static void resize1dtran(FLOAT *src, int sheight, FLOAT *dst, int dheight, int width, int chan)
{
	FLOAT scale = (FLOAT)dheight/(FLOAT)sheight;
	FLOAT invscale = (FLOAT)sheight/(FLOAT)dheight;
	// we cache the interpolation values since they can be
	// shared among different columns
	int len = (int)(dheight*invscale+0.99) + 2*dheight;
	alphainfo *ofs;
	ofs = (alphainfo*)malloc(sizeof(alphainfo)*len);
	int WD =width*dheight;
	int WS =width*sheight;

	int k = 0;
	for (int dy = 0; dy < dheight; dy++)
	{
		FLOAT fsy1 = dy * invscale;
		FLOAT fsy2 = fsy1 + invscale;
		int sy1 = (int)(fsy1+0.99);
		int sy2 = (int)fsy2;
		int dyW = dy*width;

		if (sy1 - fsy1 > 1e-3) {
			ofs[k].di = dyW;
			ofs[k].si = sy1-1;
			ofs[k++].alpha = (sy1 - fsy1) * scale;
		}

		for (int sy = sy1; sy < sy2; sy++)
		{
			ofs[k].di = dyW;
			ofs[k].si = sy;
			ofs[k++].alpha = scale;
		}

		if (fsy2 - sy2 > 1e-3)
		{
			ofs[k].di = dyW;
			ofs[k].si = sy2;
			ofs[k++].alpha = (fsy2 - sy2) * scale;
		}
	}
	// resize each column of each color channel
	memset(dst,0, chan*WD*sizeof(FLOAT));
	for (int c = 0; c < chan; c++)
	{
		int CWS = c*WS;
		int CWD = c*WD;
		FLOAT *s = src + CWS;
		FLOAT *d = dst + CWD;
		for (int x = 0; x < width; x++) {
			alphacopy(s, d, ofs, k);
			s+=sheight;
			d++;
		}
	}

	free(ofs);
}

// main function (resize)
// takes a FLOAT color image and a scaling factor
// returns resized image
FLOAT *dpm_ttic_cpu_resize(FLOAT *src,int *sdims,int *odims,FLOAT scale)
{
	FLOAT *dst;
	if(scale==1.0)
	{
		memcpy(odims, sdims,sizeof(int)*3);
		int DL = odims[0]*odims[1]*odims[2];
		dst = (FLOAT*)calloc(DL,sizeof(FLOAT));
		memcpy(dst, src,sizeof(FLOAT)*DL);
	}
	else
	{
		odims[0] = (int)((FLOAT)sdims[0]*scale+0.5);
		odims[1] = (int)((FLOAT)sdims[1]*scale+0.5);
		odims[2] = sdims[2];
		dst = (FLOAT*)calloc(odims[0]*odims[1]*sdims[2],sizeof(FLOAT));
		FLOAT *tmp = (FLOAT*)calloc(odims[0]*sdims[1]*sdims[2],sizeof(FLOAT));
		resize1dtran(src, sdims[0], tmp, odims[0], sdims[1], sdims[2]);
		resize1dtran(tmp, sdims[1], dst, odims[1], odims[0], sdims[2]);
		free(tmp);
	}
	return(dst);
}
