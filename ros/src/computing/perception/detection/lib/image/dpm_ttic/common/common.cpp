/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "common.hpp"

void dpm_ttic_add_part_calculation(FLOAT *score, FLOAT*M,int *rootsize,int *partsize,int ax,int ay)
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

//initialize accumulated score
FLOAT *dpm_ttic_init_accumulated_score(IplImage *image, size_t& accumulated_size)
{
	size_t num = image->height * image->width;
	accumulated_size = num * sizeof(FLOAT);

	FLOAT *scores = (FLOAT *)calloc(num, sizeof(FLOAT));
	for(size_t i = 0; i < num; i++)
		scores[i] = -100.0;

	return scores;
}
