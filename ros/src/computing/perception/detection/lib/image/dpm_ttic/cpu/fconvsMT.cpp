///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

/////fconvsMT.cpp  convolute features and filter  /////////////////////////////////////////////////////////////////

//C++ library (thread-functions are only supported by windows)
#include <cstdio>
#include <cstdlib>

#include <pthread.h>

//Original header
#include "MODEL_info.h"		//File information
#include "common.hpp"
#include "switch_float.h"

struct thread_data {
	FLOAT *A;
	FLOAT *B;
	FLOAT *C;
	FLOAT *F;
	FLOAT *T;
	int A_dims[3];
	int B_dims[3];
	int C_dims[2];
};

//thread process
// convolve A and B(non_symmetric)
static void* process(void *thread_arg) {
	thread_data *args = (thread_data *)thread_arg;
	FLOAT *A = args->A;	//feature
	FLOAT *B = args->B;	//filter
	FLOAT *C = args->C;	//output
	int *A_dims = args->A_dims;
	int *B_dims = args->B_dims;
	int *C_dims = args->C_dims;
	int num_features = A_dims[2];

	const int A_SQ = A_dims[0]*A_dims[1];
	const int B_SQ = B_dims[0]*B_dims[1];

	for (int f = 0; f < num_features; f++)
	{
		FLOAT *dst = C;
		FLOAT *A_src = A + f*A_SQ;
		FLOAT *B_src = B + f*B_SQ;
		int XA0 = 0;
		for (int x = 0; x < C_dims[1]; x++)
		{
			FLOAT *A_src2 =A_src+XA0;
			XA0+=A_dims[0];
			for (int y = 0; y < C_dims[0]; y++)
			{
				FLOAT val = 0;
				FLOAT *A_off = A_src2+y;
				FLOAT *B_off = B_src;
				for (int xp = 0; xp < B_dims[1]; xp++)
				{
					switch(B_dims[0])
					{
					case 20: val += A_off[19] * B_off[19];
					case 19: val += A_off[18] * B_off[18];
					case 18: val += A_off[17] * B_off[17];
					case 17: val += A_off[16] * B_off[16];
					case 16: val += A_off[15] * B_off[15];
					case 15: val += A_off[14] * B_off[14];
					case 14: val += A_off[13] * B_off[13];
					case 13: val += A_off[12] * B_off[12];
					case 12: val += A_off[11] * B_off[11];
					case 11: val += A_off[10] * B_off[10];
					case 10: val += A_off[9] * B_off[9];
					case 9: val += A_off[8] * B_off[8];
					case 8: val += A_off[7] * B_off[7];
					case 7: val += A_off[6] * B_off[6];
					case 6: val += A_off[5] * B_off[5];
					case 5: val += A_off[4] * B_off[4];
					case 4: val += A_off[3] * B_off[3];
					case 3: val += A_off[2] * B_off[2];
					case 2: val += A_off[1] * B_off[1];
					case 1: val += A_off[0] * B_off[0];
						break;
					default:
						FLOAT *A_temp = A_off;
						FLOAT *B_temp = B_off;
						for (int yp = 0; yp < B_dims[0]; yp++)
						{
							val += *(A_temp++) * *(B_temp++);
						}
					}
					A_off+=A_dims[0];
					B_off+=B_dims[0];
				}
				*(dst++) += val;
			}
		}
		A_src+=A_SQ;
		B_src+=B_SQ;
	}

	pthread_exit((void*)thread_arg);
	return nullptr;
}

// convolve A and B when B is symmetric
static void* processS(void *thread_arg)
{
	thread_data *args = (thread_data *)thread_arg;
	FLOAT *A = args->A;
	FLOAT *B = args->B;
	FLOAT *C = args->C;
	FLOAT *F = args->F;
	FLOAT *T = args->T;
	int *A_dims = args->A_dims;
	int *B_dims = args->B_dims;
	int *C_dims = args->C_dims;
	int num_features = A_dims[2];
	int width1 = (int)(B_dims[1]/2.0+0.99);
	int width2 = (int)(B_dims[1]/2.0);

	const int A_SQ = A_dims[0]*A_dims[1];
	const int B_SQ = B_dims[0]*B_dims[1];
	const int T_L  = width2*A_dims[0];
	const int CP_L = width1*A_dims[0];
	const int XF_L = A_dims[1]-width1-width2;
	const int CP_L_S = CP_L*sizeof(FLOAT);

	for (int f = 0; f < num_features; f++)
	{
		FLOAT *dst = C;
		FLOAT *A_src = A + f*A_SQ;
		FLOAT *B_src = B + f*B_SQ;
		FLOAT *F_src = F + f*A_SQ;
		int XA = 0;
		for (int x = 0; x < C_dims[1]; x++)
		{
			// generate tmp data for band of output
			memcpy(T, A_src + XA, CP_L_S);
			XA+=A_dims[0];
			int xf = XF_L-x;
			FLOAT *copy_dst = T;
			FLOAT *copy_end = T + T_L;
			FLOAT *copy_src = F_src + xf*A_dims[0];
			while (copy_dst < copy_end)
			{
				*copy_dst += *copy_src;
				copy_dst++;
				copy_src++;
			}

			for (int y = 0; y < C_dims[0]; y++)
			{
				FLOAT val = 0;
				FLOAT *T_off = T+y;
				FLOAT *B_off = B_src;
				for (int xp = 0; xp < width1; xp++)
				{
					switch(B_dims[0]) {
					case 20: val += T_off[19] * B_off[19];
					case 19: val += T_off[18] * B_off[18];
					case 18: val += T_off[17] * B_off[17];
					case 17: val += T_off[16] * B_off[16];
					case 16: val += T_off[15] * B_off[15];
					case 15: val += T_off[14] * B_off[14];
					case 14: val += T_off[13] * B_off[13];
					case 13: val += T_off[12] * B_off[12];
					case 12: val += T_off[11] * B_off[11];
					case 11: val += T_off[10] * B_off[10];
					case 10: val += T_off[9] * B_off[9];
					case 9: val += T_off[8] * B_off[8];
					case 8: val += T_off[7] * B_off[7];
					case 7: val += T_off[6] * B_off[6];
					case 6: val += T_off[5] * B_off[5];
					case 5: val += T_off[4] * B_off[4];
					case 4: val += T_off[3] * B_off[3];
					case 3: val += T_off[2] * B_off[2];
					case 2: val += T_off[1] * B_off[1];
					case 1: val += T_off[0] * B_off[0];
						break;
					default:
						FLOAT *T_temp = T_off;
						FLOAT *B_temp = B_off;
						for (int yp = 0; yp < B_dims[0]; yp++)
						{
							val += *(T_temp++) * *(B_temp++);
						}
					}
					T_off+=A_dims[0];
					B_off+=B_dims[0];
				}

				*(dst++) += val;
			}
		}
	}

	pthread_exit((void*)thread_arg);

	return nullptr;
}

//Input(feat,flipfeat,filter,symmetric info,1,length)
//Output Score
FLOAT **dpm_ttic_cpu_fconvsMT(FLOAT*feat,FLOAT*flfeat,FLOAT**filter,int *sym_info,
			      int start,int end,int *A_SIZE,int **B_SIZE,int *M_size)
{
	start=start-1;
	end=end-1;

	const int len=end-start+1;
	FLOAT **Output=(FLOAT**)malloc(sizeof(FLOAT*)*len);		//Output (cell)
	// start threads
	thread_data *td = (thread_data *)calloc(len, sizeof(thread_data));
	pthread_t *ts = (pthread_t *)calloc(len, sizeof(pthread_t));

	for(int ii=0;ii<len;ii++)
	{
		td[ii].A=feat;
		td[ii].B=filter[ii+start];
		td[ii].F=flfeat;
		td[ii].A_dims[0]=A_SIZE[0];
		td[ii].A_dims[1]=A_SIZE[1];
		td[ii].A_dims[2]=31;
		td[ii].B_dims[0]=B_SIZE[ii][0];
		td[ii].B_dims[1]=B_SIZE[ii][1];
		td[ii].B_dims[2]=31;

		//compute size of output
		int height = td[ii].A_dims[0] - td[ii].B_dims[0] + 1;
		int width = td[ii].A_dims[1] - td[ii].B_dims[1] + 1;

		if (height < 1 || width < 1)
		{
			printf("Invalid input: B should be smaller than A\n");
			exit(0);
		}

		td[ii].C_dims[0]=height;
		td[ii].C_dims[1]=width;
		td[ii].C=(FLOAT*)calloc(height*width,sizeof(FLOAT));

		int sym = sym_info[ii+start];

		//non_symmetric
		if (sym ==0)
		{
			if (pthread_create(&ts[ii], NULL, process, (void*)&td[ii]))
			{
				printf("Error creating thread\n");
				exit(0);
			}
		}
		//symmetric
		else
		{
			int T_dims[2];
			T_dims[0] = td[ii].A_dims[0];
			T_dims[1] = (int)(td[ii].B_dims[1]/2.0+0.99);
			td[ii].T=(FLOAT*)calloc(T_dims[0]*T_dims[1],sizeof(FLOAT));
			if (pthread_create(&ts[ii], NULL, processS, (void*)&td[ii]))
			{
				printf("Error creating thread\n");
				exit(0);
			}
		}

		M_size[ii*2]=height;
		M_size[ii*2+1]=width;

	}

	//close handle and get output
	for (int i = 0; i < len; i++)
	{
		pthread_join(ts[i], NULL);

		Output[i]=td[i].C;
		s_free(td[i].T);
	}
	s_free(td);
	s_free(ts);
	return(Output);
}
