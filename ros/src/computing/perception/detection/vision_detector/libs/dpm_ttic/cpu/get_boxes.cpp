// Car tracking project with laser_radar_data_fusion
// Copyright 2009-10 Akihiro Takeuchi

/////get_boxes.cpp  detect boundary-boxes-coordinate of oject

//C++ library
#include <cstdio>
#include <cstdlib>
#include <cstring>

//Header files
#include "MODEL_info.h"				//File information
#include "common.hpp"

#include "switch_float.h"
#include "get_boxes.hpp"
#include "dt.hpp"
#include "fconvsMT.hpp"

static void free_rootmatch(FLOAT **rootmatch, MODEL *MO)
{
	if (rootmatch == nullptr)
		return;

	for(int i = 0; i < MO->RF->NoR; i++) {
		free(rootmatch[i]);

	}
	s_free(rootmatch);
}

//free part-matching result
static void free_partmatch(FLOAT **partmatch, MODEL *MO)
{
	if (partmatch == nullptr)
		return;

	for(int i = 0; i < MO->PF->NoP; i++) {
		s_free(partmatch[i]);
	}
	s_free(partmatch);
}

//sub functions
//padd zeros to image
static FLOAT *padarray(FLOAT *feature,int *size,int padx,int pady)
{
	const int NEW_Y=size[0]+pady*2;
	const int NEW_X=size[1]+padx*2;
	const int L=NEW_Y*padx;
	const int SPL=size[0]+pady;
	const int M_S = sizeof(FLOAT)*size[0];
	FLOAT *new_feature = (FLOAT*)calloc(NEW_Y*NEW_X*size[2],sizeof(FLOAT)); // feature (pad-added)
	FLOAT *P=new_feature;
	FLOAT *S=feature;

	for(int ii=0;ii<size[2];ii++)	//31
	{
		P+=L; //row-all
		for(int jj=0;jj<size[1];jj++)
		{
			P+=pady;
			memcpy(P, S,M_S);
			S+=size[0];
			P+=SPL;
		}
		P+=L; //row-all
	}
	size[0]=NEW_Y;
	size[1]=NEW_X;
	return(new_feature);
}

static FLOAT *flip_feat(FLOAT *feat,int *size)
{
	const int ORD[31]={10,9,8,7,6,5,4,3,2,1,18,17,16,15,14,13,12,11,19,27,26,25,24,23,22,21,20,30,31,28,29};
	const int S_S =sizeof(FLOAT)*size[0];
	FLOAT *fliped = (FLOAT*)calloc(size[0]*size[1]*size[2],sizeof(FLOAT)); // feature (pad-added)
	FLOAT *D=fliped;
	int DIM = size[0]*size[1];
	for(int ii=0;ii<size[2];ii++)
	{
		FLOAT *S=feat+DIM*(*(ORD+ii));

		for(int jj=1;jj<=size[1];jj++)
		{
			FLOAT *P=S-*(size)*jj; //*(size)=size[0]

			memcpy(D, P,S_S);
			D+=*size;
			P+=*size;
		}
	}
	return(fliped);
}

//get good-matched pixel-coordinate
static int *get_gmpc(FLOAT *score,FLOAT thresh,int *ssize,int *GMN)
{
	const int L = ssize[0]*ssize[1];
	FLOAT *P=score;
	int NUM=0;
	FLOAT MAX_SCORE=thresh;

	for(int ii=0;ii<L;ii++)
	{
		if(*P>thresh)
		{
			NUM++;
			if(*P>MAX_SCORE) MAX_SCORE=*P;
		}
		P++;
	}

	int *Out =(int*)malloc(NUM*2*sizeof(int));
	P=score;

	int ii=0,qq=0;
	while(qq<NUM)
	{
		if(*P>thresh)
		{
			Out[2*qq]=ii%ssize[0];		//Y coordinate of match
			Out[2*qq+1]=ii/ssize[0];	//X coordinate of match
			qq++;
		}
		ii++;
		P++;
	}

	*GMN = NUM;
	return(Out);
}

//get root-box pixel coordinate
static FLOAT *rootbox(int x,int y,FLOAT scale,int padx,int pady,int *rsize)
{
	FLOAT *Out=(FLOAT*)malloc(sizeof(FLOAT)*4);
	Out[0]=((FLOAT)y-(FLOAT)pady+1)*scale;	//Y1
	Out[1]=((FLOAT)x-(FLOAT)padx+1)*scale;	//X1
	Out[2]=Out[0]+(FLOAT)rsize[0]*scale-1.0;	//Y2
	Out[3]=Out[1]+(FLOAT)rsize[1]*scale-1.0;	//X2
	return(Out);
}

//get part-box pixel coordinate
static FLOAT *partbox(int x,int y,int ax,int ay,FLOAT scale,int padx,int pady,int *psize,int *lx,int *ly,int *ssize)
{
	FLOAT *Out=(FLOAT*)malloc(sizeof(FLOAT)*4);
	int probex = (x-1)*2+ax;
	int probey = (y-1)*2+ay;
	int P = probey+probex*ssize[0];

	FLOAT px = (FLOAT)lx[P]+1.0;
	FLOAT py = (FLOAT)ly[P]+1.0;

	Out[0]=((py-2.0)/2.0+1.0-(FLOAT)pady)*scale;	//Y1
	Out[1]=((px-2.0)/2.0+1.0-(FLOAT)padx)*scale;	//X1
	Out[2]=Out[0]+(FLOAT)psize[0]*scale/2.0-1.0;	//Y2
	Out[3]=Out[1]+(FLOAT)psize[1]*scale/2.0-1.0;	//X2
	return(Out);
}

//calculate accumulated HOG detector score
static void calc_a_score(FLOAT *ac_score,FLOAT *score,int *ssize,int *rsize,Model_info *MI,FLOAT scale)
{
	const int IHEI = MI->IM_HEIGHT;
	const int IWID = MI->IM_WIDTH;
	int pady_n = MI->pady;
	int padx_n = MI->padx;
	int block_pad = (int)(scale/2.0);

	int RY = (int)((FLOAT)rsize[0]*scale/2.0-1.0+block_pad);
	int RX = (int)((FLOAT)rsize[1]*scale/2.0-1.0+block_pad);

	for(int ii=0;ii<IWID;ii++)
	{
		int Xn=(int)((FLOAT)ii/scale+padx_n);

		for(int jj=0;jj<IHEI;jj++)
		{
			int Yn =(int)((FLOAT)jj/scale+pady_n);
			if(Yn<ssize[0] && Xn<ssize[1])
			{
				FLOAT sc = score[Yn+Xn*ssize[0]];		//get score of pixel

				int Im_Y = jj+RY;
				int Im_X = ii+RX;
				if(Im_Y<IHEI && Im_X<IWID)
				{
					FLOAT *PP=ac_score+Im_Y+Im_X*IHEI;		//consider root rectangle size
					if(sc>*PP) *PP=sc;				//save max score
				}
			}
		}
	}
}

//free detected boxes result
static void free_boxes(FLOAT **boxes, int LofFeat)
{
	if (boxes == nullptr)
		return;

	for(int i = 0; i < LofFeat; i++)
	{
		s_free(boxes[i]);
	}
	s_free(boxes);
}

//detect boundary box
FLOAT *dpm_ttic_cpu_get_boxes(FLOAT **features,FLOAT *scales,int *FSIZE,MODEL *MO,
			      int *Dnum,FLOAT *A_SCORE,FLOAT thresh)
{
	//constant parameters
	const int max_scale = MO->MI->max_scale;
	const int interval = MO->MI->interval;
	const int sbin = MO->MI->sbin;
	const int padx = MO->MI->padx;
	const int pady = MO->MI->pady;
	const int NoR = MO->RF->NoR;
	const int NoP = MO->PF->NoP;
	const int NoC = MO->MI->numcomponent;
	const int *numpart = MO->MI->numpart;
	const int LofFeat=(max_scale+interval)*NoC;
	const int L_MAX = max_scale+interval;

	/* for measurement */
	struct timeval tv;
	struct timeval tv_root_score_start, tv_root_score_end;
	float time_root_score = 0;
	struct timeval tv_part_score_start, tv_part_score_end;
	float time_part_score = 0;
	struct timeval tv_dt_start, tv_dt_end;
	float time_dt = 0;
	struct timeval tv_calc_a_score_start, tv_calc_a_score_end;
	float time_calc_a_score = 0;

	int **RF_size = MO->RF->root_size;
	int *rootsym = MO->RF->rootsym;
	int *part_sym = MO->PF->part_sym;
	int **part_size = MO->PF->part_size;
	FLOAT **rootfilter = MO->RF->rootfilter;
	FLOAT **partfilter=MO->PF->partfilter;
	int **psize = MO->MI->psize;

	int *rm_size = (int*)malloc(sizeof(int)*NoC*2);			//size of root-matching-score-matrix
	int *pm_size = (int*)malloc(sizeof(int)*NoP*2);			//size of part-matching-score-matrix

	FLOAT **Tboxes=(FLOAT**)calloc(LofFeat,sizeof(FLOAT*));		//box coordinate information(Temp)
	int  *b_nums =(int*)calloc(LofFeat,sizeof(int));		//length of Tboxes
	int count = 0;
	int D_NUMS=0;							//number of detected boundary box

	///////level
	for (int level=interval;level<L_MAX;level++)
	{
		//parameters (related for level)
		int L=level-interval;

		//matched score (root and part)
		FLOAT **rootmatch = nullptr,**partmatch = nullptr;
		//matched score size matrix
		FLOAT scale=(FLOAT)sbin/scales[level];

		if(FSIZE[level*2]+2*pady<MO->MI->max_Y ||(FSIZE[level*2+1]+2*padx<MO->MI->max_X))
		{
			Tboxes[count]=nullptr;
			count++;
			continue;
		}

		///////root calculation/////////

		//convolve feature maps with filters
		int PADsize[3]={FSIZE[level*2],FSIZE[level*2+1],31};
		FLOAT *featp=padarray(features[level],PADsize,padx,pady);	//pad zero to matrix
		FLOAT *flipfeat =flip_feat(featp,PADsize);			//flip features (to reduce calculation time)

		//calculate model score (only root)
		gettimeofday(&tv_root_score_start, nullptr);
		rootmatch = dpm_ttic_cpu_fconvsMT(featp,flipfeat,rootfilter,rootsym,1,NoR,PADsize,RF_size,rm_size);
		gettimeofday(&tv_root_score_end, nullptr);
		tvsub(&tv_root_score_end, &tv_root_score_start, &tv);
		time_root_score += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

		//release feature
		s_free(featp);
		s_free(flipfeat);

		///////part calculation/////////
		if(NoP>0)
		{
			//convolve feature maps with filters
			int PADsize2[3]={FSIZE[L*2],FSIZE[L*2+1],31};
			featp=padarray(features[L],PADsize2,padx*2,pady*2);		//pad zero to matrix
			flipfeat=flip_feat(featp,PADsize2);				//flip features (to reduce calculation time)

			//calculate model score (only part)
			gettimeofday(&tv_part_score_start, nullptr);
			partmatch = dpm_ttic_cpu_fconvsMT(featp,flipfeat,partfilter,part_sym,1,
							  NoP,PADsize2,part_size,pm_size);
			gettimeofday(&tv_part_score_end, nullptr);
			tvsub(&tv_part_score_end, &tv_part_score_start, &tv);
			time_part_score += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

			//release feature
			s_free(featp);
			s_free(flipfeat);
		}

		////nucom
		//combine root and part score and detect boundary box for each-component
		for(int jj=0;jj<NoC;jj++)
		{
			//root score + offset
			int RL = rm_size[jj*2]*rm_size[jj*2+1];		//length of root-matching
			int RI = MO->MI->ridx[jj];			//root-index
			int RL_S =sizeof(FLOAT)*RL;

			FLOAT OFF = MO->MI->offw[RI];			//offset information
			FLOAT *SCORE = (FLOAT*)malloc(RL_S);		//Matching score matrix

			//add offset
			//memcpy_s(SCORE,RL_S,rootmatch[jj],RL_S);
			memcpy(SCORE, rootmatch[jj],RL_S);
			FLOAT *SC_S = SCORE;
			FLOAT *SC_E = SCORE+RL;
			while(SC_S<SC_E) *(SC_S++)+=OFF;

			//root matching size
			int R_S[2]={rm_size[jj*2],rm_size[jj*2+1]};
			int SNJ =sizeof(int*)*numpart[jj];

			//anchor matrix
			int *ax = (int*)malloc(SNJ);
			int *ay = (int*)malloc(SNJ);

			//boudary index
			int **Ix =(int**)malloc(SNJ);
			int **Iy =(int**)malloc(SNJ);

			//add parts
			if(NoP>0)
			{
				for (int kk=0;kk<numpart[jj];kk++)
				{
					int DIDX = MO->MI->didx[jj][kk];
					int DID_4 = DIDX*4;
					int PIDX = MO->MI->pidx[jj][kk];
					//anchor
					ax[kk] = MO->MI->anchor[DIDX*2]+1;
					ay[kk] = MO->MI->anchor[DIDX*2+1]+1;
					//set part-match
					FLOAT *match = partmatch[PIDX];
					//size of part-matching
					int PSSIZE[2] ={pm_size[PIDX*2],pm_size[PIDX*2+1]};

					FLOAT *Q = match;
					for(int ss=0;ss<PSSIZE[0]*PSSIZE[1];ss++) *(Q++)*=-1;

					//index matrix
					Ix[kk] =(int*)malloc(sizeof(int)*PSSIZE[0]*PSSIZE[1]);
					Iy[kk] =(int*)malloc(sizeof(int)*PSSIZE[0]*PSSIZE[1]);

					//decide position of part for all pixels
					gettimeofday(&tv_dt_start, nullptr);
					FLOAT *M = dpm_ttic_cpu_dt(match,MO->MI->def[DID_4],MO->MI->def[DID_4+1],
								   MO->MI->def[DID_4+2],MO->MI->def[DID_4+3],
								   PSSIZE,Ix[kk],Iy[kk]);
					gettimeofday(&tv_dt_end, nullptr);
					tvsub(&tv_dt_end, &tv_dt_start, &tv);
					time_dt += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;

					//add part score
					dpm_ttic_add_part_calculation(SCORE,M,R_S,PSSIZE,ax[kk],ay[kk]);
					s_free(M);
				}
			}

			//get all good matches
			int GMN;
			int *GMPC = get_gmpc(SCORE,thresh,R_S,&GMN);
			int RSIZE[2]={MO->MI->rsize[jj*2],MO->MI->rsize[jj*2+1]};

			int GL = (numpart[jj]+1)*4+3;  //31

			//calculate accumulated score
			gettimeofday(&tv_calc_a_score_start, nullptr);
			calc_a_score(A_SCORE,SCORE,R_S,RSIZE,MO->MI,scale);
			gettimeofday(&tv_calc_a_score_end, nullptr);
			tvsub(&tv_calc_a_score_end, &tv_calc_a_score_start, &tv);
			time_calc_a_score += tv.tv_sec * 1000.0 + (float)tv.tv_usec / 1000.0;


			//detected box coordinate(current level)
			FLOAT *t_boxes = (FLOAT*)calloc(GMN*GL,sizeof(FLOAT));

			for(int kk=0;kk<GMN;kk++)
			{
				FLOAT *P_temp = t_boxes+GL*kk;
				int y = GMPC[2*kk];
				int x = GMPC[2*kk+1];

				//calculate root box coordinate
				FLOAT *RB =rootbox(x,y,scale,padx,pady,RSIZE);
				//memcpy_s(P_temp,sizeof(FLOAT)*4,RB,sizeof(FLOAT)*4);
				memcpy(P_temp, RB,sizeof(FLOAT)*4);
				s_free(RB);
				P_temp+=4;

				for(int pp=0;pp<numpart[jj];pp++)
				{
					int PBSIZE[2]={psize[jj][pp*2],psize[jj][pp*2+1]};
					int Isize[2]={pm_size[MO->MI->pidx[jj][pp]*2],pm_size[MO->MI->pidx[jj][pp]*2+1]};

					//calculate part box coordinate
					FLOAT *PB = partbox(x,y,ax[pp],ay[pp],scale,padx,pady,PBSIZE,Ix[pp],Iy[pp],Isize);
					//memcpy_s(P_temp,sizeof(FLOAT)*4,PB,sizeof(FLOAT)*4);
					memcpy(P_temp, PB,sizeof(FLOAT)*4);
					P_temp+=4;
					s_free(PB);
				}
				//component number and score
				*(P_temp++)=(FLOAT)jj;		//component number
				*(P_temp++)=SCORE[x*R_S[0]+y];	//score of good match
				*P_temp = scale;
			}

			//save box information
			if(GMN>0) Tboxes[count]=t_boxes;
			else Tboxes[count]=nullptr;
			b_nums[count]=GMN;
			count++;
			D_NUMS+=GMN;	//number of detected box

			//release
			s_free(GMPC);
			s_free(SCORE);
			s_free(ax);
			s_free(ay);

			for(int ss=0;ss<numpart[jj];ss++)
			{
				s_free(Ix[ss]);
				s_free(Iy[ss]);
			}

			free(Ix);
		}
////numcom
		free_rootmatch(rootmatch,MO);
		free_partmatch(partmatch,MO);
	}
////level

	printf("root SCORE : %f\n", time_root_score);
	printf("part SCORE : %f\n", time_part_score);
	printf("dt  : %f\n", time_dt);
	printf("calc_a_score : %f\n", time_calc_a_score);

	//release
	s_free(rm_size);
	s_free(pm_size);

	//Output boundary-box coorinate information
	int GL=(numpart[0]+1)*4+3;
	FLOAT *boxes=(FLOAT*)calloc(D_NUMS*GL,sizeof(FLOAT));		//box coordinate information(Temp)
	FLOAT *T1=boxes;

	for(int ii=0;ii<LofFeat;ii++)
	{
		int num_t = b_nums[ii]*GL;
		FLOAT *T2 = Tboxes[ii];
		if(num_t>0)
		{
			memcpy(T1, T2,sizeof(FLOAT)*num_t);
			T1+=num_t;
		}
	}

	FLOAT AS_OFF = std::abs(thresh);

	//accumulated score calculation
	FLOAT max_ac = 0.0;

	//add offset to accumulated score
	for(int ii=0;ii<MO->MI->IM_HEIGHT*MO->MI->IM_WIDTH;ii++)
	{
		if(A_SCORE[ii]<thresh)
			A_SCORE[ii]=0.0;
		else
		{
			A_SCORE[ii]+=AS_OFF;
			if(A_SCORE[ii]>max_ac) max_ac=A_SCORE[ii];
		}
	}
	//normalization
	if(max_ac>0.0)
	{
		FLOAT ac_ratio = 1.0/max_ac;
		for(int ii=0;ii<MO->MI->IM_HEIGHT*MO->MI->IM_WIDTH;ii++){A_SCORE[ii]*=ac_ratio;}
	}

	//release
	free_boxes(Tboxes,LofFeat);
	s_free(b_nums);

	//output result
	*Dnum=D_NUMS;
	return(boxes);
}
