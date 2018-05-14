///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

/////load_model.cpp   load detection-model information ////////////////////////////////////////////////////////////

//C++ library
#include <cstdlib>
#include <cstdio>
#include <cmath>

//Header files
#include "MODEL_info.h"		//Model-structure definition
#include "common.hpp"

#include "switch_float.h"

#ifdef FLOAT_IS_float
#define FLOAT_SCAN_FMT	"%f,"
#else
#define FLOAT_SCAN_FMT	"%lf,"
#endif

#define FLOAT_SCAN_FMT2	FLOAT_SCAN_FMT FLOAT_SCAN_FMT
#define FLOAT_SCAN_FMT3	FLOAT_SCAN_FMT2 FLOAT_SCAN_FMT
#define FLOAT_SCAN_FMT4	FLOAT_SCAN_FMT3 FLOAT_SCAN_FMT

//load model basic information
static Model_info *load_modelinfo(const char *filename)
{
	FILE *file;		//File
	Model_info *MI=(Model_info*)malloc(sizeof(Model_info));		//Model information

	if( (file=fopen(filename, "r")) == NULL )
	{
		printf("Model information file not found \n");
		exit(-1);
	}
	FLOAT t1,t2,t3,t4;

	//load basic information
	fscanf(file,FLOAT_SCAN_FMT,&t1);
	MI->numcomponent=(int)t1;			//number of components
	fscanf(file,FLOAT_SCAN_FMT,&t1);
	MI->sbin=(int)t1;				//sbin
	fscanf(file,FLOAT_SCAN_FMT,&t1);
	MI->interval=(int)t1;				//interval
	fscanf(file,FLOAT_SCAN_FMT,&t1);
	MI->max_Y=(int)t1;				//max_Y
	fscanf(file,FLOAT_SCAN_FMT,&t1);
	MI->max_X=(int)t1;				//max_X

	//root filter information
	MI->ridx = (int*)malloc(sizeof(int)*MI->numcomponent);
	MI->oidx = (int*)malloc(sizeof(int)*MI->numcomponent);
	MI->offw = (FLOAT*)malloc(sizeof(FLOAT)*MI->numcomponent);
	MI->rsize = (int*)malloc(sizeof(int)*MI->numcomponent*2);
	MI->numpart = (int*)malloc(sizeof(int)*MI->numcomponent);

	//part filter information
	MI->pidx = (int**)malloc(sizeof(int*)*MI->numcomponent);
	MI->didx = (int**)malloc(sizeof(int*)*MI->numcomponent);
	MI->psize = (int**)malloc(sizeof(int*)*MI->numcomponent);

	for(int ii=0;ii<MI->numcomponent;ii++)	//LOOP (component)
	{
		fscanf(file,FLOAT_SCAN_FMT,&t1);
		MI->ridx[ii]=(int)t1-1;			//root index
		fscanf(file,FLOAT_SCAN_FMT,&t1);
		MI->oidx[ii]=(int)t1-1;			//offset index
		fscanf(file,FLOAT_SCAN_FMT,&t1);
		MI->offw[ii]=t1;				//offset weight (FLOAT)
		fscanf(file,FLOAT_SCAN_FMT2,&t1,&t2);
		MI->rsize[ii*2]=(int)t1;			//rsize (Y)
		MI->rsize[ii*2+1]=(int)t2;		//rsize (X)
		fscanf(file,FLOAT_SCAN_FMT,&t1);
		MI->numpart[ii]=(int)t1;			//number of part filter


		MI->pidx[ii]=(int*)malloc(sizeof(int)*MI->numpart[ii]);
		MI->didx[ii]=(int*)malloc(sizeof(int)*MI->numpart[ii]);
		MI->psize[ii]=(int*)malloc(sizeof(int)*MI->numpart[ii]*2);

		for(int jj=0;jj<MI->numpart[ii];jj++)	//LOOP (part-filter)
		{
			fscanf(file,FLOAT_SCAN_FMT,&t1);
			MI->pidx[ii][jj]=(int)t1-1;		//part index
			fscanf(file,FLOAT_SCAN_FMT,&t1);
			MI->didx[ii][jj]=(int)t1-1;		//define-index of part
			fscanf(file,FLOAT_SCAN_FMT2,&t1,&t2);
			MI->psize[ii][jj*2]=(int)t1;
			MI->psize[ii][jj*2+1]=(int)t2;
		}
	}

	//get defs information
	fscanf(file,FLOAT_SCAN_FMT,&t1);

	int DefL = int(t1);
	MI->def = (FLOAT*)malloc(sizeof(FLOAT)*DefL*4);
	MI->anchor = (int*)malloc(sizeof(int)*DefL*2);

	for (int kk=0;kk<DefL;kk++)
	{
		fscanf(file,FLOAT_SCAN_FMT4,&t1,&t2,&t3,&t4);
		MI->def[kk*4]=t1;
		MI->def[kk*4+1]=t2;
		MI->def[kk*4+2]=t3;
		MI->def[kk*4+3]=t4;
		fscanf(file,FLOAT_SCAN_FMT2,&t1,&t2);
		MI->anchor[kk*2]=(int)t1;
		MI->anchor[kk*2+1]=(int)t2;
	}

	//get least_square information
	MI->x1 = (FLOAT **)malloc(sizeof(FLOAT*)*MI->numcomponent);
	MI->x2 = (FLOAT **)malloc(sizeof(FLOAT*)*MI->numcomponent);
	MI->y1 = (FLOAT **)malloc(sizeof(FLOAT*)*MI->numcomponent);
	MI->y2 = (FLOAT **)malloc(sizeof(FLOAT*)*MI->numcomponent);

	for(int ii=0;ii<MI->numcomponent;ii++)
	{
		int GL = 1+2*(1+MI->numpart[ii]);
		MI->x1[ii] =(FLOAT *)malloc(sizeof(FLOAT)*GL);
		MI->y1[ii] =(FLOAT *)malloc(sizeof(FLOAT)*GL);
		MI->x2[ii] =(FLOAT *)malloc(sizeof(FLOAT)*GL);
		MI->y2[ii] =(FLOAT *)malloc(sizeof(FLOAT)*GL);

		for (int jj=0;jj<GL;jj++){fscanf(file,FLOAT_SCAN_FMT,&t1);	MI->x1[ii][jj]=t1;}
		for (int jj=0;jj<GL;jj++){fscanf(file,FLOAT_SCAN_FMT,&t1);	MI->y1[ii][jj]=t1;}
		for (int jj=0;jj<GL;jj++){fscanf(file,FLOAT_SCAN_FMT,&t1);	MI->x2[ii][jj]=t1;}
		for (int jj=0;jj<GL;jj++){fscanf(file,FLOAT_SCAN_FMT,&t1);	MI->y2[ii][jj]=t1;}
	}

	MI->padx=(int)ceil((FLOAT)MI->max_X/2.0+1.0);	//padx
	MI->pady=(int)ceil((FLOAT)MI->max_Y/2.0+1.0);	//padY

	MI->ini=true;


	//fclose
	fclose(file);

	return(MI);
}

static Rootfilters *load_rootfilter(const char *filename)
{
	FILE *file;
	Rootfilters *RF=(Rootfilters*)malloc(sizeof(Rootfilters));	//Root filter

	if( (file=fopen(filename, "r"))==NULL )
	{
		printf("Root-filter file not found \n");
		exit(-1);
	}

	FLOAT t1,t2,t3;
	fscanf(file,FLOAT_SCAN_FMT,&t1);
	RF->NoR=(int)t1;						//number of root filter

	RF->root_size=(int**)malloc(sizeof(int*)*RF->NoR);		//size of root filter
	RF->rootfilter=(FLOAT**)malloc(sizeof(FLOAT*)*RF->NoR);	//weight of root filter
	RF->rootsym=(int*)malloc(sizeof(int)*RF->NoR);		//symmetric information of root

	for (int ii=0;ii<RF->NoR;ii++)
	{
		fscanf(file,FLOAT_SCAN_FMT3,&t1,&t2,&t3);			//number of components
		RF->root_size[ii]=(int*)malloc(sizeof(int)*3);
		RF->root_size[ii][0]=(int)t1;
		RF->root_size[ii][1]=(int)t2;
		RF->root_size[ii][2]=(int)t3;
		int NUMB=RF->root_size[ii][0]*RF->root_size[ii][1]*RF->root_size[ii][2];
		RF->rootfilter[ii]=(FLOAT*)malloc(sizeof(FLOAT)*NUMB);	//weight of root filter
		for (int jj=0;jj<NUMB;jj++)
		{
			fscanf(file,FLOAT_SCAN_FMT,&t1);
			RF->rootfilter[ii][jj]=t1;
		}
		RF->rootsym[ii]=1;

		//test
		printf("root No.%d size %d %d \n",ii,RF->root_size[ii][0],RF->root_size[ii][1]);

	}

	//fclose
	fclose(file);
	return(RF);
}

static Partfilters *load_partfilter(const char *filename)
{
	FILE *file;
	Partfilters *PF=(Partfilters*)malloc(sizeof(Partfilters));		//Part filter

	//fopen
	if( (file=fopen(filename, "r"))==NULL )
	{
		printf("Part-filter file not found \n");
		exit(-1);
	}

	FLOAT t1,t2,t3;
	fscanf(file,FLOAT_SCAN_FMT,&t1);
	PF->NoP=(int)t1;						//number of part filter

	PF->part_size=(int**)malloc(sizeof(int*)*PF->NoP);		//size of part filter
	PF->partfilter=(FLOAT**)malloc(sizeof(FLOAT*)*PF->NoP);	//weight of part filter
	PF->part_partner=(int*)malloc(sizeof(int)*PF->NoP);		//symmetric information of part
	PF->part_sym=(int*)malloc(sizeof(int)*PF->NoP);		//symmetric information of part


	for (int ii=0;ii<PF->NoP;ii++)
	{
		fscanf(file,FLOAT_SCAN_FMT3,&t1,&t2,&t3);			//number of components
		PF->part_size[ii]=(int*)malloc(sizeof(int)*3);
		PF->part_size[ii][0]=(int)t1;
		PF->part_size[ii][1]=(int)t2;
		PF->part_size[ii][2]=(int)t3;
		//printf("%f  %f  %f\n",t1,t2,t3);
		int NUMB=PF->part_size[ii][0]*PF->part_size[ii][1]*PF->part_size[ii][2];
		PF->partfilter[ii]=(FLOAT*)malloc(sizeof(FLOAT)*NUMB);	//weight of root filter
		for (int jj=0;jj<NUMB;jj++)
		{
			fscanf(file,FLOAT_SCAN_FMT,&t1);
			PF->partfilter[ii][jj]=t1;
		}

		fscanf(file,FLOAT_SCAN_FMT,&t1);
		PF->part_partner[ii]=(int)t1;				//symmetric information of part
		if(PF->part_partner[ii]==0) PF->part_sym[ii]=1;
		else PF->part_sym[ii]=0;
	}
	//fclose
	fclose(file);
	return(PF);
}

//load model infroamtion
MODEL *dpm_ttic_cpu_load_model(FLOAT ratio, const char *com_csv, const char *root_csv, const char *part_csv)
{
	MODEL *model = (MODEL*)malloc(sizeof(MODEL));

	model->MI = load_modelinfo(com_csv);
	model->RF = load_rootfilter(root_csv);
	model->PF = load_partfilter(part_csv);
	model->MI->ratio = ratio;

	model->MI->padx = 0;
	model->MI->pady = 0;

	return model;
}

//release model
void dpm_ttic_cpu_free_model(MODEL *MO)
{
	//free model information
	for(int ii=0;ii<MO->MI->numcomponent;ii++)
	{
		s_free(MO->MI->didx[ii]);
		s_free(MO->MI->pidx[ii]);
		s_free(MO->MI->psize[ii]);
		s_free(MO->MI->x1[ii]);
		s_free(MO->MI->x2[ii]);
		s_free(MO->MI->y1[ii]);
		s_free(MO->MI->y2[ii]);
	}
	s_free(MO->MI->anchor);
	s_free(MO->MI->def);
	s_free(MO->MI->numpart);
	s_free(MO->MI->offw);
	s_free(MO->MI->oidx);
	s_free(MO->MI->ridx);
	s_free(MO->MI->rsize);
	s_free(MO->MI->x1);
	s_free(MO->MI->x2);
	s_free(MO->MI->y1);
	s_free(MO->MI->y2);
	s_free(MO->MI);

	//free root-filter information
	for(int ii=0;ii<MO->RF->NoR;ii++)
	{
		s_free(MO->RF->root_size[ii]);
		s_free(MO->RF->rootfilter[ii]);
	}
	s_free(MO->RF->rootsym);
	s_free(MO->RF);

	//free root-filter information
	for(int ii=0;ii<MO->PF->NoP;ii++)
	{
		s_free(MO->PF->part_size[ii]);
		s_free(MO->PF->partfilter[ii]);
	}
	s_free(MO->PF->part_partner);
	s_free(MO->PF->part_sym);
	s_free(MO->PF);

	s_free(MO);
}
