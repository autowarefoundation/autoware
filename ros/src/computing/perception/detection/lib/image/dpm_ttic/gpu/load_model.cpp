/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

/////load_model.cpp   load detection-model information ////////////////////////////////////////////////////////////

//C++ library
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

//Header files
#include "MODEL_info.h"		//Model-structure definition
#include "common.hpp"

#include "for_use_GPU.h"
#include "switch_float.h"
#include "switch_release.h"
#include "load_model.hpp"

#include <cuda_util.hpp>

#ifdef USE_FLOAT_AS_DECIMAL
#define FLOAT_SCAN_FMT	"%f,"
#else
#define FLOAT_SCAN_FMT	"%lf,"
#endif

#define FLOAT_SCAN_FMT2		FLOAT_SCAN_FMT FLOAT_SCAN_FMT
#define FLOAT_SCAN_FMT3		FLOAT_SCAN_FMT2 FLOAT_SCAN_FMT
#define FLOAT_SCAN_FMT4		FLOAT_SCAN_FMT3 FLOAT_SCAN_FMT

int sum_size_def_array;

//subfunctions

//load model basic information
static Model_info * load_modelinfo(const char *filename)
{
	Model_info *MI=(Model_info*)malloc(sizeof(Model_info));		//Model information

	FILE *file = fopen(filename, "r");
	if (file == nullptr) {
		std::cerr << std::strerror(errno) << std::endl;
		std::exit(-1);
	}

	FLOAT t1,t2,t3,t4;

	//load basic information
	fscanf(file,FLOAT_SCAN_FMT ",",&t1);
	MI->numcomponent=(int)t1;   //number of components
	fscanf(file,FLOAT_SCAN_FMT,&t1);
	MI->sbin=(int)t1;           //sbin
	fscanf(file,FLOAT_SCAN_FMT,&t1);
	MI->interval=(int)t1;       //interval
	fscanf(file,FLOAT_SCAN_FMT,&t1);
	MI->max_Y=(int)t1;          //max_Y
	fscanf(file,FLOAT_SCAN_FMT,&t1);
	MI->max_X=(int)t1;          //max_X

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

	for(int i = 0; i < MI->numcomponent; i++) {
		fscanf(file,FLOAT_SCAN_FMT,&t1);
		MI->ridx[i]=(int)t1-1; //root index
		fscanf(file,FLOAT_SCAN_FMT,&t1);
		MI->oidx[i]=(int)t1-1; //offset index
		fscanf(file,FLOAT_SCAN_FMT,&t1);
		MI->offw[i]=t1;        //offset weight (FLOAT)
		fscanf(file,FLOAT_SCAN_FMT2,&t1,&t2);
		MI->rsize[i*2]=(int)t1;   //rsize (Y)
		MI->rsize[i*2+1]=(int)t2; //rsize (X)
		fscanf(file,FLOAT_SCAN_FMT,&t1);
		MI->numpart[i]=(int)t1; //number of part filter

		MI->pidx[i]=(int*)malloc(sizeof(int)*MI->numpart[i]);
		MI->didx[i]=(int*)malloc(sizeof(int)*MI->numpart[i]);
		MI->psize[i]=(int*)malloc(sizeof(int)*MI->numpart[i]*2);

		//LOOP (part-filter)
		for(int j = 0;j < MI->numpart[i]; j++)
		{
			fscanf(file,FLOAT_SCAN_FMT,&t1);
			MI->pidx[i][j]=(int)t1-1; //part index
			fscanf(file,FLOAT_SCAN_FMT,&t1);
			MI->didx[i][j]=(int)t1-1; //define-index of part
			fscanf(file,FLOAT_SCAN_FMT2,&t1,&t2);
			MI->psize[i][j*2]=(int)t1;
			MI->psize[i][j*2+1]=(int)t2;
		}
	}

	//get defs information
	fscanf(file,FLOAT_SCAN_FMT,&t1);

	int DefL = int(t1);

	CUresult res = cuMemHostAlloc((void **)&(MI->def), sizeof(FLOAT)*DefL*4, CU_MEMHOSTALLOC_DEVICEMAP);
	if(res != CUDA_SUCCESS) {
		std::cerr << "Failed: cuMemHostAlloc(MI->def) : "
			  << cuda_response_to_string(res) << std::endl;
		std::exit(1);
	}
	sum_size_def_array = sizeof(FLOAT)*DefL*4;

	MI->anchor = (int*)malloc(sizeof(int)*DefL*2);

	for (int i = 0; i < DefL; i++) {
		fscanf(file,FLOAT_SCAN_FMT4,&t1,&t2,&t3,&t4);
		MI->def[i*4]=t1;
		MI->def[i*4+1]=t2;
		MI->def[i*4+2]=t3;
		MI->def[i*4+3]=t4;
		fscanf(file,FLOAT_SCAN_FMT2,&t1,&t2);
		MI->anchor[i*2]=(int)t1;
		MI->anchor[i*2+1]=(int)t2;
	}

	//get least_square information
	MI->x1 = (FLOAT **)malloc(sizeof(FLOAT*)*MI->numcomponent);
	MI->x2 = (FLOAT **)malloc(sizeof(FLOAT*)*MI->numcomponent);
	MI->y1 = (FLOAT **)malloc(sizeof(FLOAT*)*MI->numcomponent);
	MI->y2 = (FLOAT **)malloc(sizeof(FLOAT*)*MI->numcomponent);

	for(int i = 0; i < MI->numcomponent; i++) {
		int GL = 1+2*(1+MI->numpart[i]);
		MI->x1[i] =(FLOAT *)malloc(sizeof(FLOAT)*GL);
		MI->y1[i] =(FLOAT *)malloc(sizeof(FLOAT)*GL);
		MI->x2[i] =(FLOAT *)malloc(sizeof(FLOAT)*GL);
		MI->y2[i] =(FLOAT *)malloc(sizeof(FLOAT)*GL);

		for (int j = 0; j < GL; j++) {
			fscanf(file,FLOAT_SCAN_FMT,&t1);
			MI->x1[i][j] = t1;
		}
		for (int j = 0; j < GL; j++) {
			fscanf(file,FLOAT_SCAN_FMT,&t1);
			MI->y1[i][j] = t1;
		}

		for (int j = 0; j < GL; j++){
			fscanf(file,FLOAT_SCAN_FMT,&t1);
			MI->x2[i][j] = t1;
		}

		for (int j = 0; j < GL; j++){
			fscanf(file,FLOAT_SCAN_FMT,&t1);
			MI->y2[i][j] = t1;
		}
	}

	MI->padx=(int)ceil((double)MI->max_X/2.0+1.0);	//padx
	MI->pady=(int)ceil((double)MI->max_Y/2.0+1.0);	//padY

	MI->ini=true;

	fclose(file);
	return(MI);
}

static Rootfilters *load_rootfilter(const char *filename)
{
	FILE *file = fopen(filename, "r");
	if (file == nullptr) {
		std::cerr << std::strerror(errno) << std::endl;
		std::exit(-1);
	}

	FLOAT t1,t2,t3;
	FLOAT dummy_t1, dummy_t2, dummy_t3;  // variable for dummy scan in order to adjust the location of file-pointer

	fscanf(file,FLOAT_SCAN_FMT,&t1);

	Rootfilters *RF=(Rootfilters*)malloc(sizeof(Rootfilters));
	RF->NoR=(int)t1; //number of root filter

	RF->root_size=(int**)malloc(sizeof(int*)*RF->NoR);				//size of root filter
	RF->rootfilter=(FLOAT**)malloc(sizeof(FLOAT*)*RF->NoR);		//weight of root filter
	RF->rootsym=(int*)malloc(sizeof(int)*RF->NoR);					//symmetric information of root

	/* keep file pointer location */
	long before_loop_location = ftell(file);

	CUresult res;
	size_t sum_size_root = 0;
	for (int i = 0; i < RF-> NoR; i++) {
		fscanf(file,FLOAT_SCAN_FMT3,&t1,&t2,&t3);				//number of components

		RF->root_size[i]=(int*)malloc(sizeof(int)*3);
		RF->root_size[i][0]=(int)t1;
		RF->root_size[i][1]=(int)t2;
		RF->root_size[i][2]=(int)t3;

		int NUMB=RF->root_size[i][0]*RF->root_size[i][1]*RF->root_size[i][2];
#ifdef ORIGINAL
		RF->rootfilter[i]=(FLOAT*)malloc(sizeof(FLOAT)*NUMB);	//weight of root filter
#else
#ifdef SEPARETE_MEM
		res = cuMemHostAlloc((void **)&(RF->rootfilter[i]), sizeof(FLOAT)*NUMB, CU_MEMHOSTALLOC_DEVICEMAP);
		if(res != CUDA_SUCCESS){
			std::cerr << "Failed: cuMemHostAlloc(RF->rootfilter): "
				  << cuda_response_to_string(res) << std::endl;
			std::exit(1);
		}
#else
		sum_size_root += NUMB*sizeof(FLOAT);
#endif
#endif
		/* adjust the location of file-pointer */
		for(int j = 0; j < NUMB; j++) {
			fscanf(file,FLOAT_SCAN_FMT,&dummy_t1);  // this is dummy scan
		}
	}

#ifndef ORIGINAL
#ifndef SEPARETE_MEM
	/* allocate memory for root in a lump */
	FLOAT *dst_root;
	res = cuMemHostAlloc((void **)&dst_root, sum_size_root , CU_MEMHOSTALLOC_DEVICEMAP);
	if(res != CUDA_SUCCESS){
		std::cerr << "Failed: cuMemHostAlloc(dst_root): "
			  << cuda_response_to_string(res) << std::endl;
		std::exit(1);
	}

	/* distribution */
	unsigned long long int pointer = (unsigned long long int)dst_root;
	for(int i = 0; i < RF->NoR; i++) {
		RF->rootfilter[i] = (FLOAT *)pointer;
		int NUMB=RF->root_size[i][0]*RF->root_size[i][1]*RF->root_size[i][2];
		pointer += NUMB*sizeof(FLOAT);
	}
#endif
#endif

	/* reset the location of file pointer */
	fseek(file, before_loop_location, SEEK_SET);

	for(int i = 0; i < RF->NoR; i++) {
		int NUMB = RF->root_size[i][0]*RF->root_size[i][1]*RF->root_size[i][2];

		/* adjust the location of file-pointer */
		fscanf(file,FLOAT_SCAN_FMT3,&dummy_t1,&dummy_t2,&dummy_t3);  // this is dummy scan

		for (int j = 0; j < NUMB;j++) {
			fscanf(file,FLOAT_SCAN_FMT,&t1);
			RF->rootfilter[i][j]=t1;
		}
		RF->rootsym[i] = 1;

#ifdef PRINT_INFO
		printf("root No.%d size %d %d \n",i,RF->root_size[i][0],RF->root_size[i][1]);
#endif
	}

	fclose(file);
	return(RF);
}

static Partfilters *load_partfilter(const char *filename)
{
	CUresult res;

	Partfilters *PF=(Partfilters*)malloc(sizeof(Partfilters));		//Part filter

	FILE *file = fopen(filename, "r");
	if (file == nullptr) {
		std::cerr << std::strerror(errno) << std::endl;
		std::exit(-1);
	}

	FLOAT t1,t2,t3;
	FLOAT dummy_t1, dummy_t2, dummy_t3;  // variable for dummy scan in order to adjust the location of file-pointer

	fscanf(file,FLOAT_SCAN_FMT,&t1);

	PF->NoP=(int)t1;											//number of part filter

	PF->part_size=(int**)malloc(sizeof(int*)*PF->NoP);			//size of part filter
	PF->partfilter=(FLOAT**)malloc(sizeof(FLOAT*)*PF->NoP);	//weight of part filter
	PF->part_partner=(int*)malloc(sizeof(int)*PF->NoP);			//symmetric information of part
	PF->part_sym=(int*)malloc(sizeof(int)*PF->NoP);				//symmetric information of part

	/* keep file pointer location */
	long before_loop_location = ftell(file);

	int SUM_SIZE_PART = 0;
	for (int i = 0; i < PF->NoP; i++) {
		fscanf(file,FLOAT_SCAN_FMT3,&t1,&t2,&t3);			//number of components

		PF->part_size[i]=(int*)malloc(sizeof(int)*3);
		PF->part_size[i][0]=(int)t1;
		PF->part_size[i][1]=(int)t2;
		PF->part_size[i][2]=(int)t3;
		//printf("***************%f  %f  %f\n",t1,t2,t3);
		int NUMB=PF->part_size[i][0]*PF->part_size[i][1]*PF->part_size[i][2];
#ifdef ORIGINAL
		PF->partfilter[i]=(FLOAT*)malloc(sizeof(FLOAT)*NUMB);				//weight of root filter
#else
#ifdef SEPARETE_MEM
		res = cuMemHostAlloc((void **)&(PF->partfilter[i]), sizeof(FLOAT)*NUMB, CU_MEMHOSTALLOC_DEVICEMAP);
		if(res != CUDA_SUCCESS){
			std::cerr << "Failed: cuMemHostAlloc(PF->partfilter): "
				  << cuda_response_to_string(res) << std::endl;
			std::exit(1);
		}
#else
		SUM_SIZE_PART += NUMB*sizeof(FLOAT);
#endif
#endif
		/* adjust the location of file-pointer */
		for(int j = 0; j < NUMB; j++) {
			fscanf(file,FLOAT_SCAN_FMT,&dummy_t1);  // this is dummy scan
		}
		fscanf(file,FLOAT_SCAN_FMT,&dummy_t1); // this is dummy scan
	}

#ifndef ORIGINAL
#ifndef SEPARETE_MEM
	/* allocate memory region for part in a lump */
	FLOAT *dst_part;
	res = cuMemHostAlloc((void **)&dst_part, SUM_SIZE_PART, CU_MEMHOSTALLOC_DEVICEMAP);
	if(res != CUDA_SUCCESS){
		std::cerr << "Failed: cuMemHostAlloc(dst_part): "
			  << cuda_response_to_string(res) << std::endl;
		std::exit(1);
	}

	/* distribution */
	uintptr_t pointer = (uintptr_t)dst_part;
	for(int i = 0; i < PF->NoP; i++) {
		PF->partfilter[i] = (FLOAT *)pointer;
		int NUMB=PF->part_size[i][0]*PF->part_size[i][1]*PF->part_size[i][2];
		pointer += NUMB*sizeof(FLOAT);
	}
#endif
#endif
	/* reset the location of file-pointer */
	fseek(file, before_loop_location, SEEK_SET);

	for(int i = 0; i < PF->NoP; i++) {
		int NUMB = PF->part_size[i][0]*PF->part_size[i][1]*PF->part_size[i][2];

		/* adjust the location of file-pointer */
		fscanf(file,FLOAT_SCAN_FMT3,&dummy_t1,&dummy_t2,&dummy_t3);  // this is dummy scan

		for (int j = 0; j < NUMB; j++) {
			fscanf(file,FLOAT_SCAN_FMT,&t1);
			PF->partfilter[i][j]=t1;
		}
		fscanf(file,FLOAT_SCAN_FMT,&t1);

		PF->part_partner[i]=(int)t1; //symmetric information of part

		if(PF->part_partner[i]==0)
			PF->part_sym[i]=1;
		else
			PF->part_sym[i]=0;
	}

	fclose(file);
	return(PF);
}

GPUModel *dpm_ttic_gpu_load_model(FLOAT ratio, const char *com_csv, const char *root_csv, const char *part_csv)
{
	GPUModel *model = (GPUModel*)malloc(sizeof(GPUModel));

	// assign information into  model.OO
	model->MI = load_modelinfo(com_csv);
	model->RF = load_rootfilter(root_csv);
	model->PF = load_partfilter(part_csv);

	model->MI->ratio = ratio;

	/* added to reuse resized feature */
	model->MI->padx = 0;
	model->MI->pady = 0;

	return model;
}

//release model
void dpm_ttic_gpu_free_model(GPUModel *MO)
{
	//free model information
	for(int i=0; i < MO->MI->numcomponent; i++) {
		s_free(MO->MI->didx[i]);
		s_free(MO->MI->pidx[i]);
		s_free(MO->MI->psize[i]);
		s_free(MO->MI->x1[i]);
		s_free(MO->MI->x2[i]);
		s_free(MO->MI->y1[i]);
		s_free(MO->MI->y2[i]);
	}
	s_free(MO->MI->anchor);

	//  s_free(MO->MI->def);
	CUresult res = cuMemFreeHost((void *)MO->MI->def);
	if(res != CUDA_SUCCESS) {
		std::cerr << "Failed: cuMemFreeHost(MO->MI->def): "
			  << cuda_response_to_string(res) << std::endl;
		std::exit(1);
	}

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
	for(int i = 0; i < MO->RF->NoR; i++) {
		s_free(MO->RF->root_size[i]);
#ifdef ORIGINAL
		s_free(MO->RF->rootfilter[i]);
#else
#ifdef SEPARETE_MEM
		res = cuMemFreeHost((void *)MO->RF->rootfilter[i]);
		if(res != CUDA_SUCCESS){
			std::cerr << "Failed: cuMemFreeHost(MO->RF->rootfilter): "
				  << cuda_response_to_string(res) << std::endl;
			std::exit(1);
		}
#endif
#endif
	}

#ifndef ORIGINAL
#ifndef SEPARETE_MEM
	/* free heap region in a lump */
	res = cuMemFreeHost((void *)MO->RF->rootfilter[0]);
	if(res != CUDA_SUCCESS){
		std::cerr << "Failed: cuMemFreeHost(MO->RF->rootfilter[0]): "
			  << cuda_response_to_string(res) << std::endl;
		std::exit(1);
	}
#endif
#endif
	s_free(MO->RF->rootsym);
	s_free(MO->RF);

	//free root-filter information
	for(int i = 0; i < MO->PF->NoP; i++)
	{
		s_free(MO->PF->part_size[i]);
#ifdef ORIGINAL
		s_free(MO->PF->partfilter[i]);
#else
#ifdef SEPARETE_MEM
		res = cuMemFreeHost((void *)MO->PF->partfilter[i]);
		if(res != CUDA_SUCCESS){
			std::cerr << "Failed: cuMemFreeHost(MO->PF->partfilter): "
				  << cuda_response_to_string(res) << std::endl;
			std::exit(1);
		}
#endif
#endif
	}

#ifndef ORIGINAL
#ifndef SEPARETE_MEM
	/* free heap region in a lump */
	res = cuMemFreeHost((void *)MO->PF->partfilter[0]);
	if(res != CUDA_SUCCESS){
		std::cerr << "Failed: cuMemFreeHost(MO->PF->partfilter[0]): "
			  << cuda_response_to_string(res) << std::endl;
		std::exit(1);
	}
#endif
#endif
	s_free(MO->PF->part_partner);
	s_free(MO->PF->part_sym);
	s_free(MO->PF);

	s_free(MO);
}
