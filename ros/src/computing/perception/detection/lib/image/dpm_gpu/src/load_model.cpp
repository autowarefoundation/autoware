/////////////////////////Car tracking project with laser_radar_data_fusion/////////////////////////////////////////
//////////////////////////////////////////////////////////////////////Copyright 2009-10 Akihiro Takeuchi///////////

/////load_model.cpp   load detection-model information ////////////////////////////////////////////////////////////

//C++ library
#include <cstdlib>
#include <cstdio>
#include <cmath>

//Header files
#include "MODEL_info.h"		//Model-structure definition
#include "Common.h"

#include "for_use_GPU.h"
#include "switch_float.h"
#include "switch_release.h"

#ifndef WIN32
typedef int errno_t;
#endif

#if defined(ROS) // AXE

#ifdef USE_FLOAT_AS_DECIMAL
#define FLOAT_SCAN_FMT	"%f,"
#else
#define FLOAT_SCAN_FMT	"%lf,"
#endif

#define FLOAT_SCAN_FMT2		FLOAT_SCAN_FMT FLOAT_SCAN_FMT
#define FLOAT_SCAN_FMT3		FLOAT_SCAN_FMT2 FLOAT_SCAN_FMT
#define FLOAT_SCAN_FMT4		FLOAT_SCAN_FMT3 FLOAT_SCAN_FMT

#endif

int sum_size_def_array;

//definiton of functions//

//subfunctions
#if defined(ROS) // AXE
Model_info * load_modelinfo(const char *filename);	//load model basic information
Rootfilters *load_rootfilter(const char *filename);	//load root filter information
Partfilters *load_partfilter(const char *filename);	//load part filter information
#else
Model_info * load_modelinfo(char *filename);	//load model basic information
Rootfilters *load_rootfilter(char *filename);	//load root filter information
Partfilters *load_partfilter(char *filename);	//load part filter information
#endif

//load model information
MODEL *load_model(FLOAT ratio);				//load MODEL(filter) (extended to main.cpp)

//release function
void free_model(MODEL *MO);						//release model-information (externed to main.cpp)


//subfunctions

//load model basic information
#if defined(ROS) // AXE
Model_info * load_modelinfo(const char *filename)
#else
Model_info * load_modelinfo(char *filename)
#endif
{
  CUresult res;

  FILE *file;		//File
  Model_info *MI=(Model_info*)malloc(sizeof(Model_info));		//Model information

  //fopen
  //if( (err = fopen_s( &file,filename, "r"))!=0 )
  if( (file=fopen(filename, "r"))==NULL )
    {
      printf("Model information file not found \n");
      exit(-1);
    }
  FLOAT t1,t2,t3,t4;

  //load basic information
#if defined(ROS) // AXE
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
#else
  if( sizeof(FLOAT) == sizeof(double) ){
    b =fscanf(file,"%lf,",&t1);
    MI->numcomponent=(int)t1;   //number of components
    b =fscanf(file,"%lf,",&t1);
    MI->sbin=(int)t1;           //sbin
    b =fscanf(file,"%lf,",&t1);
    MI->interval=(int)t1;       //interval
    b =fscanf(file,"%lf,",&t1);
    MI->max_Y=(int)t1;          //max_Y
    b =fscanf(file,"%lf,",&t1);
    MI->max_X=(int)t1;          //max_X
  }else{
    b =fscanf(file,"%f,",&t1);
    MI->numcomponent=(int)t1;   //number of components
    b =fscanf(file,"%f,",&t1);
    MI->sbin=(int)t1;           //sbin
    b =fscanf(file,"%f,",&t1);
    MI->interval=(int)t1;       //interval
    b =fscanf(file,"%f,",&t1);
    MI->max_Y=(int)t1;          //max_Y
    b =fscanf(file,"%f,",&t1);
    MI->max_X=(int)t1;          //max_X
  }
#endif
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
#if defined(ROS) // AXE
      fscanf(file,FLOAT_SCAN_FMT,&t1);
      MI->ridx[ii]=(int)t1-1; //root index
      fscanf(file,FLOAT_SCAN_FMT,&t1);
      MI->oidx[ii]=(int)t1-1; //offset index
      fscanf(file,FLOAT_SCAN_FMT,&t1);
      MI->offw[ii]=t1;        //offset weight (FLOAT)
      fscanf(file,FLOAT_SCAN_FMT2,&t1,&t2);
      MI->rsize[ii*2]=(int)t1;   //rsize (Y)
      MI->rsize[ii*2+1]=(int)t2; //rsize (X)
      fscanf(file,FLOAT_SCAN_FMT,&t1);
      MI->numpart[ii]=(int)t1; //number of part filter
#else
      if(sizeof(FLOAT)==sizeof(double)) {
        b =fscanf(file,"%lf,",&t1);
        MI->ridx[ii]=(int)t1-1; //root index
        b =fscanf(file,"%lf,",&t1);
        MI->oidx[ii]=(int)t1-1; //offset index
        b =fscanf(file,"%lf,",&t1);
        MI->offw[ii]=t1;        //offset weight (FLOAT)
        b =fscanf(file,"%lf,%lf,",&t1,&t2);
        MI->rsize[ii*2]=(int)t1;   //rsize (Y)
        MI->rsize[ii*2+1]=(int)t2; //rsize (X)
        b =fscanf(file,"%lf,",&t1);
        MI->numpart[ii]=(int)t1; //number of part filter
      }else{
        b =fscanf(file,"%f,",&t1);
        MI->ridx[ii]=(int)t1-1; //root index
        b =fscanf(file,"%f,",&t1);
        MI->oidx[ii]=(int)t1-1; //offset index
        b =fscanf(file,"%f,",&t1);
        MI->offw[ii]=t1;        //offset weight (FLOAT)
        b =fscanf(file,"%f,%f,",&t1,&t2);
        MI->rsize[ii*2]=(int)t1;   //rsize (Y)
        MI->rsize[ii*2+1]=(int)t2; //rsize (X)
        b =fscanf(file,"%f,",&t1);
        MI->numpart[ii]=(int)t1; //number of part filter
      }
#endif

      MI->pidx[ii]=(int*)malloc(sizeof(int)*MI->numpart[ii]);
      MI->didx[ii]=(int*)malloc(sizeof(int)*MI->numpart[ii]);
      MI->psize[ii]=(int*)malloc(sizeof(int)*MI->numpart[ii]*2);

      for(int jj=0;jj<MI->numpart[ii];jj++)	//LOOP (part-filter)
        {
#if defined(ROS) // AXE
	  fscanf(file,FLOAT_SCAN_FMT,&t1);
	  MI->pidx[ii][jj]=(int)t1-1; //part index
	  fscanf(file,FLOAT_SCAN_FMT,&t1);
	  MI->didx[ii][jj]=(int)t1-1; //define-index of part
	  fscanf(file,FLOAT_SCAN_FMT2,&t1,&t2);
	  MI->psize[ii][jj*2]=(int)t1;
	  MI->psize[ii][jj*2+1]=(int)t2;
#else
          if(sizeof(FLOAT)==sizeof(double)) {
            b =fscanf(file,"%lf,",&t1);
            MI->pidx[ii][jj]=(int)t1-1; //part index
            b =fscanf(file,"%lf,",&t1);
            MI->didx[ii][jj]=(int)t1-1; //define-index of part
            b =fscanf(file,"%lf,%lf,",&t1,&t2);
            MI->psize[ii][jj*2]=(int)t1;
            MI->psize[ii][jj*2+1]=(int)t2;
          }else{
            b =fscanf(file,"%f,",&t1);
            MI->pidx[ii][jj]=(int)t1-1; //part index
            b =fscanf(file,"%f,",&t1);
            MI->didx[ii][jj]=(int)t1-1; //define-index of part
            b =fscanf(file,"%f,%f,",&t1,&t2);
            MI->psize[ii][jj*2]=(int)t1;
            MI->psize[ii][jj*2+1]=(int)t2;
          }
#endif
        }
    }

  //get defs information
#if defined(ROS) // AXE
    fscanf(file,FLOAT_SCAN_FMT,&t1);
#else
  if(sizeof(FLOAT)==sizeof(double)) {
    b =fscanf(file,"%lf,",&t1);
  }else{
    b =fscanf(file,"%f,",&t1);
  }
#endif
  int DefL = int(t1);
  //MI->def = (FLOAT*)malloc(sizeof(FLOAT)*DefL*4);
  res = cuMemHostAlloc((void **)&(MI->def), sizeof(FLOAT)*DefL*4, CU_MEMHOSTALLOC_DEVICEMAP);
  if(res != CUDA_SUCCESS) {
    printf("cuMemHostAlloc(MI->def) failed: res = %s\n", conv(res));
    exit(1);
  }
  sum_size_def_array = sizeof(FLOAT)*DefL*4;


  MI->anchor = (int*)malloc(sizeof(int)*DefL*2);


  for (int kk=0;kk<DefL;kk++)
    {
#if defined(ROS) // AXE
      fscanf(file,FLOAT_SCAN_FMT4,&t1,&t2,&t3,&t4);
      MI->def[kk*4]=t1;
      MI->def[kk*4+1]=t2;
      MI->def[kk*4+2]=t3;
      MI->def[kk*4+3]=t4;
      fscanf(file,FLOAT_SCAN_FMT2,&t1,&t2);
      MI->anchor[kk*2]=(int)t1;
      MI->anchor[kk*2+1]=(int)t2;
#else
      if(sizeof(FLOAT)==sizeof(double)) {
        b =fscanf(file,"%lf,%lf,%lf,%lf,",&t1,&t2,&t3,&t4);
        MI->def[kk*4]=t1;
        MI->def[kk*4+1]=t2;
        MI->def[kk*4+2]=t3;
        MI->def[kk*4+3]=t4;
        b =fscanf(file,"%lf,%lf,",&t1,&t2);
        MI->anchor[kk*2]=(int)t1;
        MI->anchor[kk*2+1]=(int)t2;
      }else{
        b =fscanf(file,"%f,%f,%f,%f,",&t1,&t2,&t3,&t4);
        MI->def[kk*4]=t1;
        MI->def[kk*4+1]=t2;
        MI->def[kk*4+2]=t3;
        MI->def[kk*4+3]=t4;
        b =fscanf(file,"%f,%f,",&t1,&t2);
        MI->anchor[kk*2]=(int)t1;
        MI->anchor[kk*2+1]=(int)t2;
      }
#endif
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

#if defined(ROS) // AXE
      for (int jj=0;jj<GL;jj++){fscanf(file,FLOAT_SCAN_FMT,&t1);	MI->x1[ii][jj]=t1;}
      for (int jj=0;jj<GL;jj++){fscanf(file,FLOAT_SCAN_FMT,&t1);	MI->y1[ii][jj]=t1;}
      for (int jj=0;jj<GL;jj++){fscanf(file,FLOAT_SCAN_FMT,&t1);	MI->x2[ii][jj]=t1;}
      for (int jj=0;jj<GL;jj++){fscanf(file,FLOAT_SCAN_FMT,&t1);	MI->y2[ii][jj]=t1;}
#else
      if(sizeof(FLOAT)==sizeof(double)) {
        for (int jj=0;jj<GL;jj++){b =fscanf(file,"%lf,",&t1);	MI->x1[ii][jj]=t1;}
        for (int jj=0;jj<GL;jj++){b =fscanf(file,"%lf,",&t1);	MI->y1[ii][jj]=t1;}
        for (int jj=0;jj<GL;jj++){b =fscanf(file,"%lf,",&t1);	MI->x2[ii][jj]=t1;}
        for (int jj=0;jj<GL;jj++){b =fscanf(file,"%lf,",&t1);	MI->y2[ii][jj]=t1;}
      }else{
        for (int jj=0;jj<GL;jj++){b =fscanf(file,"%f,",&t1);	MI->x1[ii][jj]=t1;}
        for (int jj=0;jj<GL;jj++){b =fscanf(file,"%f,",&t1);	MI->y1[ii][jj]=t1;}
        for (int jj=0;jj<GL;jj++){b =fscanf(file,"%f,",&t1);	MI->x2[ii][jj]=t1;}
        for (int jj=0;jj<GL;jj++){b =fscanf(file,"%f,",&t1);	MI->y2[ii][jj]=t1;}
      }
#endif
    }

  MI->padx=(int)ceil((double)MI->max_X/2.0+1.0);	//padx
  MI->pady=(int)ceil((double)MI->max_Y/2.0+1.0);	//padY

  MI->ini=true;


  //fclose
  fclose(file);

  return(MI);
}

#if defined(ROS) // AXE
Rootfilters *load_rootfilter(const char *filename)
#else
Rootfilters *load_rootfilter(char *filename)
#endif
{
  FILE *file;		//File
  CUresult res;

  Rootfilters *RF=(Rootfilters*)malloc(sizeof(Rootfilters));		//Root filter

  //fopen
  //if( (err = fopen_s( &file,filename, "r"))!=0 )
  if( (file=fopen(filename, "r"))==NULL)
    {
      printf("Root-filter file not found \n");
      exit(-1);
    }

  FLOAT t1,t2,t3;
  FLOAT dummy_t1, dummy_t2, dummy_t3;  // variable for dummy scan in order to adjust the location of file-pointer

#if defined(ROS) // AXE
  fscanf(file,FLOAT_SCAN_FMT,&t1);
#else
  if(sizeof(FLOAT)==sizeof(double)) {
    b =fscanf(file,"%lf,",&t1);
  }else{
    b =fscanf(file,"%f,",&t1);
  }
#endif
  RF->NoR=(int)t1;												//number of root filter

  RF->root_size=(int**)malloc(sizeof(int*)*RF->NoR);				//size of root filter
  RF->rootfilter=(FLOAT**)malloc(sizeof(FLOAT*)*RF->NoR);		//weight of root filter
  RF->rootsym=(int*)malloc(sizeof(int)*RF->NoR);					//symmetric information of root


  /* keep file pointer location */
  long before_loop_location = ftell(file);

  size_t SUM_SIZE_ROOT=0;
  for (int ii=0;ii<RF->NoR;ii++)
    {
#if defined(ROS) // AXE
      fscanf(file,FLOAT_SCAN_FMT3,&t1,&t2,&t3);				//number of components
#else
      if(sizeof(FLOAT)==sizeof(double)) {
        b =fscanf(file,"%lf,%lf,%lf,",&t1,&t2,&t3);				//number of components
      }else{
        b =fscanf(file,"%f,%f,%f,",&t1,&t2,&t3);				//number of components
      }
#endif

      RF->root_size[ii]=(int*)malloc(sizeof(int)*3);
      RF->root_size[ii][0]=(int)t1;
      RF->root_size[ii][1]=(int)t2;
      RF->root_size[ii][2]=(int)t3;

      int NUMB=RF->root_size[ii][0]*RF->root_size[ii][1]*RF->root_size[ii][2];
#ifdef ORIGINAL
      RF->rootfilter[ii]=(FLOAT*)malloc(sizeof(FLOAT)*NUMB);	//weight of root filter
#else
#ifdef SEPARETE_MEM
      res = cuMemHostAlloc((void **)&(RF->rootfilter[ii]), sizeof(FLOAT)*NUMB, CU_MEMHOSTALLOC_DEVICEMAP);
      if(res != CUDA_SUCCESS){
        printf("cuMemHostAlloc(RF->rootfilter) failed: res = %s\n", conv(res));
        exit(1);
      }
#else
      SUM_SIZE_ROOT += NUMB*sizeof(FLOAT);
#endif
#endif

      /* adjust the location of file-pointer */
      for(int jj=0; jj<NUMB; jj++) {
#if defined(ROS) // AXE
	fscanf(file,FLOAT_SCAN_FMT,&dummy_t1);  // this is dummy scan
#else
        if(sizeof(FLOAT)==sizeof(double)) {
          fscanf(file,"%lf,",&dummy_t1);  // this is dummy scan
        }else{
          fscanf(file,"%f,",&dummy_t1);  // this is dummy scan
        }
#endif
      }
    }

#ifndef ORIGINAL
#ifndef SEPARETE_MEM
    /* allocate memory for root in a lump */
    FLOAT *dst_root;
    res = cuMemHostAlloc((void **)&dst_root, SUM_SIZE_ROOT, CU_MEMHOSTALLOC_DEVICEMAP);
    if(res != CUDA_SUCCESS){
      printf("cuMemHostAlloc(dst_root) failed: res = %s\n", conv(res));
      exit(1);
    }

    /* distribution */
    unsigned long long int pointer = (unsigned long long int)dst_root;
    for(int ii=0; ii<RF->NoR; ii++) {
      RF->rootfilter[ii] = (FLOAT *)pointer;
      int NUMB=RF->root_size[ii][0]*RF->root_size[ii][1]*RF->root_size[ii][2];
      pointer += NUMB*sizeof(FLOAT);
    }
#endif
#endif



    /* reset the location of file pointer */
    fseek(file, before_loop_location, SEEK_SET);


    for(int ii=0; ii<RF->NoR; ii++) {

      int NUMB=RF->root_size[ii][0]*RF->root_size[ii][1]*RF->root_size[ii][2];

      /* adjust the location of file-pointer */
#if defined(ROS) // AXE
      fscanf(file,FLOAT_SCAN_FMT3,&dummy_t1,&dummy_t2,&dummy_t3);  // this is dummy scan
#else
      if(sizeof(FLOAT)==sizeof(double)) {
        fscanf(file,"%lf,%lf,%lf,",&dummy_t1,&dummy_t2,&dummy_t3);  // this is dummy scan
      }else{
        fscanf(file,"%f,%f,%f,",&dummy_t1,&dummy_t2,&dummy_t3);  // this is dummy scan
      }
#endif

      for (int jj=0;jj<NUMB;jj++)
        {
#if defined(ROS) // AXE
	  fscanf(file,FLOAT_SCAN_FMT,&t1);
#else
          if(sizeof(FLOAT)==sizeof(double)) {
            b =fscanf(file,"%lf,",&t1);
          }else{
            b =fscanf(file,"%f,",&t1);
          }
#endif
          RF->rootfilter[ii][jj]=t1;
        }
      RF->rootsym[ii]=1;

#ifdef PRINT_INFO
      //test
      printf("root No.%d size %d %d \n",ii,RF->root_size[ii][0],RF->root_size[ii][1]);
#endif  // ifdef PRINT_INFO

    }

    //fclose
    fclose(file);

    return(RF);
}


#if defined(ROS) // AXE
Partfilters *load_partfilter(const char *filename)
#else
Partfilters *load_partfilter(char *filename)
#endif
{
  FILE *file;		//File

  CUresult res;

  Partfilters *PF=(Partfilters*)malloc(sizeof(Partfilters));		//Part filter

  //fopen
  //if( (err = fopen_s( &file,filename, "r"))!=0 )
  if( (file=fopen(filename, "r"))==NULL )
    {
      printf("Part-filter file not found \n");
      exit(-1);
    }

  FLOAT t1,t2,t3;
  FLOAT dummy_t1, dummy_t2, dummy_t3;  // variable for dummy scan in order to adjust the location of file-pointer

 #if defined(ROS) // AXE
  fscanf(file,FLOAT_SCAN_FMT,&t1);
#else
  if(sizeof(FLOAT)==sizeof(double)) {
    b =fscanf(file,"%lf,",&t1);
  }else{
    b =fscanf(file,"%f,",&t1);
  }
#endif
  PF->NoP=(int)t1;											//number of part filter

  PF->part_size=(int**)malloc(sizeof(int*)*PF->NoP);			//size of part filter
  PF->partfilter=(FLOAT**)malloc(sizeof(FLOAT*)*PF->NoP);	//weight of part filter
  PF->part_partner=(int*)malloc(sizeof(int)*PF->NoP);			//symmetric information of part
  PF->part_sym=(int*)malloc(sizeof(int)*PF->NoP);				//symmetric information of part


  /* keep file pointer location */
  long before_loop_location = ftell(file);


  int SUM_SIZE_PART = 0;
  for (int ii=0;ii<PF->NoP;ii++)
    {
#if defined(ROS) // AXE
      fscanf(file,FLOAT_SCAN_FMT3,&t1,&t2,&t3);			//number of components
#else
      if(sizeof(FLOAT)==sizeof(double)) {
        b =fscanf(file,"%lf,%lf,%lf,",&t1,&t2,&t3);			//number of components
      }else{
        b =fscanf(file,"%f,%f,%f,",&t1,&t2,&t3);			//number of components
      }
#endif
      PF->part_size[ii]=(int*)malloc(sizeof(int)*3);
      PF->part_size[ii][0]=(int)t1;
      PF->part_size[ii][1]=(int)t2;
      PF->part_size[ii][2]=(int)t3;
      //printf("***************%f  %f  %f\n",t1,t2,t3);
      int NUMB=PF->part_size[ii][0]*PF->part_size[ii][1]*PF->part_size[ii][2];
#ifdef ORIGINAL
      PF->partfilter[ii]=(FLOAT*)malloc(sizeof(FLOAT)*NUMB);				//weight of root filter
#else
#ifdef SEPARETE_MEM
      res = cuMemHostAlloc((void **)&(PF->partfilter[ii]), sizeof(FLOAT)*NUMB, CU_MEMHOSTALLOC_DEVICEMAP);
      if(res != CUDA_SUCCESS){
        printf("cuMemHostAlloc(PF->partfilter) failed: res = %s\n", conv(res));
        exit(1);
      }
#else
      SUM_SIZE_PART += NUMB*sizeof(FLOAT);
#endif
#endif

       /* adjust the location of file-pointer */
#if defined(ROS) // AXE
      for(int jj=0; jj<NUMB; jj++) {
	fscanf(file,FLOAT_SCAN_FMT,&dummy_t1);  // this is dummy scan
      }
      fscanf(file,FLOAT_SCAN_FMT,&dummy_t1); // this is dummy scan
#else
      for(int jj=0; jj<NUMB; jj++) {
        if(sizeof(FLOAT)==sizeof(double)) {
          fscanf(file,"%lf,",&dummy_t1);  // this is dummy scan
        }else{
          fscanf(file,"%f,",&dummy_t1);  // this is dummy scan
        }
      }
      if(sizeof(FLOAT)==sizeof(double)) {
        fscanf(file,"%lf,",&dummy_t1); // this is dummy scan
      }else{
        fscanf(file,"%f,",&dummy_t1); // this is dummy scan
      }
#endif

    }


#ifndef ORIGINAL
#ifndef SEPARETE_MEM
  /* allocate memory region for part in a lump */
  FLOAT *dst_part;
  res = cuMemHostAlloc((void **)&dst_part, SUM_SIZE_PART, CU_MEMHOSTALLOC_DEVICEMAP);
  if(res != CUDA_SUCCESS){
    printf("cuMemHostAlloc(dst_part) failed: res = %s\n", conv(res));
    exit(1);
  }

  /* distribution */
  unsigned long long int pointer = (unsigned long long int)dst_part;
  for(int ii=0; ii<PF->NoP; ii++) {
    PF->partfilter[ii] = (FLOAT *)pointer;
    int NUMB=PF->part_size[ii][0]*PF->part_size[ii][1]*PF->part_size[ii][2];
    pointer += NUMB*sizeof(FLOAT);
  }
#endif
#endif


  /* reset the location of file-pointer */
  fseek(file, before_loop_location, SEEK_SET);

  for(int ii=0; ii<PF->NoP; ii++) {

    int NUMB=PF->part_size[ii][0]*PF->part_size[ii][1]*PF->part_size[ii][2];

    /* adjust the location of file-pointer */
#if defined(ROS) // AXE
    fscanf(file,FLOAT_SCAN_FMT3,&dummy_t1,&dummy_t2,&dummy_t3);  // this is dummy scan

    for (int jj=0;jj<NUMB;jj++)
      {
	fscanf(file,FLOAT_SCAN_FMT,&t1);
        PF->partfilter[ii][jj]=t1;
      }
    fscanf(file,FLOAT_SCAN_FMT,&t1);
#else
    if(sizeof(FLOAT)==sizeof(double)) {
      fscanf(file,"%lf,%lf,%lf,",&dummy_t1,&dummy_t2,&dummy_t3);  // this is dummy scan
    }else{
      fscanf(file,"%f,%f,%f,",&dummy_t1,&dummy_t2,&dummy_t3);  // this is dummy scan
    }

    for (int jj=0;jj<NUMB;jj++)
      {
        if(sizeof(FLOAT)==sizeof(double)) {
          b =fscanf(file,"%lf,",&t1);
        }else{
          b =fscanf(file,"%f,",&t1);
        }
        PF->partfilter[ii][jj]=t1;
      }
    if(sizeof(FLOAT)==sizeof(double)) {
      b =fscanf(file,"%lf,",&t1);
    }else{
      b =fscanf(file,"%f,",&t1);
    }
#endif
    PF->part_partner[ii]=(int)t1; //symmetric information of part
    if(PF->part_partner[ii]==0) PF->part_sym[ii]=1;
    else PF->part_sym[ii]=0;
  }
  //fclose
  fclose(file);

  return(PF);
}

//load model infroamtion
#if defined(ROS)
extern std::string com_name;
extern std::string root_name;
extern std::string part_name;
#endif

MODEL *load_model(FLOAT ratio)
{
  MODEL *MO=(MODEL*)malloc(sizeof(MODEL));		// allocate model size
  // assign information into  MO.OO
#if defined(ROS)
  MO->MI=load_modelinfo(com_name.c_str());
  MO->RF=load_rootfilter(root_name.c_str());
  MO->PF=load_partfilter(part_name.c_str());
#else
  MO->MI=load_modelinfo(F_NAME_COM);
  MO->RF=load_rootfilter(F_NAME_ROOT);
  MO->PF=load_partfilter(F_NAME_PART);
#endif
  MO->MI->ratio = ratio;

  /* added to reuse resized feature */
  MO->MI->padx = 0;
  MO->MI->pady = 0;

  return(MO);
}

//release model
void free_model(MODEL *MO)
{

  CUresult res;

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

  //  s_free(MO->MI->def);
  res = cuMemFreeHost((void *)MO->MI->def);
  if(res != CUDA_SUCCESS) {
    printf("cuMemFreeHost(MO->MI->def) failed: res = %s\n", conv(res));
    exit(1);
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
  for(int ii=0;ii<MO->RF->NoR;ii++)
    {
      s_free(MO->RF->root_size[ii]);
#ifdef ORIGINAL
      s_free(MO->RF->rootfilter[ii]);
#else
#ifdef SEPARETE_MEM
      res = cuMemFreeHost((void *)MO->RF->rootfilter[ii]);
      if(res != CUDA_SUCCESS){
        printf("cuMemFreeHost(MO->RF->rootfilter) failed: res = %s\n", conv(res));
        exit(1);
      }
#endif
#endif
    }


#ifndef ORIGINAL
#ifndef SEPARETE_MEM
  /* free heap region in a lump */
  res = cuMemFreeHost((void *)MO->RF->rootfilter[0]);
  if(res != CUDA_SUCCESS){
    printf("cuMemFreeHost(MO->RF->rootfilter[0]) failed: res = %s\n", conv(res));
    exit(1);
  }
#endif
#endif

  s_free(MO->RF->rootsym);
  s_free(MO->RF);

  //free root-filter information
  for(int ii=0;ii<MO->PF->NoP;ii++)
    {
      s_free(MO->PF->part_size[ii]);
#ifdef ORIGINAL
      s_free(MO->PF->partfilter[ii]);
#else
#ifdef SEPARETE_MEM
      res = cuMemFreeHost((void *)MO->PF->partfilter[ii]);
      if(res != CUDA_SUCCESS){
        printf("cuMemFreeHost(MO->PF->partfilter) failed: res = %s\n", conv(res));
        exit(1);
      }
#endif
#endif
    }

#ifndef ORIGINAL
#ifndef SEPARETE_MEM
  /* free heap region in a lump */
  res = cuMemFreeHost((void *)MO->PF->partfilter[0]);
  if(res != CUDA_SUCCESS){
    printf("cuMemFreeHost(MO->PF->partfilter[0] failed: res = %s\n", conv(res));
    exit(1);
  }
#endif
#endif

  s_free(MO->PF->part_partner);
  s_free(MO->PF->part_sym);
  s_free(MO->PF);

  s_free(MO);
}
