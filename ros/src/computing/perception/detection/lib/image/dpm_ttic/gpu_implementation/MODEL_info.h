#ifndef _MODEL_INFO_H_
#define _MODEL_INFO_H_

#include "switch_float.h"

//struct for model component information
struct Model_info {
	//basic information
	//from xxxcomp.csv
	int numcomponent;           //number of component
	int sbin;                   //cell size
	int interval;               //interval (for hierachical detection)
	int max_X;
	int max_Y;
	//from calculation
	int padx;                   //pad information
	int pady;
	int max_scale;
	//image size information
	int IM_WIDTH;
	int IM_HEIGHT;

//per root
	int *ridx;                  //root index information
	int *oidx;                  //offsetindex information
	FLOAT *offw;                //offset weight
	int *rsize;                 //root size
	int *numpart;               //number of part filter per component

//per part
	int **pidx;                 //part index information
	int **didx;                 //define index of part
	int **psize;

//defs
	FLOAT *def;                 //defs
	int *anchor;                //anchor

//least_square info
	FLOAT **x1;
	FLOAT **y1;
	FLOAT **x2;
	FLOAT **y2;

	bool ini;                   //flag for initialization
	FLOAT ratio;                //ratio of zooming image
};

//struct for root_filter_information
struct Rootfilters {
	int NoR;                    //number of root filter
	int **root_size;            //size of root filter
	FLOAT **rootfilter;         //weight of root filter
	int *rootsym;               //symmetric information
};

//struct for part_filter_information
struct Partfilters {
	int NoP;                    //number of part filter
	int **part_size;            //size of part filter
	FLOAT **partfilter;         //weight of root filter
	int *part_partner;          //symmetric-partner information
	int *part_sym;              //symmetric information of part filter
};

//model information
struct GPUModel {
	Model_info *MI;
	Rootfilters *RF;
	Partfilters *PF;
};

#endif /* _MODEL_INFO_H_ */
