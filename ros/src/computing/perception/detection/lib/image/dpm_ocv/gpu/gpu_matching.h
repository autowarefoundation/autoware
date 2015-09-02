#ifndef _GPU_MATCHING_H_
#define _GPU_MATCHING_H_

#define CONV_THREAD 128
#define TRANS_THREAD 512
#define DISTANCE_TRANSFORM_STREAMS 64
#define WARP_SIZE 32
#define NUM_WARP (CONV_THREAD / WARP_SIZE)
#define TRANS_RESULT_COUNT 64

typedef struct
{
    int numSectors;     //number of sectors
    int mapX;           //map sizeX * numSectors
    int mapY;           //map sizeY
    int filterX;        //filter sizeX * numSectors
    int filterY;        //filter sizeY
    int scoreX;         //mapX - filterX + 1
    int scoreY;         //mapY - filterY + 1
    int mapSize;        //mapX * mapY
    int filterSize;     //filterX * filterY
    int scoreSize;      //scoreX * scoreY
} ConvolutionParam;

typedef struct
{
    float internalDistTrans;
    int internalPoints;
    int v;
    float z;
} DistTransWork;

#endif
