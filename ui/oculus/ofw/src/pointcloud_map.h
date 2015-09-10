#ifndef POINTCLOUD_MAP_H_INCLUDED
#define POINTCLOUD_MAP_H_INCLUDED

#include "ofMain.h"
#include <stdio.h>
#include <fstream>
#include <iostream>

#define PCD_DIR "/data/data_moriyama_0.2/map/pointcloud_map"
#define STATIC_DIR "/data/data_moriyama_0.2/map/global_map"
//#define COLOR

class PCDPoint{
public:
    float x;
    float y;
    float z;
    float c;
    PCDPoint(float x,float y, float z, float c){
        this->x = x;
        this->y = y;
        this->z = z;
        this->c = c;
    }
};

class PCDfile{
public:
    int id;
    int start;
    int end;
    float max_x;
    float max_y;
    float max_z;
    float min_x;
    float min_y;
    float min_z;
    string name;

    PCDfile(int id, int start, int end,float max_x,float max_y,float max_z,float min_x,float min_y,float min_z, string name){
        this->id = id;
        this->start = start;
        this->end = end;
        this->name = name;
        this->max_x = max_x;
        this->min_x = min_x;
        this->max_y = max_y;
        this->min_y = min_y;
        this->max_z = max_z;
        this->min_z = min_z;
    }
};

class PointcloudMap{
public:
    vector<PCDPoint> points;
    vector<PCDfile> files;

    ifstream fin;
    string line;
    vector<string> lines, pieces;
    int x=0;
    int y=0;
    float max_x=0;
    float min_x=0;
    float max_y=0;
    float min_y=0;
    float max_z;
    float min_z;
    float ma_x,ma_y,mi_x,mi_y,ma_z,mi_z;
    int totalPoints = 0;
    int localPoints = 0;
    int readingFileNum = 0;
    bool f=true;

    void searcharea();
    void load(char* path, int num);
    void sload();
    int loading=-1;

    ofVboMesh mesh[9];

    ofVboMesh start;
};

#endif // POINTCLOUD_MAP_H_INCLUDED
