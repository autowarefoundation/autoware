/***************************************************************************
*   Copyright (C) 2005 by TAKEUCHI Eijiro,,,   *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU General Public License     *
*   along with this program; if not, write to the                         *
*   Free Software Foundation, Inc.,                                       *
*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
***************************************************************************/
#ifndef __NDT_TKU__
#define __NDT_TKU__

#include <GL/glut.h>
/*root cell size*/
/*
  #define MAP_CELLSIZE 0.2 //m

  #define MAP_X 128
  #define MAP_Y 80
  #define MAP_Z 48
*/

//#define MAP_CELLSIZE 1.0 //m
#define MAX_ND_NUM (20000000)

// map origin//
// meidai_map
#if 0
#define MAP_CENTER_X (-18407.4 + 215)
#define MAP_CENTER_Y (-93444.9 - 400)
#define MAP_CENTER_Z 33
#define MAP_ROTATION 0.0
//map size
#define MAP_X 1600
#define MAP_Y 1600
#define MAP_Z 200
#endif

// meidai_map_all
#if 0
#define MAP_CENTER_X (-18500)
#define MAP_CENTER_Y (-93800)
#define MAP_CENTER_Z 33
#define MAP_ROTATION 0.0
//map size
#define MAP_X 2200
#define MAP_Y 2600
#define MAP_Z 200

#endif

// moriyama_map
#if 0
#define MAP_CENTER_X -15741
#define MAP_CENTER_Y -85697
#define MAP_CENTER_Z 70
#define MAP_ROTATION 0.7
//map size
#define MAP_X 1600
#define MAP_Y 1600
#define MAP_Z 200
#endif

// tctslc map
#if 0
#define MAP_CENTER_X 3702
#define MAP_CENTER_Y -99430
#define MAP_CENTER_Z 85
#define MAP_ROTATION 0
//map size
#define MAP_X 1600
#define MAP_Y 1600
#define MAP_Z 200
#endif

#if 0
//map offset
#define MAP_CENTER_X 0
#define MAP_CENTER_Y 0
#define MAP_CENTER_Z 0
#define MAP_ROTATION 0
//map size
#define MAP_X 1600
#define MAP_Y 1600
#define MAP_Z 200
#endif

// initial position
// meidai IB (141117_run01,run02)
#if 0
#define INITIAL_X -18354.1
#define INITIAL_Y -93693.4
#define INITIAL_Z 43
#define INITIAL_ROLL 0
#define INITIAL_PITCH 0
#define INITIAL_YAW -0.5
#endif
// meidai  (141205_outdoor1)
#if 0
#define INITIAL_X -18526.5
#define INITIAL_Y -93646.0
#define INITIAL_Z 39
#define INITIAL_ROLL 0
#define INITIAL_PITCH 0.03
#define INITIAL_YAW -0.45
#endif
// incubation(0826)
#if 0
#define INITIAL_X (-18407.4 + 215 - 215 + 0.6)
#define INITIAL_Y (-93444.9 - 400 + 400 + 0.65)
#define INITIAL_Z 33
#define INITIAL_ROLL 0
#define INITIAL_PITCH -0.063
#define INITIAL_YAW -1.976
#endif
// incubation parking(1224)
#if 0
#define INITIAL_X (-18389.8)
#define INITIAL_Y (-93499.1)
#define INITIAL_Z 33
#define INITIAL_ROLL 0
#define INITIAL_PITCH 0
#define INITIAL_YAW 1.107
#endif

// moriyama
#if 0
#define INITIAL_X -14771
#define INITIAL_Y -84757
#define INITIAL_Z (39.8)
#define INITIAL_ROLL 0
#define INITIAL_PITCH 0
//#define INITIAL_YAW 0
#define INITIAL_YAW 2.324
#endif

// tctslc
#if 0
#define INITIAL_X 3702
#define INITIAL_Y -99430  // matsu
//#define INITIAL_Y -99426   //zmp
#define INITIAL_Z 87
#define INITIAL_ROLL 0
#define INITIAL_PITCH 0
#define INITIAL_YAW 0  // matsu
//#define INITIAL_YAW 3.14   //zmp
#endif

// no map
#if 0
#define INITIAL_X 0
#define INITIAL_Y 0
#define INITIAL_Z 0
#define INITIAL_ROLL 0
#define INITIAL_PITCH 0
#define INITIAL_YAW 0
#endif

// x=-14763
// y=-84780
// q=-2.34
#define LAYER_NUM 2  // 0.2 0.4 0.8 1.6 3.2(128*80*48)

#define ND_MIN 40

//#define CELL_X (MAP_X*(1<<LAYER_NUM))
//#define CELL_Y (MAP_Y*(1<<LAYER_NUM))
//#define CELL_Z (MAP_Z*(1<<LAYER_NUM))

/*point*/
typedef struct point_type *PointPtr;

typedef struct point_type
{
  double x;
  double y;
  double z;
} Point;

/*point*/
typedef struct postuer_type *PosturePtr;

typedef struct postuer_type
{
  double x;
  double y;
  double z;
  double theta;
  double theta2;
  double theta3;
} Posture;

/*Normal Distribution  data type*/
typedef struct Normaldistribution *NDPtr;

typedef struct Normaldistribution
{
  /*covariances*/
  Point mean;
  double covariance[3][3];
  double inv_covariance[3][3];
  /*for calcurate covariances*/
  int flag; /*is updated*/
  int sign;

  int num; /*data num*/

  double m_x; /*mean*/
  double m_y;
  double m_z;

  double c_xx; /*covariance*/
  double c_yy;
  double c_zz;
  double c_xy;
  double c_yz;
  double c_zx;

  double x;
  double y;
  double z;

  double w;

  double l[3]; /*eigen value*/

  int is_source; /**/
  //  NDPtr child[8]; /*upper level layer*/
} NormalDistribution;

typedef struct nd_map *NDMapPtr;

typedef struct nd_map
{
  NDPtr *nd;
  int layer;
  int x;
  int y;
  int z;
  int to_x;
  int to_y;
  double size;
  char name[30];

  NDMapPtr next;
} NDMap;

typedef struct nd_data *NDDatPtr;

typedef struct nd_data
{
  NormalDistribution nd;
  int x;
  int y;
  int z;
  int layer;
} NDData;

int add_point_covariance(NDPtr nd, PointPtr p);
int update_covariance(NDPtr nd);
int add_point_map(NDMapPtr ndmap, PointPtr point);
int get_ND(NDMapPtr ndmap, PointPtr point, NDPtr *nd, int mode);

NDMapPtr initialize_NDmap(void);
NDMapPtr initialize_NDmap_layer(int layer, NDMapPtr parent);
int round_covariance(NDPtr nd);
int print_ellipse(FILE *output_file, double mat[3][3], double cx, double cy);
int print_ellipse_nd(FILE *output_file, NDPtr nd);
void glstart(int argc, char *argv[]);
void DrawString(GLfloat x, GLfloat y, double size_x, double size_y, void *font, char *string);
void DrawLine(GLfloat x1, GLfloat y1, GLfloat x2, GLfloat y2);
void display(void);
void reshape(int w, int h);
void draw_all(void);
void draw_map(void);
void keyfunc(unsigned char key, int x, int y);
void mousefunc(int button, int status, int x, int y);
void motionfunc(int x, int y);
void idle(void);
void draw_cell(NDPtr nd);
double probability_on_ND(NDPtr nd, double x, double y, double z);

// void add_ND(NDPtr);
NDPtr add_ND(void);
double calc_summand2d(PointPtr p, NDPtr nd, PosturePtr pose, double *g, double H[3][3]);
int adjust2d(PointPtr scan, int num, PosturePtr initial);

double calc_summand3d(PointPtr p, NDPtr nd, PosturePtr pose, double *g, double H[6][6], double qd3[6][3], double dist);
double adjust3d(PointPtr scan, int num, PosturePtr initial, int target);
void set_sincos2(double a, double b, double g, double sc[3][3]);
void scan_transrate(PointPtr src, PointPtr dst, PosturePtr pose, int num);

// values
extern int g_map_x, g_map_y, g_map_z;
extern double g_map_cellsize;
#endif
