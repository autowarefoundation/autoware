#ifndef _DATA_H
#define _DATA_H

#include <string>
#include "mainwindow.h"
#include "../hev_base/common/shm_common.h"

using namespace zmp::hev;

#define DRVMODE_MANUAL 0
#define DRVMODE_PROGRAM 1

//#define DEBUG 1

typedef struct _CMDDATA{
    vel_data_t vel;
    int mode;
    int gear;
    int accell;
    int steer;
    int brake;
} CMDDATA;

//Structure to store HEV State
typedef struct _DrvState {
  DrvInf drvInf;
  StrInf strInf;
  BrakeInf brkInf;
} DrvState;


extern DrvState _drv_state;

extern std::string rosServerIP;
extern std::string candata;
extern int drvmode;
extern CMDDATA cmddata;
extern double estimate_accel;
extern double cycle_time;
extern int ndrv;
extern int pdrv;
extern int nbrk;
extern int pbrk;
extern double nstr;
extern double pstr;

int CheckDrvMode();
double KmhToMs(double v);
bool Control(vel_data_t vel,vel_data_t &current,void* p);

#endif //_DATA_H
