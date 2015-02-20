#ifndef _DATA_H
#define _DATA_H

#include <string>
#include "mainwindow.h"

using namespace zmp::hev;

#define DRVMODE_MANUAL 0
#define DRVMODE_PROGRAM 1

typedef struct _CMDDATA{
    double linear_x;
    double angular_z;
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

int CheckDrvMode();

#endif //_DATA_H
