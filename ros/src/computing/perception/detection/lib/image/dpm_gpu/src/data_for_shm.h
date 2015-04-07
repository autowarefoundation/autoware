/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/****************/
/* include file */
/****************/
#ifndef DATA_FOR_SHM_H  // multiple include guard
#define DATA_FOR_SHM_H

#ifndef __SHM_HEADER__
#define __SHM_HEADER__
#include <sys/ipc.h>
#include <sys/shm.h>
#endif  // __SHM_HEADER__

#ifndef __SEM_HEADER__
#define __SEM_HEADER__
#include <sys/sem.h>
#endif  // __SEM_HEADER__

#ifndef __PTHREAD_HEADER__
#define __PTHREAD_HEADER__
#include <pthread.h>
#endif  // __PTHREAD_HEADER__

#ifndef __SWITCH_RELEASE_HEADER__
#define __SWITCH_RELEASE_HEADER__
#include "switch_release.h"
#endif  // __SWITCH_RELEASE_HEADER__


/************************/
/* information of image */
/************************/
#define IMAGE_SIZE 640*480*3+100 // the size of 640 x 480 bitmap imagep + alpha
#define HEIGHT_OF_IMAGE 480
//#define HEIGHT_OF_IMAGE 240
#define WIDTH_OF_IMAGE 640
//#define WIDTH_OF_IMAGE 320
#define HEADER_SIZE 54  // the size of header information of bitmap image


/************************************************/
/* information of the location of shared memory */
/************************************************/
/* #define INPUT_SHM_PATH "/home/hirabayashi/dev/autonomous_driving_system/input_file" */
/* #define OUTPUT_SHM_PATH "/home/hirabayashi/dev/autonomous_driving_system/output_file" */
/* #define HEIGHT_SHM_PATH "/home/hirabayashi/dev/autonomous_driving_system/height_file" */
/* #define WIDTH_SHM_PATH "/home/hirabayashi/dev/autonomous_driving_system/width_file" */
/* #define SIZE_SHM_PATH "/home/hirabayashi/dev/autonomous_driving_system/size_file" */
#ifdef RELEASE
#define INPUT_SHM_PATH "/usr/local/geye_with_cam/shared_memory/input_file"
#define OUTPUT_SHM_PATH "/usr/local/geye_with_cam/shared_memory/output_file"
#define HEIGHT_SHM_PATH "/usr/local/geye_with_cam/shared_memory/height_file"
#define WIDTH_SHM_PATH "/usr/local/geye_with_cam/shared_memory/width_file"
#define SIZE_SHM_PATH "/usr/local/geye_with_cam/shared_memory/size_file"
#define IMGUPD_SHM_PATH "/usr/local/geye_with_cam/shared_memory/imgupd_checker"
#define CAR_FPS_SHM_PATH "/usr/local/geye_with_cam/shared_memory/car_fps"
#define PDS_FPS_SHM_PATH "/usr/local/geye_with_cam/shared_memory/pds_fps"
#define SGN_FPS_SHM_PATH "/usr/local/geye_with_cam/shared_memory/sgn_fps"
#else
#define INPUT_SHM_PATH "/home/hirabayashi/demo/autonomous_driving_system/input_file"
#define OUTPUT_SHM_PATH "/home/hirabayashi/demo/autonomous_driving_system/output_file"
#define HEIGHT_SHM_PATH "/home/hirabayashi/demo/autonomous_driving_system/height_file"
#define WIDTH_SHM_PATH "/home/hirabayashi/demo/autonomous_driving_system/width_file"
#define SIZE_SHM_PATH "/home/hirabayashi/demo/autonomous_driving_system/size_file"
#define SIZE_SHM_PATH "/home/hirabayashi/demo/autonomous_driving_system/imgupd_checker"
#define CAR_FPS_SHM_PATH "/home/hirabayashi/demo/autonomous_driving_system/car_fps"
#define PDS_FPS_SHM_PATH "/home/hirabayashi/demo/autonomous_driving_system/pds_fps"
#define SGN_FPS_SHM_PATH "/home/hirabayashi/demo/autonomous_driving_system/sgn_fps"
#endif  /* ifdef RELEASE */

/*********************************************************/
/* definition of FOR_INPUT/OUTPUT used in sign_detection */
/*********************************************************/
#define FOR_INPUT 0
#define FOR_OUTPUT 1


/*************************************/
/* information for semaphore control */
/*************************************/
#define LOCK -1
#define UNLOCK 1
union semun {
    int val;                    /* the value of SETVAL */
    struct semid_ds *buf;       /* buffer for IPC_STAT, IPC_SET */
    unsigned short *array;      /* array for GETALL, SETALL */
};
//#define SEM_PATH "/home/hirabayashi/dev/autonomous_driving_system/semaphore"
#ifdef RELEASE
#define SEM_PATH "/usr/local/geye_with_cam/shared_memory/semaphore"
#else
#define SEM_PATH "/home/hirabayashi/demo/autonomous_driving_system/semaphore"
#endif  /* ifdef RELEASE */

/* the function for semaphore control */
void My_sem_operation(int p_semid, int p_op)
{
    struct sembuf sops[1];

    sops[0].sem_num = 0;        /* semaphore number */
    sops[0].sem_op = p_op;      /* semaphore control */
    sops[0].sem_flg = 0;        /* control flag */

    if(semop(p_semid, sops, 1) == -1) {
        printf("semaphore operation error\n");
        exit(1);
    }

    return;
}

/*************************************/
/* information of reader-writer lock */
/*************************************/
#ifdef RELEASE
#define RWLOCK_SHM_PATH "/usr/local/geye_with_cam/shared_memory/rwlock"
#else
#define RWLOCK_SHM_PATH "/home/hirabayashi/demo/autonomous_driving_system/rwlock"
#endif  /* ifdef RELEASE */


/************************************************************************************/
/* data and operation for ring buffer which contains coordinates of detected object */
/************************************************************************************/
/* #define RBUF_PATH "/home/hirabayashi/dev/autonomous_driving_system/rbuf" */
/* #define RBUF_HEAD_PATH "/home/hirabayashi/dev/autonomous_driving_system/rbuf_head" */
/* #define RBUF_TAIL_PATH "/home/hirabayashi/dev/autonomous_driving_system/rbuf_tail" */
#ifdef RELEASE
#define RBUF_PATH "/usr/local/geye_with_cam/shared_memory/rbuf"
#define RBUF_HEAD_PATH "/usr/local/geye_with_cam/shared_memory/rbuf_head"
#define RBUF_TAIL_PATH "/usr/local/geye_with_cam/shared_memory/rbuf_tail"
#else
#define RBUF_PATH "/home/hirabayashi/demo/autonomous_driving_system/rbuf"
#define RBUF_HEAD_PATH "/home/hirabayashi/demo/autonomous_driving_system/rbuf_head"
#define RBUF_TAIL_PATH "/home/hirabayashi/demo/autonomous_driving_system/rbuf_tail"
#endif


#define MAX_OBJECT_NUM 64 /* # of max ofjects detected at the same time */
//#define CO_NUM 4   /* # of coordinates needed to draw one rectangle */
//#define RBUF_ELEMENT_NUM (MAX_OBJECT_NUM)*(CO_NUM) /* # of ring buffer element */

#define LEFT 0                  /* left side of rectangle */
#define UPPER 1                 /* upper side of rectangle */
#define RIGHT 2                 /* right side of rectangle */
#define BOTTOM 3                /* bottom side of rectangle */

/* object type definition*/
#define CAR 1 
#define SIGN 2
#define PEDESTRIAN 3
#define UNKNOWN_OBJECT 4

/* resolution definition */
#define QVGA 1
#define VGA 2
#define SVGA 3
#define XGA 4
#define SXGA 5
#define UXGA 6

typedef struct {
    int left;
    int upper;
    int right;
    int bottom;
    int obj_type;
} obj_coordinate;


/* the function to enqueue(set) the data */
//void apSetCoordinate(int **queue, int data, int *head, int *tail, int location)
void apSetCoordinate(obj_coordinate *queue, 
                     int *head, 
                     int *tail, 
                     int left, 
                     int upper, 
                     int right, 
                     int bottom,
                     int type
    )
{
//    if(*head % RBUF_ELEMENT_NUM != ( *tail + 1) % RBUF_ELEMENT_NUM)
    if(*head % MAX_OBJECT_NUM != ( *tail + 1) % MAX_OBJECT_NUM)
    {
//        queue[(*tail)++ % RBUF_ELEMENT_NUM][location] = data;
//        queue[(*tail)++ % MAX_OBJECT_NUM][location] = data;
        /* if(location==BOTTOM) */
        /*     queue[(*tail)++ % MAX_OBJECT_NUM][location] = data; */
        /* else */
        /*     queue[(*tail) % MAX_OBJECT_NUM][location] = data; */
        queue[(*tail) % MAX_OBJECT_NUM].left = left;
        queue[(*tail) % MAX_OBJECT_NUM].upper = upper;
        queue[(*tail) % MAX_OBJECT_NUM].right = right;
        queue[(*tail) % MAX_OBJECT_NUM].bottom = bottom;
        queue[(*tail) % MAX_OBJECT_NUM].obj_type = type;

        (*tail)++;
        if(*tail == MAX_OBJECT_NUM) *tail = 0;  // overflow guard
        //      (*tail) = ((*tail)+1) % MAX_OBJECT_NUM;

    }

//    if(*tail == RBUF_ELEMENT_NUM) *tail = 0;  // overflow guard
//    if(*tail == MAX_OBJECT_NUM) *tail = 0;  // overflow guard
}

/* the function to dequeue(get) the data */
//int apGetCoordinate(int **queue, int *head, int *tail, int location)
void apGetCoordinate(obj_coordinate *queue, 
                     int *head, 
                     int *tail, 
                     int *left, 
                     int *upper, 
                     int *right, 
                     int *bottom,
                     int *type
    )
{
//    int ret_val;

    if(*head != *tail)
    {
//        ret_val = queue[(*head)++ % RBUF_ELEMENT_NUM][location];
//        ret_val = queue[(*head)++ % MAX_OBJECT_NUM][location];
        /* if(location==BOTTOM) */
        /*     ret_val = queue[(*head)++ % MAX_OBJECT_NUM][location]; */
        /* else */
        /*     ret_val = queue[(*head) % MAX_OBJECT_NUM][location]; */
        *left = queue[(*head) % MAX_OBJECT_NUM].left;
        *upper = queue[(*head) % MAX_OBJECT_NUM].upper;
        *right = queue[(*head) % MAX_OBJECT_NUM].right;
        *bottom = queue[(*head) % MAX_OBJECT_NUM].bottom;
        *type = queue[(*head) % MAX_OBJECT_NUM].obj_type;

        (*head)++;
        if(*head == MAX_OBJECT_NUM) *head = 0;  // overflow guard
//       (*head) = ((*head)+1) % MAX_OBJECT_NUM;

    }else{
        //      ret_val = -1;
        /* no rectangle will be drawn */
        *left = -1;
        *upper = -1;
        *right = -1;
        *bottom = -1;
        *type = UNKNOWN_OBJECT;
    }

//    if(*head == RBUF_ELEMENT_NUM) *head = 0;  // overflow guard
//    if(*head == MAX_OBJECT_NUM) *head = 0;  // overflow guard

//    return ret_val;

}


#endif  // DATA_FOR_SHM_H
