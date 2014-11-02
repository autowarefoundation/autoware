/****************************************/
/* library source of dlibImage function */
/****************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <highgui.h>
#include <cv.h>
#include "deal_image.h"

/**************************************************************/
/*------------------------------------------------------------*/
/*
   [function name] : dlibImageInit
   [argument]      : pointer of image descriptor
                     path to image file or shared memory
                     size of shared memory(needed if "FROM_SHM" is specified to image source discriminater)
                     image source discriminater
   [return value]  :
   [description]   : comfirming connection to USB camera
 */
/*------------------------------------------------------------*/
/**************************************************************/
void dlibImageInit(
    DLimgdescriptor *src,
    char *pathname,
    int shm_size,
    DLimgsource src_identifier)
{
    /* to use camera */
    CvCapture *videoCapture=NULL;

    /* to use shared memory */
    key_t shm_key;
    int shrd_id;

    /* to use file */
    FILE *fp;

    switch(src_identifier){
    case FROM_CAM:
        /* get connection to camera */
        videoCapture = cvCreateCameraCapture(0);
        if(videoCapture == NULL) // error semantics
        {
            fprintf(stderr, "no camera detected...\n");
            return;
        }

        src->pointer = videoCapture;
        src->src_identifier = FROM_CAM;
        src->shm_size = 0;

        break;

    case FROM_SHM:
        /* access to the shared memory */
        shm_key = ftok(pathname, 1);
        shrd_id = shmget(shm_key, shm_size, 0666);
        if(shrd_id < 0) // error semantics
        {
            fprintf(stderr, "Can't access to the shared memory...\n");
            return ;
        }

        /* attach the shared memory */
        src->pointer =  (unsigned char*)shmat(shrd_id, NULL, 0);
        src->src_identifier = FROM_SHM;
        src->shm_size = shm_size;

        break;

    case FROM_FILE:
        /* file open */
        fp = fopen(pathname, "rb");
        if(fp == NULL)
        {
            fprintf(stderr, "file open error...\n");
            return ;
        }
        src->pointer = fp;
        src->src_identifier = FROM_FILE;
        src->shm_size = 0;

        break;

    default:
        fprintf(stderr, "no defined image source.");
        break;
    }

}

/**************************************************************/
/*------------------------------------------------------------*/
/*
   [function name] : dlibImageExit
   [argument]      : pointer of image descriptor
   [return value]  :
   [description]   : break connection to USB camera
 */
/*------------------------------------------------------------*/
/**************************************************************/
void dlibImageExit(DLimgdescriptor *src)
{
    switch(src->src_identifier){
    case FROM_CAM:
        cvReleaseCapture((CvCapture **)(&(src->pointer)));
        break;
    case FROM_SHM:
        shmdt((unsigned char*)src->pointer);
        break;
    case FROM_FILE:
        fclose((FILE*)(src->pointer));
        break;
    }

} /* dlibImageExit */



/**************************************************************/
/*------------------------------------------------------------*/
/*
   [function name] : dlibImageGet
   [argument]      : pointer to destination buffer of image
                     pointer of image descriptor
   [return value]  : allocated size of destinaton buffer
   [description]   : return char pointer of image data from various source(camera, shared memory and file).
 */
/*------------------------------------------------------------*/
/**************************************************************/
int dlibImageGet(
    unsigned char** dst_buf,
    DLimgdescriptor *src
    )
{
    /* for image from camera */
    CvCapture *videoCapture = (CvCapture*)(src->pointer);
    IplImage* img_from_cam = NULL;
    IplImage* resized_img = NULL;
    CvMat *encoded_img = NULL;

    /* for iamge from file */
    FILE* fp = (FILE*)(src->pointer);
    unsigned char* tmp_pointer = NULL;
    size_t read_size = 0;

    /* size of region allocated to dst_buf */
    int allocated_size=0;

    switch(src->src_identifier) {
    case FROM_CAM:
        /****************************************************************/
        /* get image from CAMERA */
        /****************************************************************/

        /* get 1frame from camera */
        if(videoCapture != NULL){
            img_from_cam = cvQueryFrame(videoCapture);
        }

        /* change resolution */
        resized_img = cvCreateImage(cvSize(RESIZED_IMG_WIDTH, RESIZED_IMG_HEIGHT), IPL_DEPTH_8U, 3);
        cvResize(img_from_cam, resized_img, CV_INTER_CUBIC);

        /* encode image data(to .jpg which data size is small) */
        encoded_img = cvEncodeImage(".jpg", resized_img, 0);

        /* allocate dst_buf region and data copy */
        allocated_size = encoded_img->cols * encoded_img->rows;

        *dst_buf = (unsigned char*)malloc(allocated_size);
        if(*dst_buf == NULL)
        {
            printf("allocate size %d\n", allocated_size);
            fprintf(stderr, "allocate error of bufffer for image...\n");
            return -1;
        }
        memcpy(*dst_buf, encoded_img->data.ptr, allocated_size);


        /* free temporary memory */
//        cvReleaseImage(&img_from_cam); // don't free image data gained from "cvQueryFrame"
        cvReleaseImage(&resized_img);

        cvReleaseMat(&encoded_img);

        break;

    case FROM_SHM:
        /****************************************************************/
        /* get image from SHARED MEMORY */
        /****************************************************************/
        /* allocate buffer region */
        *dst_buf = (unsigned char*)malloc(src->shm_size);
        if(*dst_buf == NULL)
        {
            fprintf(stderr, "allocate error of buffer for image...\n");
            return -1;
        }

        /* copy image data */
        memcpy(*dst_buf, (unsigned char*)src->pointer, src->shm_size);
        allocated_size = src->shm_size;

        break;

    case FROM_FILE:
        /****************************************************************/
        /* get image from FILE */
        /****************************************************************/
        /* get file size */
        fseek(fp, 0, SEEK_END);
        allocated_size = ftell(fp);
        fseek(fp, 0, SEEK_SET);

        /* allocate dst_buf region and data copy */
        *dst_buf = (unsigned char*)malloc(allocated_size);
        if(*dst_buf == NULL)
        {
            fprintf(stderr, "allocate error of bufffer for image...\n");
            return -1;
        }


        tmp_pointer = *dst_buf;

        while( feof(fp) == 0 )
        {
            read_size = fread(tmp_pointer, sizeof(unsigned char), 1, fp);
            tmp_pointer += read_size;
        }

        break;

    default:
        fprintf(stderr, "invalid image source is specified.\n");
        return -1;
    }

    return allocated_size;

} /* dlibImageGet */




/**************************************************************/
/*------------------------------------------------------------*/
/*
   [function name] : dlibImageSet
   [argument]      : pointer to source image
                     image size
                     pointer of image descriptor
   [return value]  : no value to return
   [description]   : store the image data to shared memory
*/
/*------------------------------------------------------------*/
/**************************************************************/
void dlibImageSet(
    unsigned char* buf,
    int image_size,
    DLimgdescriptor *dst
    )
{
    unsigned char* shrd_ptr_img = (unsigned char*)(dst->pointer);
    FILE* fp;

    /****************************************************************/
    /* size check */
    /****************************************************************/
    if(image_size > dst->shm_size)
    {
        fprintf(stderr, "specified image size over the shared memory size\n");
        exit(1);
    }

    /****************************************************************/
    /* write image to the shared memory */
    /****************************************************************/
    fp = fmemopen(shrd_ptr_img, image_size, "wb");
    fwrite(buf, sizeof(unsigned char), image_size, fp);
    fclose(fp);

} /* dlibImageSet */



/**************************************************************/
/*------------------------------------------------------------*/
/*
   [function name] : FreeImage
   [argument]      : pointer to image
   [return value]  : no value to return
   [description]   : free image buffer
*/
/*------------------------------------------------------------*/
/**************************************************************/
void dlibImageFree(unsigned char* buf)
{
    free(buf);
} /* dlibImageFree */
