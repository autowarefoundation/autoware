#ifndef DEAL_IMAGE
#define DEAL_IMAGE

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* define img_src_discriminater */
typedef enum { 
    /**
     * This indicates that  image will be gotten 
     * from camera.
     */
    FROM_CAM = 1, 

    /**
     * This indicates that  image will be gotten 
     * from shared memory.
     */
    FROM_SHM = 2, 

    /**
     * This indicates that  image will be gotten 
     * from file.
     */
    FROM_FILE = 3 
} DLimgsource;

    
/* data about shared memory */
/* #define SHM_PATH "/usr/local/geye/shared/input_file" */
/* #define SIZE_SHM_PATH "/usr/local/geye/shared/size_file" */
/* #define SHM_SIZE 680*480*3+100  // the size of 640x480 bitmap image + alpha */

/* resized image size */
#define RESIZED_IMG_WIDTH 640
#define RESIZED_IMG_HEIGHT 480

typedef struct {
    DLimgsource src_identifier; /* image source identifier */
    void* pointer;              /* pointer to image */
    int shm_size;               /* size of shaerd memory */
} DLimgdescriptor;


/* function declaration */
extern void dlibImageInit(
    DLimgdescriptor *src,
    char *pathname,
    int shm_size,
    DLimgsource src_identifier
    );

extern void dlibImageExit(
    DLimgdescriptor *src
    );


extern int dlibImageGet(
    unsigned char** dst_buf,
    DLimgdescriptor *src
    );


extern void dlibImageSet(
    unsigned char* buf, 
    int image_size,
    DLimgdescriptor *dst
    );

extern void dlibImageFree(
    unsigned char* buf
    );

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* DEAL_IMAGE */
