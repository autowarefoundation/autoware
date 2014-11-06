/*************************************/
/* program to operate shared memory  */
/*************************************/
#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include "data_for_shm.h"
#include "deal_image.h"


/* pointer to shared memory  */
extern unsigned char *shrd_ptr;
extern int           *shrd_ptr_height, *shrd_ptr_width;

static pthread_rwlock_t *shrd_ptr_rwlock;
static char             *shrd_ptr_imgupd;
static int               semid;
static DLimgdescriptor   imgdes_shm_output;


#define MY_ERROR_CHECK(val, txt1, txt2) {           \
    if ((val) < 0) {                                \
      printf("%s(%s) failed\n", (txt1), (txt2));    \
      exit(-1);                                     \
    }                                               \
  }


void attach_ShareMem(void)
{
  /* generate key */
  key_t shm_key = ftok(INPUT_SHM_PATH, 1);
  MY_ERROR_CHECK(shm_key, "key generation", "input_SHM");

  key_t shm_key_height = ftok(HEIGHT_SHM_PATH, 1);
  MY_ERROR_CHECK(shm_key_height, "key generation", "height_SHM");

  key_t shm_key_width = ftok(WIDTH_SHM_PATH, 1);
  MY_ERROR_CHECK(shm_key_width, "key generation", "width_SHM");

  key_t sem_key = ftok(SEM_PATH, 1);
  MY_ERROR_CHECK(sem_key, "key generation", "semaphore");

  key_t shm_key_rwlock = ftok(RWLOCK_SHM_PATH, 1);
  MY_ERROR_CHECK(shm_key_rwlock, "key generation", "reader-writer lock");

  key_t shm_key_imgupd = ftok(IMGUPD_SHM_PATH, 1);
  MY_ERROR_CHECK(shm_key_imgupd, "key generation", "image update checker");

  /* access to the shared memory */
  int shrd_id = shmget(shm_key, IMAGE_SIZE, 0666);
  MY_ERROR_CHECK(shrd_id, "Access shared memory", "input");

  int shrd_id_height = shmget(shm_key_height, sizeof(int), 0666);
  MY_ERROR_CHECK(shrd_id_height, "Access shared memory", "height");

  int shrd_id_width = shmget(shm_key_width, sizeof(int), 0666);
  MY_ERROR_CHECK(shrd_id_width, "Access shared memory", "width");

  semid = semget(sem_key, 1, 0666);
  MY_ERROR_CHECK(semid, "Access", "semaphore");

  int shrd_id_rwlock = shmget(shm_key_rwlock, sizeof(pthread_rwlock_t), 0666);
  MY_ERROR_CHECK(shrd_id_rwlock, "Access shared memory", "reader-writer lock");

  int shrd_id_imgupd = shmget(shm_key_imgupd, sizeof(char)*256, 0666);
  MY_ERROR_CHECK(shrd_id_imgupd, "Access shared memory", "image update checker");

  /* attach shared memory */
  shrd_ptr        = (unsigned char *)shmat(shrd_id, NULL, 0);
  shrd_ptr_height = (int *)shmat(shrd_id_height, NULL, 0);
  shrd_ptr_width  = (int *)shmat(shrd_id_width, NULL, 0);
  shrd_ptr_rwlock = (pthread_rwlock_t *)shmat(shrd_id_rwlock, NULL, 0);
  shrd_ptr_imgupd = (char *)shmat(shrd_id_imgupd, NULL, 0);

  /* initialize output shared memory */
  dlibImageInit(&imgdes_shm_output, OUTPUT_SHM_PATH, IMAGE_SIZE, FROM_SHM);

}

IplImage *getImage_fromSHM(void)
{
  static char imgupd_before[256] = {0};
  int upd_check = 0;

  while (1)         // loop until image in the shared memory is update
    {
      My_sem_operation(semid, LOCK); // lock semaphore
      upd_check = strcmp(shrd_ptr_imgupd, imgupd_before);
      My_sem_operation(semid, UNLOCK); // unlock semaphore

      if (upd_check != 0)
        {
          My_sem_operation(semid, LOCK); // lock semaphore
          strcpy(imgupd_before, shrd_ptr_imgupd);
          My_sem_operation(semid, UNLOCK); // unlock semaphore
          break;
        }
    }

  /* read image from buffer */
  CvMat *buf = cvCreateMat(1, IMAGE_SIZE, CV_8UC3);

  int rtn = pthread_rwlock_rdlock(shrd_ptr_rwlock); // lock reader-writer lock as reader
  MY_ERROR_CHECK(rtn, "lock", "as reader");

  memcpy(buf->data.ptr, shrd_ptr, IMAGE_SIZE);

  rtn = pthread_rwlock_unlock(shrd_ptr_rwlock); // unlock reader-writer lock
  MY_ERROR_CHECK(rtn, "unlock", "as reader");

  /* convert to IplImage format*/
  IplImage *image = cvDecodeImage(buf, CV_LOAD_IMAGE_COLOR);
  cvReleaseMat(&buf);
  return (image);
}


void setImage_toSHM(IplImage *result)
{

  /* encode image to jpeg format */
  CvMat *encoded_img = cvEncodeImage(".jpg", result);

  /* allocate buffer and copy image data */
  int image_size = encoded_img->cols * encoded_img->rows;
  unsigned char *buf = (unsigned char *)calloc(image_size, sizeof(unsigned char));
  memcpy(buf, encoded_img->data.ptr, image_size);

  /* set image to shared memory */
  int rtn = pthread_rwlock_wrlock(shrd_ptr_rwlock); // lock reader-writer lock as a writer
  MY_ERROR_CHECK(rtn, "lock", "as reader");

  dlibImageSet(buf, image_size, &imgdes_shm_output);

  rtn = pthread_rwlock_unlock(shrd_ptr_rwlock); // unlock reader-writer lock
  MY_ERROR_CHECK(rtn, "unlock", "as reader");

  /* release resources */
  free(buf);
  cvReleaseMat(&encoded_img);
}


void detach_ShareMem(void)
{
  MY_ERROR_CHECK(shmdt(shrd_ptr), "detach", "input");
  MY_ERROR_CHECK(shmdt(shrd_ptr_height), "detach", "height");
  MY_ERROR_CHECK(shmdt(shrd_ptr_width), "detach", "width");
  MY_ERROR_CHECK(shmdt(shrd_ptr_rwlock), "detach", "reader-writer lock");
  MY_ERROR_CHECK(shmdt(shrd_ptr_imgupd), "detach", "image update checker");
  dlibImageExit(&imgdes_shm_output);
}
