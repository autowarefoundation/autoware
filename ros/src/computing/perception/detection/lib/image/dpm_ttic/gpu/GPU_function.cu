#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "for_use_GPU.h"
#include "switch_release.h"
#define FROM_GPU
#include "switch_float.h"

/* declaration of texture memory */
//texture<FLOAT> A;
//texture<FLOAT> B;
texture<float, cudaTextureType1D, cudaReadModeElementType> A;
texture<float, cudaTextureType1D, cudaReadModeElementType> B;
texture<int2, cudaTextureType1D, cudaReadModeElementType> A_double;
texture<int2, cudaTextureType1D, cudaReadModeElementType> B_double;

//thread process
// convolve A and B(non_symmetric)
//unsigned __stdcall process(void *thread_arg) {

/********************************************/
/* function for calculating root */
/********************************************/
extern "C"
__global__
void
process_root
(
 //FLOAT *A,
 //FLOAT *B,
 FLOAT *C,
 int *A_dims_array,
 int *B_dims_array,
 int len,
 int interval,
 int L_MAX,
 int *error_array,
 int error_array_num,
 int pid,
 int device_number
)
{
  int idx_x = blockIdx.x * blockDim.x + threadIdx.x;
  int idx_y = blockIdx.y * blockDim.y + threadIdx.y;
  int ii = blockIdx.z % len;
  int level = blockIdx.z / len;

  int A_dims[3] = { A_dims_array[level*3], A_dims_array[level*3+1], A_dims_array[level*3+2] };
  int B_dims[3] = { B_dims_array[ii*3], B_dims_array[ii*3+1], B_dims_array[ii*3+2] };
  int C_dims[2] = { A_dims[0] - B_dims[0] + 1, A_dims[1] - B_dims[1] + 1 };

  int C_x = C_dims[1]/device_number;

  if(C_dims[1]%device_number != 0){
    C_x++;
  }

  idx_x = idx_x + pid * C_x;

  if(idx_x < C_x * pid  ||  idx_x >=  C_x * (pid + 1)){
    return ;
  }

  if(0 <= ii && ii < len && 0 <= idx_x && idx_x < C_dims[1] && 0 <= idx_y && idx_y < C_dims[0] && interval <= level && level < L_MAX ) {


    int num_features = A_dims[2];
    const int A_SQ = A_dims[0]*A_dims[1];
    const int B_SQ = B_dims[0]*B_dims[1];
    FLOAT add_val = 0;

    int x = idx_x;
    int y = idx_y;
    int XA0 = A_dims[0]*x;


    /* apply loop condition */
    for(int i=0; i<error_array_num; i++){
      if(error_array[i] == level){
        return;
      }
    }



    /* adjust the location of pointer of C */
    FLOAT *dst;
    uintptr_t pointer = (uintptr_t)C;

    for(int a=interval; a<level; a++) {
      for(int b=0; b<len; b++) {
        int height = A_dims_array[a*3] - B_dims_array[b*3] + 1;
        int width = A_dims_array[a*3 + 1] - B_dims_array[b*3 + 1] + 1;

        /* error semantics */
        if (height < 1 || width < 1){
          printf("Invalid input in GPU\n");
          return;
        }

        pointer += (uintptr_t)(height*width*sizeof(FLOAT));

      }
    }

    for(int b=0; b<ii; b++){
      int height = A_dims_array[level*3] - B_dims_array[b*3] + 1;
      int width  = A_dims_array[level*3 + 1] - B_dims_array[b*3 + 1] + 1;

      /* error semantics */
      if (height < 1 || width < 1){
        printf("Invalid input in GPU\n");
        return;
      }

      pointer += (uintptr_t)(height*width*sizeof(FLOAT));
    }

    dst = (FLOAT *)pointer;

    /* adjust the location of pointer of A */
    //unsigned long long int pointerA = (unsigned long long int)A;
    int A_index_ini = 0;
    for(int a=0; a<level; a++) {
      //      pointerA += (unsigned long long int)(A_dims_array[a*3]*A_dims_array[a*3 + 1]*A_dims_array[a*3 + 2]*sizeof(FLOAT));
      A_index_ini += A_dims_array[a*3]*A_dims_array[a*3 + 1]*A_dims_array[a*3 + 2];
    }


    /* adjust the location of pointer of B */
    //unsigned long long int pointerB = (unsigned long long int)B;
    int B_index_ini = 0;
    for(int b=0; b<ii; b++) {
      //      pointerB += (unsigned long long int)(B_dims_array[b*3]*B_dims_array[b*3 + 1]*B_dims_array[b*3 + 2]*sizeof(FLOAT));
      B_index_ini += B_dims_array[b*3]*B_dims_array[b*3 + 1]*B_dims_array[b*3 + 2];
    }


    for(int f = 0; f < num_features; f++) // num_features = 31
      {
        // FLOAT *A_src = (FLOAT *)pointerA + f*A_SQ;
        int A_index = A_index_ini + f*A_SQ;
        // FLOAT *B_src = (FLOAT *)pointerB + f*B_SQ;
        int B_index = B_index_ini + f*B_SQ;

        // FLOAT *A_src2 =A_src+XA0;
        A_index += XA0;

        FLOAT val = 0;
        // FLOAT *A_off = A_src2+y;
        A_index += y;
        // FLOAT *B_off = B_src;

        for (int xp = 0; xp < B_dims[1]; xp++)
          {
            // FLOAT *A_temp = A_off;
            int A_index_tmp = A_index;
            // FLOAT *B_temp = B_off;
            int B_index_tmp = B_index;

            for (int yp = 0; yp < B_dims[0]; yp++)
              {
                // val += *(A_temp++) * *(B_temp++);
                if(sizeof(FLOAT) == sizeof(float)) // if configured to use single precision
                  {
                    FLOAT A_val = tex1Dfetch(A, A_index_tmp);
                    FLOAT B_val = tex1Dfetch(B, B_index_tmp);
                    val += A_val * B_val;
                  }
                else
                  {      // if configured to use double precision
                    int2 A_val = tex1Dfetch(A_double, A_index_tmp);
                    int2 B_val = tex1Dfetch(B_double, B_index_tmp);
                    val += __hiloint2double(A_val.y, A_val.x) * __hiloint2double(B_val.y, B_val.x);
                  }

                A_index_tmp++;
                B_index_tmp++;
              }

            // A_off+=A_dims[0];
            A_index += A_dims[0];
            // B_off+=B_dims[0];
            B_index += B_dims[0];

          }

        add_val += val;
      }

    *(dst + (idx_x*C_dims[0] + idx_y)) += add_val;
  }


  return;
}



/********************************************/
/* function for calculating part */
/********************************************/
extern "C"
__global__
void
process_part
(
 //FLOAT *A,
 //FLOAT *B,
 FLOAT *C,
 int *A_dims_array,
 int *B_dims_array,
 int len,
 int interval,
 int L_MAX,
 int *error_array,
 int error_array_num,
 int pid,
 int device_number
)
{
  int idx_x = blockIdx.x * blockDim.x + threadIdx.x;
  int idx_y = blockIdx.y * blockDim.y + threadIdx.y;
  int ii = blockIdx.z % len;
  int level = blockIdx.z / len;

  int A_dims[3] = { A_dims_array[level*3], A_dims_array[level*3+1], A_dims_array[level*3+2] };
  int B_dims[3] = { B_dims_array[ii*3], B_dims_array[ii*3+1], B_dims_array[ii*3+2] };
  int C_dims[2] = { A_dims[0] - B_dims[0] + 1, A_dims[1] - B_dims[1] + 1 };

  int C_x = C_dims[1]/device_number;

  if(C_dims[1]%device_number != 0){
    C_x++;
  }

  idx_x = idx_x + pid * C_x;

  if(idx_x < C_x * pid  ||  idx_x >=  C_x * (pid + 1)){
    return ;
  }

  if(0 <= ii && ii < len && 0 <= idx_x && idx_x < C_dims[1] && 0 <= idx_y && idx_y < C_dims[0] && 0 <= level && level < (L_MAX - interval) ) {
    int num_features = A_dims[2];
    const int A_SQ = A_dims[0]*A_dims[1];
    const int B_SQ = B_dims[0]*B_dims[1];
    FLOAT add_val = 0;

    int x = idx_x;
    int y = idx_y;
    int XA0 = A_dims[0]*x;

    /* apply loop condition */
    for(int i=0; i<error_array_num; i++){
      if(error_array[i] == level)
        return;
    }

    /* adjust the location of pointer of C */
    FLOAT *dst;
    uintptr_t pointer = (uintptr_t)C;
    for(int a=0; a<level; a++) {
      for(int b=0; b<len; b++){
        int height = A_dims_array[a*3] - B_dims_array[b*3] + 1;
        int width = A_dims_array[a*3 + 1] - B_dims_array[b*3 + 1] + 1;

        /* error semantics */
        if(height < 1 || width < 1){
          printf("Invalid input in GPU\n");
          return;
        }

        pointer += (uintptr_t)(height*width*sizeof(FLOAT));
      }
    }

    for(int b=0; b<ii; b++){
      int height = A_dims_array[level*3] - B_dims_array[b*3] + 1;
      int width  = A_dims_array[level*3 + 1] - B_dims_array[b*3 + 1] + 1;

       /* error semantics */
        if(height < 1 || width < 1){
          printf("Invalid input in GPU\n");
          return;
        }

      pointer += (uintptr_t)(height*width*sizeof(FLOAT));
    }


    dst = (FLOAT *)pointer;

    /* adjust the location of pointer of A */
    // unsigned long long int pointerA = (unsigned long long int)A;
    int A_index_ini = 0;
    for(int a=0; a<level; a++) {
      // pointerA += (unsigned long long int)(A_dims_array[a*3]*A_dims_array[a*3 + 1]*A_dims_array[a*3 + 2]*sizeof(FLOAT));
      A_index_ini += A_dims_array[a*3]*A_dims_array[a*3 + 1]*A_dims_array[a*3 + 2];
    }

    /* adjust the location of pointer of B */
    // unsigned long long int pointerB = (unsigned long long int)B;
    int B_index_ini = 0;
    for(int b=0; b<ii; b++) {
      // pointerB += (unsigned long long int)(B_dims_array[b*3]*B_dims_array[b*3 + 1]*B_dims_array[b*3 + 2]*sizeof(FLOAT));
      B_index_ini += B_dims_array[b*3]*B_dims_array[b*3 + 1]*B_dims_array[b*3 + 2];
    }

    for(int f = 0; f < num_features; f++) // num_features = 31
      {
        // FLOAT *A_src = (FLOAT *)pointerA + f*A_SQ;
        int A_index = A_index_ini + f*A_SQ;
        // FLOAT *B_src = (FLOAT *)pointerB + f*B_SQ;
        int B_index = B_index_ini + f*B_SQ;

        // FLOAT *A_src2 =A_src+XA0;
        A_index += XA0;

        FLOAT val = 0;
        // FLOAT *A_off = A_src2+y;
        A_index += y;
        // FLOAT *B_off = B_src;

        for (int xp = 0; xp < B_dims[1]; xp++)
          {
            // FLOAT *A_temp = A_off;
            int A_index_tmp = A_index;
            // FLOAT *B_temp = B_off;
            int B_index_tmp = B_index;

            for (int yp = 0; yp < B_dims[0]; yp++)
              {
                // val += *(A_temp++) * *(B_temp++);
                if(sizeof(FLOAT) == sizeof(float)) // if configured to use single precision
                  {
                    FLOAT A_val = tex1Dfetch(A, A_index_tmp);
                    FLOAT B_val = tex1Dfetch(B, B_index_tmp);
                    val += A_val * B_val;
                  }
                else            // if configured to use double precision
                  {
                    int2 A_val = tex1Dfetch(A_double, A_index_tmp);
                    int2 B_val = tex1Dfetch(B_double, B_index_tmp);
                    val += __hiloint2double(A_val.y, A_val.x) * __hiloint2double(B_val.y, B_val.x);
                  }

                A_index_tmp++;
                B_index_tmp++;
              }

            // A_off+=A_dims[0];
            A_index += A_dims[0];
            // B_off+=B_dims[0];
            B_index += B_dims[0];

          }
        add_val += val;
      }

    *(dst + (idx_x*C_dims[0] + idx_y)) += add_val;
  }

  return;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
extern "C"
__global__
void
inverse_Q(
  FLOAT *src_start,
  int *size_array,
  int *error_array,
  int error_array_num,
  int NoP,
  int *PIDX_array,
  int *numpart,
  int NoC,
  int max_numpart,
  int interval,
  int L_MAX,
  int pid,
  int device_number
          )
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int kk = blockIdx.y * blockDim.y + threadIdx.y;
  int jj = threadIdx.z;
  int L = blockIdx.z;
  int numpart_jj;
  int C_y;


  if(0<=jj && jj<NoC)
    {
      numpart_jj = numpart[jj];
      C_y = numpart_jj/device_number;
      if(numpart_jj%device_number != 0){
        C_y++;
       }
      kk = kk + pid * C_y;
      if(kk < C_y * pid  ||  kk >=  C_y * (pid + 1)){
         return ;
       }
    } else return ;


  if(0<=L && L < (L_MAX-interval))
    {

      /* loop condition */
      for(int h=0; h<error_array_num; h++) {
        if(L==error_array[h]){
          return;
        }
      }


      if( 0<=kk && kk < numpart_jj )
        {
          int PIDX = PIDX_array[L*(NoC*max_numpart) + jj*max_numpart + kk];
          int dim0 = size_array[L*NoP*2 + PIDX*2];
          int dim1 = size_array[L*NoP*2 + PIDX*2+1];

          if( idx < 0 || dim0*dim1 <= idx) return;

              /* pointer adjustment */
          FLOAT *src;
          uintptr_t ptr_adjuster = (uintptr_t)src_start;
          for(int i=0; i<L; i++) {

                /* apply error condition */
            int error_flag=0;
            for(int h=0; h<error_array_num; h++) {
              if(i==error_array[h]){
                error_flag = 1;
              }
            }
            if(error_flag != 0) {
              continue;
            }


            for(int j=0; j<NoP; j++) {
              int height = size_array[i*NoP*2 + j*2];
              int width = size_array[i*NoP*2 + j*2+1];
              ptr_adjuster += (uintptr_t)(height*width*sizeof(FLOAT));

            }
          }



          for(int j=0; j<PIDX; j++) {
            int height = size_array[L*NoP*2 + j*2];
            int width = size_array[L*NoP*2 + j*2+1];
            ptr_adjuster += (uintptr_t)(height*width*sizeof(FLOAT));
          }

          src = (FLOAT *)ptr_adjuster;

          *(src + idx) *= -1;

      }
    }
}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// dt helper function
__device__
void
dt_helper(FLOAT *src, FLOAT *dst, int *ptr, int step, int s1, int s2, int d1, int d2, FLOAT a, FLOAT b)
{
  if (d2 >= d1)
    {
      int d = (d1+d2) >> 1;
      int ds =d*step;
      int s = s1;
      FLOAT src_ss = *(src+s*step);
      for (int p = s1+1; p <= s2; p++)
        {
          int t1 = d-s;
          int t2 = d-p;
          if (src_ss + a*t1*t1 + b*t1 > *(src+p*step) + a*t2*t2 + b*t2)
            {
              s = p;
              src_ss = *(src+s*step);
            }
        }
      int D = d-s;
      dst[ds] = *(src+s*step) + a*D*D + b*D;
      ptr[ds] = s;
      dt_helper(src, dst, ptr, step, s1, s, d1, d-1, a, b);
      dt_helper(src, dst, ptr, step, s, s2, d+1, d2, a, b);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//sub function of dt
extern "C"
__global__
void
dt1d_x(
  FLOAT *src_start,             // part_C_dev
  FLOAT *dst_start,             // tmpM_dev
  int *ptr_start,               // tmpIy_dev
  int *DID_4_array,             // DID_4_array_dev
  FLOAT *def_array,             // def_array_dev
  int *size_array,              // pm_size_array_dev
  int NoP,                      // NoP
  int *PIDX_array,              // PIDX_array_dev
  int *error_array,             // part_error_array_dev
  int error_array_num,          // part_error_array_num
  int *numpart,                 // numpart_jj
  int NoC,                      // NoC
  int max_numpart,              // max_numpart
  int interval,                 // interval
  int L_MAX,                     // L_MAX
  int pid,                       // pid
  int device_number              // device_number

       )
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int kk = blockIdx.y * blockDim.y + threadIdx.y;
  int jj = threadIdx.z;
  int L = blockIdx.z;
  int numpart_jj;
  int C_y;

  if(0<=jj && jj<NoC)
    {

      numpart_jj = numpart[jj];
      C_y = numpart_jj/device_number;

      if(numpart_jj%device_number != 0){
        C_y++;
       }

      kk = kk + pid * C_y;

      if(kk < C_y * pid  ||  kk >=  C_y * (pid + 1)){
         return ;
       }
    } else{
      return ;
    }


  if(0<=L && L<(L_MAX-interval))
    {
      /* loop condition */
      for(int h=0; h<error_array_num; h++) {
        if(L==error_array[h]){
          return;
        }
      }

      if(0<=kk && kk<numpart_jj)
        {
          int PIDX = PIDX_array[L*(NoC*max_numpart) + jj*max_numpart + kk];
          int dim1 = size_array[L*NoP*2 + PIDX*2+1];

          if( idx < 0 || dim1 <= idx ) return;

          int dim0 = size_array[L*NoP*2 + PIDX*2];
          int XD=0;
          int step = 1;
          int n = dim0;
          int DID_4 = DID_4_array[L*(NoC*max_numpart) + jj*max_numpart + kk];
          FLOAT a = def_array[DID_4+2];
          FLOAT b = def_array[DID_4+3];

          /* pointer adjustment */
          uintptr_t adj_src = (uintptr_t)src_start;
          uintptr_t adj_dst = (uintptr_t)dst_start;
          uintptr_t adj_ptr = (uintptr_t)ptr_start;
          /* for src */
          for(int i=0; i<L; i++) {

            /* apply error condition */
            int error_flag=0;
            for(int h=0; h<error_array_num; h++) {
              if(i==error_array[h]){
                error_flag = 1;
              }
            }
            if(error_flag != 0) {
              continue;
            }

            for(int j=0; j<NoP; j++) {
              int height = size_array[i*NoP*2 + j*2];
              int width = size_array[i*NoP*2 + j*2+1];
              adj_src += (uintptr_t)(height*width*sizeof(FLOAT));

            }
          }


          for(int j=0; j<PIDX; j++) {
            int height = size_array[L*NoP*2 + j*2];
            int width = size_array[L*NoP*2 + j*2+1];
            adj_src += (uintptr_t)(height*width*sizeof(FLOAT));
          }

              /* for dst, ptr */
              // adjust "dst" to tmpM[L][jj][kk]
              // adjust "ptr" to tmpIy[L][jj][kk]
          for(int i=0; i<L; i++) {

                /* apply error condition */
            int error_flag=0;
            for(int h=0; h<error_array_num; h++) {
              if(i==error_array[h]){
                error_flag = 1;
              }
            }
            if(error_flag != 0) {
              continue;
            }

            for(int j=0; j<NoC; j++) {
              for(int k=0; k<numpart[j]; k++) {
                int PIDX_tmp = PIDX_array[i*(NoC*max_numpart) + j*max_numpart + k];
                int dims0_tmp = size_array[i*NoP*2 + PIDX_tmp*2];
                int dims1_tmp = size_array[i*NoP*2 + PIDX_tmp*2+1];


                adj_dst += (uintptr_t)(dims0_tmp*dims1_tmp*sizeof(FLOAT));
                adj_ptr += (uintptr_t)(dims0_tmp*dims1_tmp*sizeof(int));


              }
            }
          }


          for(int i=0; i<jj; i++) {
            for(int j=0; j<numpart[i]; j++) {
              int PIDX_tmp = PIDX_array[L*(NoC*max_numpart) + i*max_numpart + j]; // PIDX_array[L][i][j]
              int dims0_tmp = size_array[L*NoP*2 + PIDX_tmp*2]; // size_array[L][PIDX_tmp*2]
              int dims1_tmp = size_array[L*NoP*2 + PIDX_tmp*2+1]; // size_array[L][PIDX_tmp*2+1]

              adj_dst += (uintptr_t)(dims0_tmp*dims1_tmp*sizeof(FLOAT));
              adj_ptr += (uintptr_t)(dims0_tmp*dims1_tmp*sizeof(int));

            }
          }

          for(int j=0; j<kk; j++) {
            int PIDX_tmp = PIDX_array[L*(NoC*max_numpart) + jj*max_numpart + j]; // PIDX_array[L][jj][j]
            int dims0_tmp = size_array[L*NoP*2 + PIDX_tmp*2]; // size_array[L][PIDX_tmp*2]
            int dims1_tmp = size_array[L*NoP*2 + PIDX_tmp*2+1]; // size_array[L][PIDX_tmp*2+1]

            adj_dst += (uintptr_t)(dims0_tmp*dims1_tmp*sizeof(FLOAT));
            adj_ptr += (uintptr_t)(dims0_tmp*dims1_tmp*sizeof(int));
          }


          FLOAT *src = (FLOAT *)adj_src;
          FLOAT *dst = (FLOAT *)adj_dst;
          int *ptr = (int *)adj_ptr;

          /* main calculation of di1d_x */
          XD = idx*dim0;
          dt_helper(src+XD, dst+XD, ptr+XD, step, 0, n-1, 0, n-1, a, b);

        }
    }
}


extern "C"
__global__
void
dt1d_y(
  FLOAT *src_start,             // tmpM_dev
  FLOAT *dst_start,             // M_dev
  int *ptr_start,               // tmpIx_dev
  int *DID_4_array,             // DID_4_array_dev
  FLOAT *def_array,             // def_array_dev
  int NoP,                      // NoP
  int *size_array,              // pm_size_array_dev
  int *numpart,                 // numpart_jj
  int *PIDX_array,              // PIDX_array_dev
  int NoC,                      // NoC
  int max_numpart,              // max_numpart
  int interval,                 // interval
  int L_MAX,                    // L_MAX
  int *error_array,             // part_error_array_dev
  int error_array_num,           // part_error_array_num
  int pid,                       // pid
  int device_number              // device_number
       )
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  int kk = blockIdx.y * blockDim.y + threadIdx.y;
  int jj = threadIdx.z;
  int L = blockIdx.z;
  int numpart_jj;
  int C_y;

  if(0<=jj && jj<NoC)
    {

      numpart_jj = numpart[jj];
      C_y = numpart_jj/device_number;

      if(numpart_jj%device_number != 0){
        C_y++;
       }

      kk = kk + pid * C_y;

      if(kk < C_y * pid  ||  kk >=  C_y * (pid + 1)){
         return ;
       }
    } else{
      return ;
    }


  if(0<=L && L<(L_MAX-interval))
    {
      /* loop condition */
      for(int h=0; h<error_array_num; h++) {
        if(L==error_array[h]){
          return;
        }
      }


      if( 0<=kk && kk<numpart_jj)
        {
          int PIDX = PIDX_array[L*(NoC*max_numpart) + jj*max_numpart + kk];
          int dim0 = size_array[L*NoP*2 + PIDX*2];

          if( idx < 0 || dim0 <= idx ) return;

          int dim1 = size_array[L*NoP*2 + PIDX*2+1];
          int step  = dim0;
          int n = dim1;

          int DID_4 = DID_4_array[L*(NoC*max_numpart) + jj*max_numpart + kk];

          FLOAT a = def_array[DID_4];   // ax
          FLOAT b = def_array[DID_4+1]; // bx

              /* pointer adjustment */
          uintptr_t adj_src = (uintptr_t)src_start;
          uintptr_t adj_dst = (uintptr_t)dst_start;
          uintptr_t adj_ptr = (uintptr_t)ptr_start;
              /* for src, dst, ptr */
              /* adjust "src" to tmpM[L][jj][kk] */
              /* adjust "dst" to M[L][jj][kk] */
              /* adjust "ptr" to tmpIx[L][jj][kk] */
          for(int i=0; i<L; i++) {

            /* apply error condition */
            int error_flag=0;
            for(int h=0; h<error_array_num; h++) {
              if(i==error_array[h]){
                error_flag = 1;
              }
            }
            if(error_flag != 0) {
              continue;
            }

            for(int j=0; j<NoC; j++) {
              for(int k=0; k<numpart[j]; k++) {

                int PIDX_tmp = PIDX_array[i*(NoC*max_numpart) + j*max_numpart + k];
                int dims0_tmp = size_array[i*NoP*2 + PIDX_tmp*2];
                int dims1_tmp = size_array[i*NoP*2 + PIDX_tmp*2+1];

                adj_src += (uintptr_t)(dims0_tmp*dims1_tmp*sizeof(FLOAT));
                adj_dst += (uintptr_t)(dims0_tmp*dims1_tmp*sizeof(FLOAT));
                adj_ptr += (uintptr_t)(dims0_tmp*dims1_tmp*sizeof(int));

              }
            }
          }


          for(int i=0; i<jj; i++) {
            for(int j=0; j<numpart[i]; j++) {
              int PIDX_tmp = PIDX_array[L*(NoC*max_numpart) + i*max_numpart + j]; // PIDX_array[L][i][j]
              int dims0_tmp = size_array[L*NoP*2 + PIDX_tmp*2]; // size_array[L][PIDX_tmp*2]
              int dims1_tmp = size_array[L*NoP*2 + PIDX_tmp*2+1]; // size_array[L][PIDX_tmp*2+1]

              adj_src += (uintptr_t)(dims0_tmp*dims1_tmp*sizeof(FLOAT));
              adj_dst += (uintptr_t)(dims0_tmp*dims1_tmp*sizeof(FLOAT));
              adj_ptr += (uintptr_t)(dims0_tmp*dims1_tmp*sizeof(int));

            }
          }

          for(int j=0; j<kk; j++) {
            int PIDX_tmp = PIDX_array[L*(NoC*max_numpart) + jj*max_numpart + j];
            int dims0_tmp = size_array[L*NoP*2 + PIDX_tmp*2];
            int dims1_tmp = size_array[L*NoP*2 + PIDX_tmp*2+1];

            adj_src += (uintptr_t)(dims0_tmp*dims1_tmp*sizeof(FLOAT));
            adj_dst += (uintptr_t)(dims0_tmp*dims1_tmp*sizeof(FLOAT));
            adj_ptr += (uintptr_t)(dims0_tmp*dims1_tmp*sizeof(int));
          }



          FLOAT *src = (FLOAT *)adj_src;
          FLOAT *dst = (FLOAT *)adj_dst;
          int *ptr = (int *)adj_ptr;


          dt_helper(src+idx, dst+idx, ptr+idx, step, 0, n-1, 0, n-1, a, b);


        }
    }
}

/*************************************************************/
/*************************************************************/
/* original source of dt function loop */
// for (int x = 0; x < dims[1]; x++)
//   {
//     dt1d(vals+XD, tmpM+XD, tmpIy+XD, 1, dims[0], ay, by);
//     XD+=dims[0];
//   }
// for (int y = 0; y < dims[0]; y++)
//   {
//     dt1d(tmpM+y, M+y, tmpIx+y, dims[0], dims[1], ax, bx);
//   }
/*************************************************************/
/*************************************************************/



extern "C"
__global__
void
calc_a_score(
 int IWID,
 int IHEI,
 FLOAT scale,
 int padx_n,
 int pady_n,
 int *RX_array,
 int *RY_array,
 FLOAT *ac_score,
 FLOAT *score_array,
 int *ssize_array,
 int NoC,
 int *size_score_array
)
{
  int ii = blockIdx.x * blockDim.x + threadIdx.x;
  int jj = blockIdx.y * blockDim.y + threadIdx.y;

  int component_jj = threadIdx.z;

  if(0<=component_jj && component_jj < NoC)
    {
      if(0<=ii && ii<IWID && 0<=jj && jj<IHEI)
        {
          uintptr_t pointer_score = (uintptr_t)score_array;
          uintptr_t pointer_ssize = (uintptr_t)ssize_array;
          uintptr_t pointer_RX = (uintptr_t)RX_array;
          uintptr_t pointer_RY = (uintptr_t)RY_array;
          for(int k=0; k<component_jj; k++) {
            pointer_score += (uintptr_t)size_score_array[k];
            pointer_ssize += (uintptr_t)(sizeof(int));
            pointer_RX += (uintptr_t)(sizeof(int));
            pointer_RY += (uintptr_t)(sizeof(int));
          }

          FLOAT *score = (FLOAT *)pointer_score;
          int ssize0 = *((int *)pointer_ssize);
          int ssize1 = *((int *)pointer_ssize + sizeof(int));
          int RX = *((int *)pointer_RX);
          int RY = *((int *)pointer_RY);



          // if(0<=ii && ii<IWID && 0<=jj && jj<IHEI)
          //   {
          int Xn = (int)((FLOAT)ii/scale+padx_n);
          int Yn = (int)((FLOAT)jj/scale+pady_n);


          if(Yn<ssize0 && Xn<ssize1)
            {
              FLOAT sc = score[Yn+Xn*ssize0];
              int Im_Y = jj+RY;
              int Im_X = ii+RX;
              if(Im_Y<IHEI && Im_X<IWID)
                {
                  FLOAT *PP = ac_score+Im_Y+Im_X*IHEI;
                  if(sc>*PP) *PP=sc;
                }
            }
        }
    }

  /*************************************************************/
  /*************************************************************/
  /* original source of calc_a_score loop */
  // for(int ii=0;ii<IWID;ii++)
  //   {
  //     int Xn=(int)((FLOAT)ii/scale+padx_n);

  //     for(int jj=0;jj<IHEI;jj++)
  //       {
  //         int Yn =(int)((FLOAT)jj/scale+pady_n);

  //         if(Yn<ssize[0] && Xn<ssize[1])
  //           {
  //             FLOAT sc = score[Yn+Xn*ssize[0]]; //get score of pixel

  //             int Im_Y = jj+RY;
  //             int Im_X = ii+RX;
  //             if(Im_Y<IHEI && Im_X<IWID)
  //               {
  //                 FLOAT *PP=ac_score+Im_Y+Im_X*IHEI; //consider root rectangle size
  //                 if(sc>*PP) *PP=sc;                 //save max score
  //               }
  //           }
  //       }
  //   }
  /*************************************************************/
  /*************************************************************/

}



__device__
static inline int
min_i(int x, int y)
{return (x <= y ? x : y);}

#ifdef USE_FLOAT_AS_DECIMAL
/************************************************/
/* atomic function dealing with float precision */
__device__
static inline float
atomicAdd_float(float *address, float val)
{
  return atomicAdd(address, val);      // atomicAdd must be called from "__device__" function
}
/*************************************************/

#else  /* ifdef USE_FLOAT_AS_DECIMAL */

/*************************************************/
/* atomic function dealing with double precision */
__device__
static inline double
atomicAdd_double
(double *address, double val)
{
  uintptr_t *address_as_ull = (uintptr_t *)address;
  uintptr_t old = *address_as_ull, assumed;
  do {
    assumed = old;
    old = atomicCAS(address_as_ull, assumed, __double_as_longlong(val + __longlong_as_double(assumed)));
  }while(assumed != old);
  return __longlong_as_double(old);
}
/*************************************************/
#endif  /* ifdef USE_FLOAT_AS_DECIMAL */


/***********************************************************/
/* function which cast from int2 to unsigned long long int */
__device__
static inline unsigned long long int
hiloint2uint64(int hi, int lo)
{
  int combined[] = {hi, lo};
  return *reinterpret_cast<unsigned long long int*>(combined);
}
/***********************************************************/


/* declaration of texture memory */
#ifdef USE_FLOAT_AS_DECIMAL
texture<float, cudaTextureType1D, cudaReadModeElementType> resized_image;
#else
texture<uint2, cudaTextureType1D, cudaReadModeElementType>  resized_image_double;
#endif

texture<int , cudaTextureType1D, cudaReadModeElementType>  resized_image_size;
texture<int, cudaTextureType1D, cudaReadModeElementType>   image_idx_incrementer;
texture<uint2, cudaTextureType1D, cudaReadModeElementType> hist_ptr_incrementer;
texture<uint2, cudaTextureType1D, cudaReadModeElementType> norm_ptr_incrementer;
texture<uint2, cudaTextureType1D, cudaReadModeElementType> feat_ptr_incrementer;

extern "C"
__global__
void
calc_hist
(
 FLOAT *hist_top,
 int sbin,
 int visible_0,
 int visible_1,
 int level
 )
{
  /* index of each pixels */
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  const FLOAT Hcos[9] = {1.0000, 0.9397, 0.7660, 0.5000, 0.1736, -0.1736, -0.5000, -0.7660, -0.9397};
  const FLOAT Hsin[9] = {0.0000, 0.3420, 0.6428, 0.8660, 0.9848, 0.9848, 0.8660, 0.6428, 0.3420};

  /* adjust pointer position */
  int   base_index     = tex1Dfetch(image_idx_incrementer, level);
  uint2 ptr_hist_uint2 = tex1Dfetch(hist_ptr_incrementer, level);
  uintptr_t ptr_hist = (uintptr_t)hist_top + hiloint2uint64(ptr_hist_uint2.x, ptr_hist_uint2.y); // convert uint2 -> unsigned long long int
  FLOAT *hist = (FLOAT *)ptr_hist;

  /* input size */
  const int height = tex1Dfetch(resized_image_size, level*3);
  const int width  = tex1Dfetch(resized_image_size, level*3 + 1);

  const int dims[2] = {height, width};

  /* size of Histgrams and Norm calculation space */
  const int blocks[2] = {
    (int)floor((double)height/(double)sbin+0.5),
    (int)floor((double)width/(double)sbin+0.5)
  };


  // for (int x=1; x<visible[1]-1; x++) {
  //   for (int y=1; y<visible[0]-1; y++) {
  if (1<=x && x<visible_1-1 && 1<=y && y<visible_0-1)
    {
      /* first color channel */
      //      base_index += min_i(x, dims[1]-2)*dims[0] + min_i(y, dims[0]-2);
      base_index += min_i(x, dims[1]-2) + min_i(y, dims[0]-2)*dims[1];
      FLOAT dx, dy;

#ifdef USE_FLOAT_AS_DECIMAL
        {
          // dy = tex1Dfetch(resized_image, base_index + 1) - tex1Dfetch(resized_image, base_index - 1) ;
          // dx = tex1Dfetch(resized_image, base_index + dims[0]) - tex1Dfetch(resized_image, base_index - dims[0]) ;
          dx = tex1Dfetch(resized_image, base_index + 1) - tex1Dfetch(resized_image, base_index - 1) ;
          dy = tex1Dfetch(resized_image, base_index + dims[1]) - tex1Dfetch(resized_image, base_index - dims[1]) ;
        }
#else
        {
          int2 arg1 = tex1Dfetch(resized_image_double, base_index + 1);
          int2 arg2 = tex1Dfetch(resized_image_double, base_index - 1) ;
          // dy = __hiloint2double(arg1.y, arg1.x) - __hiloint2double(arg2.y, arg2.x);
          dx = __hiloint2double(arg1.y, arg1.x) - __hiloint2double(arg2.y, arg2.x);

          // arg1 = tex1Dfetch(resized_image_double, base_index + dims[0]);
          // arg2 = tex1Dfetch(resized_image_double, base_index - dims[0]);
          // dx = __hiloint2double(arg1.y, arg1.x) - __hiloint2double(arg2.y, arg2.x);
          arg1 = tex1Dfetch(resized_image_double, base_index + dims[1]);
          arg2 = tex1Dfetch(resized_image_double, base_index - dims[1]);
          dy = __hiloint2double(arg1.y, arg1.x) - __hiloint2double(arg2.y, arg2.x);
        }
#endif
      FLOAT  v  = dx*dx + dy*dy;

      /* second color channel */
      base_index += dims[0]*dims[1];
      FLOAT dx2, dy2;

#ifdef USE_FLOAT_AS_DECIMAL
        {
          // dy2 = tex1Dfetch(resized_image, base_index + 1) - tex1Dfetch(resized_image, base_index - 1) ;
          // dx2 = tex1Dfetch(resized_image, base_index + dims[0]) - tex1Dfetch(resized_image, base_index - dims[0]) ;
          dx2 = tex1Dfetch(resized_image, base_index + 1) - tex1Dfetch(resized_image, base_index - 1) ;
          dy2 = tex1Dfetch(resized_image, base_index + dims[1]) - tex1Dfetch(resized_image, base_index - dims[1]) ;
        }
      #else
        {
          int2 arg1 = tex1Dfetch(resized_image_double, base_index + 1);
          int2 arg2 = tex1Dfetch(resized_image_double, base_index - 1) ;
          // dy2 = __hiloint2double(arg1.y, arg1.x) - __hiloint2double(arg2.y, arg2.x);
          dx2 = __hiloint2double(arg1.y, arg1.x) - __hiloint2double(arg2.y, arg2.x);

          // arg1 = tex1Dfetch(resized_image_double, base_index + dims[0]);
          // arg2 = tex1Dfetch(resized_image_double, base_index - dims[0]);
          // dx2 = __hiloint2double(arg1.y, arg1.x) - __hiloint2double(arg2.y, arg2.x);
          arg1 = tex1Dfetch(resized_image_double, base_index + dims[1]);
          arg2 = tex1Dfetch(resized_image_double, base_index - dims[1]);
          dy2 = __hiloint2double(arg1.y, arg1.x) - __hiloint2double(arg2.y, arg2.x);
        }
#endif
      FLOAT v2  = dx2*dx2 + dy2*dy2;

      /* third color channel */
      base_index += dims[0]*dims[1];
      FLOAT dx3, dy3;

#ifdef USE_FLOAT_AS_DECIMAL
        {
          // dy3 = tex1Dfetch(resized_image, base_index + 1) - tex1Dfetch(resized_image, base_index - 1) ;
          // dx3 = tex1Dfetch(resized_image, base_index + dims[0]) - tex1Dfetch(resized_image, base_index - dims[0]) ;
          dx3 = tex1Dfetch(resized_image, base_index + 1) - tex1Dfetch(resized_image, base_index - 1) ;
          dy3 = tex1Dfetch(resized_image, base_index + dims[1]) - tex1Dfetch(resized_image, base_index - dims[1]) ;
        }
#else
        {
          int2 arg1 = tex1Dfetch(resized_image_double, base_index + 1);
          int2 arg2 = tex1Dfetch(resized_image_double, base_index - 1) ;
          // dy3 = __hiloint2double(arg1.y, arg1.x) - __hiloint2double(arg2.y, arg2.x);
          dx3 = __hiloint2double(arg1.y, arg1.x) - __hiloint2double(arg2.y, arg2.x);

          // arg1 = tex1Dfetch(resized_image_double, base_index + dims[0]);
          // arg2 = tex1Dfetch(resized_image_double, base_index - dims[0]);
          // dx3 = __hiloint2double(arg1.y, arg1.x) - __hiloint2double(arg2.y, arg2.x);
          arg1 = tex1Dfetch(resized_image_double, base_index + dims[1]);
          arg2 = tex1Dfetch(resized_image_double, base_index - dims[1]);
          dy3 = __hiloint2double(arg1.y, arg1.x) - __hiloint2double(arg2.y, arg2.x);
        }
#endif
      FLOAT v3  = dx3*dx3 + dy3*dy3;

      /* pick channel with strongest gradient */
      // if (v2 > v) {
      //   v  = v2;
      //   dx = dx2;
      //   dy = dy2;
      // }
      dx = (v2 > v) ? dx2 : dx;
      dy = (v2 > v) ? dy2 : dy;
      v  = (v2 > v) ? v2 : v;
      // if (v3 > v) {
      //   v  = v3;
      //   dx = dx3;
      //   dy = dy3;
      // }
      dx = (v3 > v) ? dx3 : dx;
      dy = (v3 > v) ? dy3 : dy;
      v  = (v3 > v) ? v3 : v;


      /* snap to one of 18 orientations */
      FLOAT best_dot = 0;
      int   best_o   = 0;

#pragma unroll 9
      for (int o=0; o<9; o++) {
        FLOAT dot = Hcos[o]*dx + Hsin[o]*dy;

        if (dot > best_dot) {
          best_dot = dot;
          best_o   = o;
        }
        else if (-dot > best_dot) {
          best_dot = -dot;
          best_o   = o + 9;
        }
      }

      /*add to 4 histgrams aroud pixel using linear interpolation*/
      FLOAT xp  = ((FLOAT)x+0.5)/(FLOAT)sbin - 0.5;
      FLOAT yp  = ((FLOAT)y+0.5)/(FLOAT)sbin - 0.5;
      int   ixp = (int)floor((double)xp);
      int   iyp = (int)floor((double)yp);
      FLOAT vx0 = xp - ixp;
      FLOAT vy0 = yp - iyp;
      FLOAT vx1 = 1.0 - vx0;
      FLOAT vy1 = 1.0 - vy0;
      v = sqrtf((double)v);


#ifdef USE_FLOAT_AS_DECIMAL
      {
        /* dummy variable to reduce warp divergence */
        //        float retval = 0;
        if (ixp >= 0 && iyp >= 0)
          {
            atomicAdd_float((float *)(hist + ixp*blocks[0] + iyp + best_o*blocks[0]*blocks[1]), (float)vx1*vy1*v);
          }
        // retval = (ixp >= 0 && iyp >= 0) ?
        //   atomicAdd_float((float *)(hist + ixp*blocks[0] + iyp + best_o*blocks[0]*blocks[1]), (float)vx1*vy1*v) :
        //   0;

        if (ixp+1 < blocks[1] && iyp >= 0)
          {
            atomicAdd_float((float *)(hist + (ixp+1)*blocks[0] + iyp + best_o*blocks[0]*blocks[1]), (float)vx0*vy1*v);
          }
        // retval = (ixp+1 < blocks[1] && iyp >= 0) ?
        //   atomicAdd_float((float *)(hist + (ixp+1)*blocks[0] + iyp + best_o*blocks[0]*blocks[1]), (float)vx0*vy1*v) :
        //   0;

        if (ixp >= 0 && iyp+1 < blocks[0])
          {
            atomicAdd_float((float *)(hist + ixp*blocks[0] + (iyp+1) + best_o*blocks[0]*blocks[1]), (float)vx1*vy0*v);
          }
        // retval = (ixp >= 0 && iyp+1 < blocks[0]) ?
        //   atomicAdd_float((float *)(hist + ixp*blocks[0] + (iyp+1) + best_o*blocks[0]*blocks[1]), (float)vx1*vy0*v) :
        //   0;

        if (ixp+1 < blocks[1] && iyp+1 < blocks[0])
          {
            atomicAdd_float((float *)(hist + (ixp+1)*blocks[0] + (iyp+1) + best_o*blocks[0]*blocks[1]), (float)vx0*vy0*v);
          }
        // retval = (ixp+1 < blocks[1] && iyp+1 < blocks[0]) ?
        //   atomicAdd_float((float *)(hist + (ixp+1)*blocks[0] + (iyp+1) + best_o*blocks[0]*blocks[1]), (float)vx0*vy0*v) :
        //   0;
      }
#else  /* ifdef USE_FLOAT_AS_DECIMAL */
      {
        if (ixp >= 0 && iyp >= 0)
          {
            atomicAdd_double((double *)(hist + ixp*blocks[0] + iyp + best_o*blocks[0]*blocks[1]), (double)vx1*vy1*v);
          }

        if (ixp+1 < blocks[1] && iyp >= 0)
          {
            atomicAdd_double((double *)(hist + (ixp+1)*blocks[0] + iyp + best_o*blocks[0]*blocks[1]), (double)vx0*vy1*v);
          }

        if (ixp >= 0 && iyp+1 < blocks[0])
          {
            atomicAdd_double((double *)(hist + ixp*blocks[0] + (iyp+1) + best_o*blocks[0]*blocks[1]), (double)vx1*vy0*v);
          }

        if (ixp+1 < blocks[1] && iyp+1 < blocks[0])
          {
            atomicAdd_double((double *)(hist + (ixp+1)*blocks[0] + (iyp+1) + best_o*blocks[0]*blocks[1]), (double)vx0*vy0*v);
          }
      }
#endif  /* ifdef USE_FLOAT_AS_DECIMAL */

    }

      //   }
      // }



  /*************************************************************/
  /* original source of calc hist loop */
  // for (int x=1; x<visible[1]-1; x++) {
  //   for (int y=1; y<visible[0]-1; y++) {

  //     /* first color channel */
  //     FLOAT *s  = SRC + min_i(x, dims[1]-2)*dims[0] + min_i(y, dims[0]-2);
  //     FLOAT  dy = *(s+1) - *(s-1);
  //     FLOAT  dx = *(s+dims[0]) - *(s-dims[0]);
  //     FLOAT  v  = dx*dx + dy*dy;

  //     /* second color channel */
  //     s += dims[0]*dims[1];
  //     FLOAT dy2 = *(s+1) - *(s-1);
  //     FLOAT dx2 = *(s+dims[0]) - *(s-dims[0]);
  //     FLOAT v2  = dx2*dx2 + dy2*dy2;

  //     /* third color channel */
  //     s += dims[0]*dims[1];
  //     FLOAT dy3 = *(s+1) - *(s-1);
  //     FLOAT dx3 = *(s+dims[0]) - *(s-dims[0]);
  //     FLOAT v3  = dx3*dx3 + dy3*dy3;

  //     /* pick channel with strongest gradient */
  //     if (v2 > v) {
  //       v  = v2;
  //       dx = dx2;
  //       dy = dy2;
  //     }
  //     if (v3 > v) {
  //       v  = v3;
  //       dx = dx3;
  //       dy = dy3;
  //     }

  //     /* snap to one of 18 orientations */
  //     FLOAT best_dot = 0;
  //     int   best_o   = 0;
  //     for (int o=0; o<9; o++) {
  //       FLOAT dot = Hcos[o]*dx + Hsin[o]*dy;

  //       if (dot > best_dot) {
  //         best_dot = dot;
  //         best_o   = o;
  //       }
  //       else if (-dot > best_dot) {
  //         best_dot = -dot;
  //         best_o   = o + 9;
  //       }

  //     }

  //     /*add to 4 histgrams aroud pixel using linear interpolation*/
  //     FLOAT xp  = ((FLOAT)x+0.5)/(FLOAT)sbin - 0.5;
  //     FLOAT yp  = ((FLOAT)y+0.5)/(FLOAT)sbin - 0.5;
  //     int   ixp = (int)floor(xp);
  //     int   iyp = (int)floor(yp);
  //     FLOAT vx0 = xp - ixp;
  //     FLOAT vy0 = yp - iyp;
  //     FLOAT vx1 = 1.0 - vx0;
  //     FLOAT vy1 = 1.0 - vy0;
  //     v = sqrtf(v);

  //     if (ixp >= 0 && iyp >= 0) {
  //       *(hist + ixp*blocks[0] + iyp + best_o*blocks[0]*blocks[1]) += vx1*vy1*v;
  //     }
  //     if (ixp+1 < blocks[1] && iyp >= 0) {
  //       *(hist + (ixp+1)*blocks[0] + iyp + best_o*blocks[0]*blocks[1]) += vx0*vy1*v;
  //     }

  //     if (ixp >= 0 && iyp+1 < blocks[0]) {
  //       *(hist + ixp*blocks[0] + (iyp+1) + best_o*blocks[0]*blocks[1]) += vx1*vy0*v;
  //     }

  //     if (ixp+1 < blocks[1] && iyp+1 < blocks[0]) {
  //       *(hist + (ixp+1)*blocks[0] + (iyp+1) + best_o*blocks[0]*blocks[1]) += vx0*vy0*v;
  //     }
  //   }
  // }

  /*************************************************************/
  /*************************************************************/


}


extern "C"
__global__
void
calc_norm
(
 FLOAT *hist_top,
 FLOAT *norm_top,
 int blocks_0,
 int blocks_1,
 int level
 )
{
  /* index of each element of norm */
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  if (x<blocks_1 && y<blocks_0)
    {
      /* adjust pointer position */
      uint2 ptr_uint2 = tex1Dfetch(hist_ptr_incrementer, level);
      uintptr_t ptr_hist = (uintptr_t)hist_top + hiloint2uint64(ptr_uint2.x, ptr_uint2.y); // convert uint2 -> unsigned long long int


      ptr_uint2 = tex1Dfetch(norm_ptr_incrementer, level);
      uintptr_t ptr_norm = (uintptr_t)norm_top + hiloint2uint64(ptr_uint2.x, ptr_uint2.y); // convert uint2 -> unsigned long long int
      FLOAT *dst = (FLOAT *)(ptr_norm + (x*blocks_0 + y)*sizeof(FLOAT));


      FLOAT add_val = 0;
#pragma unroll 9
      for (int orient=0; orient<9; orient++)
        {
          FLOAT *src1 = (FLOAT *)(ptr_hist + (orient*blocks_0*blocks_1 + x*blocks_0 + y)*sizeof(FLOAT));
          FLOAT *src2 = (FLOAT *)(ptr_hist + ((orient+9)*blocks_0*blocks_1 + x*blocks_0 + y)*sizeof(FLOAT));
          add_val += (*src1 + *src2) * (*src1 + *src2);
        }
      *(dst) += add_val;
    }
  /*************************************************************/
  /* original source of compute_energy loop */

  //   /* compute energy in each block by summing over orientations */
  //   for (int o=0; o<9; o++) {
  //     FLOAT *src1 = hist + o*blocks[0]*blocks[1];
  //     FLOAT *src2 = hist + (o+9)*blocks[0]*blocks[1];
  //     FLOAT *dst  = norm;
  //     FLOAT *end  = norm + blocks[0]*blocks[1];

  //     while(dst < end) {
  //       *(dst++) += (*src1 + *src2) * (*src1 + *src2);
  //       src1++;
  //       src2++;
  //     }
  //   }


  /*************************************************************/
  /*************************************************************/

}

/* definition of constant */
#define EPS 0.0001

//return minimum number (FLOAT)
__device__
static inline FLOAT
min_2(FLOAT x)
{return (x <= 0.2 ? x :0.2);}


extern "C"
__global__
void
calc_feat
(
 FLOAT *hist_top,
 FLOAT *norm_top,
 FLOAT *feat_top,
 int out_0,
 int out_1,
 int blocks_0,
 int blocks_1,
 int level
 )
{
  /* index of each element of feat */
  int x = blockIdx.x * blockDim.x + threadIdx.x;
  int y = blockIdx.y * blockDim.y + threadIdx.y;

  /* adjust pointer position */
  uint2 ptr_uint2 = tex1Dfetch(hist_ptr_incrementer, level);
  uintptr_t ptr_hist = (uintptr_t)hist_top + hiloint2uint64(ptr_uint2.x, ptr_uint2.y); // convert uint2 -> unsigned long long int
  FLOAT *hist = (FLOAT *)ptr_hist;

  ptr_uint2 = tex1Dfetch(norm_ptr_incrementer, level);
  uintptr_t ptr_norm = (uintptr_t)norm_top + hiloint2uint64(ptr_uint2.x, ptr_uint2.y); // convert uint2 -> unsigned long long int
  FLOAT *norm = (FLOAT *)ptr_norm;

  ptr_uint2 = tex1Dfetch(feat_ptr_incrementer, level);
  uintptr_t ptr_feat = (uintptr_t)feat_top + hiloint2uint64(ptr_uint2.x, ptr_uint2.y); // convert uint2 -> unsigned long long int
  FLOAT *feat = (FLOAT *)ptr_feat;

  if (x<out_1 && y<out_0)
    {
      // for (int x=0; x<out[1]; x++) {
      //   for (int y=0; y<out[0]; y++) {
      //      FLOAT *dst = feat + x*out[0] + y;
      FLOAT *dst = feat + x*out_0 + y;
      FLOAT *src, *p, n1, n2, n3, n4;

      //      p = norm + (x+1)*blocks[0] + y+1;
      p = norm + (x+1)*blocks_0 + y+1;
      //      n1 = 1.0 / sqrtf(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + EPS);
      n1 = 1.0 / sqrtf(*p + *(p+1) + *(p+blocks_0) + *(p+blocks_0+1) + EPS);

      //      p = norm + (x+1)*blocks[0] + y;
      p = norm + (x+1)*blocks_0 + y;
      //      n2 = 1.0 / sqrtf(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + EPS);
      n2 = 1.0 / sqrtf(*p + *(p+1) + *(p+blocks_0) + *(p+blocks_0+1) + EPS);

      //      p = norm + x*blocks[0] + y+1;
      p = norm + x*blocks_0 + y+1;
      //      n3 = 1.0 / sqrtf(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + EPS);
      n3 = 1.0 / sqrtf(*p + *(p+1) + *(p+blocks_0) + *(p+blocks_0+1) + EPS);

      //      p = norm + x*blocks[0] + y;
      p = norm + x*blocks_0 + y;
      //      n4 = 1.0 / sqrtf(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + EPS);
      n4 = 1.0 / sqrtf(*p + *(p+1) + *(p+blocks_0) + *(p+blocks_0+1) + EPS);

      FLOAT t1 = 0;
      FLOAT t2 = 0;
      FLOAT t3 = 0;
      FLOAT t4 = 0;

      /* contrast-sensitive features */
      //      src = hist + (x+1)*blocks[0] + (y+1);
      src = hist + (x+1)*blocks_0 + (y+1);

#pragma unroll 18
      for (int o=0; o<18; o++) {
        FLOAT h1 = min_2(*src * n1);
        FLOAT h2 = min_2(*src * n2);
        FLOAT h3 = min_2(*src * n3);
        FLOAT h4 = min_2(*src * n4);

        *dst = 0.5 * (h1 + h2 + h3 + h4);

        t1 += h1;
        t2 += h2;
        t3 += h3;
        t4 += h4;

        //        dst += out[0]*out[1];
        dst += out_0*out_1;
        //        src += blocks[0]*blocks[1];
        src += blocks_0*blocks_1;
      }

      /* contrast-insensitive features */
      //      src = hist + (x+1)*blocks[0] + (y+1);
      src = hist + (x+1)*blocks_0 + (y+1);

#pragma unroll 9
      for (int o=0; o<9; o++) {
        //        FLOAT sum = *src + *(src + 9*blocks[0]*blocks[1]);
        FLOAT sum = *src + *(src + 9*blocks_0*blocks_1);
        FLOAT h1 = min_2(sum * n1);
        FLOAT h2 = min_2(sum * n2);
        FLOAT h3 = min_2(sum * n3);
        FLOAT h4 = min_2(sum * n4);

        *dst = 0.5 * (h1 + h2 + h3 + h4);

        //        dst += out[0]*out[1];
        dst += out_0*out_1;
        //        src += blocks[0]*blocks[1];
        src += blocks_0*blocks_1;
      }

      /* texture features */
      *dst = 0.2357 * t1;
      //      dst += out[0]*out[1];
      dst += out_0*out_1;

      *dst = 0.2357 * t2;
      //      dst += out[0]*out[1];
      dst += out_0*out_1;

      *dst = 0.2357 * t3;
      //      dst += out[0]*out[1];
      dst += out_0*out_1;

      *dst = 0.2357 * t4;
    }

  //    }
  //}


  /*************************************************************/
  /* original source of compute features loop */

  // /* compute featuers */
  // for (int x=0; x<out[1]; x++) {
  //   for (int y=0; y<out[0]; y++) {
  //     FLOAT *dst = feat + x*out[0] + y;
  //     FLOAT *src, *p, n1, n2, n3, n4;

  //     p = norm + (x+1)*blocks[0] + y+1;
  //     n1 = 1.0 / sqrtf(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);

  //     p = norm + (x+1)*blocks[0] + y;
  //     n2 = 1.0 / sqrtf(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);

  //     p = norm + x*blocks[0] + y+1;
  //     n3 = 1.0 / sqrtf(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);

  //     p = norm + x*blocks[0] + y;
  //     n4 = 1.0 / sqrtf(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);

  //     FLOAT t1 = 0;
  //     FLOAT t2 = 0;
  //     FLOAT t3 = 0;
  //     FLOAT t4 = 0;

  //     /* contrast-sensitive features */
  //     src = hist + (x+1)*blocks[0] + (y+1);
  //     for (int o=0; o<18; o++) {
  //       FLOAT h1 = min_2(*src * n1);
  //       FLOAT h2 = min_2(*src * n2);
  //       FLOAT h3 = min_2(*src * n3);
  //       FLOAT h4 = min_2(*src * n4);

  //       *dst = 0.5 * (h1 + h2 + h3 + h4);

  //       t1 += h1;
  //       t2 += h2;
  //       t3 += h3;
  //       t4 += h4;

  //       dst += out[0]*out[1];
  //       src += blocks[0]*blocks[1];
  //     }

  //     /* contrast-insensitive features */
  //     src = hist + (x+1)*blocks[0] + (y+1);
  //     for (int o=0; o<9; o++) {
  //       FLOAT sum = *src + *(src + 9*blocks[0]*blocks[1]);
  //       FLOAT h1 = min_2(sum * n1);
  //       FLOAT h2 = min_2(sum * n2);
  //       FLOAT h3 = min_2(sum * n3);
  //       FLOAT h4 = min_2(sum * n4);

  //       *dst = 0.5 * (h1 + h2 + h3 + h4);

  //       dst += out[0]*out[1];
  //       src += blocks[0]*blocks[1];
  //     }

  //     /* texture features */
  //     *dst = 0.2357 * t1;
  //     dst += out[0]*out[1];

  //     *dst = 0.2357 * t2;
  //     dst += out[0]*out[1];

  //     *dst = 0.2357 * t3;
  //     dst += out[0]*out[1];

  //     *dst = 0.2357 * t4;
  //   }
  // }

  /*************************************************************/
  /*************************************************************/

}

/* texture declaration for original image */
#ifdef USE_FLOAT_AS_DECIMAL
texture<float, cudaTextureType2DLayered, cudaReadModeElementType> org_image;
#else
texture<uint2, cudaTextureType2DLayered, cudaReadModeElementType> org_image;
#endif

#ifndef USE_FLOAT_AS_DECIMAL
#define NO_HARDWARE_SUPPORT
#endif

#ifdef NO_HARDWARE_SUPPORT
__device__
static inline
double getPixelVal(int x, int y, int width, int height, int channel)
{
  int access_x = (x < 0) ? 0 :
    (x < width) ? x : (width-1);

  int access_y = (y < 0) ? 0 :
    (y < height) ? y : (height-1);

  int2 retval = tex1Dfetch(org_image, channel*height*width + access_y*width + access_x);
  return __hiloint2double(retval.y, retval.x);
}
#endif

extern "C"
__global__
void
resize
(
 int src_height,
 int src_width,
 FLOAT *dst_top,
 int dst_height,
 int dst_width,
 FLOAT hfactor,
 FLOAT wfactor,
 int level
 )
{
  int dst_x   = blockIdx.x*blockDim.x + threadIdx.x;
  int dst_y   = blockIdx.y*blockDim.y + threadIdx.y;
  int channel = blockIdx.z;

  FLOAT *dst = dst_top + tex1Dfetch(image_idx_incrementer, level) + channel*dst_height*dst_width;
  // unsigned long long int dst_ptr = (unsigned long long int)dst_top +
  //   	  (unsigned long long int)(tex1Dfetch(image_idx_incrementer, level) + channel*dst_height*dst_width)*sizeof(FLOAT);
  // FLOAT *dst = (FLOAT *)dst_ptr;

  FLOAT src_x_decimal = wfactor * dst_x + 0.5f;
  FLOAT src_y_decimal = hfactor * dst_y + 0.5f;

#ifdef USE_FLOAT_AS_DECIMAL
  if (dst_x < dst_width && dst_y < dst_height)
    {
      dst[dst_y*dst_width + dst_x] = (FLOAT)tex2DLayered(org_image, src_x_decimal, src_y_decimal, channel);
    }
#else
  /* if "double" type is used to express decimal value, there is no hardware support */
  int src_x = (int)src_x_decimal;
  int src_y = (int)src_y_decimal;

  double color[4] = {
    getPixelVal(src_x, src_y, src_width, src_height, channel),
    getPixelVal(src_x+1, src_y, src_width, src_height, channel),
    getPixelVal(src_x, src_y+1, src_width, src_height, channel),
    getPixelVal(src_x+1, src_y+1, src_width, src_height, channel)
  };

  double new_element = (src_x + 1 - src_x_decimal)*(src_y + 1 - src_y_decimal)*color[0] +
              (src_x_decimal - src_x)*(src_y + 1 - src_y_decimal)*color[1] +
              (src_x + 1 - src_x_decimal)*(src_y_decimal - src_y)*color[2] +
              (src_x_decimal - src_x)*(src_y_decimal - src_y)*color[3];

  if (dst_x < dst_width && dst_y < dst_height)
    {
      dst[dst_y*dst_width + dst_x] = new_element;
    }

#endif
}
