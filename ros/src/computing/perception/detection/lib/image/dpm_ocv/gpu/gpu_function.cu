#include <stdio.h>
#include <float.h>
#include <math.h>
#include <cuda.h>
#include "gpu_matching.h"

// Copy from OpenCV
#define F_MAX FLT_MAX
#define F_MIN -FLT_MAX
#define NUM_SECTOR 9

texture<float, cudaTextureType1D, cudaReadModeElementType> texRef;
texture<float, cudaTextureType1D, cudaReadModeElementType> texMap;
texture<float, cudaTextureType1D, cudaReadModeElementType> texFi;

__device__
static inline float
atomicAdd_float(float * __restrict__ address, float val)
{
    return atomicAdd(address, val); // atomicAdd must be called from "__device__" function
}

__device__
int DistanceTransformOneDimensionalProblemX
(
        const int y,
        const int x,
        const float * __restrict__ score,
        const float a,
        const float b,
        DistTransWork * __restrict__ work,
        int indx
)
{
    int i, k;
    int diff;
    float pointIntersection;
    float tmp;
    const int size = x * y;

    k = 0;

    work[indx].v = 0;
    work[indx].z = (float)F_MIN;
    work[indx+1].z = (float)F_MAX;

    for (i = 1; i < x; i++)
    {
        tmp = ( score[i + size] - a * i + b * i * i );
        pointIntersection = ( tmp
                - ( score[work[indx + k].v + size] - a * work[indx + k].v + b * work[indx + k].v * work[indx + k].v ) )
        / (2 * b * (i - work[indx + k].v));
        while (pointIntersection <= work[indx + k].z)
        {
            k--;
            pointIntersection = ( tmp
                    - ( score[work[indx + k].v + size] - a * work[indx + k].v + b * work[indx + k].v * work[indx + k].v ) )
            / (2 * b * (i - work[indx + k].v));
        }
        // Addition parabola to the envelope
        k++;
        work[indx + k].v = i;
        work[indx + k].z = pointIntersection;
        work[indx + k + 1].z = (float)F_MAX;
    }

    // Computation values of generalized distance transform at all grid points
    k = 0;
    for (i = 0; i < x; i++)
    {
        while (work[indx + k + 1].z < i)
        {
            k++;
        }
        work[size + i].internalPoints = work[indx + k].v;
        diff = i - work[indx + k].v;
        work[size + i].internalDistTrans = a * diff + b * diff * diff + score[work[indx + k].v + size];
    }
    return 0;
}

__device__ int DistanceTransformOneDimensionalProblemY
(
        const int x,
        const int y,
        const int diffX,
        const float a,
        const float b,
        float * __restrict__ distanceTransform,
        int * __restrict__ points,
        DistTransWork * __restrict__ work,
        int indx
)
{
    int i, k;
    int diff;
    float pointIntersection;
    float tmp;
    k = 0;

    work[indx].v = 0;
    work[indx].z = (float)F_MIN;
    work[indx+1].z = (float)F_MAX;

    for (i = 1; i < y; i++)
    {
        tmp = ( work[x + i * diffX].internalDistTrans - a * i + b * i * i );
        pointIntersection = ( tmp
                - ( work[x + work[indx + k].v * diffX].internalDistTrans - a * work[indx + k].v + b * work[indx + k].v * work[indx + k].v ))
        / (2 * b * (i - work[indx + k].v));

        while (pointIntersection <= work[indx + k].z)
        {
            k--;
            pointIntersection = ( tmp
                    - ( work[x + work[indx + k].v * diffX].internalDistTrans - a * work[indx + k].v + b * work[indx + k].v * work[indx + k].v ))
            / (2 * b * (i - work[indx + k].v));

        }
        // Addition parabola to the envelope
        k++;
        work[indx + k].v = i;
        work[indx + k].z = pointIntersection;
        work[indx + k + 1].z = (float)F_MAX;
    }

    // Computation values of generalized distance transform at all grid points
    k = 0;
    for (i = 0; i < y; i++)
    {
        while (work[indx + k + 1].z < i)
        {
            k++;
        }
        points[x + i * diffX] = work[indx + k].v;
        diff = i - work[indx + k].v;
        distanceTransform[x + i * diffX] = a * diff + b * diff * diff + work[x + work[indx + k].v * diffX].internalDistTrans;
    }
    return 0;
}

extern "C"
{

__global__
void DistanceTransformTwoDimensionalProblemKernel
(
        float * __restrict__ score,
        const int x,
        const int y,
        const float coeff0,
        const float coeff1,
        const float coeff2,
        const float coeff3,
        DistTransWork * __restrict__ work,
        float * __restrict__ resalt,
        int * __restrict__ pointsX,
        int * __restrict__ pointsY
)
{

    int t = threadIdx.x;
    int i;
    int size = x * y;

    for (i = t; i < size; i+=TRANS_THREAD)
    {
        score[i] = -score[i];
    }

    __syncthreads();

    for (i = t; i < y; i+=TRANS_THREAD)
    {
        DistanceTransformOneDimensionalProblemX(i,x,
                score,
                coeff0, coeff2,
                work,
                i * ( x + 1 ));
    }

    __syncthreads();

    for (i = t; i < x; i+=TRANS_THREAD)
    {
        DistanceTransformOneDimensionalProblemY(i,y,x,
                coeff1, coeff3,
                resalt,
                pointsY,
                work,
                i * ( y + 1 ));
    }

    __syncthreads();

    for (i = t; i < size; i+=TRANS_THREAD)
    {
        pointsX[i] = work[pointsY[i] * (i % x) + (i / x)].internalPoints;
    }
}

__global__
void ConvolutionKernel
(
        const int idx,
        float * __restrict__ dev_score,
        const unsigned int * __restrict__ dev_filterIdxTbl,
        const ConvolutionParam prm
)
{
    
    __shared__ float cache[CONV_THREAD];
    
    unsigned int score_idx; //スコア番号
    unsigned int fi_idx;    //フィルタ距離
    unsigned int t = threadIdx.x;
    unsigned int b = gridDim.x;

    unsigned int mtable;

    score_idx = blockIdx.x; 

    mtable = (( score_idx / prm.scoreX ) * prm.mapX ) + (( score_idx % prm.scoreX ) * prm.numSectors );

    // 各ブロックでdev_score[score_idx]を計算
    for( score_idx = blockIdx.x; score_idx < prm.scoreSize; score_idx += b )
    {      
        // キャッシュの初期化
        cache[t] = 0.0f;

        // 各スレッドで対応した部分スコアを計算しキャッシュに格納
        for( fi_idx = t; fi_idx < prm.filterSize; fi_idx += CONV_THREAD)
        {
            cache[t] += tex1Dfetch(texMap,dev_filterIdxTbl[fi_idx] + mtable) * tex1Dfetch(texFi,fi_idx + idx);
        }
        //cache[t] = score;
        // キャッシュ内の部分スコアの総和をcache[0]へ畳み込む
        for( unsigned int i = CONV_THREAD >> 1; i > 0; i >>= 1)
        {
            __syncthreads();
            if( t < i )
            {
                cache[t] += cache[t + i];
            }
        }
        
        // スコアの代入
        if(t == 0)
        {
            dev_score[score_idx] = cache[0];
        }
    }
    
}

__global__
void BilinearKernelTex32F(
        float * __restrict__ out,
        const int widthIn,
        const int heightIn,
        const int widthOut,
        const int heightOut,
        const int channels,
        const int widthStepIn,
        const int widthStepOut
)
{
    const int x = blockDim.x * blockIdx.x + threadIdx.x;
    const int y = blockDim.y * blockIdx.y + threadIdx.y;
    const int c = blockIdx.z;

    if(x < widthOut && y < heightOut)
    {
        const float fx = ((float)widthIn / widthOut);
        const float fy = ((float)heightIn / heightOut);

        const float src_x = x * fx;
        const float src_y = y * fy;

        const int x1 = __float2int_rd(src_x);
        const int y1 = __float2int_rd(src_y);
        const int x2 = x1 + 1;
        const int y2 = y1 + 1;
        const int x2_read = min(x2, widthIn - 1);
        const int y2_read = min(y2, heightIn - 1);

        int width_step_out_u = widthStepOut / 4;

        float cell1 = (x2 - src_x) * (y2 - src_y);
        float cell2 = (src_x - x1) * (y2 - src_y);
        float cell3 = (x2 - src_x) * (src_y - y1);
        float cell4 = (src_x - x1) * (src_y - y1);

        out[y * width_step_out_u + x * channels + c] = (float)(
                cell1 * (float)tex1Dfetch(texRef, y1 * widthIn * channels + x1 * channels + c)
                + cell2 * (float)tex1Dfetch(texRef, y1 * widthIn * channels + x2_read * channels + c)
                + cell3 * (float)tex1Dfetch(texRef, y2_read * widthIn * channels + x1 * channels + c)
                + cell4 * (float)tex1Dfetch(texRef, y2_read * widthIn * channels + x2_read * channels + c));
    }
}

__global__
void calculateHistogram(
        const float * __restrict__ in,
        float * __restrict__ r,
        int * __restrict__ alfa,
        const int widthIn,
        const int heightIn,
        const int widthStep,
        const int channels
)
{
    const int i = blockDim.x * blockIdx.x + threadIdx.x;
    const int j = blockDim.y * blockIdx.y + threadIdx.y;

    int height, width, numChannels;
    int kk, c;

    float magnitude, x, y, tx, ty;

    const float boundary_x[NUM_SECTOR + 1] =
    {   1.000000, 0.939693, 0.766044, 0.500000, 0.173648, -0.173648, -0.500000, -0.766045, -0.939693, -1.000000};
    const float boundary_y[NUM_SECTOR + 1] =
    {   0.000000, 0.342020, 0.642788, 0.866025, 0.984808, 0.984808, 0.866025, 0.642787, 0.342020, 0.000000};
    float max, dotProd;
    int maxi;

    height = heightIn;
    width = widthIn;

    numChannels = channels;

    int width_step_u = widthStep / 4;

    if(j >= 1 && j < height - 1)
    {
        if(i >= 1 && i < width - 1)
        {

            c = 0;
            x = (-in[(j * width + (i - 1)) * numChannels + c]) + in[(j * width + (i + 1)) * numChannels + c];
            y = (-in[((j - 1) * width + i) * numChannels + c]) + in[((j + 1) * width + i) * numChannels + c];

            r[j * width + i] = sqrtf(x * x + y * y);
            for(int ch = 1; ch < numChannels; ch++)
            {
                tx = (-in[j * width_step_u + (i - 1) * numChannels + c]) + in[j * width_step_u + (i + 1) * numChannels + c];
                ty = (-in[(j - 1) * width_step_u + i * numChannels + c]) + in[(j + 1) * width_step_u + i * numChannels + c];
                magnitude = sqrtf(tx * tx + ty * ty);
                if(magnitude > r[j * width + i])
                {
                    r[j * width + i] = magnitude;
                    c = ch;
                    x = tx;
                    y = ty;
                }
            }

            max = boundary_x[0] * x + boundary_y[0] * y;
            maxi = 0;

            for (kk = 0; kk < NUM_SECTOR; kk++)
            {
                dotProd = boundary_x[kk] * x + boundary_y[kk] * y;
                if (dotProd > max)
                {
                    max = dotProd;
                    maxi = kk;
                }
                else
                {
                    if (-dotProd > max)
                    {
                        max = -dotProd;
                        maxi = kk + NUM_SECTOR;
                    }
                }
            }
            alfa[j * width * 2 + i * 2 ] = maxi % NUM_SECTOR;
            alfa[j * width * 2 + i * 2 + 1] = maxi;
        }
    }
}

__global__
void getFeatureMaps(
        const float * __restrict__ r,
        const int * __restrict__ alfa,
        const int * __restrict__ nearest,
        const float * __restrict__ w,
        float * __restrict__ map,
        const int widthMap,
        const int heightMap,
        const int k,
        const int numFeatures
)
{
    const int j = blockDim.x * blockIdx.x + threadIdx.x;
    const int i = blockDim.y * blockIdx.y + threadIdx.y;
    const int ii = blockIdx.z / k;
    const int jj = blockIdx.z % k;

    int sizeX, sizeY;
    int p, px, stringSize;
    int height, width;
    int d;

    height = heightMap;
    width = widthMap;

    sizeX = width / k;
    sizeY = height / k;
    px = 3 * NUM_SECTOR;
    p = px;
    stringSize = sizeX * p;

    if(i < sizeY)
    {
        if(j < sizeX)
        {
            if(ii < k)
            {
                if(jj < k)
                {
                    if ((i * k + ii > 0) &&
                            (i * k + ii < height - 1) &&
                            (j * k + jj > 0) &&
                            (j * k + jj < width - 1))
                    {
                        d = (k * i + ii) * width + (j * k + jj);
                        atomicAdd_float(
                                (float *)(map + (i * stringSize + j * numFeatures + alfa[d * 2 ])),
                                (float)(r[d] * w[ii * 2] * w[jj * 2])
                        );
                        atomicAdd_float(
                                (float *)(map + (i * stringSize + j * numFeatures + alfa[d * 2 + 1] + NUM_SECTOR)),
                                (float)(r[d] * w[ii * 2] * w[jj * 2])
                        );
                        if ((i + nearest[ii] >= 0) &&
                                (i + nearest[ii] <= sizeY - 1))
                        {
                            atomicAdd_float(
                                    (float *)(map + ((i + nearest[ii]) * stringSize + j * numFeatures + alfa[d * 2 ])),
                                    (float)(r[d] * w[ii * 2 + 1] * w[jj * 2])
                            );
                            atomicAdd_float(
                                    (float *)(map + ((i + nearest[ii]) * stringSize + j * numFeatures + alfa[d * 2 + 1] + NUM_SECTOR)),
                                    (float)(r[d] * w[ii * 2 + 1] * w[jj * 2])
                            );
                        }
                        if ((j + nearest[jj] >= 0) &&
                                (j + nearest[jj] <= sizeX - 1))
                        {
                            atomicAdd_float(
                                    (float *)(map + (i * stringSize + (j + nearest[jj]) * numFeatures + alfa[d * 2 ])),
                                    (float)(r[d] * w[ii * 2] * w[jj * 2 + 1])
                            );
                            atomicAdd_float(
                                    (float *)(map + (i * stringSize + (j + nearest[jj]) * numFeatures + alfa[d * 2 + 1] + NUM_SECTOR)),
                                    (float)(r[d] * w[ii * 2] * w[jj * 2 + 1])
                            );
                        }
                        if ((i + nearest[ii] >= 0) &&
                                (i + nearest[ii] <= sizeY - 1) &&
                                (j + nearest[jj] >= 0) &&
                                (j + nearest[jj] <= sizeX - 1))
                        {
                            atomicAdd_float(
                                    (float *)(map + ((i + nearest[ii]) * stringSize + (j + nearest[jj]) * numFeatures + alfa[d * 2 ])),
                                    (float)(r[d] * w[ii * 2 + 1] * w[jj * 2 + 1])
                            );
                            atomicAdd_float(
                                    (float *)(map + ((i + nearest[ii]) * stringSize + (j + nearest[jj]) * numFeatures + alfa[d * 2 + 1] + NUM_SECTOR)),
                                    (float)(r[d] * w[ii * 2 + 1] * w[jj * 2 + 1])
                            );
                        }
                    }
                }
            }
        }
    }
}

__global__
void calculateNorm(
        const float * __restrict__ map,
        float * __restrict__ partOfNorm,
        const int sizeX,
        const int sizeY,
        const int numFeatures
)
{
    const int x = blockDim.x * blockIdx.x + threadIdx.x;
    const int y = blockDim.y * blockIdx.y + threadIdx.y;

    if(y <= sizeY)
    {
        if(x <= sizeX)
        {
            int i, j, p, pos;
            float valOfNorm = 0.0f;

            p = NUM_SECTOR;

            i = y * sizeX + x;
            pos = i * numFeatures;
            for(j = 0; j < p; j++)
            {
                valOfNorm += map[pos + j] * map[pos + j];
            }
            partOfNorm[i] = valOfNorm;
        }
    }
}

__global__
void normalizeAndTruncate(
        const float * __restrict__ map,
        const float * __restrict__ partOfNorm,
        float * __restrict__ newData,
        const int mapSizeX,
        const int mapSizeY,
        const float alfa
)
{
    int x = blockDim.x * blockIdx.x + threadIdx.x;
    int y = blockDim.y * blockIdx.y + threadIdx.y;
    int ii = blockIdx.z;

    int sizeX, sizeY, p, pp, xp, pos1, pos2;
    float valOfNorm1, valOfNorm2, valOfNorm3, valOfNorm4;

    p = NUM_SECTOR;
    xp = NUM_SECTOR * 3;
    pp = NUM_SECTOR * 12;

    sizeX = mapSizeX - 2;
    sizeY = mapSizeY - 2;

    if(y >= 1 && y <= sizeY)
    {
        if(x >= 1 && x <= sizeX)
        {
            valOfNorm1 = sqrtf(
                    partOfNorm[(y )*(sizeX + 2) + (x )] +
                    partOfNorm[(y )*(sizeX + 2) + (x + 1)] +
                    partOfNorm[(y + 1)*(sizeX + 2) + (x )] +
                    partOfNorm[(y + 1)*(sizeX + 2) + (x + 1)]) + FLT_EPSILON;
            valOfNorm2 = sqrtf(
                    partOfNorm[(y )*(sizeX + 2) + (x )] +
                    partOfNorm[(y )*(sizeX + 2) + (x + 1)] +
                    partOfNorm[(y - 1)*(sizeX + 2) + (x )] +
                    partOfNorm[(y - 1)*(sizeX + 2) + (x + 1)]) + FLT_EPSILON;
            valOfNorm3 = sqrtf(
                    partOfNorm[(y )*(sizeX + 2) + (x )] +
                    partOfNorm[(y )*(sizeX + 2) + (x - 1)] +
                    partOfNorm[(y + 1)*(sizeX + 2) + (x )] +
                    partOfNorm[(y + 1)*(sizeX + 2) + (x - 1)]) + FLT_EPSILON;
            valOfNorm4 = sqrtf(
                    partOfNorm[(y )*(sizeX + 2) + (x )] +
                    partOfNorm[(y )*(sizeX + 2) + (x - 1)] +
                    partOfNorm[(y - 1)*(sizeX + 2) + (x )] +
                    partOfNorm[(y - 1)*(sizeX + 2) + (x - 1)]) + FLT_EPSILON;
            pos1 = (y ) * (sizeX + 2) * xp + (x ) * xp;
            pos2 = (y-1) * (sizeX ) * pp + (x-1) * pp;

            if(ii < p)
            {
                newData[pos2 + ii ] = fminf(map[pos1 + ii] / valOfNorm1, alfa);
                newData[pos2 + ii + p ] = fminf(map[pos1 + ii] / valOfNorm2, alfa);
                newData[pos2 + ii + p * 2] = fminf(map[pos1 + ii] / valOfNorm3, alfa);
                newData[pos2 + ii + p * 3] = fminf(map[pos1 + ii] / valOfNorm4, alfa);
            }

            newData[pos2 + ii + p * 4 ] = fminf(map[pos1 + ii + p] / valOfNorm1, alfa);
            newData[pos2 + ii + p * 6 ] = fminf(map[pos1 + ii + p] / valOfNorm2, alfa);
            newData[pos2 + ii + p * 8 ] = fminf(map[pos1 + ii + p] / valOfNorm3, alfa);
            newData[pos2 + ii + p * 10] = fminf(map[pos1 + ii + p] / valOfNorm4, alfa);
        }
    }
}

__global__
void PCAFeatureMapsAddNullableBorder(
        const float * __restrict__ map,
        float * __restrict__ newData,
        const int borderMapSizeX,
        const int borderMapSizeY,
        const int numFeatures,
        const int bx,
        const int by
)
{
    const int j = blockDim.x * blockIdx.x + threadIdx.x;
    const int i = blockDim.y * blockIdx.y + threadIdx.y;

    int ii, jj, k;
    int sizeX, sizeY, p, pp, xp, yp, pos1, pos2;
    float val;
    float nx, ny;

    sizeX = borderMapSizeX;
    sizeY = borderMapSizeY;
    p = numFeatures;
    pp = NUM_SECTOR * 3 + 4;
    yp = 4;
    xp = NUM_SECTOR;

    nx = 1.0f / sqrtf((float)(xp * 2));
    ny = 1.0f / sqrtf((float)(yp ));

    if(i < sizeY)
    {
        if(j < sizeX)
        {
            pos1 = ((i)*sizeX + j)*p;
            pos2 = ((i + by)*(sizeX + 2 * bx) + j + bx)*pp;
            k = 0;
            for(jj = 0; jj < xp * 2; jj++)
            {
                newData[pos2 + k] = ( map[pos1 + yp * xp + jj]
                        + map[pos1 + (yp + 2) * xp + jj]
                        + map[pos1 + (yp + 4) * xp + jj]
                        + map[pos1 + (yp + 6) * xp + jj] ) * ny;

                k++;
            }
            for(jj = 0; jj < xp; jj++)
            {
                newData[pos2 + k] = ( map[pos1 + jj]
                        + map[pos1 + xp + jj]
                        + map[pos1 + 2 * xp + jj]
                        + map[pos1 + 3 * xp + jj]) * ny;
                k++;
            }
            for(ii = 0; ii < yp; ii++)
            {
                val = 0;
                for(jj = 0; jj < 2 * xp; jj++)
                {
                    val += map[pos1 + yp * xp + ii * xp * 2 + jj];
                }
                newData[pos2 + k] = val * nx;
                k++;
            }
        }
    }

}

} // extern "C"
