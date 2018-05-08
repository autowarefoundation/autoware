#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include "for_use_gpu.h"
#include "drvapi_error_string.h"
#include "cuda_check.h"

#include "gpu_matching.h"
#include "matching_gpu.hpp"

// OpenCV non public functions
extern int allocFeatureMapObject(CvLSVMFeatureMap **obj, const int sizeX, const int sizeY,
                                const int p);

#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

/*
 // Calculate the size of scores.
 //
 // API
 // int calculationScoreSize(const CvLSVMFilterObject *filter,
 //        const CvLSVMFeatureMap *map, int *diffX, int *diffY);
 // INPUT
 // filter             - filter object
 // map                - feature map
 // OUTPUT
 // diffX              - diff between filter sizeX and map sizeX
 // diffY              - diff between filter sizeY and map sizeY
 // RESULT
 // Error status
 */
static int calculationScoreSize(const CvLSVMFilterObject *filter,
        const CvLSVMFeatureMap *map, int *diffX, int *diffY)
{
    if (map->sizeX < filter->sizeX || map->sizeY < filter->sizeY)
    {
        *diffX = 0;
        *diffY = 0;
        return LATENT_SVM_FAILED_SUPERPOSITION;
    }

    *diffX = map->sizeX - filter->sizeX + 1;
    *diffY = map->sizeY - filter->sizeY + 1;
    return LATENT_SVM_OK;
}

/*
 // Function for convolution computation in GPU
 //
 // API
 // int convolutionGPU(const CvLSVMFilterObject *filter,
 //        const CvLSVMFeatureMap *map, CUdeviceptr *dev_filter,
 //        CUdeviceptr *dev_map, CUdeviceptr *dev_filterIdxTbl,
 //        CUdeviceptr *dev_score, int filterIdx, CUstream stream);
 // INPUT
 // filter            - filter object
 // map               - feature map
 // dev_filter        - device filter object
 // dev_map           - device feature map
 // dev_filterIdxTbl  - device filter index table
 // OUTPUT
 // dev_score         - device score
 // RESULT
 // Error status
 */
static int convolutionGPU(const CvLSVMFilterObject *filter,
        const CvLSVMFeatureMap *map, CUdeviceptr *dev_filter,
        CUdeviceptr *dev_map, CUdeviceptr *dev_filterIdxTbl,
        CUdeviceptr *dev_score, int filterIdx, CUstream stream)
{
    int diffX, diffY, res, i;

    res = calculationScoreSize(filter, map, &diffX, &diffY);

    if (res == LATENT_SVM_OK)
    {
        ConvolutionParam prm;

        CUresult res = cuMemAlloc(dev_score, (diffX * diffY) * sizeof(float));
        CUDA_CHECK(res, "cuMemAlloc(dev_score): %zd bytes", (diffX * diffY) * sizeof(float));

        prm.numSectors = map->numFeatures;
        prm.mapX = map->sizeX * prm.numSectors;
        prm.mapY = map->sizeY;
        prm.filterX = filter->sizeX * prm.numSectors;
        prm.filterY = filter->sizeY;
        prm.scoreX = map->sizeX - filter->sizeX + 1;
        prm.scoreY = map->sizeY - filter->sizeY + 1;
        prm.mapSize = prm.mapX * prm.mapY;
        prm.filterSize = prm.filterX * prm.filterY;
        prm.scoreSize = prm.scoreX * prm.scoreY;

        // compute indexes to access filter elements.
        unsigned int filterIdxTbl[prm.filterSize];
        for (i = 0; i < prm.filterSize; i++)
        {
            filterIdxTbl[i] = (i / (prm.filterX)) * prm.mapX
                    + (i % prm.filterX);
        }

        res = cuMemAlloc(dev_filterIdxTbl, prm.filterSize * sizeof(float));
        CUDA_CHECK(res, "cuMemAlloc(dev_filterIdxTbl) %zd bytes", prm.filterSize * sizeof(float));

        res = cuMemcpyHtoDAsync(*dev_filterIdxTbl, filterIdxTbl,
                    prm.filterSize * sizeof(float), stream);
        CUDA_CHECK(res, "cuMemcpyHtoDAsync(*dev_filterIdxTbl, filterIdxTbl) %zd bytes",
                   prm.filterSize * sizeof(float));

        void *kernel_arg[] =
        { (void *) &filterIdx, (void *) dev_score, (void *) dev_filterIdxTbl,
                (void *) &prm };

        res = cuLaunchKernel(ConvolutionKernel_func[0], prm.scoreSize, 1, 1,
                     CONV_THREAD, 1, 1, CONV_THREAD * sizeof(float), stream,
                     kernel_arg, NULL);
        CUDA_CHECK(res, "cuLaunchKernel(ConvolutionKernel)");
    }

    return res;
}

/*
 // Decision of two dimensional problem generalized distance transform
 // on the regular grid at all points in GPU
 //
 // API
 // int DistanceTransformTwoDimensionalProblemGPU(const CvLSVMFilterObject *filter,
 //       const CvLSVMFeatureMap *map, CUdeviceptr *dev_score,
 //       CUdeviceptr *dev_distTransWork, CUdeviceptr *dev_distTransScore,
 //       CUdeviceptr *dev_x, CUdeviceptr *dev_y, int diffX, int diffY,
 //       CUstream stream);
 // INPUT
 // filter             - filter object
 // map                - feature map
 // dev_score          - device score
 // dev_distTransWork  - device distance transform work space
 // diffX              - diff between filter sizeX and map sizeX
 // diffY              - diff between filter sizeY and map sizeY
 // stream             - CUDA stream
 // OUTPUT
 // dev_distTransScore - device distance transform score
 // dev_x              - device disposition x
 // dev_y              - device disposition y
 // RESULT
 // Error status
 */
static int DistanceTransformTwoDimensionalProblemGPU(const CvLSVMFilterObject *filter,
        const CvLSVMFeatureMap *map, CUdeviceptr *dev_score,
        CUdeviceptr *dev_distTransWork, CUdeviceptr *dev_distTransScore,
        CUdeviceptr *dev_x, CUdeviceptr *dev_y, int diffX, int diffY,
        CUstream stream)
{

    void *kernel_arg[] =
    { (void *) dev_score, (void *) &diffX, (void *) &diffY,
            (void *) &filter->fineFunction[0],
            (void *) &filter->fineFunction[1],
            (void *) &filter->fineFunction[2],
            (void *) &filter->fineFunction[3], (void *) dev_distTransWork,
            (void *) dev_distTransScore, (void *) dev_x, (void *) dev_y };

    CUresult res = cuLaunchKernel(DistanceTransformTwoDimensionalProblem_func[0], 1, 1, 1,
                       TRANS_THREAD, 1, 1, 0, stream, kernel_arg, NULL);
    CUDA_CHECK(res, "cuLaunchKernel(DistanceTransformTwoDimensionalProblem)");

    return LATENT_SVM_OK;
}

/*
 // Calculate scores from the scores of each filter
 //
 // API
 // int sumScore(const CvLSVMFilterObject **all_F, const int n,
 //        const CvLSVMFeaturePyramid *H, const int level, const float b,
 //        const float scoreThreshold, const CvLSVMFilterDisposition disposition[],
 //        CUdeviceptr dev_score[], CvLSVMFeatureMap *map, float **score,
 //        CvPoint **points, int *kPoints, CvPoint ***partsDisplacement);
 // INPUT
 // all_F              - the set of filters
 //                      (the first element is root filter,the other - part filters)
 // n                  - the number of part filters
 // H                  - feature pyramid
 // level              - feature pyramid level for computation maximum score
 // b                  - linear term of the score function
 // scoreThreshold     - score threshold
 // disposition        - filter disposition
 // dev_score          - device score
 // map                - feature map
 // OUTPUT
 // score              - the maximum of the score function at the level
 // points             - the set of root filter positions (in the block space)
 // kpoints            - number of root filter positions
 // partsDisplacement  - displacement of part filters (in the block space)
 // RESULT
 // Error status
 */
static int sumScore(const CvLSVMFilterObject **all_F, const int n,
        const CvLSVMFeaturePyramid *H, const int level, const float b,
        const float scoreThreshold, const CvLSVMFilterDisposition disposition[],
        CUdeviceptr dev_score[], CvLSVMFeatureMap *map, float **score,
        CvPoint **points, int *kPoints, CvPoint ***partsDisplacement)
{
    int i, j, k;
    int index, last;
    float sumScorePartDisposition;
    int diffX, diffY, res;

    res = calculationScoreSize(all_F[0], H->pyramid[level], &diffX, &diffY);

    if (res != LATENT_SVM_OK)
    {
        return res;
    }

    sumScorePartDisposition = 0.0;

    float root_score[diffX * diffY];
    CUresult cu_res = cuMemcpyDtoH(root_score, dev_score[0], diffX * diffY * sizeof(float));
    CUDA_CHECK(cu_res, "cuMemcpyDtoH(root_score, dev_score[0]): %zd bytes", diffX * diffY * sizeof(float));

    cu_res = cuMemFree(dev_score[0]);
    CUDA_CHECK(cu_res, "cuMemFree(dev_score[0]");

    // The scores of hypothesis is given by the scores of each filter at
    // their respective locations minus deformation cost plus the bias.
    (*kPoints) = 0;
    for (i = 0; i < diffY; i++)
    {
        for (j = 0; j < diffX; j++)
        {
            sumScorePartDisposition = 0.0;
            for (k = 1; k <= n; k++)
            {
                if ((2 * i + all_F[k]->V.y < map->sizeY - all_F[k]->sizeY + 1)
                        && (2 * j + all_F[k]->V.x
                                < map->sizeX - all_F[k]->sizeX + 1))
                {
                    index = (2 * i + all_F[k]->V.y)
                            * (map->sizeX - all_F[k]->sizeX + 1)
                            + (2 * j + all_F[k]->V.x);
                    sumScorePartDisposition += disposition[k - 1].score[index];
                }
            }
            root_score[i * diffX + j] = root_score[i * diffX + j]
                    - sumScorePartDisposition + b;
            if (root_score[i * diffX + j] > scoreThreshold)
            {
                (*kPoints)++;
            }
        }
    }

    // Store part displacements that its scores are higher than the threshold
    (*points) = (CvPoint *) malloc(sizeof(CvPoint) * (*kPoints));
    (*partsDisplacement) = (CvPoint **) malloc(sizeof(CvPoint *) * (*kPoints));
    for (i = 0; i < (*kPoints); i++)
    {
        (*partsDisplacement)[i] = (CvPoint *) malloc(sizeof(CvPoint) * n);
    }
    (*score) = (float *) malloc(sizeof(float) * (*kPoints));
    last = 0;
    for (i = 0; i < diffY; i++)
    {
        for (j = 0; j < diffX; j++)
        {
            if (root_score[i * diffX + j] > scoreThreshold)
            {
                (*score)[last] = root_score[i * diffX + j];
                (*points)[last].y = i;
                (*points)[last].x = j;
                for (k = 1; k <= n; k++)
                {
                    if ((2 * i + all_F[k]->V.y
                            < map->sizeY - all_F[k]->sizeY + 1)
                            && (2 * j + all_F[k]->V.x
                                    < map->sizeX - all_F[k]->sizeX + 1))
                    {
                        index = (2 * i + all_F[k]->V.y)
                                * (map->sizeX - all_F[k]->sizeX + 1)
                                + (2 * j + all_F[k]->V.x);
                        (*partsDisplacement)[last][k - 1].x =
                                disposition[k - 1].x[index];
                        (*partsDisplacement)[last][k - 1].y =
                                disposition[k - 1].y[index];
                    }
                }
                last++;
            }
        }
    }
    return LATENT_SVM_OK;
}

/*
 // Computation border size for feature map
 //
 // API
 // int computeBorderSize(int maxXBorder, int maxYBorder, int *bx, int *by);
 // INPUT
 // maxXBorder        - the largest root filter size (X-direction)
 // maxYBorder        - the largest root filter size (Y-direction)
 // OUTPUT
 // bx                - border size (X-direction)
 // by                - border size (Y-direction)
 // RESULT
 // Error status
 */
int computeBorderSize(int maxXBorder, int maxYBorder, int *bx, int *by)
{
    *bx = (int) ceilf(((float) maxXBorder) / 2.0f + 1.0f);
    *by = (int) ceilf(((float) maxYBorder) / 2.0f + 1.0f);
    return LATENT_SVM_OK;
}

/*
 //
 // API
 // CvLSVMFeatureMap* featureMapBorderPartFilter(CvLSVMFeatureMap *map,
 //     int maxXBorder, int maxYBorder);
 // INPUT
 // map               - feature map
 // maxXBorder        - the largest root filter size (X-direction)
 // maxYBorder        - the largest root filter size (Y-direction)
 // OUTPUT
 // RESULT
 // new_map           - new map
 */
CvLSVMFeatureMap* featureMapBorderPartFilter(CvLSVMFeatureMap *map,
        int maxXBorder, int maxYBorder)
{
    int bx, by;
    int sizeX, sizeY, i, j, k;
    CvLSVMFeatureMap *new_map;

    computeBorderSize(maxXBorder, maxYBorder, &bx, &by);
    sizeX = map->sizeX + 2 * bx;
    sizeY = map->sizeY + 2 * by;
    allocFeatureMapObject(&new_map, sizeX, sizeY, map->numFeatures);
    for (i = 0; i < sizeX * sizeY * map->numFeatures; i++)
    {
        new_map->map[i] = 0.0f;
    }
    for (i = by; i < map->sizeY + by; i++)
    {
        for (j = bx; j < map->sizeX + bx; j++)
        {
            for (k = 0; k < map->numFeatures; k++)
            {
                new_map->map[(i * sizeX + j) * map->numFeatures + k] =
                        map->map[((i - by) * map->sizeX + j - bx)
                                * map->numFeatures + k];
            }
        }
    }
    return new_map;
}

/*
 //
 // API
 // void dispositionMalloc(const CvLSVMFilterObject *filter,
 //        const CvLSVMFeatureMap *map, CvLSVMFilterDisposition *disposition,
 //        int *maxSize);
 // INPUT
 // filter             - filter object
 // map                - feature map
 // OUTPUT
 // disposition        - filter disposition
 // maxSize            -
 // RESULT
 // Error status
 */
static void dispositionMalloc(const CvLSVMFilterObject *filter,
        const CvLSVMFeatureMap *map, CvLSVMFilterDisposition *disposition,
        int *maxSize)
{
    int diffX, diffY, res;
    res = calculationScoreSize(filter, map, &diffX, &diffY);
    if (res == LATENT_SVM_OK)
    {
        disposition->size = diffX * diffY;
        if (*maxSize < disposition->size)
        {
            *maxSize = disposition->size;
        }
        disposition->score = (float *) malloc(
                sizeof(float) * disposition->size);
        disposition->x = (int *) malloc(sizeof(int) * disposition->size);
        disposition->y = (int *) malloc(sizeof(int) * disposition->size);
    }
}

/*
 //
 // API
 // void dispositionCpyDtoH(CUdeviceptr *dev_distTransScore, CUdeviceptr *dev_x,
 //        CUdeviceptr *dev_y, CvLSVMFilterDisposition *disposition,
 //        CvLSVMFilterDisposition *tmp_disposition, CUstream *stream);
 // INPUT
 // dev_distTransScore - device distance transform score
 // dev_x              - device disposition x
 // dev_y              - device disposition y
 // tmp_disposition    - MemAllocHost
 // stream             - CUDA stream
 // OUTPUT
 // disposition        - filter disposition
 // RESULT
 // none
 */
static void dispositionCpyDtoH(CUdeviceptr *dev_distTransScore, CUdeviceptr *dev_x,
        CUdeviceptr *dev_y, CvLSVMFilterDisposition *disposition,
        CvLSVMFilterDisposition *tmp_disposition, CUstream *stream)
{
    // Copy data from the device to the host by using the page-locked memory
    // with fast transfer rate, and copy from the page-locked memory to
    // normal memory in the host.
    CUresult res = cuMemcpyDtoHAsync(tmp_disposition->score, *dev_distTransScore,
                      disposition->size * sizeof(float), stream[0]);
    CUDA_CHECK(res, "cuMemcpyDtoHAsync(tmp_disposition->score, *dev_distTransScore): %zd bytes",
               disposition->size * sizeof(float));
    res = cuMemcpyDtoHAsync(tmp_disposition->x, *dev_x,
                 disposition->size * sizeof(int), stream[1]);
    CUDA_CHECK(res, "cuMemcpyDtoHAsync(tmp_disposition->x, *dev_x): %zd bytes",
               disposition->size * sizeof(int));
    res = cuMemcpyDtoHAsync(tmp_disposition->y, *dev_y,
            disposition->size * sizeof(int), stream[2]);
    CUDA_CHECK(res, "cuMemcpyDtoHAsync(tmp_disposition->y, *dev_y): %zd bytes",
               disposition->size * sizeof(int));

    res = cuStreamSynchronize(stream[0]);
    CUDA_CHECK(res, "cuStreamSynchronize(stream[0])");

    memcpy(disposition->score, tmp_disposition->score,
            disposition->size * sizeof(float));
    res = cuStreamSynchronize(stream[1]);
    CUDA_CHECK(res, "cuStreamSynchronize(stream[1])");

    memcpy(disposition->x, tmp_disposition->x, disposition->size * sizeof(int));
    res = cuStreamSynchronize(stream[2]);
    CUDA_CHECK(res, "cuStreamSynchronize(stream[2])");

    memcpy(disposition->y, tmp_disposition->y, disposition->size * sizeof(int));
}

/*
 //
 // API
 // void dispositionFree(CvLSVMFilterDisposition *disposition);
 // INPUT
 // disposition - filter disposition
 // OUTPUT
 // none
 // RESULT
 // none
 */
static void dispositionFree(CvLSVMFilterDisposition *disposition)
{
    free(disposition->score);
    free(disposition->x);
    free(disposition->y);
}

/*
 // Calculate root and part scores.
 //
 // API
 // void calculationScore(int numLevels, int n, const CvLSVMFeaturePyramid* H,
 //        const CvLSVMFilterObject** all_F, int *res, CvLSVMFeatureMap* map[],
 //        CUdeviceptr** dev_score);
 // INPUT
 // b                  - linear term of the score function
 // numLevels          -
 // n                  - the number of part filters
 // H                  - feature pyramid
 // all_F              - the set of filters
 //                      (the first element is root filter,the other - part filters)
 // res                - response
 // map                - feature map
 // OUTPUT
 // dev_score          - device score
 // RESULT
 // none
 */
static void calculationScore(int numLevels, int n, const CvLSVMFeaturePyramid* H,
        const CvLSVMFilterObject** all_F, int *res, CvLSVMFeatureMap* map[],
        CUdeviceptr** dev_score)
{
    int i, j, k;
    CUdeviceptr dev_filterIdxTbl[numLevels][n + 1];
    CUdeviceptr dev_filter;
    CUdeviceptr dev_map[2][numLevels];
    CUstream convStreams[numLevels + 3];
    int size[2][numLevels];
    int filterSize[n + 1];
    int filterIdx[n + 1];
    int totalFilterSize;
    int numSectors;
    float* filters;

#ifdef PROFILE
    TickMeter tm;
    cout << "root start" << endl;
    tm.reset();
    tm.start();
#endif

    numSectors = H->pyramid[0]->numFeatures;
    for (i = 0; i < numLevels + 3; i++)
    {
        CUresult cu_res = cuStreamCreate(&convStreams[i], CU_STREAM_DEFAULT);
        CUDA_CHECK(cu_res, "cuStreamCreate(&convStreams[%d], CU_STREAM_DEFAULT)", i);
    }
    totalFilterSize = 0;
    for (i = 0; i < n + 1; i++)
    {
        filterIdx[i] = totalFilterSize;
        filterSize[i] = all_F[i]->sizeX * all_F[i]->sizeY * numSectors;
        totalFilterSize += filterSize[i];
    }

    CUresult cu_res = cuMemAlloc(&dev_filter, totalFilterSize * sizeof(float));
    CUDA_CHECK(cu_res, "cuMemAlloc(&dev_filter): %zd bytes", totalFilterSize * sizeof(float));

    filters = (float*) (malloc(totalFilterSize * sizeof(float)));
    for (i = 0; i < n + 1; i++)
    {
        memcpy(&(filters[filterIdx[i]]), all_F[i]->H,
                filterSize[i] * sizeof(float));
    }

    cu_res = cuMemcpyHtoD(dev_filter, filters, totalFilterSize * sizeof(float));
    CUDA_CHECK(cu_res, "cuMemcpyHtoD(dev_filter, filters): %zd bytes", totalFilterSize * sizeof(float));

    free(filters);
    for (k = Lambda; k < H->numLevels; k++)
    {
        j = k - Lambda;
        // map size of root filter
        size[0][j] = H->pyramid[k]->sizeX * H->pyramid[k]->sizeY * numSectors;
        // map size of part filters
        size[1][j] = map[j]->sizeX * map[j]->sizeY * numSectors;
    }
    CUtexref image_texMap;
    cu_res = cuModuleGetTexRef(&image_texMap, module[0], "texMap");
    CUDA_CHECK(cu_res, "cuModuleGetTexRef(&image_texMap)");
    cu_res = cuTexRefSetFlags(image_texMap, CU_TRSF_READ_AS_INTEGER);
    CUDA_CHECK(cu_res, "cuTexRefSetFlags(image_texMap)");
    cu_res = cuTexRefSetFormat(image_texMap, CU_AD_FORMAT_FLOAT, 1);
    CUDA_CHECK(cu_res, "cuTexRefSetFormat(image_texMap)");

    CUtexref image_texFi;
    cu_res = cuModuleGetTexRef(&image_texFi, module[0], "texFi");
    CUDA_CHECK(cu_res, "cuModuleGetTexRef(&image_texFi)");
    cu_res = cuTexRefSetFlags(image_texFi, CU_TRSF_READ_AS_INTEGER);
    CUDA_CHECK(cu_res, "cuTexRefSetFlags(image_texFi)");
    cu_res = cuTexRefSetFormat(image_texFi, CU_AD_FORMAT_FLOAT, 1);
    CUDA_CHECK(cu_res, "cuTexRefSetFormat(image_texFi)");
    cu_res = cuTexRefSetAddress(NULL, image_texFi, dev_filter,
            totalFilterSize * sizeof(float));
    CUDA_CHECK(cu_res, "cuTexRefSetAddress(image_texFi)");

    // Transfer of root maps
    for (k = Lambda; k < H->numLevels; k++)
    {
        j = k - Lambda;

        CUresult cu_res = cuMemAlloc(&dev_map[0][j], size[0][j] * sizeof(float));
        CUDA_CHECK(cu_res, "cuMemAlloc(&dev_map[0][%d]): %zd bytes", j, size[0][j] * sizeof(float));

        cu_res = cuMemcpyHtoDAsync(dev_map[0][j], H->pyramid[k]->map,
                        size[0][j] * sizeof(float), convStreams[j]);
        CUDA_CHECK(cu_res, "cuMemcpyHtoDAsync(dev_map[0][%d], H->pyramid[%d]->map): %zd bytes",
                   j, k, size[0][j] * sizeof(float));
    }

    // Calculate root scores
    for (k = Lambda; k < H->numLevels; k++)
    {
        j = k - Lambda;

        CUresult cu_res = cuStreamSynchronize(convStreams[0]);
        CUDA_CHECK(cu_res, "cuStreamSynchronize(convStreams[0])");
        cu_res = cuTexRefSetAddress(NULL, image_texMap, dev_map[0][j],
                        size[0][j] * sizeof(float));
        CUDA_CHECK(cu_res, "cuTexRefSetAddress(image_texMap)");
        res[j] = convolutionGPU(all_F[0], H->pyramid[k], &dev_filter,
                &dev_map[0][j], &dev_filterIdxTbl[j][0], &dev_score[j][0],
                filterIdx[0], convStreams[0]);
    }

    cu_res = cuStreamSynchronize(convStreams[0]);
    CUDA_CHECK(cu_res, "cuStreamSynchronize(convStreams[0])");

#ifdef PROFILE
    tm.stop();
    cout << "END root time = " << tm.getTimeSec() << " sec" << endl;
    cout << "part start" << endl;
    tm.reset();
    tm.start();
#endif

    for (k = Lambda; k < H->numLevels; k++)
    {
        // Transfer of part maps
        j = k - Lambda;

        CUresult cu_res = cuMemAlloc(&dev_map[1][j], size[1][j] * sizeof(float));
        CUDA_CHECK(cu_res, "cuMemAlloc(&dev_map[1][%d]): %zd bytes", j, size[1][j] * sizeof(float));
        cu_res = cuMemcpyHtoDAsync(dev_map[1][j], map[j]->map,
                        size[1][j] * sizeof(float), convStreams[numLevels + 2]);
        CUDA_CHECK(cu_res, "cuMemcpyHtoDAsync(dev_map[1][%d], map[%d]->map): %zd bytes",
                   j, j, size[1][j] * sizeof(float));
        cu_res = cuStreamSynchronize(convStreams[numLevels + 2]);
        CUDA_CHECK(cu_res, "cuStreamSynchronize(convStreams[%d])", numLevels + 2);
        cu_res = cuTexRefSetAddress(NULL, image_texMap, dev_map[1][j],
                        size[1][j] * sizeof(float));
        CUDA_CHECK(cu_res, "cuTexRefSetAddress(image_texMap)");
        // Calculate part scores
        for (i = 0; i < n; i++)
        {
            if (res[j] == LATENT_SVM_OK)
            {
                res[j] = convolutionGPU(all_F[i + 1], map[j], &dev_filter,
                        &dev_map[1][j], &dev_filterIdxTbl[j][i + 1],
                        &dev_score[j][i + 1], filterIdx[i + 1],
                        convStreams[i + 1]);
            }
        }
    }

    for (i = 0; i < n + 1; i++)
    {
        CUresult cu_res = cuStreamSynchronize(convStreams[i]);
        CUDA_CHECK(cu_res, "cuStreamSynchronize(convStreams[%d])", i);
    }
    for (i = 0; i < numLevels + 3; i++)
    {
        CUresult cu_res = cuStreamDestroy(convStreams[i]);
        CUDA_CHECK(cu_res, "cuStreamDestroy(convStreams[%d])", i);
    }

    cuMemFree(dev_filter);
    for (j = 0; j < numLevels; j++)
    {
        CUresult cu_res = cuMemFree(dev_map[0][j]);
        CUDA_CHECK(cu_res, "cuMemFree(dev_map[0][%d])", j);
        cu_res = cuMemFree(dev_map[1][j]);
        CUDA_CHECK(cu_res, "cuMemFree(dev_map[1][%d])", j);
        for (i = 0; i < n + 1; i++)
        {
            cu_res = cuMemFree(dev_filterIdxTbl[j][i]);
            CUDA_CHECK(cu_res, "cuMemFree(dev_filterIdxTbl[%d][%d])", j, i);
        }
    }
#ifdef PROFILE
    tm.stop();
    cout << "END part time = " << tm.getTimeSec() << " sec" << endl;
#endif
}

/*
 // Distance transformation
 //
 // API
 // void distanceTransform(int numLevels, int n, int max_size,
 //        const CvLSVMFilterObject** all_F, CvLSVMFeatureMap* map[],
 //        CvLSVMFilterDisposition** disposition, CUdeviceptr** dev_score);
 // INPUT
 // numLevels          -
 // n                  - the number of part filters
 // max_size           - max cuMemAllocHost size
 // all_F              - the set of filters
 //                      (the first element is root filter,the other - part filters)
 // map                - feature map
 // dev_score          - device score
 // OUTPUT
 // disposition        - filter disposition
 // RESULT
 // none
 */
static void distanceTransform(int numLevels, int n, int max_size,
        const CvLSVMFilterObject** all_F, CvLSVMFeatureMap* map[],
        CvLSVMFilterDisposition** disposition, CUdeviceptr** dev_score)
{
    CvLSVMFilterDisposition tmp_disposition;
    CUstream ditTransStreams[DISTANCE_TRANSFORM_STREAMS];
    CUdeviceptr dev_distTransScore[DISTANCE_TRANSFORM_STREAMS];
    CUdeviceptr dev_x[DISTANCE_TRANSFORM_STREAMS];
    CUdeviceptr dev_y[DISTANCE_TRANSFORM_STREAMS];
    CUdeviceptr dev_distTransWork[DISTANCE_TRANSFORM_STREAMS];
    int diffX, diffY, size;
    int i, j, k, l;

    for (i = 0; i < DISTANCE_TRANSFORM_STREAMS; i++)
    {
        CUresult res = cuStreamCreate(&ditTransStreams[i], CU_STREAM_DEFAULT);
        CUDA_CHECK(res, "cuStreamCreate(&ditTransStreams[%d], CU_STREAM_DEFAULT)", i);
    }

    CUresult res = cuMemAllocHost((void**) &tmp_disposition.score, sizeof(float) * max_size);
    CUDA_CHECK(res, "cuMemAllocHost(&tmp_disposition.score): %zd bytes", sizeof(float) * max_size);
    res = cuMemAllocHost((void**) &tmp_disposition.x, sizeof(int) * max_size);
    CUDA_CHECK(res, "cuMemAllocHost(&tmp_disposition.x): %zd bytes", sizeof(int) * max_size);
    res = cuMemAllocHost((void**) &tmp_disposition.y, sizeof(int) * max_size);
    CUDA_CHECK(res, "cuMemAllocHost(&tmp_disposition.y): %zd bytes", sizeof(int) * max_size);

    bool isNew = true;
    for (j = 0; j < n * numLevels; j++)
    {
        // Computation by dividing the number of streams and using 
        // the same memory area of device for processing speed by
        // reducing the number of calling cuMemFree function
        l = j % DISTANCE_TRANSFORM_STREAMS;
        k = j / n;
        i = j % n;

        calculationScoreSize(all_F[i + 1], map[k], &diffX, &diffY);
        if (isNew == true)
        {
            size = (diffX + 1) * (diffY + 1);
            CUresult res = cuMemAlloc(&dev_distTransWork[l], sizeof(DistTransWork) * size);
            CUDA_CHECK(res, "cuMemAlloc(&dev_distTransWork[%d]): %zd bytes", l, sizeof(DistTransWork) * size);
            res = cuMemAlloc(&dev_distTransScore[l], sizeof(float) * size);
            CUDA_CHECK(res, "cuMemAlloc(&dev_distTransScore[%d]): %zd bytes", l, sizeof(float) * size);
            res = cuMemAlloc(&dev_x[l], sizeof(int) * size);
            CUDA_CHECK(res, "cuMemAlloc(&dev_x[%d]): %zd bytes", l, sizeof(int) * size);
            res = cuMemAlloc(&dev_y[l], sizeof(int) * size);
            CUDA_CHECK(res, "cuMemAlloc(&dev_y[%d]): %zd bytes", l, sizeof(int) * size);
        }

        DistanceTransformTwoDimensionalProblemGPU(all_F[i + 1], map[k],
                &dev_score[k][i + 1], &dev_distTransWork[l],
                &dev_distTransScore[l], &dev_x[l], &dev_y[l], diffX, diffY,
                ditTransStreams[l]);

        if (l == DISTANCE_TRANSFORM_STREAMS - 1 || j == n * numLevels - 1)
        {
            isNew = false;

            for (int m = 0; m <= l; m++)
            {
                CUresult res = cuStreamSynchronize(ditTransStreams[m]);
                CUDA_CHECK(res, "cuStreamSynchronize(ditTransStreams[%d])", m);
                k = (j - l + m) / n;
                i = (j - l + m) % n;
                dispositionCpyDtoH(&dev_distTransScore[m], &dev_x[m], &dev_y[m],
                        &disposition[k][i], &tmp_disposition, ditTransStreams);
            }
        }
    }

    res = cuMemFreeHost(tmp_disposition.score);
    CUDA_CHECK(res, "cuMemFreeHost(tmp_disposition.score)");
    res = cuMemFreeHost(tmp_disposition.x);
    CUDA_CHECK(res, "cuMemFreeHost(tmp_disposition.x)");
    res = cuMemFreeHost(tmp_disposition.y);
    CUDA_CHECK(res, "cuMemFreeHost(tmp_disposition.y)");

    for (i = 0; i < DISTANCE_TRANSFORM_STREAMS; i++)
    {
        CUresult res = cuStreamSynchronize(ditTransStreams[i]);
        CUDA_CHECK(res, "cuStreamSynchronize(ditTransStreams[%d])", i);
        res = cuStreamDestroy(ditTransStreams[i]);
        CUDA_CHECK(res, "cuStreamDestroy(ditTransStreams[%d])", i);
        res = cuMemFree(dev_distTransWork[i]);
        CUDA_CHECK(res, "cuMemFree(dev_distTransWork[%d])", i);
        res = cuMemFree(dev_distTransScore[i]);
        CUDA_CHECK(res, "cuMemFree(dev_distTransScore[%d])", i);
        res = cuMemFree(dev_x[i]);
        CUDA_CHECK(res, "cuMemFree(dev_x[%d])", i);
        res = cuMemFree(dev_y[i]);
        CUDA_CHECK(res, "cuMemFree(dev_y[i])");
    }
    for (k = 0; k < numLevels; k++)
    {
        for (i = 0; i < n; i++)
        {
            CUresult res = cuMemFree(dev_score[k][i + 1]);
            CUDA_CHECK(res, "cuMemFree(dev_score[%d][%d])", k, i + 1);
        }
    }
}

/*
 // Computation score function that exceed threshold
 //
 // API
 // int thresholdFunctionalScore(const CvLSVMFilterObject **all_F, int n,
 //        const CvLSVMFeaturePyramid *H, float b, CvLSVMFeatureMap *map[],
 //        float scoreThreshold, float **score, CvPoint **points, int **levels,
 //        int *kPoints, CvPoint ***partsDisplacement);
 // INPUT
 // all_F             - the set of filters
 //                     (the first element is root filter,the other - part filters)
 // n                 - the number of part filters
 // H                 - feature pyramid
 // b                 - linear term of the score function
 // maxXBorder        - the largest root filter size (X-direction)
 // maxYBorder        - the largest root filter size (Y-direction)
 // scoreThreshold    - score threshold
 // OUTPUT
 // score             - score function values that exceed threshold
 // points            - the set of root filter positions (in the block space)
 // levels            - the set of levels
 // kPoints           - number of root filter positions
 // partsDisplacement - displacement of part filters (in the block space)
 // RESULT
 // Error status
 */
int thresholdFunctionalScore(const CvLSVMFilterObject **all_F, int n,
        const CvLSVMFeaturePyramid *H, float b, CvLSVMFeatureMap *map[],
        float scoreThreshold, float **score, CvPoint **points, int **levels,
        int *kPoints, CvPoint ***partsDisplacement)
{
    int l, i, j, k, s, f, level, numLevels;
    float **tmpScore;
    CvPoint ***tmpPoints;
    CvPoint ****tmpPartsDisplacement;
    int *tmpKPoints;

#ifdef PROFILE
    TickMeter tm;
    TickMeter all_tm;

    cout << "(matching.cpp)thresholdFunctionalScore START" << endl << "{"
    << endl;
    all_tm.reset();
    all_tm.start();
#endif

    // Computation the number of levels for seaching object,
    // first lambda-levels are used for computation values
    // of score function for each position of root filter
    numLevels = H->numLevels - Lambda;

    // Allocation memory for values of score function for each level
    // that exceed threshold
    tmpScore = (float **) malloc(sizeof(float*) * numLevels);
    // Allocation memory for the set of points that corresponds
    // to the maximum of score function
    tmpPoints = (CvPoint ***) malloc(sizeof(CvPoint **) * numLevels);
    for (i = 0; i < numLevels; i++)
    {
        tmpPoints[i] = (CvPoint **) malloc(sizeof(CvPoint *));
    }
    // Allocation memory for memory for saving parts displacement on each level
    tmpPartsDisplacement = (CvPoint ****) malloc(
            sizeof(CvPoint ***) * numLevels);
    for (i = 0; i < numLevels; i++)
    {
        tmpPartsDisplacement[i] = (CvPoint ***) malloc(sizeof(CvPoint **));
    }
    // Number of points that corresponds to the maximum
    // of score function on each level
    tmpKPoints = (int *) malloc(sizeof(int) * numLevels);
    for (i = 0; i < numLevels; i++)
    {
        tmpKPoints[i] = 0;
    }

    (*kPoints) = 0;

    CUdeviceptr **dev_score;

    dev_score = (CUdeviceptr **) malloc(sizeof(CUdeviceptr *) * (numLevels));

    for (i = 0; i < numLevels; i++)
    {
        dev_score[i] = (CUdeviceptr *) malloc(sizeof(CUdeviceptr) * (n + 1));
    }

    CvLSVMFilterDisposition **disposition;
    disposition = (CvLSVMFilterDisposition **) malloc(
            sizeof(CvLSVMFilterDisposition *) * (numLevels));

    for (i = 0; i < numLevels; i++)
    {
        disposition[i] = (CvLSVMFilterDisposition *) malloc(
                sizeof(CvLSVMFilterDisposition) * n);
    }

    int res[numLevels];

    // Calculation root and part scores
    calculationScore(numLevels, n, H, all_F, res, map, dev_score);

#ifdef PROFILE
    cout << "dt start" << endl;
    tm.reset();
    tm.start();
#endif

    // Computation distance transform
    int max_size;
    max_size = 0;
    for (k = 0; k < numLevels; k++)
    {
        for (i = 0; i < n; i++)
        {
            dispositionMalloc(all_F[i + 1], map[k], &disposition[k][i],
                    &max_size);
        }
    }
    distanceTransform(numLevels, n, max_size, all_F, map, disposition,
            dev_score);

#ifdef PROFILE
    tm.stop();
    cout << "END dt time = " << tm.getTimeSec() << " sec" << endl;
    cout << "sum start" << endl;
    tm.reset();
    tm.start();
#endif

    // Computation displacements that its scores are higher than the threshold
    for (l = Lambda; l < H->numLevels; l++)
    {
        k = l - Lambda;
        if (res[k] == LATENT_SVM_OK)
        {
            res[k] = sumScore(all_F, n, H, l, b, scoreThreshold,
                    &disposition[k][0], &dev_score[k][0], map[k],
                    &(tmpScore[k]), tmpPoints[k], &(tmpKPoints[k]),
                    tmpPartsDisplacement[k]);

            if (res[k] == LATENT_SVM_OK)
            {
                (*kPoints) += tmpKPoints[k];
            }
        }
    }

    for (k = 0; k < numLevels; k++)
    {
        for (i = 0; i < n; i++)
        {
            dispositionFree(&disposition[k][i]);

        }
    }

    // Allocation memory for levels
    (*levels) = (int *) malloc(sizeof(int) * (*kPoints));
    // Allocation memory for the set of points
    (*points) = (CvPoint *) malloc(sizeof(CvPoint) * (*kPoints));
    // Allocation memory for parts displacement
    (*partsDisplacement) = (CvPoint **) malloc(sizeof(CvPoint *) * (*kPoints));
    // Allocation memory for score function values
    (*score) = (float *) malloc(sizeof(float) * (*kPoints));

    // Filling the set of points, levels and parts displacement
    s = 0;
    f = 0;
    for (i = 0; i < numLevels; i++)
    {
        // Computation the number of level
        level = i + Lambda;

        // Addition a set of points
        f += tmpKPoints[i];
        for (j = s; j < f; j++)
        {
            (*levels)[j] = level;
            (*points)[j] = (*tmpPoints[i])[j - s];
            (*score)[j] = tmpScore[i][j - s];
            (*partsDisplacement)[j] = (*(tmpPartsDisplacement[i]))[j - s];
        }
        s = f;
    }

    // Release allocated memory
    for (i = 0; i < numLevels; i++)
    {
        free(*tmpPoints[i]);
        free(tmpPoints[i]);
        free(*tmpPartsDisplacement[i]);
        free(tmpPartsDisplacement[i]);
        free(tmpScore[i]);
        free(dev_score[i]);
        free(disposition[i]);
    }
    free(tmpPoints);
    free(tmpScore);
    free(tmpKPoints);
    free(tmpPartsDisplacement);
    free(dev_score);
    free(disposition);

#ifdef PROFILE
    tm.stop();
    cout << "END sum time = " << tm.getTimeSec() << " sec" << endl;
    all_tm.stop();
    cout << "}" << endl << "(matching.cpp)thresholdFunctionalScore END time = "
    << all_tm.getTimeSec() << " sec" << endl;
#endif

    return LATENT_SVM_OK;
}

/*
 //
 // API
 // static void sort(int n, const float* x, int* indices);
 // INPUT
 // n
 // x
 // OUTPUT
 // indices
 // RESULT
 // none
 */
static void sort(int n, const float* x, int* indices)
{
    int i, j;
    for (i = 0; i < n; i++)
        for (j = i + 1; j < n; j++)
        {
            if (x[indices[j]] > x[indices[i]])
            {
                int index_tmp = indices[i];
                indices[i] = indices[j];
                indices[j] = index_tmp;
            }
        }
}

/*
 // Perform non-maximum suppression algorithm (described in original paper)
 // to remove "similar" bounding boxes
 //
 // API
 // int nonMaximumSuppression(int numBoxes, const CvPoint *points,
 //        const CvPoint *oppositePoints, const float *score,
 //        float overlapThreshold, int *numBoxesOut, CvPoint **pointsOut,
 //        CvPoint **oppositePointsOut, float **scoreOut);
 // INPUT
 // numBoxes          - number of bounding boxes
 // points            - array of left top corner coordinates
 // oppositePoints    - array of right bottom corner coordinates
 // score             - array of detection scores
 // overlapThreshold  - threshold: bounding box is removed if overlap part
 //                     is greater than passed value
 // OUTPUT
 // numBoxesOut       - the number of bounding boxes algorithm returns
 // pointsOut         - array of left top corner coordinates
 // oppositePointsOut - array of right bottom corner coordinates
 // scoreOut          - array of detection scores
 // RESULT
 // Error status
 */
int nonMaximumSuppression(int numBoxes, const CvPoint *points,
        const CvPoint *oppositePoints, const float *score,
        float overlapThreshold, int *numBoxesOut, CvPoint **pointsOut,
        CvPoint **oppositePointsOut, float **scoreOut)
{
    int i, j, index;
    float* box_area = (float*) malloc(numBoxes * sizeof(float));
    int* indices = (int*) malloc(numBoxes * sizeof(int));
    int* is_suppressed = (int*) malloc(numBoxes * sizeof(int));

    for (i = 0; i < numBoxes; i++)
    {
        indices[i] = i;
        is_suppressed[i] = 0;
        box_area[i] = (float) ((oppositePoints[i].x - points[i].x + 1)
                * (oppositePoints[i].y - points[i].y + 1));
    }
    sort(numBoxes, score, indices);
    for (i = 0; i < numBoxes; i++)
    {
        if (!is_suppressed[indices[i]])
        {
            for (j = i + 1; j < numBoxes; j++)
            {
                if (!is_suppressed[indices[j]])
                {
                    int x1max = max(points[indices[i]].x, points[indices[j]].x);
                    int x2min =
                            min(oppositePoints[indices[i]].x, oppositePoints[indices[j]].x);
                    int y1max = max(points[indices[i]].y, points[indices[j]].y);
                    int y2min =
                            min(oppositePoints[indices[i]].y, oppositePoints[indices[j]].y);
                    int overlapWidth = x2min - x1max + 1;
                    int overlapHeight = y2min - y1max + 1;
                    if (overlapWidth > 0 && overlapHeight > 0)
                    {
                        float overlapPart = (overlapWidth * overlapHeight)
                                / box_area[indices[j]];
                        if (overlapPart > overlapThreshold)
                        {
                            is_suppressed[indices[j]] = 1;
                        }
                    }
                }
            }
        }
    }

    *numBoxesOut = 0;
    for (i = 0; i < numBoxes; i++)
    {
        if (!is_suppressed[i])
            (*numBoxesOut)++;
    }

    *pointsOut = (CvPoint *) malloc((*numBoxesOut) * sizeof(CvPoint));
    *oppositePointsOut = (CvPoint *) malloc((*numBoxesOut) * sizeof(CvPoint));
    *scoreOut = (float *) malloc((*numBoxesOut) * sizeof(float));
    index = 0;
    for (i = 0; i < numBoxes; i++)
    {
        if (!is_suppressed[indices[i]])
        {
            (*pointsOut)[index].x = points[indices[i]].x;
            (*pointsOut)[index].y = points[indices[i]].y;
            (*oppositePointsOut)[index].x = oppositePoints[indices[i]].x;
            (*oppositePointsOut)[index].y = oppositePoints[indices[i]].y;
            (*scoreOut)[index] = score[indices[i]];
            index++;
        }

    }

    free(indices);
    free(box_area);
    free(is_suppressed);

    return LATENT_SVM_OK;
}
