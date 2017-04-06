#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <dpm_ocv.hpp>

// Macros in OpenCV private header files
#define VAL_OF_TRUNCATE 0.2f
#define LATENT_SVM_OK 0

// Data structures in OpenCV private header files
typedef struct{
    int sizeX;
    int sizeY;
    int numFeatures;
    float *map;
} CvLSVMFeatureMap;

typedef struct{
    int numLevels;
    CvLSVMFeatureMap **pyramid;
} CvLSVMFeaturePyramid;

// OpenCV non public functions
extern int allocFeatureMapObject(CvLSVMFeatureMap **obj, const int sizeX, const int sizeY,
                                const int p);
extern IplImage * resize_opencv(IplImage * img, float scale);
extern int computeBorderSize(int maxXBorder, int maxYBorder, int *bx, int *by);
extern int addNullableBorder(CvLSVMFeatureMap *map, int bx, int by);

extern "C" {
extern int allocFeaturePyramidObject(CvLSVMFeaturePyramid **obj, const int countLevel);
extern int freeFeaturePyramidObject(CvLSVMFeaturePyramid **obj);
extern int getMaxFilterDims(const CvLSVMFilterObject **filters, int kComponents,
                            const int *kPartFilters,
                            unsigned int *maxXBorder, unsigned int *maxYBorder);
extern int searchObjectThresholdSomeComponents(const CvLSVMFeaturePyramid *H,
                                               const CvLSVMFilterObject **filters,
                                               int kComponents, const int *kPartFilters,
                                               const float *b, float scoreThreshold,
                                               CvPoint **points, CvPoint **oppPoints,
                                               float **score, int *kPoints, int numThreads);
extern int clippingBoxes(int width, int height,
                         CvPoint *points, int kPoints);
extern int nonMaximumSuppression(int numBoxes, const CvPoint *points,
                                 const CvPoint *oppositePoints, const float *score,
                                 float overlapThreshold,
                                 int *numBoxesOut, CvPoint **pointsOut,
                                 CvPoint **oppositePointsOut, float **scoreOut);
extern CvLatentSvmDetector* cvLoadLatentSvmDetector(const char* filename);
};

// [COPY] static functions in OpenCV
static int normalizeAndTruncate(CvLSVMFeatureMap *map, const float alfa, int num_bins)
{
    int i,j, ii;
    int sizeX, sizeY, p, pos, pp, xp, pos1, pos2;
    float * partOfNorm; // norm of C(i, j)
    float * newData;
    float   valOfNorm;

    sizeX     = map->sizeX;
    sizeY     = map->sizeY;
    partOfNorm = (float *)malloc (sizeof(float) * (sizeX * sizeY));

    p  = num_bins;
    xp = num_bins * 3;
    pp = num_bins * 12;

    for(i = 0; i < sizeX * sizeY; i++)
    {
        valOfNorm = 0.0f;
        pos = i * map->numFeatures;
        for(j = 0; j < p; j++)
        {
            valOfNorm += map->map[pos + j] * map->map[pos + j];
        }/*for(j = 0; j < p; j++)*/
        partOfNorm[i] = valOfNorm;
    }/*for(i = 0; i < sizeX * sizeY; i++)*/

    sizeX -= 2;
    sizeY -= 2;

    newData = (float *)malloc (sizeof(float) * (sizeX * sizeY * pp));
    //normalization
    for(i = 1; i <= sizeY; i++)
    {
        for(j = 1; j <= sizeX; j++)
        {
            valOfNorm = sqrtf(
                partOfNorm[(i    )*(sizeX + 2) + (j    )] +
                partOfNorm[(i    )*(sizeX + 2) + (j + 1)] +
                partOfNorm[(i + 1)*(sizeX + 2) + (j    )] +
                partOfNorm[(i + 1)*(sizeX + 2) + (j + 1)]) + FLT_EPSILON;
            pos1 = (i  ) * (sizeX + 2) * xp + (j  ) * xp;
            pos2 = (i-1) * (sizeX    ) * pp + (j-1) * pp;
            for(ii = 0; ii < p; ii++)
            {
                newData[pos2 + ii        ] = map->map[pos1 + ii    ] / valOfNorm;
            }/*for(ii = 0; ii < p; ii++)*/
            for(ii = 0; ii < 2 * p; ii++)
            {
                newData[pos2 + ii + p * 4] = map->map[pos1 + ii + p] / valOfNorm;
            }/*for(ii = 0; ii < 2 * p; ii++)*/
            valOfNorm = sqrtf(
                partOfNorm[(i    )*(sizeX + 2) + (j    )] +
                partOfNorm[(i    )*(sizeX + 2) + (j + 1)] +
                partOfNorm[(i - 1)*(sizeX + 2) + (j    )] +
                partOfNorm[(i - 1)*(sizeX + 2) + (j + 1)]) + FLT_EPSILON;
            for(ii = 0; ii < p; ii++)
            {
                newData[pos2 + ii + p    ] = map->map[pos1 + ii    ] / valOfNorm;
            }/*for(ii = 0; ii < p; ii++)*/
            for(ii = 0; ii < 2 * p; ii++)
            {
                newData[pos2 + ii + p * 6] = map->map[pos1 + ii + p] / valOfNorm;
            }/*for(ii = 0; ii < 2 * p; ii++)*/
            valOfNorm = sqrtf(
                partOfNorm[(i    )*(sizeX + 2) + (j    )] +
                partOfNorm[(i    )*(sizeX + 2) + (j - 1)] +
                partOfNorm[(i + 1)*(sizeX + 2) + (j    )] +
                partOfNorm[(i + 1)*(sizeX + 2) + (j - 1)]) + FLT_EPSILON;
            for(ii = 0; ii < p; ii++)
            {
                newData[pos2 + ii + p * 2] = map->map[pos1 + ii    ] / valOfNorm;
            }/*for(ii = 0; ii < p; ii++)*/
            for(ii = 0; ii < 2 * p; ii++)
            {
                newData[pos2 + ii + p * 8] = map->map[pos1 + ii + p] / valOfNorm;
            }/*for(ii = 0; ii < 2 * p; ii++)*/
            valOfNorm = sqrtf(
                partOfNorm[(i    )*(sizeX + 2) + (j    )] +
                partOfNorm[(i    )*(sizeX + 2) + (j - 1)] +
                partOfNorm[(i - 1)*(sizeX + 2) + (j    )] +
                partOfNorm[(i - 1)*(sizeX + 2) + (j - 1)]) + FLT_EPSILON;
            for(ii = 0; ii < p; ii++)
            {
                newData[pos2 + ii + p * 3 ] = map->map[pos1 + ii    ] / valOfNorm;
            }/*for(ii = 0; ii < p; ii++)*/
            for(ii = 0; ii < 2 * p; ii++)
            {
                newData[pos2 + ii + p * 10] = map->map[pos1 + ii + p] / valOfNorm;
            }/*for(ii = 0; ii < 2 * p; ii++)*/
        }/*for(j = 1; j <= sizeX; j++)*/
    }/*for(i = 1; i <= sizeY; i++)*/
    //truncation
    for(i = 0; i < sizeX * sizeY * pp; i++)
    {
        if(newData [i] > alfa) newData [i] = alfa;
    }/*for(i = 0; i < sizeX * sizeY * pp; i++)*/
    //swap data

    map->numFeatures  = pp;
    map->sizeX = sizeX;
    map->sizeY = sizeY;

    free (map->map);
    free (partOfNorm);

    map->map = newData;

    return LATENT_SVM_OK;
}

static int PCAFeatureMaps(CvLSVMFeatureMap *map, int num_bins)
{
    int i,j, ii, jj, k;
    int sizeX, sizeY, p,  pp, xp, yp, pos1, pos2;
    float * newData;
    float val;
    float nx, ny;

    sizeX = map->sizeX;
    sizeY = map->sizeY;
    p     = map->numFeatures;
    pp    = num_bins * 3 + 4;
    yp    = 4;
    xp    = num_bins;

    nx    = 1.0f / sqrtf((float)(xp * 2));
    ny    = 1.0f / sqrtf((float)(yp    ));

    newData = (float *)malloc (sizeof(float) * (sizeX * sizeY * pp));

    for(i = 0; i < sizeY; i++)
    {
        for(j = 0; j < sizeX; j++)
        {
            pos1 = ((i)*sizeX + j)*p;
            pos2 = ((i)*sizeX + j)*pp;
            k = 0;
            for(jj = 0; jj < xp * 2; jj++)
            {
                val = 0;
                for(ii = 0; ii < yp; ii++)
                {
                    val += map->map[pos1 + yp * xp + ii * xp * 2 + jj];
                }/*for(ii = 0; ii < yp; ii++)*/
                newData[pos2 + k] = val * ny;
                k++;
            }/*for(jj = 0; jj < xp * 2; jj++)*/
            for(jj = 0; jj < xp; jj++)
            {
                val = 0;
                for(ii = 0; ii < yp; ii++)
                {
                    val += map->map[pos1 + ii * xp + jj];
                }/*for(ii = 0; ii < yp; ii++)*/
                newData[pos2 + k] = val * ny;
                k++;
            }/*for(jj = 0; jj < xp; jj++)*/
            for(ii = 0; ii < yp; ii++)
            {
                val = 0;
                for(jj = 0; jj < 2 * xp; jj++)
                {
                    val += map->map[pos1 + yp * xp + ii * xp * 2 + jj];
                }/*for(jj = 0; jj < xp; jj++)*/
                newData[pos2 + k] = val * nx;
                k++;
            } /*for(ii = 0; ii < yp; ii++)*/
        }/*for(j = 0; j < sizeX; j++)*/
    }/*for(i = 0; i < sizeY; i++)*/
    //swap data

    map->numFeatures = pp;

    free (map->map);

    map->map = newData;

    return LATENT_SVM_OK;
}

static int getFeatureMaps(const IplImage* image, const int k, CvLSVMFeatureMap **map, int num_bins)
{
    int sizeX, sizeY;
    int p, px, stringSize;
    int height, width, numChannels;
    int i, j, kk, c, ii, jj, d;
    float  * datadx, * datady;

    int   ch;
    float magnitude, x, y, tx, ty;

    IplImage * dx, * dy;
    int *nearest;
    float *w, a_x, b_x;

    float kernel[3] = {-1.f, 0.f, 1.f};
    CvMat kernel_dx = cvMat(1, 3, CV_32F, kernel);
    CvMat kernel_dy = cvMat(3, 1, CV_32F, kernel);

    float * r;
    int   * alfa;

    float boundary_x[num_bins + 1];
    float boundary_y[num_bins + 1];
    float max, dotProd;
    int   maxi;

    height = image->height;
    width  = image->width ;

    numChannels = image->nChannels;

    dx    = cvCreateImage(cvSize(image->width, image->height),
                          IPL_DEPTH_32F, 3);
    dy    = cvCreateImage(cvSize(image->width, image->height),
                          IPL_DEPTH_32F, 3);

    sizeX = width  / k;
    sizeY = height / k;
    px    = 3 * num_bins;
    p     = px;
    stringSize = sizeX * p;
    allocFeatureMapObject(map, sizeX, sizeY, p);

    cvFilter2D(image, dx, &kernel_dx, cvPoint(-1, 0));
    cvFilter2D(image, dy, &kernel_dy, cvPoint(0, -1));

    float arg_vector;
    for(i = 0; i <= num_bins; i++)
    {
        arg_vector    = ( (float) i ) * ( (float)(CV_PI) / (float)(num_bins) );
        boundary_x[i] = cosf(arg_vector);
        boundary_y[i] = sinf(arg_vector);
    }/*for(i = 0; i <= NUM_SECTOR; i++) */

    r    = (float *)malloc( sizeof(float) * (width * height));
    alfa = (int   *)malloc( sizeof(int  ) * (width * height * 2));

    for(j = 1; j < height - 1; j++)
    {
        datadx = (float*)(dx->imageData + dx->widthStep * j);
        datady = (float*)(dy->imageData + dy->widthStep * j);
        for(i = 1; i < width - 1; i++)
        {
            c = 0;
            x = (datadx[i * numChannels + c]);
            y = (datady[i * numChannels + c]);

            r[j * width + i] =sqrtf(x * x + y * y);
            for(ch = 1; ch < numChannels; ch++)
            {
                tx = (datadx[i * numChannels + ch]);
                ty = (datady[i * numChannels + ch]);
                magnitude = sqrtf(tx * tx + ty * ty);
                if(magnitude > r[j * width + i])
                {
                    r[j * width + i] = magnitude;
                    c = ch;
                    x = tx;
                    y = ty;
                }
            }/*for(ch = 1; ch < numChannels; ch++)*/

            max  = boundary_x[0] * x + boundary_y[0] * y;
            maxi = 0;
            for (kk = 0; kk < num_bins; kk++)
            {
                dotProd = boundary_x[kk] * x + boundary_y[kk] * y;
                if (dotProd > max)
                {
                    max  = dotProd;
                    maxi = kk;
                }
                else
                {
                    if (-dotProd > max)
                    {
                        max  = -dotProd;
                        maxi = kk + num_bins;
                    }
                }
            }
            alfa[j * width * 2 + i * 2    ] = maxi % num_bins;
            alfa[j * width * 2 + i * 2 + 1] = maxi;
        }/*for(i = 0; i < width; i++)*/
    }/*for(j = 0; j < height; j++)*/

    nearest = (int  *)malloc(sizeof(int  ) *  k);
    w       = (float*)malloc(sizeof(float) * (k * 2));

    for(i = 0; i < k / 2; i++)
    {
        nearest[i] = -1;
    }/*for(i = 0; i < k / 2; i++)*/
    for(i = k / 2; i < k; i++)
    {
        nearest[i] = 1;
    }/*for(i = k / 2; i < k; i++)*/

    for(j = 0; j < k / 2; j++)
    {
        b_x = k / 2 + j + 0.5f;
        a_x = k / 2 - j - 0.5f;
        w[j * 2    ] = 1.0f/a_x * ((a_x * b_x) / ( a_x + b_x));
        w[j * 2 + 1] = 1.0f/b_x * ((a_x * b_x) / ( a_x + b_x));
    }/*for(j = 0; j < k / 2; j++)*/
    for(j = k / 2; j < k; j++)
    {
        a_x = j - k / 2 + 0.5f;
        b_x =-j + k / 2 - 0.5f + k;
        w[j * 2    ] = 1.0f/a_x * ((a_x * b_x) / ( a_x + b_x));
        w[j * 2 + 1] = 1.0f/b_x * ((a_x * b_x) / ( a_x + b_x));
    }/*for(j = k / 2; j < k; j++)*/


    for(i = 0; i < sizeY; i++)
    {
      for(j = 0; j < sizeX; j++)
      {
        for(ii = 0; ii < k; ii++)
        {
          for(jj = 0; jj < k; jj++)
          {
            if ((i * k + ii > 0) &&
                (i * k + ii < height - 1) &&
                (j * k + jj > 0) &&
                (j * k + jj < width  - 1))
            {
              d = (k * i + ii) * width + (j * k + jj);
              (*map)->map[ i * stringSize + j * (*map)->numFeatures + alfa[d * 2    ]] +=
                  r[d] * w[ii * 2] * w[jj * 2];
              (*map)->map[ i * stringSize + j * (*map)->numFeatures + alfa[d * 2 + 1] + num_bins] +=
                  r[d] * w[ii * 2] * w[jj * 2];
              if ((i + nearest[ii] >= 0) &&
                  (i + nearest[ii] <= sizeY - 1))
              {
                (*map)->map[(i + nearest[ii]) * stringSize + j * (*map)->numFeatures + alfa[d * 2    ]             ] +=
                  r[d] * w[ii * 2 + 1] * w[jj * 2 ];
                (*map)->map[(i + nearest[ii]) * stringSize + j * (*map)->numFeatures + alfa[d * 2 + 1] + num_bins] +=
                  r[d] * w[ii * 2 + 1] * w[jj * 2 ];
              }
              if ((j + nearest[jj] >= 0) &&
                  (j + nearest[jj] <= sizeX - 1))
              {
                (*map)->map[i * stringSize + (j + nearest[jj]) * (*map)->numFeatures + alfa[d * 2    ]             ] +=
                  r[d] * w[ii * 2] * w[jj * 2 + 1];
                (*map)->map[i * stringSize + (j + nearest[jj]) * (*map)->numFeatures + alfa[d * 2 + 1] + num_bins] +=
                  r[d] * w[ii * 2] * w[jj * 2 + 1];
              }
              if ((i + nearest[ii] >= 0) &&
                  (i + nearest[ii] <= sizeY - 1) &&
                  (j + nearest[jj] >= 0) &&
                  (j + nearest[jj] <= sizeX - 1))
              {
                (*map)->map[(i + nearest[ii]) * stringSize + (j + nearest[jj]) * (*map)->numFeatures + alfa[d * 2    ]             ] +=
                  r[d] * w[ii * 2 + 1] * w[jj * 2 + 1];
                (*map)->map[(i + nearest[ii]) * stringSize + (j + nearest[jj]) * (*map)->numFeatures + alfa[d * 2 + 1] + num_bins] +=
                  r[d] * w[ii * 2 + 1] * w[jj * 2 + 1];
              }
            }
          }/*for(jj = 0; jj < k; jj++)*/
        }/*for(ii = 0; ii < k; ii++)*/
      }/*for(j = 1; j < sizeX - 1; j++)*/
    }/*for(i = 1; i < sizeY - 1; i++)*/

    cvReleaseImage(&dx);
    cvReleaseImage(&dy);


    free(w);
    free(nearest);

    free(r);
    free(alfa);

    return LATENT_SVM_OK;
}

static int getPathOfFeaturePyramid(IplImage * image,
                            float step, int numStep, int startIndex,
                            int sideLength, CvLSVMFeaturePyramid **maps, int num_bins)
{
    CvLSVMFeatureMap *map;
    IplImage *scaleTmp;
    float scale;
    int   i;

    for(i = 0; i < numStep; i++)
    {
        scale = 1.0f / powf(step, (float)i);
        scaleTmp = resize_opencv (image, scale);
        getFeatureMaps(scaleTmp, sideLength, &map, num_bins);
        normalizeAndTruncate(map, VAL_OF_TRUNCATE, num_bins);
        PCAFeatureMaps(map, num_bins);
        (*maps)->pyramid[startIndex + i] = map;
        cvReleaseImage(&scaleTmp);
    }/*for(i = 0; i < numStep; i++)*/
    return LATENT_SVM_OK;
}

/* add some parameters to getFeaturePyramid OpenCV API for parameters configuration */
static int getFeaturePyramid(IplImage * image, CvLSVMFeaturePyramid **maps,
                             int lambda, int num_cells, int num_bins)
{
    IplImage *imgResize;
    float step;
    int   numStep;
    int   maxNumCells;
    int   W, H;

    if(image->depth == IPL_DEPTH_32F)
    {
        imgResize = image;
    }
    else
    {
        imgResize = cvCreateImage(cvSize(image->width , image->height) ,
                                  IPL_DEPTH_32F , 3);
        cvConvert(image, imgResize);
    }

    W = imgResize->width;
    H = imgResize->height;

    step = powf(2.0f, 1.0f / ((float)lambda));
    maxNumCells = W / num_cells;
    if( maxNumCells > H / num_cells )
    {
        maxNumCells = H / num_cells;
    }
    numStep = (int)(logf((float) maxNumCells / (5.0f)) / logf( step )) + 1;

    allocFeaturePyramidObject(maps, numStep + lambda);

    getPathOfFeaturePyramid(imgResize, step   , lambda, 0,
                            num_cells / 2, maps, num_bins);
    getPathOfFeaturePyramid(imgResize, step, numStep, lambda,
                            num_cells    , maps, num_bins);
    if(image->depth != IPL_DEPTH_32F)
    {
        cvReleaseImage(&imgResize);
    }

    return LATENT_SVM_OK;
}

static CvLSVMFeaturePyramid* createFeaturePyramidWithBorder(IplImage *image,
                                                            int maxXBorder, int maxYBorder,
                                                            int lambda, int num_cells, int num_bins)
{
    int opResult;
    int bx, by;
    int level;
    CvLSVMFeaturePyramid *H;

    // Obtaining feature pyramid
    opResult = getFeaturePyramid(image, &H, lambda, num_cells, num_bins);

    if (opResult != LATENT_SVM_OK)
    {
        freeFeaturePyramidObject(&H);
        return NULL;
    } /* if (opResult != LATENT_SVM_OK) */

    // Addition nullable border for each feature map
    // the size of the border for root filters
    computeBorderSize(maxXBorder, maxYBorder, &bx, &by);
    for (level = 0; level < H->numLevels; level++)
    {
        addNullableBorder(H->pyramid[level], bx, by);
    }
    return H;
}

static CvSeq* cvLatentSvmDetectObjects(IplImage* image,
                                       CvLatentSvmDetector* detector,
                                       CvMemStorage* storage,
                                       float overlap_threshold, int numThreads,
                                       double score_threshold,
                                       int lambda,
                                       int num_cells,
                                       int num_bins)
{
    CvLSVMFeaturePyramid *H = 0;
    CvPoint *points = 0, *oppPoints = 0;
    int kPoints = 0;
    float *score = 0;
    unsigned int maxXBorder = 0, maxYBorder = 0;
    int numBoxesOut = 0;
    CvPoint *pointsOut = 0;
    CvPoint *oppPointsOut = 0;
    float *scoreOut = 0;
    CvSeq* result_seq = 0;
    int error = 0;

    if(image->nChannels == 3)
        cvCvtColor(image, image, CV_BGR2RGB);

    // Getting maximum filter dimensions
    getMaxFilterDims((const CvLSVMFilterObject**)(detector->filters), detector->num_components,
                     detector->num_part_filters, &maxXBorder, &maxYBorder);
    // Create feature pyramid with nullable border
    H = createFeaturePyramidWithBorder(image, maxXBorder, maxYBorder, lambda, num_cells, num_bins);
    
    // Search object
    error = searchObjectThresholdSomeComponents(H, (const CvLSVMFilterObject**)(detector->filters),
        detector->num_components, detector->num_part_filters, detector->b, score_threshold,
        &points, &oppPoints, &score, &kPoints, numThreads);
    if (error != LATENT_SVM_OK)
    {
        return NULL;
    }
    // Clipping boxes
    clippingBoxes(image->width, image->height, points, kPoints);
    clippingBoxes(image->width, image->height, oppPoints, kPoints);
    // NMS procedure
    nonMaximumSuppression(kPoints, points, oppPoints, score, overlap_threshold,
                &numBoxesOut, &pointsOut, &oppPointsOut, &scoreOut);

    result_seq = cvCreateSeq( 0, sizeof(CvSeq), sizeof(CvObjectDetection), storage );

    for (int i = 0; i < numBoxesOut; i++)
    {
        CvObjectDetection detection = {{0, 0, 0, 0}, 0};
        detection.score = scoreOut[i];
        CvRect bounding_box = {0, 0, 0, 0};
        bounding_box.x = pointsOut[i].x;
        bounding_box.y = pointsOut[i].y;
        bounding_box.width = oppPointsOut[i].x - pointsOut[i].x;
        bounding_box.height = oppPointsOut[i].y - pointsOut[i].y;
        detection.rect = bounding_box;
        cvSeqPush(result_seq, &detection);
    }

    if(image->nChannels == 3)
        cvCvtColor(image, image, CV_RGB2BGR);

    freeFeaturePyramidObject(&H);
    free(points);
    free(oppPoints);
    free(score);
    free(scoreOut);

    return result_seq;
}

DPMOCVCPULatentSvmDetector::DPMOCVCPULatentSvmDetector(const std::vector<std::string>& filenames)
{
    for( size_t i = 0; i < filenames.size(); i++ )
    {
        const std::string filename = filenames[i];
        if( filename.length() < 5 || filename.substr(filename.length()-4, 4) != ".xml" )
            continue;

        CvLatentSvmDetector* detector = cvLoadLatentSvmDetector( filename.c_str() );
        if( detector )
        {
            detectors.push_back( detector );
        }
    }
}

bool DPMOCVCPULatentSvmDetector::empty() const
{
    return detectors.empty();
}

void DPMOCVCPULatentSvmDetector::detect( const cv::Mat& image,
                                         std::vector<ObjectDetection>& objectDetections,
                                         float overlapThreshold,
                                         int numThreads,
                                         double score_threshold,
                                         int lambda,
                                         int num_cells,
                                         int num_bins)
{
    objectDetections.clear();
    if( numThreads <= 0 )
        numThreads = 1;

    for( size_t classID = 0; classID < detectors.size(); classID++ )
    {
        IplImage image_ipl = image;
        CvMemStorage* storage = cvCreateMemStorage(0);
        CvSeq* detections = cvLatentSvmDetectObjects( &image_ipl,
                                                      detectors[classID],
                                                      storage,
                                                      overlapThreshold,
                                                      numThreads,
                                                      score_threshold,
                                                      lambda,
                                                      num_cells,
                                                      num_bins );

        // convert results
        objectDetections.reserve( objectDetections.size() + detections->total );
        for( int detectionIdx = 0; detectionIdx < detections->total; detectionIdx++ )
        {
            CvObjectDetection detection = *(CvObjectDetection*)cvGetSeqElem( detections, detectionIdx );
            objectDetections.push_back( ObjectDetection(cv::Rect(detection.rect), detection.score, (int)classID) );
        }

        cvReleaseMemStorage( &storage );
    }
}
