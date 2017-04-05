#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include "for_use_gpu.h"
#include "resizeimg_gpu.h"
#include "featurepyramid_gpu.hpp"
#include "matching_gpu.hpp"
#include "routine_gpu.hpp"

#include <dpm_ocv.hpp>

// OpenCV non public functions
extern int freeFeatureMapObject(CvLSVMFeatureMap **obj);
extern IplImage * resize_opencv(IplImage * img, float scale);
extern int computeBorderSize(int maxXBorder, int maxYBorder, int *bx, int *by);
extern int addNullableBorder(CvLSVMFeatureMap *map, int bx, int by);

extern "C" {
extern int allocFeaturePyramidObject(CvLSVMFeaturePyramid **obj, const int countLevel);
extern int freeFeaturePyramidObject(CvLSVMFeaturePyramid **obj);
extern int getMaxFilterDims(const CvLSVMFilterObject **filters, int kComponents,
                            const int *kPartFilters,
                            unsigned int *maxXBorder, unsigned int *maxYBorder);
extern int clippingBoxes(int width, int height,
                         CvPoint *points, int kPoints);
extern int nonMaximumSuppression(int numBoxes, const CvPoint *points,
                                 const CvPoint *oppositePoints, const float *score,
                                 float overlapThreshold,
                                 int *numBoxesOut, CvPoint **pointsOut,
                                 CvPoint **oppositePointsOut, float **scoreOut);
extern int loadModel(const char *modelPath, CvLSVMFilterObject ***filters,
        int *kFilters, int *kComponents, int **kPartFilters, float **b,
        float *scoreThreshold);
};

// Original global variables
int Lambda = LAMBDA;
int Side_Length = SIDE_LENGTH;
float Val_Of_Truncate = VAL_OF_TRUNCATE;

static int convertPoints(int /*countLevel*/, int lambda, int initialImageLevel,
        CvPoint *points, int *levels, CvPoint **partsDisplacement, int kPoints,
        int n, int maxXBorder, int maxYBorder)
{
    int i, j, bx, by;
    float step, scale;
    step = powf(2.0f, 1.0f / ((float) lambda));

    computeBorderSize(maxXBorder, maxYBorder, &bx, &by);

    for (i = 0; i < kPoints; i++)
    {
        // scaling factor for root filter
        scale = Side_Length
                * powf(step, (float) (levels[i] - initialImageLevel));
        points[i].x = (int) ((points[i].x - bx + 1) * scale);
        points[i].y = (int) ((points[i].y - by + 1) * scale);

        // scaling factor for part filters
        scale = Side_Length
                * powf(step, (float) (levels[i] - lambda - initialImageLevel));
        for (j = 0; j < n; j++)
        {
            partsDisplacement[i][j].x = (int) ((partsDisplacement[i][j].x
                    - 2 * bx + 1) * scale);
            partsDisplacement[i][j].y = (int) ((partsDisplacement[i][j].y
                    - 2 * by + 1) * scale);
        }
    }
    return LATENT_SVM_OK;
}

/*
 // load trained detector from a file
 //
 // API
 // CvLatentSvmDetector* cvLoadLatentSvmDetector(const char* filename);
 // INPUT
 // filename             - path to the file containing the parameters of
 //                      - trained Latent SVM detector
 // OUTPUT
 // trained Latent SVM detector in internal representation
 */
static CvLatentSvmDetector* cvLoadLatentSvmDetector(const char* filename,
        const float scoreThreshold)
{
    CvLatentSvmDetector* detector = 0;
    CvLSVMFilterObject** filters = 0;
    int kFilters = 0;
    int kComponents = 0;
    int* kPartFilters = 0;
    float* b = 0;
    float modelsScoreThreshold = 0.f;
    int err_code = 0;

#ifdef PROFILE
    TickMeter tm;
    tm.start();
    cout << "Loading model START" << endl;
#endif
    err_code = loadModel(filename, &filters, &kFilters, &kComponents,
            &kPartFilters, &b, &modelsScoreThreshold);
#ifdef PROFILE
    tm.stop();
    cout << "Loading model time = " << tm.getTimeSec() << " sec" << endl;
#endif
    if (err_code != LATENT_SVM_OK)
        return 0;

    detector = (CvLatentSvmDetector*) malloc(sizeof(CvLatentSvmDetector));
    detector->filters = filters;
    detector->b = b;
    detector->num_components = kComponents;
    detector->num_filters = kFilters;
    detector->num_part_filters = kPartFilters;
    detector->score_threshold = modelsScoreThreshold;
    if (scoreThreshold != 0.0f)
    {
        detector->score_threshold = scoreThreshold;
    }

    return detector;
}

static std::string extractModelName(const std::string& filename)
{
    size_t startPos = filename.rfind('/');
    if (startPos == std::string::npos)
        startPos = filename.rfind('\\');

    if (startPos == std::string::npos)
        startPos = 0;
    else
        startPos++;

    const int extentionSize = 4; //.xml

    int substrLength = (int) (filename.size() - startPos - extentionSize);

    return filename.substr(startPos, substrLength);
}


bool DPMOCVGPULatentSvmDetector::load(const std::vector<std::string>& filenames, const float test_t)
{
    clear();

    for (size_t i = 0; i < filenames.size(); i++)
    {
        const std::string filename = filenames[i];
        if (filename.length() < 5
                || filename.substr(filename.length() - 4, 4) != ".xml")
            continue;

        CvLatentSvmDetector* detector = cvLoadLatentSvmDetector(
                filename.c_str(), test_t);
        if (detector)
        {
            detectors.push_back(detector);
            classNames.push_back(extractModelName(filenames[i]));
        }
    }

    return !empty();
}

DPMOCVGPULatentSvmDetector::DPMOCVGPULatentSvmDetector(const std::vector<std::string>& filenames,
        const float scoreThreshold)
{
    load(filenames, scoreThreshold);
    init_cuda();
}

DPMOCVGPULatentSvmDetector::~DPMOCVGPULatentSvmDetector()
{
    clear();
    clean_cuda();
}

bool DPMOCVGPULatentSvmDetector::empty() const
{
    return detectors.empty();
}

static CvLSVMFeaturePyramid* createFeaturePyramidWithBorder(IplImage *image,
    int maxXBorder, int maxYBorder)
{
    int opResult;
    int bx, by;
    CvLSVMFeaturePyramid *H;

    // the size of the border for root filters
    computeBorderSize(maxXBorder, maxYBorder, &bx, &by);

    // Obtaining feature pyramid
    opResult = getFeaturePyramid(image, &H, bx, by);

    if (opResult != LATENT_SVM_OK)
    {
        freeFeaturePyramidObject(&H);
        return NULL;
    } /* if (opResult != LATENT_SVM_OK) */

    return H;
}

static int searchObjectThreshold(const CvLSVMFeaturePyramid *H,
    const CvLSVMFilterObject **all_F, int n, float b,
    CvLSVMFeatureMap *map[], int maxXBorder, int maxYBorder,
    float scoreThreshold, CvPoint **points, int **levels, int *kPoints,
    float **score, CvPoint ***partsDisplacement, int numThreads)
{
    int opResult;

    opResult = thresholdFunctionalScore(all_F, n, H, b, map, scoreThreshold,
            score, points, levels, kPoints, partsDisplacement);

    (void) numThreads;

    if (opResult != LATENT_SVM_OK)
    {
        return LATENT_SVM_SEARCH_OBJECT_FAILED;
    }

    // Transformation filter displacement from the block space
    // to the space of pixels at the initial image
    // that settles at the level number Lambda
    convertPoints(H->numLevels, Lambda, Lambda, (*points), (*levels),
            (*partsDisplacement), (*kPoints), n, maxXBorder, maxYBorder);

    return LATENT_SVM_OK;
}

static int getOppositePoint(CvPoint point, int sizeX, int sizeY, float step,
    int degree, CvPoint *oppositePoint)
{
    float scale;
    scale = Side_Length * powf(step, (float) degree);
    oppositePoint->x = (int) (point.x + sizeX * scale);
    oppositePoint->y = (int) (point.y + sizeY * scale);
    return LATENT_SVM_OK;
}

static int estimateBoxes(CvPoint *points, int *levels, int kPoints, int sizeX,
    int sizeY, CvPoint **oppositePoints)
{
    int i;
    float step;

    step = powf(2.0f, 1.0f / ((float) (Lambda)));

    *oppositePoints = (CvPoint *) malloc(sizeof(CvPoint) * kPoints);
    for (i = 0; i < kPoints; i++)
    {
        getOppositePoint(points[i], sizeX, sizeY, step, levels[i] - Lambda,
            &((*oppositePoints)[i]));
    }
    return LATENT_SVM_OK;
}

static int searchObjectThresholdSomeComponents(const CvLSVMFeaturePyramid *H,
    const CvLSVMFilterObject **filters, int kComponents,
    const int *kPartFilters, const float *b, float scoreThreshold,
    CvPoint **points, CvPoint **oppPoints, float **score, int *kPoints,
    int numThreads)
{
    //int error = 0;
    int i, j, s, f, componentIndex;
    unsigned int maxXBorder, maxYBorder;
    CvPoint **pointsArr, **oppPointsArr, ***partsDisplacementArr;
    float **scoreArr;
    int *kPointsArr, **levelsArr;

    // Allocation memory
    pointsArr = (CvPoint **) malloc(sizeof(CvPoint *) * kComponents);
    oppPointsArr = (CvPoint **) malloc(sizeof(CvPoint *) * kComponents);
    scoreArr = (float **) malloc(sizeof(float *) * kComponents);
    kPointsArr = (int *) malloc(sizeof(int) * kComponents);
    levelsArr = (int **) malloc(sizeof(int *) * kComponents);
    partsDisplacementArr = (CvPoint ***) malloc(sizeof(CvPoint **) * kComponents);

    // Getting maximum filter dimensions
    getMaxFilterDims(filters, kComponents, kPartFilters, &maxXBorder,
            &maxYBorder);

    CvLSVMFeatureMap *map[H->numLevels - Lambda];

    for (i = 0; i < H->numLevels - Lambda; i++)
    {
        map[i] = featureMapBorderPartFilter(H->pyramid[i], maxXBorder,
                                            maxYBorder);
    }

    componentIndex = 0;
    *kPoints = 0;
    // For each component perform searching
    for (i = 0; i < kComponents; i++)
    {
        int error = searchObjectThreshold(H, &(filters[componentIndex]),
                kPartFilters[i], b[i], map, maxXBorder, maxYBorder,
                scoreThreshold, &(pointsArr[i]), &(levelsArr[i]),
                &(kPointsArr[i]), &(scoreArr[i]), &(partsDisplacementArr[i]),
                numThreads);

        if (error != LATENT_SVM_OK)
        {
            // Release allocated memory
            free(pointsArr);
            free(oppPointsArr);
            free(scoreArr);
            free(kPointsArr);
            free(levelsArr);
            free(partsDisplacementArr);
            return LATENT_SVM_SEARCH_OBJECT_FAILED;
        }

        estimateBoxes(pointsArr[i], levelsArr[i], kPointsArr[i],
                filters[componentIndex]->sizeX, filters[componentIndex]->sizeY,
                &(oppPointsArr[i]));
        componentIndex += (kPartFilters[i] + 1);
        *kPoints += kPointsArr[i];
    }
    for (i = 0; i < H->numLevels - Lambda; i++)
    {
        freeFeatureMapObject(&map[i]);
    }
    //freeFeatureMapObject(map);
    *points = (CvPoint *) malloc(sizeof(CvPoint) * (*kPoints));
    *oppPoints = (CvPoint *) malloc(sizeof(CvPoint) * (*kPoints));
    *score = (float *) malloc(sizeof(float) * (*kPoints));
    s = 0;
    for (i = 0; i < kComponents; i++)
    {
        f = s + kPointsArr[i];
        for (j = s; j < f; j++)
        {
            (*points)[j].x = pointsArr[i][j - s].x;
            (*points)[j].y = pointsArr[i][j - s].y;
            (*oppPoints)[j].x = oppPointsArr[i][j - s].x;
            (*oppPoints)[j].y = oppPointsArr[i][j - s].y;
            (*score)[j] = scoreArr[i][j - s];
        }
        s = f;
    }

    // Release allocated memory
    for (i = 0; i < kComponents; i++)
    {
        free(pointsArr[i]);
        free(oppPointsArr[i]);
        free(scoreArr[i]);
        free(levelsArr[i]);
        for (j = 0; j < kPointsArr[i]; j++)
        {
            free(partsDisplacementArr[i][j]);
        }
        free(partsDisplacementArr[i]);
    }
    free(pointsArr);
    free(oppPointsArr);
    free(scoreArr);
    free(kPointsArr);
    free(levelsArr);
    free(partsDisplacementArr);
    return LATENT_SVM_OK;
}

static CvSeq* cvLatentSvmDetectObjectsGPU(IplImage* image, CvLatentSvmDetector* detector,
    CvMemStorage* storage, float overlap_threshold, int numThreads)
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

    if (image->nChannels == 3)
        cvCvtColor(image, image, CV_BGR2RGB);

    // Getting maximum filter dimensions
    getMaxFilterDims((const CvLSVMFilterObject**) (detector->filters),
                detector->num_components, detector->num_part_filters, &maxXBorder,
                &maxYBorder);
    // Create feature pyramid with nullable border
    H = createFeaturePyramidWithBorder(image, maxXBorder, maxYBorder);

    error = searchObjectThresholdSomeComponents(H,
                                            (const CvLSVMFilterObject**) (detector->filters),
                                            detector->num_components, detector->num_part_filters, detector->b,
                                            detector->score_threshold, &points, &oppPoints, &score, &kPoints,
                                            numThreads);

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

    result_seq = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvObjectDetection),
                             storage);

    for (int i = 0; i < numBoxesOut; i++)
    {
        CvObjectDetection detection = {{ 0, 0, 0, 0 }, 0 };
        detection.score = scoreOut[i];
        CvRect bounding_box = { 0, 0, 0, 0 };
        bounding_box.x = pointsOut[i].x;
        bounding_box.y = pointsOut[i].y;
        bounding_box.width = oppPointsOut[i].x - pointsOut[i].x;
        bounding_box.height = oppPointsOut[i].y - pointsOut[i].y;
        detection.rect = bounding_box;
        cvSeqPush(result_seq, &detection);
    }

    if (image->nChannels == 3)
        cvCvtColor(image, image, CV_RGB2BGR);

    freeFeaturePyramidObject(&H);
    free(points);
    free(pointsOut);
    free(oppPointsOut);
    free(oppPoints);
    free(score);
    free(scoreOut);

    return result_seq;
}

void DPMOCVGPULatentSvmDetector::detect(const cv::Mat& image,
        std::vector<ObjectDetection>& objectDetections, float overlapThreshold,
        int numThreads, const unsigned int lambda,
        const unsigned int sideLength, const float valOfTruncate)
{
    Lambda = lambda;
    Side_Length = sideLength;
    Val_Of_Truncate = valOfTruncate;

    objectDetections.clear();
    if (numThreads <= 0)
        numThreads = 1;
    for (size_t classID = 0; classID < detectors.size(); classID++)
    {
        IplImage image_ipl = image;
        CvMemStorage* storage = cvCreateMemStorage(0);
        CvSeq* detections = cvLatentSvmDetectObjectsGPU(&image_ipl,
                detectors[classID], storage, overlapThreshold, numThreads);

        // convert results
        objectDetections.reserve(objectDetections.size() + detections->total);
        for (int detectionIdx = 0; detectionIdx < detections->total;
                detectionIdx++)
        {
            CvObjectDetection detection = *(CvObjectDetection*) cvGetSeqElem(
                    detections, detectionIdx);
            objectDetections.push_back(
                    ObjectDetection(cv::Rect(detection.rect), detection.score,
                            (int) classID));
        }

        cvReleaseMemStorage(&storage);
    }
}
