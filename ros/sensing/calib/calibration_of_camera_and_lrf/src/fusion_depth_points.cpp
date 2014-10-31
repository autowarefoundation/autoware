#include "cv.h"
#include "highgui.h"
#include "cxcore.h"
#ifdef _DEBUG
    //Debugモードの場合
#pragma comment(lib,"cv200d.lib")
#pragma comment(lib,"cxcore200d.lib")
#pragma comment(lib,"cvaux200d.lib")
#pragma comment(lib,"highgui200d.lib")
#else
//Releaseモードの場合
#pragma comment(lib,"cv200.lib")
#pragma comment(lib,"cxcore200.lib")
#pragma comment(lib,"cvaux200.lib")
#pragma comment(lib,"highgui200.lib")
#endif
//C++ library
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/resource.h>

//Header files
#include "Depth_points_func.h"
#include "MODEL_info.h"		//File information
#include "Laser_info.h"
#include "Common.h"

void trans_depth_points_to_image_points(IplImage *Image, volatile Three_dimensional_vector* depth_points, volatile Two_dimensional_vector* image_points, double *distance);
void show_depth_points(IplImage* Image, Two_dimensional_vector* image_points);
double get_processing_time(struct timespec start, struct timespec end);
void write_file_processing_time(struct timespec start, struct timespec end);
void init_depth_points(Three_dimensional_vector depth_points);
void print_all_point(double *x_src, double *y_src, double *z_src, double *x_dst, double *y_dst, double *z_dst, double *u, double *v);
void show_distance(IplImage* Image, RESULT* CUR, Two_dimensional_vector* image_points, double* distance);


void show_distance(IplImage* Image, RESULT* CUR, Two_dimensional_vector* image_points, double* distance)
{
    int i, j;
	for(i = 0; i < CUR->num; i++)
	{
	   	//int *P = CUR->point+4*i;
	 	int *P = CUR->OR_point+4*i;
        CvPoint p1=cvPoint(P[0],P[1]);
        CvPoint p2=cvPoint(P[2],P[3]);
        double obstacle_distance = -1;
        char distance_string[32];
        CvScalar col = cvScalar(0.0, 0.0, 255.0);

        printf("%d, %d, %d, %d\n", P[0], P[1], P[2], P[3]);
        for(j = 0; j < SCAN_POINT_NUM; j++) {
            if(image_points->x[j] > P[0] && image_points->x[j] < P[2]) {
                if(image_points->y[j] > P[1] && image_points->y[j] < P[3]) {
                    if(obstacle_distance > distance[j] || obstacle_distance == -1) {
                        obstacle_distance = distance[j];
                    }
                }
            }
        }
        CvFont dfont;
        float hscale      = 1.0f;
        float vscale      = 1.0f;
        float italicscale = 0.0f;
        int  thickness    = 2;

        if(obstacle_distance != -1) {
            cvInitFont (&dfont, CV_FONT_HERSHEY_SIMPLEX , hscale, vscale, italicscale, thickness, CV_AA);
            sprintf(distance_string, "%d mm", (int)floor(obstacle_distance + 0.5));
            cvPutText(Image, distance_string, cvPoint(P[0] , P[3]), &dfont, CV_RGB(255, 0, 0));
        } else {
            cvInitFont (&dfont, CV_FONT_HERSHEY_SIMPLEX , hscale, vscale, italicscale, thickness, CV_AA);
            sprintf(distance_string, "No data");
            cvPutText(Image, distance_string, cvPoint(P[0] , P[3]), &dfont, CV_RGB(255, 0, 0));
        }
    }
}



//show laser_range_point(yukky)
void show_depth_points(IplImage *Image, Two_dimensional_vector* image_points)
{
    CvSeq *points;
    CvPoint pt;
    CvMemStorage *storage = cvCreateMemStorage (0);
    int i;

    //画像に点群データをプロット
    points = cvCreateSeq (CV_SEQ_ELTYPE_POINT, sizeof (CvSeq), sizeof (CvPoint), storage);
    for (i = 0; i < SCAN_POINT_NUM; i++) {
        if(0 > image_points->x[i] || image_points->x[i] > Image->width) {
            continue;
        }
        if(0 > image_points->y[i] || image_points->y[i] > Image->height) {
            continue;
        }
        pt.x = image_points->x[i];
        pt.y = image_points->y[i];
        cvSeqPush (points, &pt);
        cvCircle (Image, pt, 2, CV_RGB (0, 255, 0), CV_FILLED, 8, 0);
    }
}

void trans_depth_points_to_image_points(IplImage* Image, volatile Three_dimensional_vector* depth_points, volatile Two_dimensional_vector* image_points, double *distance) {
    struct timespec start, end;
    double x_dst[SCAN_POINT_NUM];
    double y_dst[SCAN_POINT_NUM];
    double z_dst[SCAN_POINT_NUM];
    int i;

    clock_gettime(CLOCK_REALTIME,&start);

    for(i = 0; i < SCAN_POINT_NUM; i++) {
        /*
         * 座標変換
         * LRF座標系からカメラ座標系への変換
         */
/*
        x_dst[i] = depth_points->x[i] * (cos(X_THETA / 180.0 * M_PI) * cos(Y_THETA / 180.0 * M_PI) * cos(Z_THETA / 180.0 * M_PI) - sin(X_THETA / 180.0 * M_PI) * sin(Z_THETA / 180.0 * M_PI)) + depth_points->y[i] * (-1.0 * cos(X_THETA / 180.0 * M_PI) * cos(Y_THETA / 180.0 * M_PI) * sin(Z_THETA / 180.0 * M_PI) - sin(X_THETA / 180.0 * M_PI) * cos(Z_THETA / 180.0 * M_PI)) + depth_points->z[i] * (cos(X_THETA / 180.0 * M_PI) * sin(Y_THETA / 180.0 * M_PI));

        y_dst[i] = depth_points->x[i] * (sin(X_THETA / 180.0 * M_PI) * cos(Y_THETA / 180.0 * M_PI) * cos(Z_THETA / 180.0 * M_PI) + cos(X_THETA / 180.0 * M_PI) * sin(Z_THETA / 180.0 * M_PI)) + depth_points->y[i] * (-1.0 * sin(X_THETA / 180.0 * M_PI) * cos(Y_THETA / 180.0 * M_PI) * sin(Z_THETA / 180.0 * M_PI) + cos(X_THETA / 180.0 * M_PI) * cos(Z_THETA / 180.0 * M_PI)) + depth_points->z[i] * (sin(X_THETA / 180.0 * M_PI) * sin(Y_THETA / 180.0 * M_PI));

        z_dst[i] = depth_points->x[i] * (-1.0 * sin(Y_THETA / 180.0 * M_PI) * cos(Z_THETA / 180.0 * M_PI)) + depth_points->y[i] * (sin(Y_THETA / 180.0 * M_PI) * sin(Z_THETA / 180.0 * M_PI)) + depth_points->z[i] * (cos(Y_THETA / 180.0 * M_PI));

*/
        //X軸回転
        y_dst[i] = depth_points->y[i] * cos(X_THETA / 180.0 * M_PI) - depth_points->z[i] * sin(X_THETA / 180.0 * M_PI);
        z_dst[i] = depth_points->y[i] * sin(X_THETA / 180.0 * M_PI) + depth_points->z[i] * cos(X_THETA / 180.0 * M_PI);

        //Y軸回転
        x_dst[i] = depth_points->x[i] * cos(Y_THETA / 180.0 * M_PI) + z_dst[i] * sin(Y_THETA / 180.0 * M_PI);
        z_dst[i] = -1.0 * depth_points->x[i] * sin(Y_THETA / 180.0 * M_PI) + z_dst[i] * cos(Y_THETA / 180.0 * M_PI);

        //Z軸回転
        x_dst[i] = x_dst[i] * cos(Z_THETA / 180.0 * M_PI) - y_dst[i] * sin(Z_THETA / 180.0 * M_PI);
        y_dst[i] = x_dst[i] * sin(Z_THETA / 180.0 * M_PI) + y_dst[i] * cos(Z_THETA / 180.0 * M_PI);

        //平行移動
        x_dst[i] = x_dst[i] + X_VECTOR;
        y_dst[i] = y_dst[i] + Y_VECTOR;
        z_dst[i] = z_dst[i] + Z_VECTOR;

        /*
         * ユークリッド距離算出（カメラと障害物）
         */
        distance[i] = sqrt(x_dst[i] * x_dst[i] + y_dst[i] * y_dst[i] + z_dst[i] * z_dst[i]);
        /*
         * 投影変換
         */
        if (z_dst[i] > 0.0) {
            image_points->x[i] = x_dst[i] * 1000 / z_dst[i] + (Image->width / 2);
            image_points->y[i] = y_dst[i] * 1000 / z_dst[i] + (Image->height / 2);
//  image_points->x[i] = FX_CAMERA_PARAM * x_dst[i] + CX_CAMERA_PARAM * z_dst[i] + (Image->width / 2);
//  image_points->y[i] = FY_CAMERA_PARAM * y_dst[i] + CY_CAMERA_PARAM * z_dst[i] + (Image->height / 2);
        } else {
            image_points->x[i] = -1;
            image_points->y[i] = -1;
        }
    }
    clock_gettime(CLOCK_REALTIME,&end);
    printf("\ntime:%f msec\n", get_processing_time(start, end));
}

double get_processing_time(struct timespec start, struct timespec end)
{
    long sec, nsec;
    sec =  end.tv_sec - start.tv_sec;
    nsec =  end.tv_nsec - start.tv_nsec;
    if(nsec < 0) {
        sec--;
        nsec += 1000000000L;
    }
    return (double)sec * 100 + (double)nsec/1000000;
}

void write_file_processing_time(struct timespec start, struct timespec end)
{
    FILE *fp;
    fp = fopen("./result.txt", "a+");
    if(fp == NULL){
        printf("%s : cannot open file\n", "result.txt");
        exit(EXIT_FAILURE);
    }
    fprintf(fp,"%lf\n", get_processing_time(start, end));
    fclose(fp);
}


void print_all_point(double *x_src, double *y_src, double *z_src, double *x_dst, double *y_dst, double *z_dst, double *u, double *v) {
    int i;
    for(i = 0; i < SCAN_POINT_NUM; i++) {
        printf("変換前:%f, %f, %f\n変換後:%f, %f, %f\nカメラ上の点:%f, %f\n", x_src[i], y_src[i], z_src[i], x_dst[i], y_dst[i], z_dst[i], u[i], v[i]);
    }
}

void init_depth_points(Three_dimensional_vector *depth_points) {

    /*
     * x yoko, y tate, z okuyuki
     */

    int i;

    for (i = 0; i < SCAN_POINT_NUM; i++) {
        depth_points->x[i] = (double)500.0 * cos(i * 2.0 * M_PI / SCAN_POINT_NUM);
        depth_points->y[i] = 0.0;
        depth_points->z[i] = (double)500.0 * sin(i * 2.0 * M_PI / SCAN_POINT_NUM);
    }
/*
    FILE *fpx = NULL;
    FILE *fpy = NULL;
    FILE *fpz = NULL;
    char buffer_x[READ_POINT_BUFFER_SIZE];
    char buffer_y[READ_POINT_BUFFER_SIZE];
    char buffer_z[READ_POINT_BUFFER_SIZE];
    char temp = 0;
    int i = 0;
    long int j = 0;

    fpx = fopen("./x.txt", "r");
    if(fpx == NULL){
        printf("%s : cannot open file\n", "x.txt");
        exit(EXIT_FAILURE);
    }

    fpy = fopen("./y.txt", "r");
    if(fpy == NULL){
        printf("%s : cannot open file\n", "y.txt");
        exit(EXIT_FAILURE);
    }
    fpz = fopen("./z.txt", "r");
    if(fpz == NULL){
        printf("%s : cannot open file\n", "z.txt");
        exit(EXIT_FAILURE);
    }

    for(j = 0; j < SCAN_POINT_NUM; j++) {
        for(i = 0; temp != EOF && i < READ_POINT_BUFFER_SIZE - 1;) {
            temp = fgetc(fpx);
            if(temp == '[' || temp == ' ' || temp == '\n') {
                continue;
            }
            if(temp == ',' || temp == ']') {
                buffer_x[i] = '\0';
                break;
            }
            buffer_x[i] = temp;
            i++;
        }
        if(temp == EOF) {
            printf("点数がオーバーしています\n");
            exit(EXIT_FAILURE);
        }

        for(i = 0; temp != EOF && i < READ_POINT_BUFFER_SIZE - 1;) {
            temp = fgetc(fpy);
            if(temp == '[' || temp == ' ' || temp == '\n') {
                continue;
            }
            if(temp == ',' || temp == ']') {
                buffer_y[i] = '\0';
                break;
            }
            buffer_y[i] = temp;
            i++;
        }
        if(temp == EOF) {
            printf("点数がオーバーしています\n");
            exit(EXIT_FAILURE);
        }

        for(i = 0; temp != EOF && i < READ_POINT_BUFFER_SIZE - 1;) {
            temp = fgetc(fpz);
            if(temp == '[' || temp == ' ' || temp == '\n') {
                continue;
            }
            if(temp == ',' || temp == ']') {
                buffer_z[i] = '\0';
                break;
            }
            buffer_z[i] = temp;
            i++;
        }
        if(temp == EOF) {
            printf("点数がオーバーしています\n");
            exit(EXIT_FAILURE);
        }
        depth_points->x[j] = (double)atoi(buffer_x);
        depth_points->y[j] = (double)atoi(buffer_y);
//        depth_points->y[j] = 0.0;
        depth_points->z[j] = (double)atoi(buffer_z);
    }

    fclose(fpx);
    fclose(fpy);
    fclose(fpz);
*/
}
