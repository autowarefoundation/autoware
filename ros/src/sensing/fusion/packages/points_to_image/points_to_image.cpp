#include "points_to_image.h"


Points_to_image::Points_to_image()
{
}


cv::Mat invR=cameraextrinsicmat(cv::Rect(0,0,3,3)).t();
cv::Mat invT=-invR*(cameraextrinsicmat(cv::Rect(3,0,1,3)));
int i,n=calibvelodynespoints.size();
for(i=0;i<n;i++)
{
    cv::Mat tmpimage=calibimages[i].clone();
    int j,m=calibvelodynespoints[i]->points.size();
    cv::Mat camerapoints(m,3,CV_64F);
    for(j=0;j<m;j++)
    {
        camerapoints.at<double>(j,0)=double(calibvelodynespoints[i]->points[j].x);
        camerapoints.at<double>(j,1)=double(calibvelodynespoints[i]->points[j].y);
        camerapoints.at<double>(j,2)=double(calibvelodynespoints[i]->points[j].z);
    }
    camerapoints=camerapoints*invR.t()+cv::Mat::ones(m,1,CV_64F)*invT.t();
    cv::vector<cv::Point2d> planepoints;
    planepoints.resize(m);
    for(j=0;j<m;j++)
    {
        double tmpx=camerapoints.at<double>(j,0)/camerapoints.at<double>(j,2);
        double tmpy=camerapoints.at<double>(j,1)/camerapoints.at<double>(j,2);
        double r2=tmpx*tmpx+tmpy*tmpy;
        double tmpdist=1+distcoeff.at<double>(0)*r2+distcoeff.at<double>(1)*r2*r2+distcoeff.at<double>(4)*r2*r2*r2;
        planepoints[j].x=tmpx*tmpdist+2*distcoeff.at<double>(2)*tmpx*tmpy+distcoeff.at<double>(3)*(r2+2*tmpx*tmpx);
        planepoints[j].y=tmpy*tmpdist+distcoeff.at<double>(2)*(r2+2*tmpy*tmpy)+2*distcoeff.at<double>(3)*tmpx*tmpy;
        planepoints[j].x=cameramat.at<double>(0,0)*planepoints[j].x+cameramat.at<double>(0,2);
        planepoints[j].y=cameramat.at<double>(1,1)*planepoints[j].y+cameramat.at<double>(1,2);
        cv::circle(tmpimage,planepoints[j],2,cv::Scalar(0,0,255));
    }
    if(tmpimage.type()==CV_8UC3)
    {
        QImage img(tmpimage.data, tmpimage.cols, tmpimage.rows, tmpimage.step, QImage::Format_RGB888);
        img=img.rgbSwapped();
        QScrollArea * scrollarea=(QScrollArea *)(calibimagesshow->widget(i));
        QLabel * tmpcalibimageshow=(QLabel *)(scrollarea->widget());
        tmpcalibimageshow->setPixmap(QPixmap::fromImage(img));
    }
    else if(tmpimage.type()==CV_8UC1)
    {
        QImage img(tmpimage.data, tmpimage.cols, tmpimage.rows, tmpimage.step, QImage::Format_Indexed8);
        img.setColorTable(colorTable);
        QScrollArea * scrollarea=(QScrollArea *)(calibimagesshow->widget(i));
        QLabel * tmpcalibimageshow=(QLabel *)(scrollarea->widget());
        tmpcalibimageshow->setPixmap(QPixmap::fromImage(img));
    }
}
