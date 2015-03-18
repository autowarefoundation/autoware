/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef SELECTIONWIDGET_H
#define SELECTIONWIDGET_H

#include<qlabel.h>
#include<QVector>
#include<qrgb.h>
#include<QMouseEvent>

#include<opencv2/opencv.hpp>

#include<Eigen/Dense>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree_flann.h>

#include<sensor_msgs/Image.h>
#include<sensor_msgs/PointCloud2.h>

#include<glviewer.h>

class PlaneExtractor : public GLViewer
{
    Q_OBJECT
public:
    PlaneExtractor(sensor_msgs::PointCloud2ConstPtr velodynePoints, int id, double neighborRadius=0.2, double distanceThreshold=0.05, QWidget * parent=0);
    PlaneExtractor(pcl::PointCloud<pcl::PointXYZI>::Ptr velodynePoints, int id, double neighborRadius=0.2, double distanceThreshold=0.05, QWidget * parent=0);
protected:
    int pointsid;
    double neighborradius;
    double distance;
    sensor_msgs::PointCloud2ConstPtr pointsptr;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    GLuint pointsdisplaylist;
    GLuint selecteddisplaylist;
    GLuint mousedisplaylist;
    bool extracted;
protected slots:
    void mousePositionSlot(QMouseEvent * event, CAMERAPARAMETERS * parameters);
signals:
    void extractionResultSignal(pcl::PointCloud<pcl::PointXYZI>::Ptr extraction, cv::Mat normal, int id);
protected:
    void drawMouse(Eigen::Vector3d point, Eigen::Vector3d norm, Eigen::Vector3d direction,Eigen::Vector3d nearestpoint);
    void extractPlane(Eigen::Vector3d seed, Eigen::Matrix3d eigenvectors);
};


#endif // SELECTIONWIDGET_H
