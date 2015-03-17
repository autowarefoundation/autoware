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

#ifndef GLVIEWER_H
#define GLVIEWER_H

#include <qwidget.h>
#include <qobject.h>
#include <Eigen/Dense>
#include <qopengl.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <QGLWidget>
#include <qqueue.h>
#include <qthread.h>
#include <qreadwritelock.h>
#include <QKeyEvent>
#include <QColorDialog>

struct CAMERAPARAMETERS
{
    double viewAngle;
    double viewheight;
    double minView, maxView;
    int width,height;
    Eigen::Vector4d background;
	GLfloat lightambient[4];
    Eigen::Matrix4d transform;
	GLfloat eye[4];
    double tspeed,rspeed;
    int pointsize;
};

struct DISPLAYLIST
{
    bool show;
    GLuint listid;
	Eigen::Matrix4d transform;
	Eigen::Matrix4d scale;
};

class GLViewer : public QGLWidget
{
    Q_OBJECT
public:
    explicit GLViewer(QWidget *parent = 0);
    ~GLViewer();
protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void keyPressEvent(QKeyEvent * event);
    void mousePressEvent(QMouseEvent * event);
    void mouseReleaseEvent(QMouseEvent * event);
    void wheelEvent(QWheelEvent * event);
    void mouseMoveEvent(QMouseEvent * event);
signals:
    void mousePositionSignal(QMouseEvent * event, CAMERAPARAMETERS * parameters);
private:
    CAMERAPARAMETERS cameraparameters;
    std::vector<DISPLAYLIST> displaylist;
    bool bperspective;
private:
    void setProjection();
public:
    void addDisplayList(GLuint listid);
    void addDisplayLists(GLuint listid, GLuint num);
    void enableShow(GLuint listid, bool show, bool islistid=0);
    void deleteDisplayList(GLuint listid, bool islistid=0);
    void clearDisplayList();
    int listSize();
    void setCameraPose(Eigen::Matrix4d transform);
    Eigen::Matrix4d getCameraPose();
    void setBackground(QColor color);
	void setDisplayListScale(GLuint listid, double sx, double sy, double sz, bool islistid=1);
	void setDisplayListRotation(GLuint listid, double rx, double ry, double rz, bool islistid=1);
	void setDisplayListTranslation(GLuint listid, double tx, double ty, double tz, bool islistid=1);
	void setDisplayListTransform(GLuint listid, Eigen::Matrix4d transform, bool islistid=1);
};

#endif
