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
