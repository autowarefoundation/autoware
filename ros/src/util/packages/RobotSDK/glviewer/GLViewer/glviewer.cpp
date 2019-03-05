/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "glviewer/glviewer.h"

#define GL_PI 3.1415926535897932384626433832795

GLViewer::GLViewer(QWidget *parent) :
    QGLWidget(parent)
{
    cameraparameters.viewAngle=60;
    cameraparameters.viewheight=50;
    cameraparameters.minView=0.001;
    cameraparameters.maxView=1000;
    cameraparameters.width=0;
    cameraparameters.height=0;
    cameraparameters.background=Eigen::Vector4d(0,0,0,1);
    cameraparameters.lightambient[0]=1.0;
    cameraparameters.lightambient[1]=1.0;
    cameraparameters.lightambient[2]=1.0;
    cameraparameters.lightambient[3]=1.0;
    cameraparameters.transform.setIdentity();
    cameraparameters.tspeed=10;
    cameraparameters.rspeed=1;
    cameraparameters.pointsize=1;
    bperspective=1;
    setMouseTracking(1);
}

GLViewer::~GLViewer()
{
    makeCurrent();
    int i,n=displaylist.size();
    for(i=0;i<n;i++)
    {
        glDeleteLists(displaylist[i].listid,1);
    }
    displaylist.clear();
}


void GLViewer::initializeGL()
{
    glShadeModel(GL_FLAT);
    glClearColor(cameraparameters.background(0),cameraparameters.background(1),cameraparameters.background(2),cameraparameters.background(3));
    glClearDepth(1.0);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    return;
}

void GLViewer::paintGL()
{
    Eigen::Vector4d eye=cameraparameters.transform*Eigen::Vector4d(0,0,0,1);
    Eigen::Vector4d center=cameraparameters.transform*Eigen::Vector4d(0,0,-1,1);
    Eigen::Vector4d up=cameraparameters.transform*Eigen::Vector4d(0,1,0,0);
    makeCurrent();
    cameraparameters.eye[0]=eye(0);
    cameraparameters.eye[1]=eye(1);
    cameraparameters.eye[2]=eye(2);
    cameraparameters.eye[3]=eye(3);
    glLightfv(GL_LIGHT0, GL_POSITION,cameraparameters.eye);
    glLightfv(GL_LIGHT0, GL_AMBIENT, cameraparameters.lightambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,cameraparameters.lightambient);
    glLightfv(GL_LIGHT0, GL_SPECULAR,cameraparameters.lightambient);
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, cameraparameters.lightambient);
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER,GL_TRUE);
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,GL_TRUE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(cameraparameters.background(0),cameraparameters.background(1),cameraparameters.background(2),0.0);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
    gluLookAt(eye(0),eye(1),eye(2),center(0),center(1),center(2),up(0),up(1),up(2));
	glPushMatrix();
    int i,n=displaylist.size();
    glPointSize(cameraparameters.pointsize);
    for(i=0;i<n;i++)
    {
        if(displaylist[i].show)
        {
			glPushMatrix();
			Eigen::Matrix4d matrix=displaylist[i].scale*displaylist[i].transform;
			glMultMatrixd(matrix.data());
            glCallList(displaylist[i].listid);
			glPopMatrix();
        }
    }
	glPopMatrix();
    glBegin(GL_LINES);
    glColor4d(1,0,0,1);
    glVertex3d(0,0,0);glVertex3d(1,0,0);
    glColor4d(0,1,0,1);
    glVertex3d(0,0,0);glVertex3d(0,1,0);
    glColor4d(0,0,1,1);
    glVertex3d(0,0,0);glVertex3d(0,0,1);
    glEnd();
    return;
}

void GLViewer::setProjection()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if(bperspective)
    {
        gluPerspective((GLdouble)cameraparameters.viewAngle,(GLfloat)cameraparameters.width/(GLfloat)cameraparameters.height,(GLdouble) cameraparameters.minView,(GLdouble) cameraparameters.maxView);
    }
    else
    {
        double viewheight=cameraparameters.viewheight;
        double viewwidth=viewheight*(double)cameraparameters.width/(double)cameraparameters.height;
        glOrtho((GLdouble)-viewwidth/2.0,(GLdouble)viewwidth/2.0,(GLdouble)-viewheight/2.0,(GLdouble)viewheight/2.0,cameraparameters.minView,cameraparameters.maxView);
    }
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    return;
}

void GLViewer::resizeGL(int width, int height)
{
    makeCurrent();
    if(height==0)
    {
        height=1;
    }
    cameraparameters.width=width;
    cameraparameters.height=height;
    glViewport(0,0,(GLint)cameraparameters.width,(GLint)cameraparameters.height);
    setProjection();
    return;
}

void GLViewer::keyPressEvent(QKeyEvent * event)
{
    switch(event->key())
    {
    case Qt::Key_Up:
        {
            cameraparameters.transform.col(3)=cameraparameters.transform*Eigen::Vector4d(0,0,-cameraparameters.tspeed,1);
        }
        break;
    case Qt::Key_Down:
        {
            cameraparameters.transform.col(3)=cameraparameters.transform*Eigen::Vector4d(0,0,cameraparameters.tspeed,1);
        }
        break;
    case Qt::Key_Left:
        {
            cameraparameters.transform.col(3)=cameraparameters.transform*Eigen::Vector4d(-cameraparameters.tspeed,0,0,1);
        }
        break;
    case Qt::Key_Right:
        {
            cameraparameters.transform.col(3)=cameraparameters.transform*Eigen::Vector4d(cameraparameters.tspeed,0,0,1);
        }
        break;
    case Qt::Key_PageUp:
        {
            cameraparameters.transform.col(3)=cameraparameters.transform*Eigen::Vector4d(0,cameraparameters.tspeed,0,1);
        }
        break;
    case Qt::Key_PageDown:
        {
            cameraparameters.transform.col(3)=cameraparameters.transform*Eigen::Vector4d(0,-cameraparameters.tspeed,0,1);
        }
        break;
    case Qt::Key_W://w
        {
            cameraparameters.transform.block<3,3>(0,0)=cameraparameters.transform.block<3,3>(0,0)*Eigen::AngleAxisd(cameraparameters.rspeed*GL_PI/180.0,Eigen::Vector3d::UnitX());
        }
        break;
    case Qt::Key_S://s
        {
            cameraparameters.transform.block<3,3>(0,0)=cameraparameters.transform.block<3,3>(0,0)*Eigen::AngleAxisd(-cameraparameters.rspeed*GL_PI/180.0,Eigen::Vector3d::UnitX());
        }
        break;
    case Qt::Key_A://a
        {
            cameraparameters.transform.block<3,3>(0,0)=cameraparameters.transform.block<3,3>(0,0)*Eigen::AngleAxisd(cameraparameters.rspeed*GL_PI/180.0,Eigen::Vector3d::UnitY());
        }
        break;
    case Qt::Key_D://d
        {
            cameraparameters.transform.block<3,3>(0,0)=cameraparameters.transform.block<3,3>(0,0)*Eigen::AngleAxisd(-cameraparameters.rspeed*GL_PI/180.0,Eigen::Vector3d::UnitY());
        }
        break;
    case Qt::Key_Q://q
        {
            cameraparameters.transform.block<3,3>(0,0)=cameraparameters.transform.block<3,3>(0,0)*Eigen::AngleAxisd(cameraparameters.rspeed*GL_PI/180.0,Eigen::Vector3d::UnitZ());
        }
        break;
    case Qt::Key_E://e
        {
            cameraparameters.transform.block<3,3>(0,0)=cameraparameters.transform.block<3,3>(0,0)*Eigen::AngleAxisd(-cameraparameters.rspeed*GL_PI/180.0,Eigen::Vector3d::UnitZ());
        }
        break;
    case Qt::Key_Home:
        {
            cameraparameters.transform.setIdentity();
        }
        break;
    case Qt::Key_1:
        {
            bperspective=1;
            setProjection();
        }
        break;
    case Qt::Key_2:
        {
            bperspective=0;
            setProjection();
        }
        break;
    case Qt::Key_Equal:
        {
            if(bperspective)
            {
                cameraparameters.viewAngle+=1;
            }
            else
            {
                cameraparameters.viewheight+=1;
            }
            setProjection();
        }
        break;
    case Qt::Key_Minus:
        {
            if(bperspective)
            {
                cameraparameters.viewAngle-=1;
            }
            else
            {
                cameraparameters.viewheight-=1;
            }
            setProjection();
        }
        break;
    case Qt::Key_Period:
        {
            if(bperspective)
            {
                cameraparameters.viewAngle+=10;
            }
            else
            {
                cameraparameters.viewheight+=10;
            }
            setProjection();
        }
        break;
    case Qt::Key_Comma:
        {
            if(bperspective)
            {
                cameraparameters.viewAngle-=10;
            }
            else
            {
                cameraparameters.viewheight-=10;
            }
            setProjection();
        }
        break;
    case Qt::Key_B:
        {
            QColor color(cameraparameters.background(0)*255,cameraparameters.background(1)*255,cameraparameters.background(2)*255,cameraparameters.background(3)*255);
            color=QColorDialog::getColor(color,this);
            if(color.isValid())
            {
                cameraparameters.background(0)=color.red()/255.0;
                cameraparameters.background(1)=color.green()/255.0;
                cameraparameters.background(2)=color.blue()/255.0;
                cameraparameters.background(3)=color.alpha()/255.0;
            }
         }
        break;
	case Qt::Key_N:
		{
            QColor color(cameraparameters.lightambient[0]*255,cameraparameters.lightambient[1]*255,cameraparameters.lightambient[2]*255,cameraparameters.lightambient[3]*255);
			color=QColorDialog::getColor(color,this);
			if(color.isValid())
			{
                cameraparameters.lightambient[0]=color.red()/255.0;
                cameraparameters.lightambient[1]=color.green()/255.0;
                cameraparameters.lightambient[2]=color.blue()/255.0;
                cameraparameters.lightambient[3]=color.alpha()/255.0;
			}
		}
		break;
    case Qt::Key_O:
        {
               cameraparameters.pointsize--;
               if(cameraparameters.pointsize<1)
               {
                   cameraparameters.pointsize=1;
               }
        }
        break;
    case Qt::Key_P:
        {
            cameraparameters.pointsize++;
        }
        break;
	case Qt::Key_Delete:
		{
			int i,n=displaylist.size();
			this->makeCurrent();
			for(i=0;i<n;i++)
			{
				glNewList(displaylist[i].listid,GL_COMPILE);
				glEndList();
			}
			this->update();
		}
		break;
    }
    update();
    return;
}

void GLViewer::mousePressEvent(QMouseEvent *event)
{
    setFocus();
    emit mousePositionSignal(event,&cameraparameters);
    return;
}

void GLViewer::mouseReleaseEvent(QMouseEvent *event)
{
    emit mousePositionSignal(event,&cameraparameters);
    return;
}

void GLViewer::wheelEvent(QWheelEvent * event)
{
    int numdegrees=event->delta()/8;
    int numsteps=numdegrees/15;
    cameraparameters.tspeed*=pow(2.0,(double)numsteps);
    if(cameraparameters.tspeed>10)
    {
        cameraparameters.tspeed=10;
    }
    else if(cameraparameters.tspeed<0.01)
    {
        cameraparameters.tspeed=0.01;
    }
}

void GLViewer::mouseMoveEvent(QMouseEvent *event)
{
    emit mousePositionSignal(event,&cameraparameters);
    return;
}

void GLViewer::addDisplayList(GLuint listid)
{
    if(listid==0)
    {
        return;
    }
    else
    {
        DISPLAYLIST templist;
        templist.listid=listid;
        templist.show=1;
		templist.transform.setIdentity();
		templist.scale.setIdentity();
        displaylist.push_back(templist);
    }
    return;
}

void GLViewer::addDisplayLists(GLuint listid, GLuint num)
{
    makeCurrent();
    int i,n=num;
    for(i=0;i<n;i++)
    {
        DISPLAYLIST list;
        list.listid=listid+i;
        list.show=1;
        displaylist.push_back(list);
    }
    return;
}

void GLViewer::enableShow(GLuint listid, bool show, bool islistid)
{
    if(islistid)
    {
        int i;
        int n=displaylist.size();
        for(i=0;i<n;i++)
        {
            if(displaylist[i].listid==listid)
            {
                displaylist[i].show=show;
                break;
            }
        }
    }
    else
    {
        displaylist[listid].show=show;
    }
    return;
}

void GLViewer::deleteDisplayList(GLuint listid, bool islistid)
{
    if(islistid)
    {
        int i;
        int n=displaylist.size();
        for(i=0;i<n;i++)
        {
            if(displaylist[i].listid==listid)
            {
                displaylist.erase(displaylist.begin()+i);
                break;
            }
        }
    }
    else
    {
        displaylist.erase(displaylist.begin()+listid);
    }
    return;
}

void GLViewer::clearDisplayList()
{
    displaylist.clear();
    return;
}

int GLViewer::listSize()
{
    return displaylist.size();
}

void GLViewer::setCameraPose(Eigen::Matrix4d transform)
{
    transform.block<3,3>(0,0).normalize();
    cameraparameters.transform=transform;
    return;
}

Eigen::Matrix4d GLViewer::getCameraPose()
{
    return cameraparameters.transform;
}

void GLViewer::setBackground(QColor color)
{
    cameraparameters.background(0)=color.red()/255.0;
    cameraparameters.background(1)=color.green()/255.0;
    cameraparameters.background(2)=color.blue()/255.0;
    cameraparameters.background(3)=color.alpha()/255.0;
    return;
}

void GLViewer::setDisplayListScale(GLuint listid, double sx, double sy, double sz, bool islistid)
{
    if(islistid)
    {
        int i;
        int n=displaylist.size();
        for(i=0;i<n;i++)
        {
            if(displaylist[i].listid==listid)
            {
				displaylist[i].scale(0,0)=sx;
				displaylist[i].scale(1,1)=sy;
				displaylist[i].scale(2,2)=sz;
                break;
            }
        }
    }
    else
    {
        displaylist[listid].scale(0,0)=sx;
		displaylist[listid].scale(1,1)=sy;
		displaylist[listid].scale(2,2)=sz;
    }
    return;
}

void GLViewer::setDisplayListRotation(GLuint listid, double rx, double ry, double rz, bool islistid)
{
    if(islistid)
    {
        int i;
        int n=displaylist.size();
        for(i=0;i<n;i++)
        {
            if(displaylist[i].listid==listid)
			{
				Eigen::Matrix3d rotation;
				rotation=Eigen::AngleAxisd(rz,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(ry,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(rx,Eigen::Vector3d::UnitX());
				displaylist[i].transform.block<3,3>(0,0)=rotation;
				break;
			}
		}
	}
	else
	{
		Eigen::Matrix3d rotation;
		rotation=Eigen::AngleAxisd(rz,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(ry,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(rx,Eigen::Vector3d::UnitX());
		displaylist[listid].transform.block<3,3>(0,0)=rotation;
	}
    return;
}

void GLViewer::setDisplayListTranslation(GLuint listid, double tx, double ty, double tz, bool islistid)
{
    if(islistid)
    {
        int i;
        int n=displaylist.size();
        for(i=0;i<n;i++)
        {
            if(displaylist[i].listid==listid)
            {
				displaylist[i].transform(0,3)=tx;
				displaylist[i].transform(1,3)=ty;
				displaylist[i].transform(2,3)=tz;
                break;
            }
        }
    }
    else
    {
        displaylist[listid].transform(0,3)=tx;
		displaylist[listid].transform(1,3)=ty;
		displaylist[listid].transform(2,3)=tz;
    }
    return;
}

void GLViewer::setDisplayListTransform(GLuint listid, Eigen::Matrix4d transform, bool islistid)
{
    if(islistid)
    {
        int i;
        int n=displaylist.size();
        for(i=0;i<n;i++)
        {
            if(displaylist[i].listid==listid)
            {
				displaylist[i].transform=transform;
                break;
            }
        }
    }
    else
    {
        displaylist[listid].transform=transform;
    }
    return;
}
