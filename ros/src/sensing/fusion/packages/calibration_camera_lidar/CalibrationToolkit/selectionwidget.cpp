#include"selectionwidget.h"

PlaneExtractor::PlaneExtractor(sensor_msgs::PointCloud2ConstPtr velodynePoints, int id, double neighborRadius, double distanceThreshold, QWidget *parent)
    : GLViewer(parent)
{
    pointsid=id;
    neighborradius=neighborRadius;
    distance=distanceThreshold;
    pointsptr=velodynePoints;

    int i,n=pointsptr->height*pointsptr->width;
    cloud.resize(n);
    QVector<float> colors(3*n);
    char * data=(char *)(pointsptr->data.data());
    for(i=0;i<n;i++)
    {
        float * base=(float *)(data+i*pointsptr->point_step);
        cloud[i].x=base[0];
        cloud[i].y=base[1];
        cloud[i].z=base[2];
        cloud[i].intensity=base[4];
        colors[i*3+0]=base[4]/255.0;
        colors[i*3+1]=base[4]/255.0;
        colors[i*3+2]=base[4]/255.0;
    }
    kdtree.setInputCloud(cloud.makeShared());

    this->makeCurrent();

    pointsdisplaylist=glGenLists(1);
    selecteddisplaylist=glGenLists(1);
    mousedisplaylist=glGenLists(1);

    this->addDisplayList(pointsdisplaylist);
    this->addDisplayList(selecteddisplaylist);
    this->addDisplayList(mousedisplaylist);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glVertexPointer(3,GL_FLOAT,sizeof(pcl::PointXYZI),cloud.points.data());
    glColorPointer(3,GL_FLOAT,3*sizeof(float),colors.data());

    glNewList(pointsdisplaylist,GL_COMPILE);
    glDrawArrays(GL_POINTS,0,n);
    glEndList();

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

    this->update();

    extracted=0;

    connect(this,SIGNAL(mousePositionSignal(QMouseEvent*,CAMERAPARAMETERS*)),this,SLOT(mousePositionSlot(QMouseEvent*,CAMERAPARAMETERS*)));
}

PlaneExtractor::PlaneExtractor(pcl::PointCloud<pcl::PointXYZI>::Ptr velodynePoints, int id, double neighborRadius, double distanceThreshold, QWidget *parent)
    : GLViewer(parent)
{
    pointsid=id;
    neighborradius=neighborRadius;
    distance=distanceThreshold;

    int i,n=velodynePoints->size();
    cloud.resize(n);
    QVector<float> colors(3*n);
    for(i=0;i<n;i++)
    {
        cloud[i].x=velodynePoints->points[i].x;
        cloud[i].y=velodynePoints->points[i].y;
        cloud[i].z=velodynePoints->points[i].z;
        cloud[i].intensity=velodynePoints->points[i].intensity;
        colors[i*3+0]=cloud[i].intensity/255.0;
        colors[i*3+1]=cloud[i].intensity/255.0;
        colors[i*3+2]=cloud[i].intensity/255.0;
    }
    kdtree.setInputCloud(cloud.makeShared());

    this->makeCurrent();

    pointsdisplaylist=glGenLists(1);
    selecteddisplaylist=glGenLists(1);
    mousedisplaylist=glGenLists(1);

    this->addDisplayList(pointsdisplaylist);
    this->addDisplayList(selecteddisplaylist);
    this->addDisplayList(mousedisplaylist);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glVertexPointer(3,GL_FLOAT,sizeof(pcl::PointXYZI),cloud.points.data());
    glColorPointer(3,GL_FLOAT,3*sizeof(float),colors.data());

    glNewList(pointsdisplaylist,GL_COMPILE);
    glDrawArrays(GL_POINTS,0,n);
    glEndList();

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

    this->update();

    extracted=1;

    connect(this,SIGNAL(mousePositionSignal(QMouseEvent*,CAMERAPARAMETERS*)),this,SLOT(mousePositionSlot(QMouseEvent*,CAMERAPARAMETERS*)));
}

void PlaneExtractor::mousePositionSlot(QMouseEvent * event, CAMERAPARAMETERS * parameters)
{
    this->makeCurrent();
    GLint iViewPort[4];
    GLdouble dProjMatrix[16];
    GLdouble dModelMatrix[16];
    glGetDoublev(GL_MODELVIEW_MATRIX,dModelMatrix);
    glGetDoublev(GL_PROJECTION_MATRIX,dProjMatrix);
    glGetIntegerv(GL_VIEWPORT,iViewPort);
    Eigen::Vector3d eye=parameters->transform.block<3,1>(0,3);
    Eigen::Vector3d dir;
    GLdouble tmpd[3];
    GLint tmpx=(GLint)(event->x());
    GLint tmpy=(GLint)(event->y());
    gluUnProject(tmpx,iViewPort[3]-tmpy-1,1
            ,dModelMatrix,dProjMatrix,iViewPort
            ,&(tmpd[0]),&(tmpd[1]),&(tmpd[2]));
    dir(0)=tmpd[0];dir(1)=tmpd[1];dir(2)=tmpd[2];
    dir=dir-eye;
    dir.normalize();
    double dis=parameters->minView;
    double cdis=0;
    double tmpmaxdis=parameters->maxView;
    double err;
    double radius=neighborradius;
    double errthreshold=0.01;
    Eigen::Vector3d point;
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    Eigen::Vector3d eigenvalues;
    Eigen::Matrix3d eigenvectors;
    int N=1000;
    do
    {
        if(dis>tmpmaxdis)
        {
            break;
        }
        point=eye+dir*dis;
        pcl::PointXYZI tmppoint;
        tmppoint.x=point(0);tmppoint.y=point(1);tmppoint.z=point(2);
        kdtree.radiusSearchT(tmppoint,radius,k_indices,k_sqr_distances);
        int n=k_indices.size();
        if(n<3)
        {
            dis+=radius;
            continue;
        }
        int i;
        Eigen::Vector3d center;
        center.setZero();
        std::vector<Eigen::Vector3d> points(n);
        for(i=0;i<n;i++)
        {
            points[i](0)=cloud[k_indices[i]].x;
            points[i](1)=cloud[k_indices[i]].y;
            points[i](2)=cloud[k_indices[i]].z;
            center=center+points[i];
        }
        center=center/n;
        Eigen::Matrix3d covariance;
        covariance.setZero();
        for(i=0;i<n;i++)
        {
            points[i]=points[i]-center;
            covariance(0,0)=covariance(0,0)+points[i](0)*points[i](0); covariance(0,1)=covariance(0,1)+points[i](0)*points[i](1); covariance(0,2)=covariance(0,2)+points[i](0)*points[i](2);
            covariance(1,0)=covariance(1,0)+points[i](1)*points[i](0); covariance(1,1)=covariance(1,1)+points[i](1)*points[i](1); covariance(1,2)=covariance(1,2)+points[i](1)*points[i](2);
            covariance(2,0)=covariance(2,0)+points[i](2)*points[i](0); covariance(2,1)=covariance(2,1)+points[i](2)*points[i](1); covariance(2,2)=covariance(2,2)+points[i](2)*points[i](2);
        }
        covariance=covariance/n;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance);
        eigenvalues=eigensolver.eigenvalues();
        eigenvectors=eigensolver.eigenvectors();
        if(eigenvectors.col(0).dot(dir)>0)
        {
            eigenvectors.col(0)=-eigenvectors.col(0);
        }
        if(eigenvectors.col(2).dot(dir)>0)
        {
            eigenvectors.col(2)=-eigenvectors.col(2);
        }
        Eigen::Vector3d pp=center-eye;
        double tempdis=eigenvectors.col(0).dot(dir);
        if(tempdis==0)
        {
            dis+=radius;
            continue;
        }
        dis=(pp.dot(eigenvectors.col(0)))/tempdis;
        err=fabs(dis-cdis);
        cdis=dis;
        N--;
        if(err<errthreshold)
        {
            point=eye+dir*dis;
            break;
        }
    }while(N>0);
    if(dis<=tmpmaxdis)
    {
        Eigen::Vector3d nearestpoint;
        nearestpoint(0)=cloud[k_indices[0]].x;
        nearestpoint(1)=cloud[k_indices[0]].y;
        nearestpoint(2)=cloud[k_indices[0]].z;
        switch(event->button())
        {
        case Qt::LeftButton:
            if(!extracted)
            {
                extractPlane(point,eigenvectors);
                extracted=1;
            }
            break;
        case Qt::RightButton:
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr extraction;
                emit extractionResultSignal(extraction,cv::Mat(1,3,CV_64F),pointsid);
                this->makeCurrent();
                glNewList(selecteddisplaylist,GL_COMPILE);
                glEndList();
                this->update();
                extracted=0;
            }
            break;
        case Qt::NoButton:
            drawMouse(point,eigenvectors.col(0),eigenvectors.col(2),nearestpoint);
            break;
        default:
            break;
        }
    }
    else
    {
        this->makeCurrent();
        glNewList(mousedisplaylist,GL_COMPILE);
        glEndList();
        this->update();
    }
}

void PlaneExtractor::drawMouse(Eigen::Vector3d point, Eigen::Vector3d norm, Eigen::Vector3d direction, Eigen::Vector3d nearestpoint)
{
    this->makeCurrent();
    glNewList(mousedisplaylist,GL_COMPILE);
    glBegin(GL_LINES);
    glColor4d(0.0,1.0,0.0,1.0);
    glVertex3d(point(0),point(1),point(2));
    Eigen::Vector3d temppoint=point+norm;
    glVertex3d(temppoint(0),temppoint(1),temppoint(2));
    glVertex3d(point(0),point(1),point(2));
    temppoint=point+direction;
    glVertex3d(temppoint(0),temppoint(1),temppoint(2));
    glVertex3d(point(0),point(1),point(2));
    glVertex3d(nearestpoint(0),nearestpoint(1),nearestpoint(2));
    glEnd();
    temppoint(0)=0;temppoint(1)=0;temppoint(2)=1;
    Eigen::Vector3d axis=temppoint.cross(norm);
    Eigen::Matrix3d rot;
    if(axis.norm()!=0)
    {
        axis.normalize();
        double theta=acos(temppoint.dot(norm));
        rot=Eigen::AngleAxisd(theta,axis);
    }
    else
    {
        rot.setIdentity();
    }
    int i;
    double radius=neighborradius;
    double PI=3.141592654;
    glBegin(GL_LINES);
    for(i=0;i<360;i++)
    {
        temppoint(0)=radius*cos(i*PI/180.0);
        temppoint(1)=radius*sin(i*PI/180.0);
        temppoint(2)=0;
        temppoint=rot*temppoint+point;
        glVertex3d(temppoint(0),temppoint(1),temppoint(2));
    }
    glEnd();
    glEndList();
    this->update();
}

void PlaneExtractor::extractPlane(Eigen::Vector3d seed, Eigen::Matrix3d eigenvectors)
{
    std::vector<int> k_indices;
    std::vector<float> k_sqr_distances;
    pcl::PointXYZI tmpseed;
    tmpseed.x=float(seed(0));tmpseed.y=float(seed(1));tmpseed.z=float(seed(2));
    std::vector<int> tmpresult;
    int n=kdtree.radiusSearchT(tmpseed,neighborradius,k_indices,k_sqr_distances);
    int i;
    for(i=0;i<n;i++)
    {
        int index=k_indices[i];
        Eigen::Vector3d temppoint(double(cloud[index].x), double(cloud[index].y), double(cloud[index].z));
        Eigen::Vector3d disvec=temppoint-seed;
        double tempdistance=fabs(disvec.dot(eigenvectors.col(0)));
        if(tempdistance<=distance)
        {
            tmpresult.push_back(index);
        }
    }
    if(tmpresult.size()>0)
    {
        n=tmpresult.size();
        pcl::PointCloud<pcl::PointXYZI>::Ptr extraction(new pcl::PointCloud<pcl::PointXYZI>);
        extraction->resize(n);
        for(i=0;i<n;i++)
        {
            extraction->points[i]=cloud.points[tmpresult[i]];
        }
        cv::Mat normal(1,3,CV_64F);
        normal.at<double>(0)=eigenvectors(0,0);
        normal.at<double>(1)=eigenvectors(1,0);
        normal.at<double>(2)=eigenvectors(2,0);
        emit extractionResultSignal(extraction,normal,pointsid);

        this->makeCurrent();
        glNewList(selecteddisplaylist,GL_COMPILE);
        glBegin(GL_POINTS);
        glColor4d(1.0,0.0,0.0,1.0);
        for(i=0;i<n;i++)
        {
            glVertex3f(extraction->points[i].x,extraction->points[i].y,extraction->points[i].z);
        }
        glEnd();
        glEndList();
        this->update();
    }
}

PointsExtractor::PointsExtractor(int imageSize, double maxRange, double gridSize)
{
    pointsid=-1;
    imagesize=imageSize;
    maxrange=maxRange;
    gridsize=gridSize;
    extracted=0;
    justshow=1;
}

PointsExtractor::PointsExtractor(sensor_msgs::LaserScanConstPtr lidarPoints, int id, int imageSize, double maxRange, double gridSize)
{
    updateLaserScan(lidarPoints);
    pointsid=id;
    imagesize=imageSize;
    maxrange=maxRange;
    gridsize=gridSize;
    extracted=0;
    justshow=0;
    setMouseTracking(1);
}

PointsExtractor::PointsExtractor(QVector<QPointF> lidarPoints, int id, int imageSize, double maxRange, double gridSize)\
{
    points=lidarPoints;
    drawPoints();
    pointsid=id;
    imagesize=imageSize;
    maxrange=maxRange;
    gridsize=gridSize;
    extracted=0;
    justshow=0;
    setMouseTracking(1);
}

void PointsExtractor::updateLaserScan(sensor_msgs::LaserScanConstPtr lidarPoints)
{
    pointsptr=lidarPoints;
    int i,pointsnum=(pointsptr->angle_max-pointsptr->angle_min)/pointsptr->angle_increment+1;
    points.resize(pointsnum);
    for(i=0;i<pointsnum;i++)
    {
        float angle=pointsptr->angle_min+i*pointsptr->angle_increment;
        points[i]=QPointF(pointsptr->ranges[i]*cos(angle),pointsptr->ranges[i]*sin(angle));
    }
    drawPoints();
}

void PointsExtractor::update()
{
    drawPoints();
    drawRectangle();
}

void PointsExtractor::mousePressEvent(QMouseEvent *ev)
{
    if(!justshow)
    {
        switch(ev->button())
        {
        case Qt::LeftButton:
            if(!extracted)
            {
                startcorner=ev->pos();
                endcorner=startcorner;
                startextraction=1;
            }
            break;
        case Qt::RightButton:
            extracted=0;
            startcorner=QPoint(0,0);
            endcorner=QPoint(0,0);
            startextraction=0;
            break;
        default:
            break;
        }
        drawPoints();
    }
    QLabel::mousePressEvent(ev);
}

void PointsExtractor::mouseMoveEvent(QMouseEvent *ev)
{
    if(!justshow)
    {
        if(startextraction)
        {
            endcorner=ev->pos();
            drawPoints();
            drawRectangle();
        }
    }
    QLabel::mouseMoveEvent(ev);
}

void PointsExtractor::mouseReleaseEvent(QMouseEvent *ev)
{
    if(!justshow)
    {
        if(ev->button()==Qt::LeftButton)
        {
            extracted=1;
            emit extractionResultSignal(extractPoints(),pointsid);
            startextraction=0;
        }
    }
    QLabel::mouseReleaseEvent(ev);
}

void PointsExtractor::wheelEvent(QWheelEvent *ev)
{
    if(ev->angleDelta().y()>0)
    {
        QPointF tmpstartcorner=convert2RealPoint(startcorner);
        QPointF tmpendcorder=convert2RealPoint(endcorner);
        if(ev->modifiers()!=Qt::ControlModifier)
        {
            maxrange-=gridsize;
            if(maxrange<=0)
            {
                maxrange=gridsize;
            }
        }
        else
        {
            imagesize-=100;
            if(imagesize<=0)
            {
                imagesize=100;
            }
        }
        startcorner=convert2ImagePoint(tmpstartcorner);
        endcorner=convert2ImagePoint(tmpendcorder);
    }
    else
    {
        QPointF tmpstartcorner=convert2RealPoint(startcorner);
        QPointF tmpendcorder=convert2RealPoint(endcorner);
        if(ev->modifiers()!=Qt::ControlModifier)
        {
            maxrange+=gridsize;
        }
        else
        {
            imagesize+=100;
        }
        startcorner=convert2ImagePoint(tmpstartcorner);
        endcorner=convert2ImagePoint(tmpendcorder);
    }
    drawPoints();
    drawRectangle();
}

QPoint PointsExtractor::convert2ImagePoint(QPointF point)
{
    double ratio=double(imagesize)/(2*maxrange);
    QPoint result;
    result.setX(imagesize/2+point.x()*ratio);
    result.setY(imagesize/2-point.y()*ratio);
    return result;
}

QPointF PointsExtractor::convert2RealPoint(QPoint point)
{
    double ratio=(2*maxrange)/double(imagesize);
    QPointF result;
    result.setX((point.x()-imagesize/2)*ratio);
    result.setY((imagesize/2-point.y())*ratio);
    return result;
}

void PointsExtractor::drawPoints()
{
    image=QImage(imagesize,imagesize,QImage::Format_RGB888);
    this->resize(imagesize,imagesize);
    image.fill(QColor(255,255,255));
    QPainter painter;
    painter.begin(&image);

    painter.setPen(QColor(128,128,128));
    int i,n=int(maxrange/gridsize);
    double ratio=double(imagesize)/(2*maxrange);
    QPoint center(imagesize/2,imagesize/2);
    for(i=1;i<=n;i++)
    {
        painter.drawEllipse(center,int(gridsize*i*ratio),int(gridsize*i*ratio));
    }

    n=points.size();
    painter.setPen(QColor(255,0,0));
    for(i=0;i<n;i++)
    {
        painter.drawEllipse(convert2ImagePoint(points[i]),1,1);
    }
    painter.end();

    this->setPixmap(QPixmap::fromImage(image));
}

void PointsExtractor::drawRectangle()
{
    if(justshow)
    {
        return;
    }
    QPainter painter;
    painter.begin(&image);

    painter.setPen(QColor(0,255,0));
    int startx=startcorner.x()<endcorner.x()?startcorner.x():endcorner.x();
    int starty=startcorner.y()<endcorner.y()?startcorner.y():endcorner.y();
    int endx=startcorner.x()>endcorner.x()?startcorner.x():endcorner.x();
    int endy=startcorner.y()>endcorner.y()?startcorner.y():endcorner.y();
    painter.drawRect(startx,starty,endx-startx,endy-starty);

    QVector<QPointF> extraction=extractPoints();
    int i,n=extraction.size();
    painter.setPen(QColor(0,0,255));
    for(i=0;i<n;i++)
    {
        painter.drawEllipse(convert2ImagePoint(extraction[i]),1,1);
    }
    painter.end();

    this->setPixmap(QPixmap::fromImage(image));
}

QVector<QPointF> PointsExtractor::extractPoints()
{
    int startx=startcorner.x()<endcorner.x()?startcorner.x():endcorner.x();
    int starty=startcorner.y()<endcorner.y()?startcorner.y():endcorner.y();
    int endx=startcorner.x()>endcorner.x()?startcorner.x():endcorner.x();
    int endy=startcorner.y()>endcorner.y()?startcorner.y():endcorner.y();
    int i,n=points.size();
    QVector<QPointF> result;
    for(i=0;i<n;i++)
    {
        QPoint tmp=convert2ImagePoint(points[i]);
        if(tmp.x()>=startx&&tmp.x()<=endx)
        {
            if(tmp.y()>=starty&&tmp.y()<=endy)
            {
                result.push_back(points[i]);
            }
        }
    }
    return result;
}
