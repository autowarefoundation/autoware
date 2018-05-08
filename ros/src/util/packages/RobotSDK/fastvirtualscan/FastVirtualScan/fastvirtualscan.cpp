#include "fastvirtualscan/fastvirtualscan.h"

#define MAXVIRTUALSCAN 1e6

FastVirtualScan::FastVirtualScan()
{
    beamnum=1000;
    step=0.3;
    minfloor=-3;
    maxceiling=3;
    rotation=0;
    minrange=0;
}

FastVirtualScan::~FastVirtualScan()
{

}

bool compareDistance(const SimpleVirtualScan & svs1, const SimpleVirtualScan & svs2)
{
    if(svs1.rotlength==svs2.rotlength)
    {
        return svs1.rotid>svs2.rotid;
    }
    else
    {
        return svs1.rotlength<svs2.rotlength;
    }
}

void FastVirtualScan::calculateVirtualScans(int beamNum, double heightStep, double minFloor, double maxCeiling, double obstacleMinHeight, double maxBackDistance, double beamRotation, double minRange)
{

    assert(minFloor<maxCeiling);

    beamnum=beamNum;
    step=heightStep;
    minfloor=minFloor;
    maxceiling=maxCeiling;
    rotation=beamRotation;
    minrange=minRange;
    double c=cos(rotation);
    double s=sin(rotation);

    double PI=3.141592654;
    double density=2*PI/beamnum;

    int size=int((maxceiling-minfloor)/step+0.5);

    //initial Simple Virtual Scan
    {
        svs.resize(beamnum);
        svsback.resize(beamnum);
        for(int i=0;i<beamnum;i++)
        {
            svs[i].resize(size);
            for(int j=0;j<size;j++)
            {
                svs[i][j].rotid=j;
                svs[i][j].length=MAXVIRTUALSCAN;
                svs[i][j].rotlength=MAXVIRTUALSCAN;
                svs[i][j].rotheight=minfloor+(j+0.5)*step;
                svs[i][j].height=minfloor+(j+0.5)*step;
            }
        }
    }
    //set SVS
    {
        char * tmpdata=(char *)(velodyne->data.data());
        int i,n=velodyne->height*velodyne->width;

        //O(P)
        for(i=0;i<n;i++)
        {
            float * point=(float *)(tmpdata+i*velodyne->point_step);
            double length=sqrt(point[0]*point[0]+point[1]*point[1]);
            double rotlength=length*c-point[2]*s;
            double rotheight=length*s+point[2]*c;
            int rotid=int((rotheight-minfloor)/step+0.5);
            if(rotid>=0&&rotid<size)
            {
                double theta=atan2(point[1],point[0]);
                int beamid=int((theta+PI)/density);
                if(beamid<0)
                {
                    beamid=0;
                }
                else if(beamid>=beamnum)
                {
                    beamid=beamnum-1;
                }
                if(length>minrange&&svs[beamid][rotid].rotlength>rotlength)
                {
                    svs[beamid][rotid].rotlength=rotlength;
                    svs[beamid][rotid].length=svs[beamid][rotid].rotlength*c+svs[beamid][rotid].rotheight*s;
                    svs[beamid][rotid].height=-svs[beamid][rotid].rotlength*s+svs[beamid][rotid].rotheight*c;
                }
            }
        }
    }
    //sorts
    {
#ifdef USEOMP
#ifndef QT_DEBUG
#pragma omp parallel for \
    default(shared) \
    schedule(static)
#endif
#endif
        for(int i=0;i<beamnum;i++)
        {
            int j;
            bool flag=1;
            int startid=0;
            for(j=0;j<size;j++)
            {
                if(flag)
                {
                    if(svs[i][j].rotlength<MAXVIRTUALSCAN)
                    {
                        flag=0;
                        startid=j;
                    }
                    continue;
                }
                if(svs[i][j].rotlength<MAXVIRTUALSCAN&&startid==j-1)
                {
                    startid=j;
                }
                else if(svs[i][j].rotlength<MAXVIRTUALSCAN)
                {
                    if(svs[i][j].height-svs[i][startid].height<obstacleMinHeight&&svs[i][j].rotlength-svs[i][startid].rotlength>-maxBackDistance)
                    {
                        double delta=(svs[i][j].rotlength-svs[i][startid].rotlength)/(j-startid);
                        int k;
                        for(k=startid+1;k<j;k++)
                        {
                            svs[i][k].rotlength=svs[i][j].rotlength-(j-k)*delta;
                            svs[i][k].length=svs[i][k].rotlength*c+svs[i][k].rotheight*s;
                            svs[i][k].height=-svs[i][k].rotlength*s+svs[i][k].rotheight*c;
                        }
                    }
                    startid=j;
                }
            }
            svs[i].back().rotlength=MAXVIRTUALSCAN;
            svsback[i]=svs[i];
            qSort(svs[i].begin(),svs[i].end(),compareDistance);
        }
    }
}

void FastVirtualScan::getVirtualScan(double thetaminheight, double thetamaxheight, double maxFloor, double minCeiling, double passHeight, QVector<double> &virtualScan)
{
    virtualScan.fill(MAXVIRTUALSCAN,beamnum);
    minheights.fill(minfloor,beamnum);
    maxheights.fill(maxceiling,beamnum);

    QVector<double> rotVirtualScan;
    rotVirtualScan.fill(MAXVIRTUALSCAN,beamnum);

    int size=int((maxceiling-minfloor)/step+0.5);
    double deltaminheight=fabs(step/tan(thetaminheight));
    double deltamaxheight=fabs(step/tan(thetamaxheight));

#ifdef USEOMP
#ifndef QT_DEBUG
#pragma omp parallel for \
    default(shared) \
    schedule(static)
#endif
#endif
    for(int i=0;i<beamnum;i++)
    {
        int candid=0;
        bool roadfilterflag=1;
        bool denoiseflag=1;
        while(candid<size&&svs[i][candid].height>minCeiling)
        {
            candid++;
        }
        if(candid>=size||svs[i][candid].rotlength==MAXVIRTUALSCAN)
        {
            virtualScan[i]=0;
            minheights[i]=0;
            maxheights[i]=0;
            continue;
        }
        if(svs[i][candid].height>maxFloor)
        {
            virtualScan[i]=svs[i][candid].length;
            minheights[i]=svs[i][candid].height;
            denoiseflag=0;
            roadfilterflag=0;
        }
        int firstcandid=candid;
        for(int j=candid+1;j<size;j++)
        {
            if(svs[i][j].rotid<=svs[i][candid].rotid)
            {
                continue;
            }
            int startrotid=svs[i][candid].rotid;
            int endrotid=svs[i][j].rotid;

            if(svs[i][j].rotlength==MAXVIRTUALSCAN)
            {
                if(roadfilterflag)
                {
                    virtualScan[i]=MAXVIRTUALSCAN;
                    minheights[i]=0;//svsback[i][startrotid].height;
                    maxheights[i]=0;//svsback[i][startrotid].height;
                }
                else
                {
                    maxheights[i]=svsback[i][startrotid].height;
                }
                break;
            }
            else
            {
                if(denoiseflag)
                {
                    if(startrotid+1==endrotid)
                    {
                        if(svs[i][j].rotlength-svs[i][candid].rotlength>=deltaminheight)
                        {
                            denoiseflag=0;
                            roadfilterflag=1;
                        }
                        else if(svs[i][j].height>maxFloor)
                        {
                            virtualScan[i]=svs[i][firstcandid].length;
                            minheights[i]=svs[i][firstcandid].height;
                            denoiseflag=0;
                            roadfilterflag=0;
                        }
                    }
                    else
                    {
                        if(svs[i][j].height-svs[i][candid].height<=passHeight)
                        {
                            if(svs[i][j].rotlength-svs[i][candid].rotlength<=deltaminheight)
                            {
                                virtualScan[i]=svsback[i][startrotid].length;
                                minheights[i]=svsback[i][startrotid].height;
                                denoiseflag=0;
                                roadfilterflag=0;
                            }
                            else
                            {
                                virtualScan[i]=svs[i][j].length;
                                for(int k=startrotid+1;k<endrotid;k++)
                                {
                                    if(virtualScan[i]>svsback[i][k].length)
                                    {
                                        virtualScan[i]=svsback[i][k].length;
                                    }
                                }
                                minheights[i]=svsback[i][startrotid+1].height;
                                denoiseflag=0;
                                roadfilterflag=0;
                            }
                        }
                        else
                        {
                            continue;
                        }
                    }
                }
                else
                {
                    if(roadfilterflag)
                    {
                        if(startrotid+1==endrotid)
                        {
                            if(svs[i][j].rotlength-svs[i][candid].rotlength<=deltaminheight)
                            {
                                virtualScan[i]=svsback[i][startrotid].length;
                                minheights[i]=svsback[i][startrotid].height;
                                roadfilterflag=0;
                            }
                        }
                        else
                        {
                            if(svs[i][j].height-svs[i][candid].height<=passHeight)
                            {
                                if(svs[i][j].rotlength-svs[i][candid].rotlength<=deltaminheight)
                                {
                                    virtualScan[i]=svsback[i][startrotid].length;
                                    minheights[i]=svsback[i][startrotid].height;
                                    roadfilterflag=0;
                                }
                                else
                                {
                                    virtualScan[i]=svs[i][j].length;
                                    for(int k=startrotid+1;k<endrotid;k++)
                                    {
                                        if(virtualScan[i]>svsback[i][k].length)
                                        {
                                            virtualScan[i]=svsback[i][k].length;
                                        }
                                    }
                                    minheights[i]=svsback[i][startrotid+1].height;
                                    roadfilterflag=0;
                                }
                            }
                            else
                            {
                                continue;
                            }
                        }
                    }
                    else
                    {
                        if(svs[i][j].rotlength-svs[i][candid].rotlength>deltamaxheight)
                        {
                            maxheights[i]=svsback[i][startrotid].height;
                            break;
                        }
                    }
                }
            }
            candid=j;
        }
        if(virtualScan[i]<=0)
        {
            virtualScan[i]=0;
            minheights[i]=0;
            maxheights[i]=0;
        }
    }
}
