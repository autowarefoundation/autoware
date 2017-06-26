#include"rbsspf_geometry.cuh"

//====================================================
//1: init control and particles

__host__
void hostCalculateGeometryControl(TrackerSampleControl & control)
{
    double ratio=1,vratio=1,maxratio=1;

    CALRATIO(ratio,vratio,maxratio,control.geometryoffset.theta,control.geometryprec.theta);
    CALRATIO(ratio,vratio,maxratio,control.geometryoffset.wl,control.geometryprec.wl);
    CALRATIO(ratio,vratio,maxratio,control.geometryoffset.wr,control.geometryprec.wr);
    CALRATIO(ratio,vratio,maxratio,control.geometryoffset.lf,control.geometryprec.lf);
    CALRATIO(ratio,vratio,maxratio,control.geometryoffset.lb,control.geometryprec.lb);

    control.geometryiteration=log(vratio)/log(2);
    control.geometryanneal=maxratio*maxratio;
    control.geometryannealratio=pow(control.geometryanneal,-1/control.geometryiteration);

    CALZOOM(control.geometryzoom.theta,control.geometryoffset.theta,control.geometryprec.theta,control.geometryiteration);
    CALZOOM(control.geometryzoom.wl,control.geometryoffset.wl,control.geometryprec.wl,control.geometryiteration);
    CALZOOM(control.geometryzoom.wr,control.geometryoffset.wr,control.geometryprec.wr,control.geometryiteration);
    CALZOOM(control.geometryzoom.lf,control.geometryoffset.lf,control.geometryprec.lf,control.geometryiteration);
    CALZOOM(control.geometryzoom.lb,control.geometryoffset.lb,control.geometryprec.lb,control.geometryiteration);
}

__host__
void hostSetupGeometryOffset(TrackerState & sigma, TrackerSampleControl & control)
{
    control.geometryoffset=UPDATEGEOMETRYOFFSET;
    if(sigma.wl<control.geometryprec.wl) control.geometryoffset.wl=control.geometryprec.wl;
    if(sigma.wr<control.geometryprec.wr) control.geometryoffset.wr=control.geometryprec.wr;
    if(sigma.lf<control.geometryprec.lf) control.geometryoffset.lf=control.geometryprec.lf;
    if(sigma.lb<control.geometryprec.lb) control.geometryoffset.lb=control.geometryprec.lb;
}

__host__
bool hostInitializeGeometry(Tracker & tracker, TrackerSampleControl & control)
{
    control.geometrymin=GEOMETRYMIN;
    control.geometrymax=GEOMETRYMAX;
    control.geometryprec=GEOMETRYPREC;

    switch(tracker.status)
    {
    case StatusInitGeometry:
        {
            control.geometryoffset=INITGEOMETRYOFFSET;
            control.pnum=1;
            hostCalculateGeometryControl(control);
            control.geometrymin.theta=tracker.mean.theta-control.geometryoffset.theta;
            control.geometrymax.theta=tracker.mean.theta+control.geometryoffset.theta;
        }
        break;
    case StatusUpdateTracker_SSPF:
        {
            hostSetupGeometryOffset(tracker.sigma,control);
            control.pnum=1;
            hostCalculateGeometryControl(control);
        }
        break;
    default:
        {
            control.geometryiteration=-1;
            control.pnum=0;
        }
        break;
    }
    return control.geometryiteration>=0;
}

__host__
double hostInitGeometryEstimation(int trackernum, std::vector<Tracker> & trackers, std::vector<TrackerSampleControl> & controls, int & pnum, std::vector<TrackerParticle> & particles)
{
    double maxgeometryiteration=-1;
    pnum=0;
    for(int i=0;i<trackernum;i++)
    {
        controls[i].id=i;
        if(hostInitializeGeometry(trackers[i],controls[i]))
        {
            if(maxgeometryiteration<controls[i].geometryiteration) maxgeometryiteration=controls[i].geometryiteration;
            particles[pnum].state=trackers[i].mean;
            particles[pnum].controlid=i;
            particles[pnum].weight=0;
            particles[pnum].beamcount=0;
            pnum++;
        }
    }
    return maxgeometryiteration;
}

//====================================================
//2: upsample

__global__
void kernelGeometryUpSample(TrackerParticle * particles, TrackerSampleControl * controls, TrackerParticle * tmpparticles, int tmppnum, thrust::random::minstd_rand * rng, int beamnum, int * beamcount)
{
    GetThreadID_1D(tmppid);
    if(tmppid>=tmppnum) return;
    int pid=int(tmppid/SPN);
    int cid=particles[pid].controlid;
    int rid=tmppid%RNGNUM;

    TrackerSampleControl control=controls[cid];
    TrackerParticle particle=particles[pid];

    if(control.geometryiteration<1)
    {
        tmpparticles[tmppid]=particle;
        beamcount[tmppid]=0;
        return;
    }

    if(control.motioniteration<0)
    {
        double thetamin=particle.state.theta-control.geometryoffset.theta; thetamin=thetamin>control.geometrymin.theta?thetamin:control.geometrymin.theta;
        double thetamax=particle.state.theta+control.geometryoffset.theta; thetamax=thetamax<control.geometrymax.theta?thetamax:control.geometrymax.theta;
        particle.state.theta=thrust::random::uniform_real_distribution<double>(thetamin,thetamax)(rng[rid]);
    }

    double wlmin=particle.state.wl-control.geometryoffset.wl; wlmin=wlmin>control.geometrymin.wl?wlmin:control.geometrymin.wl;
    double wlmax=particle.state.wl+control.geometryoffset.wl; wlmax=wlmax<control.geometrymax.wl?wlmax:control.geometrymax.wl;
    particle.state.wl=thrust::random::uniform_real_distribution<double>(wlmin,wlmax)(rng[rid]);

    double wrmin=particle.state.wr-control.geometryoffset.wr; wrmin=wrmin>control.geometrymin.wr?wrmin:control.geometrymin.wr;
    double wrmax=particle.state.wr+control.geometryoffset.wr; wrmax=wrmax<control.geometrymax.wr?wrmax:control.geometrymax.wr;
    particle.state.wr=thrust::random::uniform_real_distribution<double>(wrmin,wrmax)(rng[rid]);

    double lfmin=particle.state.lf-control.geometryoffset.lf; lfmin=lfmin>control.geometrymin.lf?lfmin:control.geometrymin.lf;
    double lfmax=particle.state.lf+control.geometryoffset.lf; lfmax=lfmax<control.geometrymax.lf?lfmax:control.geometrymax.lf;
    particle.state.lf=thrust::random::uniform_real_distribution<double>(lfmin,lfmax)(rng[rid]);

    double lbmin=particle.state.lb-control.geometryoffset.lb; lbmin=lbmin>control.geometrymin.lb?lbmin:control.geometrymin.lb;
    double lbmax=particle.state.lb+control.geometryoffset.lb; lbmax=lbmax<control.geometrymax.lb?lbmax:control.geometrymax.lb;
    particle.state.lb=thrust::random::uniform_real_distribution<double>(lbmin,lbmax)(rng[rid]);

    deviceBuildModel(particle,beamnum);

    tmpparticles[tmppid]=particle;

    beamcount[tmppid]=particle.geometry.beamcount;
}

//====================================================
//8. estimate tracker

__host__
void hostEstimateGeometryTracker(int pnum, std::vector<TrackerParticle> & particles, std::vector<Tracker> & trackers, std::vector<TrackerSampleControl> & controls, int beamnum)
{
    TrackerState minstate;
    TrackerState maxstate;
    Tracker pretracker;
    double weightsum;
    int cid=-1;
    bool rbflag;
    for(int i=0;i<=pnum;i++)
    {
        bool flag=(i>=pnum)||(cid!=particles[i].controlid);
        if(flag)
        {
            if(cid>=0)
            {
                if(!rbflag)
                {
                    trackers[cid].mean.theta/=weightsum;
                    trackers[cid].sigma.theta=std::max(trackers[cid].mean.theta-minstate.theta,maxstate.theta-trackers[cid].mean.theta);
                    trackers[cid].status=StatusInitMotion;
                }

                trackers[cid].mean.wl/=weightsum;
                trackers[cid].mean.wr/=weightsum;
                trackers[cid].mean.lf/=weightsum;
                trackers[cid].mean.lb/=weightsum;
                trackers[cid].beamcount/=weightsum;

                trackers[cid].sigma.wl=std::max(trackers[cid].mean.wl-minstate.wl,maxstate.wl-trackers[cid].mean.wl);
                trackers[cid].sigma.wr=std::max(trackers[cid].mean.wr-minstate.wr,maxstate.wr-trackers[cid].mean.wr);
                trackers[cid].sigma.lf=std::max(trackers[cid].mean.lf-minstate.lf,maxstate.lf-trackers[cid].mean.lf);
                trackers[cid].sigma.lb=std::max(trackers[cid].mean.lb-minstate.lb,maxstate.lb-trackers[cid].mean.lb);

                trackers[cid].sigma.wl=trackers[cid].sigma.wl>MINSIGMA?trackers[cid].sigma.wl:MINSIGMA;
                trackers[cid].sigma.wr=trackers[cid].sigma.wr>MINSIGMA?trackers[cid].sigma.wr:MINSIGMA;
                trackers[cid].sigma.lf=trackers[cid].sigma.lf>MINSIGMA?trackers[cid].sigma.lf:MINSIGMA;
                trackers[cid].sigma.lb=trackers[cid].sigma.lb>MINSIGMA?trackers[cid].sigma.lb:MINSIGMA;

                trackers[cid].sigma.wl=trackers[cid].sigma.wl<UNCERTAINTHRESH?trackers[cid].sigma.wl:MAXSIGMA;
                trackers[cid].sigma.wr=trackers[cid].sigma.wr<UNCERTAINTHRESH?trackers[cid].sigma.wr:MAXSIGMA;
                trackers[cid].sigma.lf=trackers[cid].sigma.lf<UNCERTAINTHRESH?trackers[cid].sigma.lf:MAXSIGMA;
                trackers[cid].sigma.lb=trackers[cid].sigma.lb<UNCERTAINTHRESH?trackers[cid].sigma.lb:MAXSIGMA;

                if(rbflag)
                {
                    trackers[cid].mean.wl=(pretracker.mean.wl*pow(trackers[cid].sigma.wl,2)+trackers[cid].mean.wl*pow(pretracker.sigma.wl,2))/(pow(pretracker.sigma.wl,2)+pow(trackers[cid].sigma.wl,2));
                    trackers[cid].sigma.wl=sqrt((pow(pretracker.sigma.wl,2)*pow(trackers[cid].sigma.wl,2))/(pow(pretracker.sigma.wl,2)+pow(trackers[cid].sigma.wl,2)));

                    trackers[cid].mean.wr=(pretracker.mean.wr*pow(trackers[cid].sigma.wr,2)+trackers[cid].mean.wr*pow(pretracker.sigma.wr,2))/(pow(pretracker.sigma.wr,2)+pow(trackers[cid].sigma.wr,2));
                    trackers[cid].sigma.wr=sqrt((pow(pretracker.sigma.wr,2)*pow(trackers[cid].sigma.wr,2))/(pow(pretracker.sigma.wr,2)+pow(trackers[cid].sigma.wr,2)));

                    trackers[cid].mean.lf=(pretracker.mean.lf*pow(trackers[cid].sigma.lf,2)+trackers[cid].mean.lf*pow(pretracker.sigma.lf,2))/(pow(pretracker.sigma.lf,2)+pow(trackers[cid].sigma.lf,2));
                    trackers[cid].sigma.lf=sqrt((pow(pretracker.sigma.lf,2)*pow(trackers[cid].sigma.lf,2))/(pow(pretracker.sigma.lf,2)+pow(trackers[cid].sigma.lf,2)));

                    trackers[cid].mean.lb=(pretracker.mean.lb*pow(trackers[cid].sigma.lb,2)+trackers[cid].mean.lb*pow(pretracker.sigma.lb,2))/(pow(pretracker.sigma.lb,2)+pow(trackers[cid].sigma.lb,2));
                    trackers[cid].sigma.lb=sqrt((pow(pretracker.sigma.lb,2)*pow(trackers[cid].sigma.lb,2))/(pow(pretracker.sigma.lb,2)+pow(trackers[cid].sigma.lb,2)));

                    trackers[cid].sigma.wl=trackers[cid].sigma.wl>MINSIGMA?trackers[cid].sigma.wl:MINSIGMA;
                    trackers[cid].sigma.wr=trackers[cid].sigma.wr>MINSIGMA?trackers[cid].sigma.wr:MINSIGMA;
                    trackers[cid].sigma.lf=trackers[cid].sigma.lf>MINSIGMA?trackers[cid].sigma.lf:MINSIGMA;
                    trackers[cid].sigma.lb=trackers[cid].sigma.lb>MINSIGMA?trackers[cid].sigma.lb:MINSIGMA;
                }
                hostBuildModel(trackers[cid],beamnum);
            }
            if(i<pnum)
            {
                cid=particles[i].controlid;
                rbflag=controls[cid].motioniteration>=0;

                if(rbflag)
                {
                    pretracker=trackers[cid];
                }
                else
                {
                    trackers[cid].mean.theta=0;
                }

                trackers[cid].mean.wl=0;
                trackers[cid].mean.wr=0;
                trackers[cid].mean.lf=0;
                trackers[cid].mean.lb=0;
                trackers[cid].beamcount=0;

                weightsum=0;
                minstate=particles[i].state;
                maxstate=particles[i].state;
            }
            else
            {
                break;
            }
        }

        weightsum+=particles[i].weight;

        if(!rbflag)
        {
            trackers[cid].mean.theta+=particles[i].state.theta*particles[i].weight;
            minstate.theta=minstate.theta<particles[i].state.theta?minstate.theta:particles[i].state.theta;
            maxstate.theta=maxstate.theta>particles[i].state.theta?maxstate.theta:particles[i].state.theta;
        }

        trackers[cid].mean.wl+=particles[i].state.wl*particles[i].weight;
        trackers[cid].mean.wr+=particles[i].state.wr*particles[i].weight;
        trackers[cid].mean.lf+=particles[i].state.lf*particles[i].weight;
        trackers[cid].mean.lb+=particles[i].state.lb*particles[i].weight;
        trackers[cid].beamcount+=particles[i].beamcount*particles[i].weight;

        minstate.wl=minstate.wl<particles[i].state.wl?minstate.wl:particles[i].state.wl;
        maxstate.wl=maxstate.wl>particles[i].state.wl?maxstate.wl:particles[i].state.wl;
        minstate.wr=minstate.wr<particles[i].state.wr?minstate.wr:particles[i].state.wr;
        maxstate.wr=maxstate.wr>particles[i].state.wr?maxstate.wr:particles[i].state.wr;
        minstate.lf=minstate.lf<particles[i].state.lf?minstate.lf:particles[i].state.lf;
        maxstate.lf=maxstate.lf>particles[i].state.lf?maxstate.lf:particles[i].state.lf;
        minstate.lb=minstate.lb<particles[i].state.lb?minstate.lb:particles[i].state.lb;
        maxstate.lb=maxstate.lb>particles[i].state.lb?maxstate.lb:particles[i].state.lb;
    }
}
