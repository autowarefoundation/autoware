#include"rbsspf_motion.cuh"

//====================================================
//1: init control and particles

__host__
void hostCalculateMotionControl(TrackerSampleControl & control)
{
    double ratio=1,vratio=1,maxratio=1;

    CALRATIO(ratio,vratio,maxratio,control.motionoffset.a,control.motionprec.a);
    CALRATIO(ratio,vratio,maxratio,control.motionoffset.v,control.motionprec.v);
    CALRATIO(ratio,vratio,maxratio,control.motionoffset.omega,control.motionprec.omega);

    control.motioniteration=log(vratio)/log(2);
    control.motionanneal=maxratio*maxratio;
    control.motionannealratio=pow(control.motionanneal,-1/control.motioniteration);

    CALZOOM(control.motionzoom.a,control.motionoffset.a,control.motionprec.a,control.motioniteration);
    CALZOOM(control.motionzoom.v,control.motionoffset.v,control.motionprec.v,control.motioniteration);
    CALZOOM(control.motionzoom.omega,control.motionoffset.omega,control.motionprec.omega,control.motioniteration);
}

__host__
bool hostInitializeMotion(Tracker & tracker, TrackerSampleControl & control)
{
    control.motionmin=MOTIONMIN;
    control.motionmax=MOTIONMAX;
    control.motionprec=MOTIONPREC;

    switch(tracker.status)
    {
    case StatusInitMotion:
        {
            control.pfflag=0;
            control.motionoffset=INITMOTIONOFFSET;
            control.pnum=1;
            hostCalculateMotionControl(control);
        }
        break;
    case StatusUpdateTracker_SSPF:
        {
            control.pfflag=0;
            control.motionoffset=UPDATEMOTIONOFFSET_SSPF;
            control.pnum=1;
            hostCalculateMotionControl(control);
        }
        break;
    case StatusUpdateTracker_PF:
        {
            control.pfflag=1;
            control.motionoffset=UPDATEMOTIONOFFSET_PF;
            control.pnum=MRQPN;
            control.motioniteration=0;
        }
        break;
    default:
        {
            control.motioniteration=-1;
            control.pnum=0;
        }
        break;
    }
    return control.motioniteration>=0;
}

__host__
double hostInitMotionEstimation(int trackernum, std::vector<Tracker> & trackers, std::vector<TrackerSampleControl> & controls, int & pnum, std::vector<TrackerParticle> & particles)
{
    double maxmotioniteration=-1;
    pnum=0;
    for(int i=0;i<trackernum;i++)
    {
        controls[i].id=i;
        if(hostInitializeMotion(trackers[i],controls[i]))
        {
            if(maxmotioniteration<controls[i].motioniteration) maxmotioniteration=controls[i].motioniteration;
            for(int j=0;j<controls[i].pnum;j++)
            {
                particles[pnum].state=trackers[i].mean;
                particles[pnum].controlid=i;
                particles[pnum].weight=0;
                particles[pnum].beamcount=0;
                pnum++;
            }
        }
    }
    return maxmotioniteration;
}

//====================================================
//2: upsample

__host__ __device__
void deviceAckermannModel(TrackerParticle & particle, EgoMotion & egomotion)
{
    double c=cos(particle.state.theta);
    double s=sin(particle.state.theta);

    if(particle.state.k==0)
    {
        particle.state.x=particle.state.x+c*particle.state.v*egomotion.dt;
        particle.state.y=particle.state.y+s*particle.state.v*egomotion.dt;
        particle.state.a=0;
    }
    else
    {
        double c0=cos(particle.state.theta+particle.state.a);
        double s0=sin(particle.state.theta+particle.state.a);

        particle.state.omega=particle.state.v*particle.state.k;
        double dtheta=particle.state.omega*egomotion.dt;
        particle.state.theta+=dtheta;

        double c1=cos(particle.state.theta+particle.state.a);
        double s1=sin(particle.state.theta+particle.state.a);
        double R=1/particle.state.k;

        particle.state.x=particle.state.x+R*(-s0+s1);
        particle.state.y=particle.state.y+R*(c0-c1);
    }
}

__host__ __device__
void deviceEgoMotion(TrackerParticle & particle, EgoMotion & egomotion)
{
    double c=cos(egomotion.dtheta);
    double s=sin(egomotion.dtheta);
    double tmpx=c*particle.state.x-s*particle.state.y+egomotion.dx;
    double tmpy=s*particle.state.x+c*particle.state.y+egomotion.dy;
    particle.state.x=tmpx;
    particle.state.y=tmpy;
    particle.state.theta+=egomotion.dtheta;
}

__global__
void kernelMotionUpSample(TrackerParticle *particles, TrackerSampleControl *controls, TrackerParticle *tmpparticles, TrackerParticle *tmpparticles_forward, int tmppnum, thrust::random::minstd_rand *rng, EgoMotion egomotion, int beamnum, int *beamcount)
{
    GetThreadID_1D(tmppid);
    if(tmppid>=tmppnum) return;
    int pid=int(tmppid/SPN);
    int cid=particles[pid].controlid;
    int rid=tmppid%RNGNUM;

    TrackerSampleControl control=controls[cid];
    TrackerParticle particle=particles[pid];

    if(control.motioniteration<1)
    {
        tmpparticles[tmppid]=particle;
        tmpparticles_forward[tmppid]=particle;
        beamcount[tmppid]=0;
        return;
    }

    if(control.pfflag)
    {
        particle.state.v=thrust::random::normal_distribution<double>(particle.state.v,control.motionoffset.v)(rng[rid]);
        particle.state.v=particle.state.v>control.motionmin.v?particle.state.v:control.motionmin.v;
        particle.state.v=particle.state.v<control.motionmax.v?particle.state.v:control.motionmax.v;

        particle.state.omega=thrust::random::normal_distribution<double>(particle.state.omega,control.motionoffset.omega)(rng[rid]);
        particle.state.omega=particle.state.omega>control.motionmin.omega?particle.state.omega:control.motionmin.omega;
        particle.state.omega=particle.state.omega<control.motionmax.omega?particle.state.omega:control.motionmax.omega;
    }
    else
    {
        double vmin=particle.state.v-control.motionoffset.v; vmin=vmin>control.motionmin.v?vmin:control.motionmin.v;
        double vmax=particle.state.v+control.motionoffset.v; vmax=vmax<control.motionmax.v?vmax:control.motionmax.v;
        particle.state.v=thrust::random::uniform_real_distribution<double>(vmin,vmax)(rng[rid]);

        double omegamin=particle.state.omega-control.motionoffset.omega; omegamin=omegamin>control.motionmin.omega?omegamin:control.motionmin.omega;
        double omegamax=particle.state.omega+control.motionoffset.omega; omegamax=omegamax<control.motionmax.omega?omegamax:control.motionmax.omega;
        particle.state.omega=thrust::random::uniform_real_distribution<double>(omegamin,omegamax)(rng[rid]);
    }

    if(particle.state.v==0)
    {
        particle.state.k=0;
    }
    else
    {
        particle.state.k=particle.state.omega/particle.state.v;
        particle.state.k=particle.state.k>control.motionmin.k?particle.state.k:control.motionmin.k;
        particle.state.k=particle.state.k<control.motionmax.k?particle.state.k:control.motionmax.k;
    }
    particle.state.omega=particle.state.v*particle.state.k;

    if(control.pfflag)
    {
        particle.state.a=thrust::random::normal_distribution<double>(particle.state.a,control.motionoffset.a)(rng[rid]);
        particle.state.a=particle.state.a>control.motionmin.a?particle.state.a:control.motionmin.a;
        particle.state.a=particle.state.a<control.motionmax.a?particle.state.a:control.motionmax.a;
    }
    else
    {
        double R,phi;

        if(particle.state.k!=0)
        {
            R=1/fabs(particle.state.k);
            phi=atan2(6.0,R);
        }

        double amin,amax;
        if(particle.state.omega>0)
        {
            amin=DEG2RAD(-30);
            amax=phi;
            amax=amax>amin?amax:amin;
        }
        else if(particle.state.omega<0)
        {
            amax=DEG2RAD(30);
            amin=-phi;
            amin=amin<amax?amin:amax;
        }
        else
        {
            amin=0;
            amax=0;
        }
        particle.state.a=thrust::random::uniform_real_distribution<double>(amin,amax)(rng[rid]);
    }

    tmpparticles[tmppid]=particle;

    deviceAckermannModel(particle,egomotion);
    deviceEgoMotion(particle,egomotion);
    deviceBuildModel(particle,beamnum);

    tmpparticles_forward[tmppid]=particle;

    tmpparticles[tmppid].geometry.validflag=particle.geometry.validflag;

    beamcount[tmppid]=particle.geometry.beamcount;
}

//====================================================
//8: estimate tracker

__host__
void hostEstimateMotionTracker(int pnum, std::vector<TrackerParticle> & particles, std::vector<Tracker> & trackers, int beamnum)
{
    TrackerState minstate;
    TrackerState maxstate;
    double weightsum;
    int cid=-1;
    for(int i=0;i<=pnum;i++)
    {
        bool flag=(i>=pnum)||(cid!=particles[i].controlid);
        if(flag)
        {
            if(cid>=0)
            {
                trackers[cid].mean.x/=weightsum;
                trackers[cid].mean.y/=weightsum;
                trackers[cid].mean.theta/=weightsum;
                trackers[cid].mean.a/=weightsum;
                trackers[cid].mean.v/=weightsum;
                trackers[cid].mean.k/=weightsum;
                trackers[cid].mean.omega/=weightsum;
                trackers[cid].beamcount/=weightsum;

                trackers[cid].sigma.x=std::max(trackers[cid].mean.x-minstate.x,maxstate.x-trackers[cid].mean.x);
                trackers[cid].sigma.y=std::max(trackers[cid].mean.y-minstate.y,maxstate.y-trackers[cid].mean.y);
                trackers[cid].sigma.theta=std::max(trackers[cid].mean.theta-minstate.theta,maxstate.theta-trackers[cid].mean.theta);
                trackers[cid].sigma.a=std::max(trackers[cid].mean.a-minstate.a,maxstate.a-trackers[cid].mean.a);
                trackers[cid].sigma.v=std::max(trackers[cid].mean.v-minstate.v,maxstate.v-trackers[cid].mean.v);
                trackers[cid].sigma.k=std::max(trackers[cid].mean.k-minstate.k,maxstate.k-trackers[cid].mean.k);
                trackers[cid].sigma.omega=std::max(trackers[cid].mean.omega-minstate.omega,maxstate.omega-trackers[cid].mean.omega);

                if(trackers[cid].sigma.x<SSPF_SIGMA_X&&trackers[cid].sigma.y<SSPF_SIGMA_Y&&trackers[cid].sigma.theta<SSPF_SIGMA_THETA)
                {
                    if(trackers[cid].beamcount>SSPF_BEAMCOUNT)
                    {
                        trackers[cid].pfcount/=2;
                    }
                    else
                    {
                        trackers[cid].pfcount++;
                    }
                    trackers[cid].status=StatusUpdateTracker_SSPF;
                }
                else
                {
                    trackers[cid].pfcount++;
                    trackers[cid].status=StatusUpdateTracker_PF;
                }
                hostBuildModel(trackers[cid],beamnum);
            }
            if(i<pnum)
            {
                cid=particles[i].controlid;
                trackers[cid].mean.x=0;
                trackers[cid].mean.y=0;
                trackers[cid].mean.theta=0;
                trackers[cid].mean.a=0;
                trackers[cid].mean.v=0;
                trackers[cid].mean.k=0;
                trackers[cid].mean.omega=0;
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

        trackers[cid].mean.x+=particles[i].state.x*particles[i].weight;
        trackers[cid].mean.y+=particles[i].state.y*particles[i].weight;
        trackers[cid].mean.theta+=particles[i].state.theta*particles[i].weight;
        trackers[cid].mean.a+=particles[i].state.a*particles[i].weight;
        trackers[cid].mean.v+=particles[i].state.v*particles[i].weight;
        trackers[cid].mean.k+=particles[i].state.k*particles[i].weight;
        trackers[cid].mean.omega+=particles[i].state.omega;
        trackers[cid].beamcount+=particles[i].beamcount*particles[i].weight;

        minstate.x=minstate.x<particles[i].state.x?minstate.x:particles[i].state.x;
        maxstate.x=maxstate.x>particles[i].state.x?maxstate.x:particles[i].state.x;
        minstate.y=minstate.y<particles[i].state.y?minstate.y:particles[i].state.y;
        maxstate.y=maxstate.y>particles[i].state.y?maxstate.y:particles[i].state.y;
        minstate.theta=minstate.theta<particles[i].state.theta?minstate.theta:particles[i].state.theta;
        maxstate.theta=maxstate.theta>particles[i].state.theta?maxstate.theta:particles[i].state.theta;
        minstate.a=minstate.a<particles[i].state.a?minstate.a:particles[i].state.a;
        maxstate.a=maxstate.a>particles[i].state.a?maxstate.a:particles[i].state.a;
        minstate.v=minstate.v<particles[i].state.v?minstate.v:particles[i].state.v;
        maxstate.v=maxstate.v>particles[i].state.v?maxstate.v:particles[i].state.v;
        minstate.k=minstate.k<particles[i].state.k?minstate.k:particles[i].state.k;
        maxstate.k=maxstate.k>particles[i].state.k?maxstate.k:particles[i].state.k;
        minstate.omega=minstate.omega<particles[i].state.omega?minstate.omega:particles[i].state.omega;
        maxstate.omega=maxstate.omega>particles[i].state.omega?maxstate.omega:particles[i].state.omega;
    }
}
