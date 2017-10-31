#include"rbsspfvehicletracker.cuh"

//==============================================================================

LaserScan * d_scan=NULL;
LaserScan h_scan;
EgoMotion h_egomotion;

//==============================================================================

__host__ __device__
void deviceBuildModel(VehicleState & state, double & density)
{
    double c=cos(state.theta);
    double s=sin(state.theta);

    state.ox=-c*state.x-s*state.y;
    state.oy=s*state.x-c*state.y;

    state.cx[0]=c*state.lf-s*state.wl+state.x; state.cy[0]=s*state.lf+c*state.wl+state.y;
    state.cx[1]=c*state.lf+s*state.wr+state.x; state.cy[1]=s*state.lf-c*state.wr+state.y;
    state.cx[2]=-c*state.lb+s*state.wr+state.x; state.cy[2]=-s*state.lb-c*state.wr+state.y;
    state.cx[3]=-c*state.lb-s*state.wl+state.x; state.cy[3]=-s*state.lb+c*state.wl+state.y;

    state.cl[0]=state.cl[2]=state.wl+state.wr;
    state.cl[1]=state.cl[3]=state.lf+state.lb;

    state.bid[0]=(atan2(state.cy[0],state.cx[0])+PI)/density;
    state.bid[1]=(atan2(state.cy[1],state.cx[1])+PI)/density;
    state.bid[2]=(atan2(state.cy[2],state.cx[2])+PI)/density;
    state.bid[3]=(atan2(state.cy[3],state.cx[3])+PI)/density;

    if(state.ox>state.lf)
    {
        if(state.oy>state.wl)
        {
            state.eid[0]=0;state.eid[1]=3;
        }
        else if(state.oy<-state.wr)
        {
            state.eid[0]=0;state.eid[1]=1;
        }
        else
        {
            state.eid[0]=0;state.eid[1]=-1;
        }
    }
    else if(state.ox<-state.lb)
    {
        if(state.oy>state.wl)
        {
            state.eid[0]=2;state.eid[1]=3;
        }
        else if(state.oy<-state.wr)
        {
            state.eid[0]=2;state.eid[1]=1;
        }
        else
        {
            state.eid[0]=2;state.eid[1]=-1;
        }
    }
    else
    {
        if(state.oy>state.wl)
        {
            state.eid[0]=3;state.eid[1]=-1;
        }
        else if(state.oy<-state.wr)
        {
            state.eid[0]=1;state.eid[1]=-1;
        }
        else
        {
            state.eid[0]=-1;state.eid[1]=-1;
        }
    }
    return;
}

__host__ __device__
void deviceMeasureEdge(VehicleState & state, int edgeid, LaserScan * scan, double anneal, int * beamnum, int * beamid, bool uncertainflag)
{
    if(state.eid[edgeid]<0)
    {
        return;
    }

    if(uncertainflag)
    {
        switch(state.eid[edgeid])
        {
        case 0:
            if(state.dlf>UNCERTAINTHRESH)
            {
                return;
            }
            break;
        case 1:
            if(state.dwr>UNCERTAINTHRESH)
            {
                return;
            }
            break;
        case 2:
            if(state.dlb>UNCERTAINTHRESH)
            {
                return;
            }
            break;
        case 3:
            if(state.dwl>UNCERTAINTHRESH)
            {
                return;
            }
            break;
        default:
            break;
        }
    }

    int starteid=state.eid[edgeid];
    int endeid=(state.eid[edgeid]+1)%4;

    int startbid=state.bid[starteid];
    int endbid=state.bid[endeid];
    if(startbid>endbid)
    {
        endbid+=scan->beamnum;
    }

    int totalbeam=(endbid-startbid)+1;
    if(totalbeam<=UNCERTAINTHRESH_CNT)
    {
        state.eid[edgeid]=-1;
    }

    double dx1=state.cx[endeid]-state.cx[starteid];
    double dy1=state.cy[endeid]-state.cy[starteid];
    double dx2=-dy1/state.cl[starteid];
    double dy2=dx1/state.cl[starteid];

    double density=2*PI/scan->beamnum;
    for(int i=startbid;i<=endbid;i++)
    {
        double P[4]={MAXBEAMLENGTH,MAXBEAMLENGTH,MAXBEAMLENGTH,MAXBEAMLENGTH};
        int tmpid=i%scan->beamnum;
        double bear=tmpid*density-PI;
        double c=cos(bear);
        double s=sin(bear);
        double tmpx=c*dx1+s*dy1;
        double tmpy=s*dx1-c*dy1;
        if(tmpy!=0)
        {
            double beta=tmpx/tmpy*(c*state.cy[starteid]-s*state.cx[starteid])+(c*state.cx[starteid]+s*state.cy[starteid]);
            if(beta>=MINBEAMLENGTH&&beta<=MAXBEAMLENGTH)
            {
                P[2]=beta;
                double gamma0,gamma1,gamma2;
                if(beta<NEARESTRING)
                {
                    gamma0=fabs(beta-(tmpx/tmpy*(c*(state.cy[starteid]+dy2*beta)-s*(state.cx[starteid]+dx2*beta))+c*(state.cx[starteid]+dx2*beta)+s*(state.cy[starteid]+dy2*beta)));
                    gamma1=fabs(beta-(tmpx/tmpy*(c*(state.cy[starteid]+dy2*2)-s*(state.cx[starteid]+dx2*2))+c*(state.cx[starteid]+dx2*2)+s*(state.cy[starteid]+dy2*2)));
                    gamma2=fabs(beta-(tmpx/tmpy*(c*(state.cy[starteid]+dy2*beta)-s*(state.cx[starteid]+dx2*beta))+c*(state.cx[starteid]+dx2*beta)+s*(state.cy[starteid]+dy2*beta)));
                }
                else
                {
                    gamma0=fabs(beta-(tmpx/tmpy*(c*(state.cy[starteid]+dy2*MARGIN0)-s*(state.cx[starteid]+dx2*MARGIN0))+c*(state.cx[starteid]+dx2*MARGIN0)+s*(state.cy[starteid]+dy2*MARGIN0)));
                    gamma1=fabs(beta-(tmpx/tmpy*(c*(state.cy[starteid]+dy2*MARGIN1)-s*(state.cx[starteid]+dx2*MARGIN1))+c*(state.cx[starteid]+dx2*MARGIN1)+s*(state.cy[starteid]+dy2*MARGIN1)));
                    gamma2=fabs(beta-(tmpx/tmpy*(c*(state.cy[starteid]+dy2*MARGIN2)-s*(state.cx[starteid]+dx2*MARGIN2))+c*(state.cx[starteid]+dx2*MARGIN2)+s*(state.cy[starteid]+dy2*MARGIN2)));
                }
                P[1]=P[2]-gamma0>=MINBEAMLENGTH?P[2]-gamma0:MINBEAMLENGTH;
                P[3]=P[2]+gamma1<=MAXBEAMLENGTH?P[2]+gamma1:MAXBEAMLENGTH;
                P[0]=P[2]-gamma2>=MINBEAMLENGTH?P[2]-gamma2:MINBEAMLENGTH;
                double tmplogweight;
                if(scan->length[tmpid]<=P[0])
                {
                    double delta=scan->length[tmpid]-P[0];
                    double w1=WEIGHT0-WEIGHT0;
                    double w2=WEIGHT1-WEIGHT0;
                    tmplogweight=w1+(w2-w1)*exp(-delta*delta/0.01);
                }
                else if(scan->length[tmpid]<=P[1])
                {
                    double delta=scan->length[tmpid]-P[1];
                    double w1=WEIGHT1-WEIGHT0;
                    double w2=WEIGHT2-WEIGHT0;
                    tmplogweight=w1+(w2-w1)*exp(-delta*delta/0.01);
                }
                else if(scan->length[tmpid]<=P[3])
                {
                    if(beta>=NEARESTRING)
                    {
                        if(beamnum!=NULL&&beamid!=NULL&&totalbeam>UNCERTAINTHRESH_CNT)
                        {
                            if((*beamnum)<MAXEDGEPOINT)
                            {
                                beamid[*beamnum]=tmpid;
                                (*beamnum)++;
                            }
                        }
                        state.count++;
                    }
                    double delta=scan->length[tmpid]-P[2];
                    double w1=WEIGHT2-WEIGHT0;
                    double w2=2*w1;
                    tmplogweight=w1+(w2-w1)*exp(-delta*delta/0.01);
                }
                else
                {
                    double delta=scan->length[tmpid]-P[3];
                    double w1=WEIGHT3-WEIGHT0;
                    double w2=WEIGHT2-WEIGHT0;
                    tmplogweight=w1+(w2-w1)*exp(-delta*delta/0.01);
                }
                state.weight+=tmplogweight/anneal;
            }
        }
    }
}

__host__ __device__
void deviceEgoMotion(VehicleState & state, EgoMotion & egomotion)
{
    double c=cos(egomotion.dtheta);
    double s=sin(egomotion.dtheta);
    double tmpx=c*state.x-s*state.y+egomotion.dx;
    double tmpy=s*state.x+c*state.y+egomotion.dy;
    state.x=tmpx;
    state.y=tmpy;
    state.theta+=egomotion.dtheta;
    return;
}

__host__ __device__
void deviceAckermannModel(VehicleState & state0, VehicleState & state1, EgoMotion & egomotion)
{
    state1=state0;
    if(state1.v==0)
    {
        deviceEgoMotion(state1,egomotion);
        return;
    }

    double c=cos(state1.theta);
    double s=sin(state1.theta);

    if(state1.k==0)
    {
        state1.x=state1.x+c*state1.v*egomotion.dt/1000;
        state1.y=state1.y+s*state1.v*egomotion.dt/1000;
        state1.a=0;
        deviceEgoMotion(state1,egomotion);
        return;
    }

    double c0=cos(state1.theta+state1.a);
    double s0=sin(state1.theta+state1.a);
    state1.omega=state1.v*state1.k;
    double dtheta=state1.omega*egomotion.dt/1000;
    state1.theta+=dtheta;
    double c1=cos(state1.theta+state1.a);
    double s1=sin(state1.theta+state1.a);
    double R=1/state1.k;

    state1.x=state1.x+R*(-s0+s1);
    state1.y=state1.y+R*(c0-c1);
    deviceEgoMotion(state1,egomotion);
    return;
}

//==============================================================================

__global__
void kernelSetRandomSeed(int * seed, thrust::minstd_rand * rng, int tmppnum)
{
    GetThreadID_1D(id);
    if(id>=tmppnum)
    {
        return;
    }
    rng[id]=thrust::minstd_rand(seed[id]);
    return;
}

__global__
void kernelGeometryModel(LaserScan * scan, int pnum, VehicleState * particle, int tmppnum, VehicleState * tmpparticle, thrust::minstd_rand * rng, ObjectStateOffset objectstateoffset, StateConstrain stateconstrain, EgoMotion egomotion)
{
    GetThreadID_1D(id);
    if(id>=tmppnum)
    {
        return;
    }
    double index=double(pnum)/double(tmppnum);
    int pid=int(id*index);

    tmpparticle[id]=particle[pid];

    if(objectstateoffset.thetaoff>objectstateoffset.thetaprec)
    {
        double thetamin=tmpparticle[id].theta-objectstateoffset.thetaoff;thetamin=thetamin>stateconstrain.thetamin?thetamin:stateconstrain.thetamin;
        double thetamax=tmpparticle[id].theta+objectstateoffset.thetaoff;thetamax=thetamax<stateconstrain.thetamax?thetamax:stateconstrain.thetamax;
        tmpparticle[id].theta=thrust::random::uniform_real_distribution<double>(thetamin,thetamax)(rng[id]);
    }

    double wlmin=tmpparticle[id].wl-objectstateoffset.wloff;wlmin=wlmin>stateconstrain.wlmin?wlmin:stateconstrain.wlmin;
    double wlmax=tmpparticle[id].wl+objectstateoffset.wloff;wlmax=wlmax<stateconstrain.wlmax?wlmax:stateconstrain.wlmax;
    tmpparticle[id].wl=thrust::random::uniform_real_distribution<double>(wlmin,wlmax)(rng[id]);

    double wrmin=tmpparticle[id].wr-objectstateoffset.wroff;wrmin=wrmin>stateconstrain.wrmin?wrmin:stateconstrain.wrmin;
    double wrmax=tmpparticle[id].wr+objectstateoffset.wroff;wrmax=wrmax<stateconstrain.wrmax?wrmax:stateconstrain.wrmax;
    tmpparticle[id].wr=thrust::random::uniform_real_distribution<double>(wrmin,wrmax)(rng[id]);

    double lfmin=tmpparticle[id].lf-objectstateoffset.lfoff;lfmin=lfmin>stateconstrain.lfmin?lfmin:stateconstrain.lfmin;
    double lfmax=tmpparticle[id].lf+objectstateoffset.lfoff;lfmax=lfmax<stateconstrain.lfmax?lfmax:stateconstrain.lfmax;
    tmpparticle[id].lf=thrust::random::uniform_real_distribution<double>(lfmin,lfmax)(rng[id]);

    double lbmin=tmpparticle[id].lb-objectstateoffset.lboff;lbmin=lbmin>stateconstrain.lbmin?lbmin:stateconstrain.lbmin;
    double lbmax=tmpparticle[id].lb+objectstateoffset.lboff;lbmax=lbmax<stateconstrain.lbmax?lbmax:stateconstrain.lbmax;
    tmpparticle[id].lb=thrust::random::uniform_real_distribution<double>(lbmin,lbmax)(rng[id]);

    deviceBuildModel(tmpparticle[id],egomotion.density);

    tmpparticle[id].weight=0;
    tmpparticle[id].count=0;
    deviceMeasureEdge(tmpparticle[id],0,scan,objectstateoffset.anneal,NULL,NULL,0);
    deviceMeasureEdge(tmpparticle[id],1,scan,objectstateoffset.anneal,NULL,NULL,0);

    return;
}

__global__
void kernelMotionModel(LaserScan * scan, int pnum, VehicleState * particle, int tmppnum, VehicleState * tmpparticle, thrust::minstd_rand * rng, ObjectStateOffset objectstateoffset, StateConstrain stateconstrain, EgoMotion egomotion)
{
    GetThreadID_1D(id);
    if(id>=tmppnum)
    {
        return;
    }
    double index=double(pnum)/double(tmppnum);
    int pid=int(id*index);

    tmpparticle[id]=particle[pid];

    if(egomotion.pfflag)
    {
        tmpparticle[id].v=thrust::random::normal_distribution<double>(tmpparticle[id].v,objectstateoffset.voff)(rng[id]);
        tmpparticle[id].v=tmpparticle[id].v>stateconstrain.vmin?tmpparticle[id].v:stateconstrain.vmin;
        tmpparticle[id].v=tmpparticle[id].v<stateconstrain.vmax?tmpparticle[id].v:stateconstrain.vmax;

        tmpparticle[id].omega=thrust::random::normal_distribution<double>(tmpparticle[id].omega,objectstateoffset.omegaoff)(rng[id]);
        tmpparticle[id].omega=tmpparticle[id].omega>stateconstrain.omegamin?tmpparticle[id].omega:stateconstrain.omegamin;
        tmpparticle[id].omega=tmpparticle[id].omega<stateconstrain.omegamax?tmpparticle[id].omega:stateconstrain.omegamax;
    }
    else
    {
        double vmin=tmpparticle[id].v-objectstateoffset.voff;vmin=vmin>stateconstrain.vmin?vmin:stateconstrain.vmin;
        double vmax=tmpparticle[id].v+objectstateoffset.voff;vmax=vmax<stateconstrain.vmax?vmax:stateconstrain.vmax;
        tmpparticle[id].v=thrust::random::uniform_real_distribution<double>(vmin,vmax)(rng[id]);

        double omegamin=tmpparticle[id].omega-objectstateoffset.omegaoff;omegamin=omegamin>stateconstrain.omegamin?omegamin:stateconstrain.omegamin;
        double omegamax=tmpparticle[id].omega+objectstateoffset.omegaoff;omegamax=omegamax<stateconstrain.omegamax?omegamax:stateconstrain.omegamax;
        tmpparticle[id].omega=thrust::random::uniform_real_distribution<double>(omegamin,omegamax)(rng[id]);
    }

    if(tmpparticle[id].v==0)
    {
        tmpparticle[id].k=(stateconstrain.kmin+stateconstrain.kmax)/2;
    }
    else
    {
        tmpparticle[id].k=tmpparticle[id].omega/tmpparticle[id].v;
        if(tmpparticle[id].k<stateconstrain.kmin)
        {
            tmpparticle[id].k=stateconstrain.kmin;
        }
        if(tmpparticle[id].k>stateconstrain.kmax)
        {
            tmpparticle[id].k=stateconstrain.kmax;
        }
    }
    tmpparticle[id].omega=tmpparticle[id].v*tmpparticle[id].k;

    double R,phi;
    if(tmpparticle[id].k!=0)
    {
        R=1/fabs(tmpparticle[id].k);
        phi=atan2(4.0,R);
    }

    if(tmpparticle[id].omega>0)
    {
        stateconstrain.amin=-MAXANGLEOFFSET;
        stateconstrain.amax=phi;
        stateconstrain.amax=stateconstrain.amax>stateconstrain.amin?stateconstrain.amax:stateconstrain.amin;
    }
    else if(tmpparticle[id].omega<0)
    {
        stateconstrain.amax=MAXANGLEOFFSET;
        stateconstrain.amin=-phi;
        stateconstrain.amin=stateconstrain.amin<stateconstrain.amax?stateconstrain.amin:stateconstrain.amax;
    }
    else if(tmpparticle[id].omega==0)
    {
        stateconstrain.amin=0;
        stateconstrain.amax=0;
    }

    if(egomotion.pfflag)
    {
        tmpparticle[id].a=thrust::random::normal_distribution<double>(tmpparticle[id].a,objectstateoffset.aoff)(rng[id]);
        tmpparticle[id].a=tmpparticle[id].a>stateconstrain.amin?tmpparticle[id].a:stateconstrain.amin;
        tmpparticle[id].a=tmpparticle[id].a<stateconstrain.amax?tmpparticle[id].a:stateconstrain.amax;
    }
    else
    {
        double amin=tmpparticle[id].a-objectstateoffset.aoff;amin=amin>stateconstrain.amin?amin:stateconstrain.amin;
        double amax=tmpparticle[id].a+objectstateoffset.aoff;amax=amax<stateconstrain.amax?amax:stateconstrain.amax;
        tmpparticle[id].a=thrust::random::uniform_real_distribution<double>(amin,amax)(rng[id]);
    }

    VehicleState movedparticle;
    deviceAckermannModel(tmpparticle[id],movedparticle,egomotion);
    deviceBuildModel(movedparticle,egomotion.density);

    movedparticle.weight=0;
    movedparticle.count=0;
    deviceMeasureEdge(movedparticle,0,scan,objectstateoffset.anneal,NULL,NULL,1);
    deviceMeasureEdge(movedparticle,1,scan,objectstateoffset.anneal,NULL,NULL,1);
    tmpparticle[id].weight=movedparticle.weight;
    tmpparticle[id].count=movedparticle.count;

    return;
}

__global__
void kernelMotionUpdate(int pnum, VehicleState * particle, EgoMotion egomotion)
{
    GetThreadID_1D(id);
    if(id>=pnum)
    {
        return;
    }
    deviceAckermannModel(particle[id],particle[id],egomotion);
    deviceBuildModel(particle[id],egomotion.density);
}

//==============================================================================

void sampleParticle(int & pnum, VehicleState * d_particle, int & tmppnum, VehicleState * d_tmpparticle, VehicleState & estimate)
{
    VehicleState h_particle[RQPN];
    VehicleState h_tmpparticle[MAXPN];
    bool h_flag[MAXPN];

    cudaMemcpy(h_tmpparticle,d_tmpparticle,sizeof(VehicleState)*tmppnum,cudaMemcpyDeviceToHost);

    double maxlogweight=h_tmpparticle[0].weight;
    double minlogweight=h_tmpparticle[0].weight;
    for(int j=0;j<tmppnum;j++)
    {
        if(maxlogweight<h_tmpparticle[j].weight)
        {
            maxlogweight=h_tmpparticle[j].weight;
        }
        if(minlogweight>h_tmpparticle[j].weight)
        {
            minlogweight=h_tmpparticle[j].weight;
        }
        h_flag[j]=1;
    }

    double maxscale=maxlogweight<=30?1:30/maxlogweight;
    double minscale=minlogweight>=-30?1:-30/minlogweight;
    double scale=maxscale<minscale?maxscale:minscale;
    for(int j=0;j<tmppnum;j++)
    {
        h_tmpparticle[j].weight=exp(h_tmpparticle[j].weight*scale);
        if(j>0)
        {
            h_tmpparticle[j].weight+=h_tmpparticle[j-1].weight;
        }
    }

    int planpnum=tmppnum<RQPN?tmppnum:RQPN;
    double weightstep=1.0/planpnum;
    int accuracy=1000000;
    double samplebase=(rand()%accuracy)*weightstep/accuracy;
    double weightsum=h_tmpparticle[tmppnum-1].weight;
    pnum=0;

    estimate.weight=0;
    estimate.x=0;estimate.y=0;estimate.theta=0;
    estimate.wl=0;estimate.wr=0;estimate.lf=0;estimate.lb=0;
    estimate.a=0;estimate.v=0;estimate.k=0;estimate.omega=0;
    estimate.count=0;

    VehicleState minstate,maxstate;
    for(int j=0, k=0;j<planpnum;j++)
    {
        double sample=samplebase+j*weightstep;
        while(k<tmppnum)
        {
            if(sample>h_tmpparticle[k].weight/weightsum)
            {
                k++;
                continue;
            }
            if(h_flag[k])
            {
                h_flag[k]=0;
                h_particle[pnum]=h_tmpparticle[k];
                h_particle[pnum].weight=weightstep;
                if(pnum==0)
                {
                    minstate.x=h_particle[pnum].x;maxstate.x=h_particle[pnum].x;
                    minstate.y=h_particle[pnum].y;maxstate.y=h_particle[pnum].y;
                    minstate.theta=h_particle[pnum].theta;maxstate.theta=h_particle[pnum].theta;
                    minstate.wl=h_particle[pnum].wl;maxstate.wl=h_particle[pnum].wl;
                    minstate.wr=h_particle[pnum].wr;maxstate.wr=h_particle[pnum].wr;
                    minstate.lf=h_particle[pnum].lf;maxstate.lf=h_particle[pnum].lf;
                    minstate.lb=h_particle[pnum].lb;maxstate.lb=h_particle[pnum].lb;
                    minstate.a=h_particle[pnum].a;maxstate.a=h_particle[pnum].a;
                    minstate.v=h_particle[pnum].v;maxstate.v=h_particle[pnum].v;
                    minstate.k=h_particle[pnum].k;maxstate.k=h_particle[pnum].k;
                    minstate.omega=h_particle[pnum].omega;maxstate.omega=h_particle[pnum].omega;
                }
                else
                {
                    minstate.x=minstate.x<h_particle[pnum].x?minstate.x:h_particle[pnum].x;
                    maxstate.x=maxstate.x>h_particle[pnum].x?maxstate.x:h_particle[pnum].x;
                    minstate.y=minstate.y<h_particle[pnum].y?minstate.y:h_particle[pnum].y;
                    maxstate.y=maxstate.y>h_particle[pnum].y?maxstate.y:h_particle[pnum].y;
                    minstate.theta=minstate.theta<h_particle[pnum].theta?minstate.theta:h_particle[pnum].theta;
                    maxstate.theta=maxstate.theta>h_particle[pnum].theta?maxstate.theta:h_particle[pnum].theta;
                    minstate.wl=minstate.wl<h_particle[pnum].wl?minstate.wl:h_particle[pnum].wl;
                    maxstate.wl=maxstate.wl>h_particle[pnum].wl?maxstate.wl:h_particle[pnum].wl;
                    minstate.wr=minstate.wr<h_particle[pnum].wr?minstate.wr:h_particle[pnum].wr;
                    maxstate.wr=maxstate.wr>h_particle[pnum].wr?maxstate.wr:h_particle[pnum].wr;
                    minstate.lf=minstate.lf<h_particle[pnum].lf?minstate.lf:h_particle[pnum].lf;
                    maxstate.lf=maxstate.lf>h_particle[pnum].lf?maxstate.lf:h_particle[pnum].lf;
                    minstate.lb=minstate.lb<h_particle[pnum].lb?minstate.lb:h_particle[pnum].lb;
                    maxstate.lb=maxstate.lb>h_particle[pnum].lb?maxstate.lb:h_particle[pnum].lb;
                    minstate.a=minstate.a<h_particle[pnum].a?minstate.a:h_particle[pnum].a;
                    maxstate.a=maxstate.a>h_particle[pnum].a?maxstate.a:h_particle[pnum].a;
                    minstate.v=minstate.v<h_particle[pnum].v?minstate.v:h_particle[pnum].v;
                    maxstate.v=maxstate.v>h_particle[pnum].v?maxstate.v:h_particle[pnum].v;
                    minstate.k=minstate.k<h_particle[pnum].k?minstate.k:h_particle[pnum].k;
                    maxstate.k=maxstate.k>h_particle[pnum].k?maxstate.k:h_particle[pnum].k;
                    minstate.omega=minstate.omega<h_particle[pnum].omega?minstate.omega:h_particle[pnum].omega;
                    maxstate.omega=maxstate.omega>h_particle[pnum].omega?maxstate.omega:h_particle[pnum].omega;
                }
                pnum++;
            }
            else
            {
                h_particle[pnum-1].weight+=weightstep;
            }
            estimate.weight+=weightstep;
            estimate.x+=h_particle[pnum-1].x*weightstep;
            estimate.y+=h_particle[pnum-1].y*weightstep;
            estimate.theta+=h_particle[pnum-1].theta*weightstep;
            estimate.wl+=h_particle[pnum-1].wl*weightstep;
            estimate.wr+=h_particle[pnum-1].wr*weightstep;
            estimate.lf+=h_particle[pnum-1].lf*weightstep;
            estimate.lb+=h_particle[pnum-1].lb*weightstep;
            estimate.a+=h_particle[pnum-1].a*weightstep;
            estimate.v+=h_particle[pnum-1].v*weightstep;
            estimate.k+=h_particle[pnum-1].k*weightstep;
            estimate.omega+=h_particle[pnum-1].omega*weightstep;
            estimate.count+=h_particle[pnum-1].count*weightstep;

            break;
        }
    }

    estimate.x/=estimate.weight;
    estimate.y/=estimate.weight;
    estimate.theta/=estimate.weight;
    estimate.wl/=estimate.weight;
    estimate.wr/=estimate.weight;
    estimate.lf/=estimate.weight;
    estimate.lb/=estimate.weight;
    estimate.a/=estimate.weight;
    estimate.v/=estimate.weight;
    estimate.k/=estimate.weight;
    estimate.omega=estimate.v*estimate.k;
    estimate.count/=estimate.weight;
    estimate.weight/=estimate.weight;

    estimate.dx=std::max(estimate.x-minstate.x,maxstate.x-estimate.x);
    estimate.dy=std::max(estimate.y-minstate.y,maxstate.y-estimate.y);
    estimate.dtheta=std::max(estimate.theta-minstate.theta,maxstate.theta-estimate.theta);
    estimate.dwl=std::max(estimate.wl-minstate.wl,maxstate.wl-estimate.wl);
    estimate.dwr=std::max(estimate.wr-minstate.wr,maxstate.wr-estimate.wr);
    estimate.dlf=std::max(estimate.lf-minstate.lf,maxstate.lf-estimate.lf);
    estimate.dlb=std::max(estimate.lb-minstate.lb,maxstate.lb-estimate.lb);
    estimate.da=std::max(estimate.a-minstate.a,maxstate.a-estimate.a);
    estimate.dv=std::max(estimate.v-minstate.v,maxstate.v-estimate.v);
    estimate.dk=std::max(estimate.k-minstate.k,maxstate.k-estimate.k);
    estimate.domega=std::max(estimate.omega-minstate.omega,maxstate.omega-estimate.omega);

    deviceBuildModel(estimate,h_egomotion.density);

    cudaMemcpy(d_particle,h_particle,sizeof(VehicleState)*pnum,cudaMemcpyHostToDevice);
    return;
}

#define CALRATIO(ratio, vratio, maxratio, maxrange, minrange) \
    ratio=maxrange/minrange; vratio*=ratio; maxratio=ratio>maxratio?ratio:maxratio;
#define CALZOOM(zoom, maxrange, minrange, N) \
    zoom=log(maxrange/minrange)/log(2)/N;zoom=1/pow(2,zoom);

void SSPF_GeometryModel(LaserScan * scan, int & pnum, VehicleState * d_particle, VehicleState * d_tmpparticle, thrust::minstd_rand * d_rng, VehicleState & estimate, ObjectStateOffset & objectstateoffset, EgoMotion & egomotion)
{
    double ratio=1,vratio=1,maxratio=1;
    CALRATIO(ratio,vratio,maxratio,objectstateoffset.thetaoff,objectstateoffset.thetaprec);
    CALRATIO(ratio,vratio,maxratio,objectstateoffset.wloff,objectstateoffset.wlprec);
    CALRATIO(ratio,vratio,maxratio,objectstateoffset.wroff,objectstateoffset.wrprec);
    CALRATIO(ratio,vratio,maxratio,objectstateoffset.lfoff,objectstateoffset.lfprec);
    CALRATIO(ratio,vratio,maxratio,objectstateoffset.lboff,objectstateoffset.lbprec);
    objectstateoffset.anneal=maxratio*maxratio;
    double N=log(vratio)/log(2);

    CALZOOM(objectstateoffset.thetazoom,objectstateoffset.thetaoff,objectstateoffset.thetaprec,N);
    CALZOOM(objectstateoffset.wlzoom,objectstateoffset.wloff,objectstateoffset.wlprec,N);
    CALZOOM(objectstateoffset.wrzoom,objectstateoffset.wroff,objectstateoffset.wrprec,N);
    CALZOOM(objectstateoffset.lfzoom,objectstateoffset.lfoff,objectstateoffset.lfprec,N);
    CALZOOM(objectstateoffset.lbzoom,objectstateoffset.lboff,objectstateoffset.lbprec,N);
    objectstateoffset.annealratio=pow(objectstateoffset.anneal,-1/N);

    StateConstrain stateconstrain;
    stateconstrain.thetamin=estimate.theta-objectstateoffset.thetaoff;
    stateconstrain.thetamax=estimate.theta+objectstateoffset.thetaoff;

    int tmppnum;
    for(int i=1;i<=N;i++)
    {
        tmppnum=pnum*SPN;

        GetKernelDim_1D(blocknum,threadnum,tmppnum);
        kernelGeometryModel<<<blocknum,threadnum>>>(scan,pnum,d_particle,tmppnum,d_tmpparticle,d_rng,objectstateoffset,stateconstrain,egomotion);
        sampleParticle(pnum,d_particle,tmppnum,d_tmpparticle,estimate);

        objectstateoffset.thetaoff*=objectstateoffset.thetazoom;
        objectstateoffset.wloff*=objectstateoffset.wlzoom;
        objectstateoffset.wroff*=objectstateoffset.wrzoom;
        objectstateoffset.lfoff*=objectstateoffset.lfzoom;
        objectstateoffset.lboff*=objectstateoffset.lbzoom;
        objectstateoffset.anneal*=objectstateoffset.annealratio;
    }
    {
        objectstateoffset.thetaoff=objectstateoffset.thetaprec;
        objectstateoffset.wloff=objectstateoffset.wlprec;
        objectstateoffset.wroff=objectstateoffset.wrprec;
        objectstateoffset.lfoff=objectstateoffset.lfprec;
        objectstateoffset.lboff=objectstateoffset.lbprec;
        objectstateoffset.anneal=1;
        tmppnum=pnum*SPN;
        GetKernelDim_1D(blocknum,threadnum,tmppnum);
        kernelGeometryModel<<<blocknum,threadnum>>>(scan,pnum,d_particle,tmppnum,d_tmpparticle,d_rng,objectstateoffset,stateconstrain,egomotion);
        sampleParticle(pnum,d_particle,tmppnum,d_tmpparticle,estimate);
    }
}

void SSPF_MotionModel(LaserScan * scan, int & pnum, VehicleState * d_particle, VehicleState * d_tmpparticle, thrust::minstd_rand * d_rng, VehicleState & estimate, ObjectStateOffset & objectstateoffset, EgoMotion & egomotion)
{
    double ratio=1,vratio=1,maxratio=1;
    CALRATIO(ratio,vratio,maxratio,objectstateoffset.aoff,objectstateoffset.aprec);
    CALRATIO(ratio,vratio,maxratio,objectstateoffset.voff,objectstateoffset.vprec);
    CALRATIO(ratio,vratio,maxratio,objectstateoffset.omegaoff,objectstateoffset.omegaprec);
    objectstateoffset.anneal=maxratio*maxratio;
    double N=log(vratio)/log(2);

    CALZOOM(objectstateoffset.azoom,objectstateoffset.aoff,objectstateoffset.aprec,N);
    CALZOOM(objectstateoffset.vzoom,objectstateoffset.voff,objectstateoffset.vprec,N);
    CALZOOM(objectstateoffset.omegazoom,objectstateoffset.omegaoff,objectstateoffset.omegaprec,N);
    objectstateoffset.annealratio=pow(objectstateoffset.anneal,-1/N);

    StateConstrain stateconstrain;
    if(!(egomotion.pfflag))
    {
        stateconstrain.amin=std::max(stateconstrain.amin,estimate.a-objectstateoffset.aoff);
        stateconstrain.amax=std::min(stateconstrain.amax,estimate.a+objectstateoffset.aoff);
        stateconstrain.vmin=std::max(stateconstrain.vmin,estimate.v-objectstateoffset.voff);
        stateconstrain.vmax=std::min(stateconstrain.vmax,estimate.v+objectstateoffset.voff);
        stateconstrain.kmin=std::max(stateconstrain.kmin,estimate.k-objectstateoffset.koff);
        stateconstrain.kmax=std::min(stateconstrain.kmax,estimate.k+objectstateoffset.koff);
        stateconstrain.omegamin=std::max(stateconstrain.omegamin,estimate.omega-objectstateoffset.omegaoff);
        stateconstrain.omegamax=std::min(stateconstrain.omegamax,estimate.omega+objectstateoffset.omegaoff);
    }

    int tmppnum;
    for(int i=1;i<=N&&!(egomotion.pfflag);i++)
    {
        tmppnum=pnum*SPN;

        GetKernelDim_1D(blocknum,threadnum,tmppnum);
        kernelMotionModel<<<blocknum,threadnum>>>(scan,pnum,d_particle,tmppnum,d_tmpparticle,d_rng,objectstateoffset,stateconstrain,egomotion);
        sampleParticle(pnum,d_particle,tmppnum,d_tmpparticle,estimate);

        objectstateoffset.aoff*=objectstateoffset.azoom;
        objectstateoffset.voff*=objectstateoffset.vzoom;
        objectstateoffset.omegaoff*=objectstateoffset.omegazoom;
        objectstateoffset.anneal*=objectstateoffset.annealratio;
    }
    {
        if(!(egomotion.pfflag))
        {
            objectstateoffset.aoff=objectstateoffset.aprec;
            objectstateoffset.voff=objectstateoffset.vprec;
            objectstateoffset.omegaoff=objectstateoffset.omegaprec;
            objectstateoffset.anneal=1;
            tmppnum=pnum*SPN;
        }
        else
        {
            objectstateoffset.anneal=1;
            tmppnum=MAXPN;
        }
        GetKernelDim_1D(blocknum,threadnum,tmppnum);
        kernelMotionModel<<<blocknum,threadnum>>>(scan,pnum,d_particle,tmppnum,d_tmpparticle,d_rng,objectstateoffset,stateconstrain,egomotion);
        kernelMotionUpdate<<<blocknum,threadnum>>>(tmppnum,d_tmpparticle,egomotion);
        sampleParticle(pnum,d_particle,tmppnum,d_tmpparticle,estimate);
    }
}

//==============================================================================

extern "C" void cuda_InitLaserScan()
{
    if(d_scan==NULL)
    {
        cudaMalloc(&(d_scan),sizeof(LaserScan));
    }
}

extern "C" void cuda_SetLaserScan(LaserScan & laserScan)
{
    cudaMemcpy(d_scan,&laserScan,sizeof(LaserScan),cudaMemcpyHostToDevice);
    h_scan=laserScan;
    if(h_egomotion.validflag)
    {
        double tmpdx=h_egomotion.x-laserScan.x;
        double tmpdy=h_egomotion.y-laserScan.y;
        double c=cos(laserScan.theta);
        double s=sin(laserScan.theta);
        h_egomotion.dx=c*tmpdx+s*tmpdy;
        h_egomotion.dy=-s*tmpdx+c*tmpdy;
        h_egomotion.dtheta=h_egomotion.theta-laserScan.theta;
        h_egomotion.dt=laserScan.timestamp-h_egomotion.timestamp;
    }
    h_egomotion.x=laserScan.x;
    h_egomotion.y=laserScan.y;
    h_egomotion.theta=laserScan.theta;
    h_egomotion.timestamp=laserScan.timestamp;
    h_egomotion.validflag=1;
    h_egomotion.density=2*PI/laserScan.beamnum;
}

extern "C" void cuda_FreeLaserScan()
{
    CUDAFREE(d_scan);
}

//==============================================================================

extern "C" void cuda_OpenTracker(TrackerDataContainer & trackerDataContainer)
{
    trackerDataContainer.pnum=0;
    cudaMalloc(&(trackerDataContainer.d_particle),sizeof(VehicleState)*RQPN);
    cudaMalloc(&(trackerDataContainer.d_tmpparticle),sizeof(VehicleState)*MAXPN);
    cudaMalloc(&(trackerDataContainer.d_rng),sizeof(thrust::minstd_rand)*MAXPN);

    int h_seed[MAXPN];
    thrust::generate(h_seed,h_seed+MAXPN,rand);
    int * d_seed;
    cudaMalloc(&(d_seed),sizeof(int)*MAXPN);
    cudaMemcpy(d_seed,h_seed,sizeof(int)*MAXPN,cudaMemcpyHostToDevice);
    GetKernelDim_1D(blocks,threads,MAXPN);
    kernelSetRandomSeed<<<blocks,threads>>>(d_seed,trackerDataContainer.d_rng,MAXPN);
    CUDAFREE(d_seed);
}

extern "C" void cuda_CloseTracker(TrackerDataContainer & trackerDataContainer)
{
    CUDAFREE(trackerDataContainer.d_particle);
    CUDAFREE(trackerDataContainer.d_tmpparticle);
    CUDAFREE(trackerDataContainer.d_rng);
}

//==============================================================================

extern "C" void cuda_InitGeometry(TrackerDataContainer & trackerDataContainer, TrackerResultContainer & trackerResultContainer)
{
    ObjectStateOffset objectstateoffset;

    EgoMotion egomotion=h_egomotion;
    egomotion.pfflag=0;

    trackerDataContainer.pnum=1;
    cudaMemcpy(trackerDataContainer.d_particle,&(trackerResultContainer.estimate),sizeof(VehicleState),cudaMemcpyHostToDevice);

    SSPF_GeometryModel(d_scan,trackerDataContainer.pnum,trackerDataContainer.d_particle,trackerDataContainer.d_tmpparticle,trackerDataContainer.d_rng,trackerResultContainer.estimate,objectstateoffset,egomotion);

    trackerResultContainer.estimate.dwl=trackerResultContainer.estimate.dwl>MINSIGMA?trackerResultContainer.estimate.dwl:MINSIGMA;
    trackerResultContainer.estimate.dwr=trackerResultContainer.estimate.dwr>MINSIGMA?trackerResultContainer.estimate.dwr:MINSIGMA;
    trackerResultContainer.estimate.dlf=trackerResultContainer.estimate.dlf>MINSIGMA?trackerResultContainer.estimate.dlf:MINSIGMA;
    trackerResultContainer.estimate.dlb=trackerResultContainer.estimate.dlb>MINSIGMA?trackerResultContainer.estimate.dlb:MINSIGMA;

    trackerResultContainer.estimate.dwl=trackerResultContainer.estimate.dwl<UNCERTAINTHRESH?trackerResultContainer.estimate.dwl:MAXSIGMA;
    trackerResultContainer.estimate.dwr=trackerResultContainer.estimate.dwr<UNCERTAINTHRESH?trackerResultContainer.estimate.dwr:MAXSIGMA;
    trackerResultContainer.estimate.dlf=trackerResultContainer.estimate.dlf<UNCERTAINTHRESH?trackerResultContainer.estimate.dlf:MAXSIGMA;
    trackerResultContainer.estimate.dlb=trackerResultContainer.estimate.dlb<UNCERTAINTHRESH?trackerResultContainer.estimate.dlb:MAXSIGMA;

    deviceBuildModel(trackerResultContainer.estimate,egomotion.density);
    trackerResultContainer.estimate.weight=0;
    trackerResultContainer.estimate.count=0;
    trackerResultContainer.edgepointnum[0]=0;
    deviceMeasureEdge(trackerResultContainer.estimate,0,&h_scan,1,&(trackerResultContainer.edgepointnum[0]),trackerResultContainer.edgepointid[0],1);
    trackerResultContainer.edgepointnum[1]=0;
    deviceMeasureEdge(trackerResultContainer.estimate,1,&h_scan,1,&(trackerResultContainer.edgepointnum[1]),trackerResultContainer.edgepointid[1],1);
}

extern "C" void cuda_InitMotion(TrackerDataContainer & trackerDataContainer, TrackerResultContainer & trackerResultContainer)
{
    VehicleState preestimate=trackerResultContainer.estimate;
    VehicleState curestimate=preestimate;

    ObjectStateOffset objectstateoffset;
    objectstateoffset.thetaoff=objectstateoffset.thetaprec;
    if(preestimate.dwl<objectstateoffset.wlprec)
    {
        objectstateoffset.wloff=objectstateoffset.wlprec;
    }
    if(preestimate.dwr<objectstateoffset.wrprec)
    {
        objectstateoffset.wroff=objectstateoffset.wrprec;
    }
    if(preestimate.dlf<objectstateoffset.lfprec)
    {
        objectstateoffset.lfoff=objectstateoffset.lfprec;
    }
    if(preestimate.dlb<objectstateoffset.lbprec)
    {
        objectstateoffset.lboff=objectstateoffset.lbprec;
    }

    EgoMotion egomotion=h_egomotion;
    egomotion.pfflag=0;

    trackerDataContainer.pnum=1;
    cudaMemcpy(trackerDataContainer.d_particle,&(preestimate),sizeof(VehicleState),cudaMemcpyHostToDevice);

    SSPF_MotionModel(d_scan,trackerDataContainer.pnum,trackerDataContainer.d_particle,trackerDataContainer.d_tmpparticle,trackerDataContainer.d_rng,curestimate,objectstateoffset,egomotion);

    double dx=curestimate.dx;
    double dy=curestimate.dy;
    double dtheta=curestimate.dtheta;

    trackerDataContainer.pnum=1;
    cudaMemcpy(trackerDataContainer.d_particle,&(curestimate),sizeof(VehicleState),cudaMemcpyHostToDevice);

    SSPF_GeometryModel(d_scan,trackerDataContainer.pnum,trackerDataContainer.d_particle,trackerDataContainer.d_tmpparticle,trackerDataContainer.d_rng,curestimate,objectstateoffset,egomotion);

    trackerResultContainer.estimate=curestimate;

    curestimate.dwl=curestimate.dwl>MINSIGMA?curestimate.dwl:MINSIGMA;
    curestimate.dwr=curestimate.dwr>MINSIGMA?curestimate.dwr:MINSIGMA;
    curestimate.dlf=curestimate.dlf>MINSIGMA?curestimate.dlf:MINSIGMA;
    curestimate.dlb=curestimate.dlb>MINSIGMA?curestimate.dlb:MINSIGMA;

    curestimate.dwl=curestimate.dwl<UNCERTAINTHRESH?curestimate.dwl:MAXSIGMA;
    curestimate.dwr=curestimate.dwr<UNCERTAINTHRESH?curestimate.dwr:MAXSIGMA;
    curestimate.dlf=curestimate.dlf<UNCERTAINTHRESH?curestimate.dlf:MAXSIGMA;
    curestimate.dlb=curestimate.dlb<UNCERTAINTHRESH?curestimate.dlb:MAXSIGMA;

    trackerResultContainer.estimate.dx=dx;trackerResultContainer.estimate.dy=dy;trackerResultContainer.estimate.dtheta=dtheta;

    trackerResultContainer.estimate.wl=(preestimate.wl*curestimate.dwl*curestimate.dwl+curestimate.wl*preestimate.dwl*preestimate.dwl)/(preestimate.dwl*preestimate.dwl+curestimate.dwl*curestimate.dwl);
    trackerResultContainer.estimate.dwl=sqrt((preestimate.dwl*preestimate.dwl*curestimate.dwl*curestimate.dwl)/(preestimate.dwl*preestimate.dwl+curestimate.dwl*curestimate.dwl));
    trackerResultContainer.estimate.dwl=trackerResultContainer.estimate.dwl>MINSIGMA?trackerResultContainer.estimate.dwl:MINSIGMA;

    trackerResultContainer.estimate.wr=(preestimate.wr*curestimate.dwr*curestimate.dwr+curestimate.wr*preestimate.dwr*preestimate.dwr)/(preestimate.dwr*preestimate.dwr+curestimate.dwr*curestimate.dwr);
    trackerResultContainer.estimate.dwr=sqrt((preestimate.dwr*preestimate.dwr*curestimate.dwr*curestimate.dwr)/(preestimate.dwr*preestimate.dwr+curestimate.dwr*curestimate.dwr));
    trackerResultContainer.estimate.dwr=trackerResultContainer.estimate.dwr>MINSIGMA?trackerResultContainer.estimate.dwr:MINSIGMA;

    trackerResultContainer.estimate.lf=(preestimate.lf*curestimate.dlf*curestimate.dlf+curestimate.lf*preestimate.dlf*preestimate.dlf)/(preestimate.dlf*preestimate.dlf+curestimate.dlf*curestimate.dlf);
    trackerResultContainer.estimate.dlf=sqrt((preestimate.dlf*preestimate.dlf*curestimate.dlf*curestimate.dlf)/(preestimate.dlf*preestimate.dlf+curestimate.dlf*curestimate.dlf));
    trackerResultContainer.estimate.dlf=trackerResultContainer.estimate.dlf>MINSIGMA?trackerResultContainer.estimate.dlf:MINSIGMA;

    trackerResultContainer.estimate.lb=(preestimate.lb*curestimate.dlb*curestimate.dlb+curestimate.lb*preestimate.dlb*preestimate.dlb)/(preestimate.dlb*preestimate.dlb+curestimate.dlb*curestimate.dlb);
    trackerResultContainer.estimate.dlb=sqrt((preestimate.dlb*preestimate.dlb*curestimate.dlb*curestimate.dlb)/(preestimate.dlb*preestimate.dlb+curestimate.dlb*curestimate.dlb));
    trackerResultContainer.estimate.dlb=trackerResultContainer.estimate.dlb>MINSIGMA?trackerResultContainer.estimate.dlb:MINSIGMA;

    deviceBuildModel(trackerResultContainer.estimate,egomotion.density);
    trackerResultContainer.estimate.weight=0;
    trackerResultContainer.estimate.count=0;
    trackerResultContainer.edgepointnum[0]=0;
    deviceMeasureEdge(trackerResultContainer.estimate,0,&h_scan,1,&(trackerResultContainer.edgepointnum[0]),trackerResultContainer.edgepointid[0],1);
    trackerResultContainer.edgepointnum[1]=0;
    deviceMeasureEdge(trackerResultContainer.estimate,1,&h_scan,1,&(trackerResultContainer.edgepointnum[1]),trackerResultContainer.edgepointid[1],1);
}

extern "C" bool cuda_UpdateTracker(TrackerDataContainer & trackerDataContainer, TrackerResultContainer & trackerResultContainer)
{
    VehicleState preestimate=trackerResultContainer.estimate;
    VehicleState curestimate=preestimate;

    ObjectStateOffset objectstateoffset;
    objectstateoffset.thetaoff=objectstateoffset.thetaprec;
    if(preestimate.dwl<objectstateoffset.wlprec)
    {
        objectstateoffset.wloff=objectstateoffset.wlprec;
    }
    if(preestimate.dwr<objectstateoffset.wrprec)
    {
        objectstateoffset.wroff=objectstateoffset.wrprec;
    }
    if(preestimate.dlf<objectstateoffset.lfprec)
    {
        objectstateoffset.lfoff=objectstateoffset.lfprec;
    }
    if(preestimate.dlb<objectstateoffset.lbprec)
    {
        objectstateoffset.lboff=objectstateoffset.lbprec;
    }

    EgoMotion egomotion=h_egomotion;
    if(preestimate.dx<=2*UNCERTAINTHRESH&&preestimate.dy<=2*UNCERTAINTHRESH&&preestimate.dtheta<=UNCERTAINTHRESH_ANG&&preestimate.count>=UNCERTAINTHRESH_CNT)
    {
        objectstateoffset.aoff=DEG2RAD(30);
        objectstateoffset.voff=10;
        objectstateoffset.koff=0.5;
        objectstateoffset.omegaoff=DEG2RAD(60);

        egomotion.pfflag=0;
        trackerDataContainer.pnum=1;
        cudaMemcpy(trackerDataContainer.d_particle,&(preestimate),sizeof(VehicleState),cudaMemcpyHostToDevice);
    }
    else
    {
        objectstateoffset.aoff=DEG2RAD(5);
        objectstateoffset.voff=2;
        objectstateoffset.koff=0.05;
        objectstateoffset.omegaoff=DEG2RAD(3);

        egomotion.pfflag=1;
    }

    SSPF_MotionModel(d_scan,trackerDataContainer.pnum,trackerDataContainer.d_particle,trackerDataContainer.d_tmpparticle,trackerDataContainer.d_rng,curestimate,objectstateoffset,egomotion);

    if(curestimate.count>=10||curestimate.dx<=2*UNCERTAINTHRESH&&curestimate.dy<=2*UNCERTAINTHRESH&&curestimate.dtheta<=UNCERTAINTHRESH_ANG&&curestimate.count>=UNCERTAINTHRESH_CNT)
    {
        double dx=curestimate.dx;
        double dy=curestimate.dy;
        double dtheta=curestimate.dtheta;

        trackerDataContainer.pnum=1;
        cudaMemcpy(trackerDataContainer.d_particle,&(curestimate),sizeof(VehicleState),cudaMemcpyHostToDevice);

        SSPF_GeometryModel(d_scan,trackerDataContainer.pnum,trackerDataContainer.d_particle,trackerDataContainer.d_tmpparticle,trackerDataContainer.d_rng,curestimate,objectstateoffset,egomotion);

        trackerResultContainer.estimate=curestimate;

        curestimate.dwl=curestimate.dwl>MINSIGMA?curestimate.dwl:MINSIGMA;
        curestimate.dwr=curestimate.dwr>MINSIGMA?curestimate.dwr:MINSIGMA;
        curestimate.dlf=curestimate.dlf>MINSIGMA?curestimate.dlf:MINSIGMA;
        curestimate.dlb=curestimate.dlb>MINSIGMA?curestimate.dlb:MINSIGMA;

        curestimate.dwl=curestimate.dwl<UNCERTAINTHRESH?curestimate.dwl:MAXSIGMA;
        curestimate.dwr=curestimate.dwr<UNCERTAINTHRESH?curestimate.dwr:MAXSIGMA;
        curestimate.dlf=curestimate.dlf<UNCERTAINTHRESH?curestimate.dlf:MAXSIGMA;
        curestimate.dlb=curestimate.dlb<UNCERTAINTHRESH?curestimate.dlb:MAXSIGMA;

        trackerResultContainer.estimate.dx=dx;trackerResultContainer.estimate.dy=dy;trackerResultContainer.estimate.dtheta=dtheta;

        trackerResultContainer.estimate.wl=(preestimate.wl*curestimate.dwl*curestimate.dwl+curestimate.wl*preestimate.dwl*preestimate.dwl)/(preestimate.dwl*preestimate.dwl+curestimate.dwl*curestimate.dwl);
        trackerResultContainer.estimate.dwl=sqrt((preestimate.dwl*preestimate.dwl*curestimate.dwl*curestimate.dwl)/(preestimate.dwl*preestimate.dwl+curestimate.dwl*curestimate.dwl));
        trackerResultContainer.estimate.dwl=trackerResultContainer.estimate.dwl>MINSIGMA?trackerResultContainer.estimate.dwl:MINSIGMA;

        trackerResultContainer.estimate.wr=(preestimate.wr*curestimate.dwr*curestimate.dwr+curestimate.wr*preestimate.dwr*preestimate.dwr)/(preestimate.dwr*preestimate.dwr+curestimate.dwr*curestimate.dwr);
        trackerResultContainer.estimate.dwr=sqrt((preestimate.dwr*preestimate.dwr*curestimate.dwr*curestimate.dwr)/(preestimate.dwr*preestimate.dwr+curestimate.dwr*curestimate.dwr));
        trackerResultContainer.estimate.dwr=trackerResultContainer.estimate.dwr>MINSIGMA?trackerResultContainer.estimate.dwr:MINSIGMA;

        trackerResultContainer.estimate.lf=(preestimate.lf*curestimate.dlf*curestimate.dlf+curestimate.lf*preestimate.dlf*preestimate.dlf)/(preestimate.dlf*preestimate.dlf+curestimate.dlf*curestimate.dlf);
        trackerResultContainer.estimate.dlf=sqrt((preestimate.dlf*preestimate.dlf*curestimate.dlf*curestimate.dlf)/(preestimate.dlf*preestimate.dlf+curestimate.dlf*curestimate.dlf));
        trackerResultContainer.estimate.dlf=trackerResultContainer.estimate.dlf>MINSIGMA?trackerResultContainer.estimate.dlf:MINSIGMA;

        trackerResultContainer.estimate.lb=(preestimate.lb*curestimate.dlb*curestimate.dlb+curestimate.lb*preestimate.dlb*preestimate.dlb)/(preestimate.dlb*preestimate.dlb+curestimate.dlb*curestimate.dlb);
        trackerResultContainer.estimate.dlb=sqrt((preestimate.dlb*preestimate.dlb*curestimate.dlb*curestimate.dlb)/(preestimate.dlb*preestimate.dlb+curestimate.dlb*curestimate.dlb));
        trackerResultContainer.estimate.dlb=trackerResultContainer.estimate.dlb>MINSIGMA?trackerResultContainer.estimate.dlb:MINSIGMA;
    }
    else
    {
        trackerResultContainer.estimate=curestimate;

        trackerResultContainer.estimate.wl=preestimate.wl;trackerResultContainer.estimate.dwl=preestimate.dwl;
        trackerResultContainer.estimate.wr=preestimate.wr;trackerResultContainer.estimate.dwr=preestimate.dwr;
        trackerResultContainer.estimate.lf=preestimate.lf;trackerResultContainer.estimate.dlf=preestimate.dlf;
        trackerResultContainer.estimate.lb=preestimate.lb;trackerResultContainer.estimate.dlb=preestimate.dlb;
    }

    deviceBuildModel(trackerResultContainer.estimate,egomotion.density);
    trackerResultContainer.estimate.weight=0;
    trackerResultContainer.estimate.count=0;
    trackerResultContainer.edgepointnum[0]=0;
    deviceMeasureEdge(trackerResultContainer.estimate,0,&h_scan,1,&(trackerResultContainer.edgepointnum[0]),trackerResultContainer.edgepointid[0],1);
    trackerResultContainer.edgepointnum[1]=0;
    deviceMeasureEdge(trackerResultContainer.estimate,1,&h_scan,1,&(trackerResultContainer.edgepointnum[1]),trackerResultContainer.edgepointid[1],1);

    return egomotion.pfflag;
}

//==============================================================================
