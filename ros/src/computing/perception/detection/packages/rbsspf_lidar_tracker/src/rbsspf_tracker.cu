#include"rbsspf_tracker.cuh"

using namespace std::chrono;

LaserScan h_scan;
double * d_scan;

EgoMotion h_egomotion;

int h_seed[RNGNUM];
thrust::minstd_rand * d_rng=NULL;

//==============================================================================

extern "C" void cudaOpenTracker()
{
    srand(time(NULL));
    cudaCloseTracker();
    //==============================
    //initialize rand seed
    int * d_seed;
    cudaMalloc(&d_seed,sizeof(int)*RNGNUM);
    thrust::generate(h_seed,h_seed+RNGNUM,rand);
    cudaMemcpy(d_seed,h_seed,sizeof(int)*RNGNUM,cudaMemcpyHostToDevice);
    cudaMalloc(&d_rng,sizeof(thrust::minstd_rand)*RNGNUM);
    GetKernelDim_1D(blocks,threads,RNGNUM);
    kernelSetupRandomSeed<<<blocks,threads>>>(d_seed,d_rng);
    CUDAFREE(d_seed);

    cudaMalloc(&d_scan,sizeof(double)*MAXBEAMNUM);
}

extern "C" void cudaCloseTracker()
{
    CUDAFREE(d_rng);
    CUDAFREE(d_scan);
}

//==============================================================================

extern "C" void cudaSetLaserScan(LaserScan & scan)
{
    h_scan=scan;
    cudaMemcpy(d_scan,h_scan.length,sizeof(double)*MAXBEAMNUM,cudaMemcpyHostToDevice);
    if(h_egomotion.validflag)
    {
        double tmpdx=h_egomotion.x-h_scan.x;
        double tmpdy=h_egomotion.y-h_scan.y;
        double c=cos(h_scan.theta);
        double s=sin(h_scan.theta);
        h_egomotion.dx=c*tmpdx+s*tmpdy;
        h_egomotion.dy=-s*tmpdx+c*tmpdy;
        h_egomotion.dtheta=h_egomotion.theta-h_scan.theta;
        h_egomotion.dt=h_scan.timestamp-h_egomotion.timestamp;
    }
    h_egomotion.x=h_scan.x;
    h_egomotion.y=h_scan.y;
    h_egomotion.theta=h_scan.theta;
    h_egomotion.timestamp=h_scan.timestamp;
    h_egomotion.validflag=1;
}

//==============================================================================

void SSPF_Motion(int & pnum, TrackerParticle * d_particles, int trackernum, std::vector<TrackerSampleControl> & h_controls, TrackerSampleControl * d_controls, TrackerParticle * d_tmpparticles, TrackerParticle * d_tmpparticles_forward, bool forwardflag)
{
    cudaMemcpy(d_controls,h_controls.data(),sizeof(TrackerSampleControl)*trackernum,cudaMemcpyHostToDevice);

    //Allocate memory
    int tmppnum=pnum*SPN;
    MALLOCARRAY(h_beamcount,d_beamcount,int,tmppnum);
    MALLOCARRAY(h_weights,d_weights,double,tmppnum);
    MALLOCARRAY(h_controlids,d_controlids,int,tmppnum);
    MALLOCARRAY(h_sampleids,d_sampleids,int,MRQPN*trackernum);
    MALLOCARRAY(h_wcount,d_wcount,int,MRQPN*trackernum);

    //2: upsample
    {
        GetKernelDim_1D(blocks,threads,tmppnum);
        kernelMotionUpSample<<<blocks,threads>>>(d_particles,d_controls,d_tmpparticles,d_tmpparticles_forward,tmppnum,d_rng,h_egomotion,h_scan.beamnum,d_beamcount);
    }
    cudaDeviceSynchronize();

    //3: collect beamcount and generate beamweight buffer
    int beamcount=hostCollectBeamCount(d_beamcount,h_beamcount.data(),tmppnum);
    TrackerBeamEvaluator * d_beamevaluators;
    cudaMalloc(&d_beamevaluators,sizeof(TrackerBeamEvaluator)*beamcount);

    //4: setup beam array
    {
        GetKernelDim_1D(blocks,threads,tmppnum);
        kernelSetupBeamArray<<<blocks,threads>>>(d_beamcount,tmppnum,d_beamevaluators);
    }

    //5: measure scan
    {
        GetKernelDim_1D(blocks,threads,beamcount);
        kernelMeasureScan<<<blocks,threads>>>(d_beamevaluators,beamcount,d_tmpparticles_forward,d_controls,d_scan,h_scan.beamnum,1);
    }

    //6: accumulate beam weight
    {
        GetKernelDim_1D(blocks,threads,tmppnum);
        kernelAccumulateWeight<<<blocks,threads>>>(d_weights,d_controlids,d_tmpparticles,d_beamcount,tmppnum,d_beamevaluators,d_tmpparticles_forward);
    }
    cudaDeviceSynchronize();

    //7: get down sample ids
    cudaMemcpy(h_weights.data(),d_weights,sizeof(double)*tmppnum,cudaMemcpyDeviceToHost);
    cudaMemcpy(h_controlids.data(),d_controlids,sizeof(int)*tmppnum,cudaMemcpyDeviceToHost);
    pnum=0;
    int startid=0;
    while(startid<tmppnum)
    {
        hostDownSampleIDs(startid,h_controlids,h_weights,tmppnum,h_controls,pnum,h_sampleids,h_wcount,1);
    }

    //8: down sample particles
    cudaMemcpy(d_sampleids,h_sampleids.data(),sizeof(int)*pnum,cudaMemcpyHostToDevice);
    cudaMemcpy(d_wcount,h_wcount.data(),sizeof(int)*pnum,cudaMemcpyHostToDevice);
    {
        GetKernelDim_1D(blocks,threads,pnum);
        if(forwardflag)
        {
            kernelDownSample<<<blocks,threads>>>(d_particles,d_sampleids,d_wcount,pnum,d_tmpparticles_forward);
        }
        else
        {
            kernelDownSample<<<blocks,threads>>>(d_particles,d_sampleids,d_wcount,pnum,d_tmpparticles);
        }
    }
    cudaDeviceSynchronize();

    //Recycle memory
    CUDAFREE(d_sampleids);
    CUDAFREE(d_wcount);
    CUDAFREE(d_weights);
    CUDAFREE(d_beamcount);
    CUDAFREE(d_controlids);
    CUDAFREE(d_beamevaluators);
}

void SSPF_Geometry(int & pnum, TrackerParticle * d_particles, int trackernum, std::vector<TrackerSampleControl> & h_controls, TrackerSampleControl * d_controls, TrackerParticle * d_tmpparticles)
{
    cudaMemcpy(d_controls,h_controls.data(),sizeof(TrackerSampleControl)*trackernum,cudaMemcpyHostToDevice);

    //Allocate memory
    int tmppnum=pnum*SPN;
    MALLOCARRAY(h_beamcount,d_beamcount,int,tmppnum);
    MALLOCARRAY(h_weights,d_weights,double,tmppnum);
    MALLOCARRAY(h_controlids,d_controlids,int,tmppnum);
    MALLOCARRAY(h_sampleids,d_sampleids,int,GRQPN*trackernum);
    MALLOCARRAY(h_wcount,d_wcount,int,GRQPN*trackernum);

    //2: upsample
    {
        GetKernelDim_1D(blocks,threads,tmppnum);
        kernelGeometryUpSample<<<blocks,threads>>>(d_particles,d_controls,d_tmpparticles,tmppnum,d_rng,h_scan.beamnum,d_beamcount);
    }
    cudaDeviceSynchronize();

    //3: collect beamcount and generate beamweight buffer
    int beamcount=hostCollectBeamCount(d_beamcount,h_beamcount.data(),tmppnum);
    TrackerBeamEvaluator * d_beamevaluators;
    cudaMalloc(&d_beamevaluators,sizeof(TrackerBeamEvaluator)*beamcount);

    //4: setup beam array
    {
        GetKernelDim_1D(blocks,threads,tmppnum);
        kernelSetupBeamArray<<<blocks,threads>>>(d_beamcount,tmppnum,d_beamevaluators);
    }

    //5: measure scan
    {
        GetKernelDim_1D(blocks,threads,beamcount);
        kernelMeasureScan<<<blocks,threads>>>(d_beamevaluators,beamcount,d_tmpparticles,d_controls,d_scan,h_scan.beamnum,0);
    }

    //6: accumulate beam weight
    {
        GetKernelDim_1D(blocks,threads,tmppnum);
        kernelAccumulateWeight<<<blocks,threads>>>(d_weights,d_controlids,d_tmpparticles,d_beamcount,tmppnum,d_beamevaluators,NULL);
    }
    cudaDeviceSynchronize();

    //7: get down sample ids
    cudaMemcpy(h_weights.data(),d_weights,sizeof(double)*tmppnum,cudaMemcpyDeviceToHost);
    cudaMemcpy(h_controlids.data(),d_controlids,sizeof(int)*tmppnum,cudaMemcpyDeviceToHost);
    pnum=0;
    int startid=0;
    while(startid<tmppnum)
    {
        hostDownSampleIDs(startid,h_controlids,h_weights,tmppnum,h_controls,pnum,h_sampleids,h_wcount,0);
    }

    //8: down sample particles
    cudaMemcpy(d_sampleids,h_sampleids.data(),sizeof(int)*pnum,cudaMemcpyHostToDevice);
    cudaMemcpy(d_wcount,h_wcount.data(),sizeof(int)*pnum,cudaMemcpyHostToDevice);
    {
        GetKernelDim_1D(blocks,threads,pnum);
        kernelDownSample<<<blocks,threads>>>(d_particles,d_sampleids,d_wcount,pnum,d_tmpparticles);
    }
    cudaDeviceSynchronize();

    //Recycle memory
    CUDAFREE(d_sampleids);
    CUDAFREE(d_wcount);
    CUDAFREE(d_weights);
    CUDAFREE(d_beamcount);
    CUDAFREE(d_controlids);
    CUDAFREE(d_beamevaluators);
}

extern "C" void cudaUpdateTracker(std::vector<Tracker> & trackers)
{
    int trackernum=trackers.size();
    if(trackernum<=0) return;
    //==============================
    //Allocate memory
    MALLOCARRAY(h_controls,d_controls,TrackerSampleControl,trackernum);
    MALLOCARRAY(h_particles,d_particles,TrackerParticle,RQPN*trackernum);

    TrackerParticle * d_tmpparticles;
    cudaMalloc(&d_tmpparticles,sizeof(TrackerParticle)*MAXPN*trackernum);
    TrackerParticle * d_tmpparticles_forward;
    cudaMalloc(&d_tmpparticles_forward,sizeof(TrackerParticle)*MAXPN*trackernum);

    int pnum;

    //==============================
    //Motion estimate

    //1: init control and particles
    double maxmotioniteration=hostInitMotionEstimation(trackernum,trackers,h_controls,pnum,h_particles);

    if(maxmotioniteration>=0)
    {
        cudaMemcpy(d_particles,h_particles.data(),sizeof(TrackerParticle)*pnum,cudaMemcpyHostToDevice);

        //SSPF-loop
        for(int i=1;i<=maxmotioniteration;i++)
        {
            //SSPF_Motion
            SSPF_Motion(pnum,d_particles,trackernum,h_controls,d_controls,d_tmpparticles,d_tmpparticles_forward,0);

            //update controls
            for(int j=0;j<trackernum;j++)
            {
                if(h_controls[j].motioniteration>=1)
                {
                    h_controls[j].motioniteration--;
                    h_controls[j].motionoffset.a*=h_controls[j].motionzoom.a;
                    h_controls[j].motionoffset.v*=h_controls[j].motionzoom.v;
                    h_controls[j].motionoffset.omega*=h_controls[j].motionzoom.omega;
                    h_controls[j].motionanneal*=h_controls[j].motionannealratio;
                }
            }
        }

        //Final estimate
        //setup controls
        for(int j=0;j<trackernum;j++)
        {
            if(h_controls[j].motioniteration>=0)
            {
                h_controls[j].motioniteration=1;
                h_controls[j].motionoffset=MOTIONPREC;
                h_controls[j].motionanneal=1;
            }
        }

        //SSPF_Motion
        SSPF_Motion(pnum,d_particles,trackernum,h_controls,d_controls,d_tmpparticles,d_tmpparticles_forward,1);

        //Motion results
        cudaMemcpy(h_particles.data(),d_particles,sizeof(TrackerParticle)*pnum,cudaMemcpyDeviceToHost);
        hostEstimateMotionTracker(pnum,h_particles,trackers,h_scan.beamnum);
    }

    //==============================
    //Geometry estimate

    //1: init control and particles
    double maxgeometryiteration=hostInitGeometryEstimation(trackernum,trackers,h_controls,pnum,h_particles);

    if(maxgeometryiteration>=0)
    {
        cudaMemcpy(d_particles,h_particles.data(),sizeof(TrackerParticle)*pnum,cudaMemcpyHostToDevice);

        //SSPF-loop
        for(int i=1;i<=maxgeometryiteration;i++)
        {
            //SSPF_Geometry
            SSPF_Geometry(pnum,d_particles,trackernum,h_controls,d_controls,d_tmpparticles);

            //update controls
            for(int j=0;j<trackernum;j++)
            {
                if(h_controls[j].geometryiteration>=1)
                {
                    h_controls[j].geometryiteration--;
                    h_controls[j].geometryoffset.theta*=h_controls[j].geometryzoom.theta;
                    h_controls[j].geometryoffset.wl*=h_controls[j].geometryzoom.wl;
                    h_controls[j].geometryoffset.wr*=h_controls[j].geometryzoom.wr;
                    h_controls[j].geometryoffset.lf*=h_controls[j].geometryzoom.lf;
                    h_controls[j].geometryoffset.lb*=h_controls[j].geometryzoom.lb;
                    h_controls[j].geometryanneal*=h_controls[j].geometryannealratio;
                }
            }
        }

        //Final estimate
        //setup controls
        for(int j=0;j<trackernum;j++)
        {
            if(h_controls[j].geometryiteration>=0)
            {
                h_controls[j].geometryiteration=1;
                h_controls[j].geometryoffset=GEOMETRYPREC;
                h_controls[j].geometryanneal=1;
            }
        }

        //SSPF_Geometry
        SSPF_Geometry(pnum,d_particles,trackernum,h_controls,d_controls,d_tmpparticles);

        //Geometry results
        cudaMemcpy(h_particles.data(),d_particles,sizeof(TrackerParticle)*pnum,cudaMemcpyDeviceToHost);
        hostEstimateGeometryTracker(pnum,h_particles,trackers,h_controls,h_scan.beamnum);
    }

    //==============================
    //Recycle memory
    CUDAFREE(d_controls);
    CUDAFREE(d_particles);
    CUDAFREE(d_tmpparticles);
    CUDAFREE(d_tmpparticles_forward);
}
