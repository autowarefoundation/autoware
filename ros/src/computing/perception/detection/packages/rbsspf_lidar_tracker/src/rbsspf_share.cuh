#ifndef RBSSPF_CUH
#define RBSSPF_CUH

#include<cuda.h>
#include<cuda_runtime.h>

#include<thrust/random/linear_congruential_engine.h>
#include<thrust/random/uniform_real_distribution.h>
#include<thrust/random/normal_distribution.h>
#include<thrust/generate.h>

#include<random>
#include<time.h>

//==========================

#define PI 3.14159265359
#define DEG2RAD(ang) (ang*PI/180)

//==========================

#define RNGNUM 1024

#define SPN 4 //split particle number for upsample

#define MRQPN 256 //motion
#define MAXMPN (SPN*MRQPN)

#define GRQPN 1024 //geometry
#define MAXGPN (SPN*GRQPN)

#define RQPN (MRQPN>GRQPN?MRQPN:GRQPN)
#define MAXPN (MAXMPN>MAXGPN?MAXMPN:MAXGPN)

#define MAXBEAMNUM 2048

#define CUDAFREE(pointer) if(pointer!=NULL){cudaFree(pointer);pointer=NULL;}

#define THREAD_1D 1024
#define THREAD_2D 16
#define GetKernelDim_1D(numBlocks, threadsPerBlock, dim) int numBlocks=(dim+THREAD_1D-1)/THREAD_1D; int threadsPerBlock=THREAD_1D;
#define GetKernelDim_2D(numBlocks, threadsPerBlock, xdim, ydim) dim3 numBlocks(int((xdim+THREAD_2D-1)/THREAD_2D), int((ydim+THREAD_2D-1)/THREAD_2D)); dim3 threadsPerBlock(THREAD_2D, THREAD_2D);
#define GetThreadID_1D(id) int id=blockDim.x*blockIdx.x+threadIdx.x;
#define GetThreadID_2D(xid,yid) int xid=blockDim.x*blockIdx.x+threadIdx.x;int yid=blockDim.y*blockIdx.y+threadIdx.y;

#define DEBUGARRAY(src,dst,type,size) std::vector<type> dst(size); cudaMemcpy(dst.data(),src,sizeof(type)*size,cudaMemcpyDeviceToHost);
#define MALLOCARRAY(h_array,d_array,type,size) std::vector<type> h_array(size); type * d_array; cudaMalloc(&d_array,sizeof(type)*size);

//==========================

#define MINSIGMA 1e-2
#define UNCERTAINTHRESH 0.4
#define MAXSIGMA 1e6

#define NEARESTRING 3.35
#define MINBEAM 2
#define MAXBEAM 100

//==========================

#define SIGMA 0.01

//dmin->A
#define COST0 1
#define WEIGHT0 -2 //weight=-2*cost^2
//A->B
#define COST1 2
#define WEIGHT1 -8
//B->D
#define COST2 0
#define WEIGHT2 0
//D->dmax
#define COST3 1.6
#define WEIGHT3 -5.12

//C->A
#define MARGIN0 0.2
//C->B
#define MARGIN1 0.1
//C->D
#define MARGIN2 0.1

//==========================

#define CALRATIO(ratio, vratio, maxratio, maxrange, minrange) \
    ratio=maxrange/minrange; vratio*=ratio; maxratio=ratio>maxratio?ratio:maxratio;
#define CALZOOM(zoom, maxrange, minrange, N) \
    zoom=log(maxrange/minrange)/log(2)/N;zoom=1/pow(2,zoom);

//==========================
#define MOTIONMIN {DEG2RAD(-60),-10,-0.5,DEG2RAD(-90)} //a,v,k,omega
#define MOTIONMAX {DEG2RAD(60),30,0.5,DEG2RAD(90)}
#define MOTIONPREC {DEG2RAD(1),1,0.001,DEG2RAD(1)}
#define INITMOTIONOFFSET {DEG2RAD(60),20,0.5,DEG2RAD(90)}
#define UPDATEMOTIONOFFSET_SSPF {DEG2RAD(30),15,0.3,DEG2RAD(60)}
#define UPDATEMOTIONOFFSET_PF {DEG2RAD(10),5,0.1,DEG2RAD(20)}

#define GEOMETRYMIN {DEG2RAD(-30),0,0,0,0}
#define GEOMETRYMAX {DEG2RAD(30),3,3,5,5}
#define GEOMETRYPREC {DEG2RAD(1),0.1,0.1,0.1,0.1}
#define INITGEOMETRYOFFSET {DEG2RAD(30),1.5,1.5,2.5,2.5}
#define UPDATEGEOMETRYOFFSET {DEG2RAD(1),1.5,1.5,2.5,2.5}

//==========================

#define SSPF_SIGMA_X 0.5
#define SSPF_SIGMA_Y 0.5
#define SSPF_SIGMA_THETA DEG2RAD(10)
#define SSPF_BEAMCOUNT 3

//==========================

struct GeometrySampleParam
{
    double theta;
    double wl,wr,lf,lb;
};

struct MotionSampleParam
{
    double a,v,k,omega;
};

struct TrackerSampleControl
{
    int id;
    bool pfflag;

    MotionSampleParam motionmin;
    MotionSampleParam motionmax;
    MotionSampleParam motionprec;
    MotionSampleParam motionoffset;
    MotionSampleParam motionzoom;

    double motioniteration;
    double motionanneal;
    double motionannealratio;

    GeometrySampleParam geometrymin;
    GeometrySampleParam geometrymax;
    GeometrySampleParam geometryprec;
    GeometrySampleParam geometryoffset;
    GeometrySampleParam geometryzoom;

    double geometryiteration;
    double geometryanneal;
    double geometryannealratio;

    int pnum;
};

//====================================================

enum TrackerStatus
{
    StatusInitGeometry,
    StatusInitMotion,
    StatusUpdateTracker_SSPF,
    StatusUpdateTracker_PF
};

struct TrackerState
{
    double x,y,theta;
    double wl,wr,lf,lb;
    double a,v,k,omega;
};

struct TrackerGeometry
{
    double cx[4],cy[4];//corners
    double dx[4],dy[4];//unit directions
    double cn[4],sa[4];//distance to origin and sin(alpha)
    int startid,startbeamid;
    int midid,midbeamid;
    int endid,endbeamid;
    int beamcount;
    bool validflag;
};

struct Tracker
{
    int id;
    TrackerStatus status;
    TrackerState mean;
    TrackerState sigma;
    double cx[4],cy[4];
    int startbeamid,midbeamid,endbeamid;
    int pfcount;
    double beamcount;
};

struct TrackerParticle
{
    double weight;
    int beamcount;
    TrackerState state;
    TrackerGeometry geometry;
    int controlid;
};

struct TrackerBeamEvaluator
{
    int tmppid;
    int beamdelta;
    double weight;
    bool validflag;
};

//====================================================

struct LaserScan
{
    double timestamp;
    double x,y,theta;
    int beamnum;
    double length[MAXBEAMNUM];
};

struct EgoMotion
{
    bool validflag=0;
    double x,y,theta;
    double timestamp;
    double dx=0,dy=0,dtheta=0;
    double dt=0;
};

//====================================================
//0 -> Start Track Update ->
//1 -> Start Motion Track Iteration -> (2) -> 3 -> (4) -> (5) -> (6) -> 7 -> (8) -> (2) -> End Motion Track Iteration -> 9 ->
//1 -> Start Geometry Track Iteration -> (2) -> 3 -> (4) -> (5) -> (6) -> 7 -> (8) -> (2) -> End Motion Track Iteration -> 9 ->
//End Track Update
//# executes on the CPU-side, and (#) executes on the GPU-side.

//0: init rng
__global__
void kernelSetupRandomSeed(int * seed, thrust::minstd_rand * rng); //0. RNGNUM

//1: init control and particles (motion & geometry)
//double hostInitMotionEstimation(int trackernum, std::vector<Tracker> &trackers, std::vector<TrackerSampleControl> &controls, int & pnum, std::vector<TrackerParticle> &particles);
//double hostInitGeometryEstimation(int trackernum, std::vector<Tracker> & trackers, std::vector<TrackerSampleControl> & controls, int & pnum, std::vector<TrackerParticle> & particles);

//2: upsample (motion & geometry)
//void kernelMotionUpSample(TrackerParticle * particles, TrackerSampleControl * controls, TrackerParticle * tmpparticles, TrackerParticle * tmpparticles_forward, int tmppnum, thrust::minstd_rand * rng, EgoMotion egomotion, int beamnum, int * beamcount);
//void kernelGeometryUpSample(TrackerParticle * particles, TrackerSampleControl * controls, TrackerParticle * tmpparticles, int tmppnum, thrust::minstd_rand * rng, int beamnum, int * beamcount);

//3: collect beamcount and generate beamweight buffer
__host__
int hostCollectBeamCount(int * d_beamcount, int * h_beamcount, int tmppnum);

//4: setup beam array
__global__
void kernelSetupBeamArray(int * beamcount, int tmppnum, TrackerBeamEvaluator * beamevaluators); //3. tmppnum

//5: measure scan
__global__
void kernelMeasureScan(TrackerBeamEvaluator * beamevaluators, int beamcount, TrackerParticle * tmpparticles, TrackerSampleControl * controls, double * scan, int beamnum, bool motionflag); //4. beamcount

//6: accumulate beam weight
__global__
void kernelAccumulateWeight(double * weights, int * controlids, TrackerParticle * tmpparticles, int *beamcount, int tmppnum, TrackerBeamEvaluator * beamevaluators, TrackerParticle *tmpparticles_forward); //5. tmppnum

//7: get down sample ids
__host__
void hostDownSampleIDs(int & startid, std::vector<int> & controlids, std::vector<double> & weights, int tmppnum, std::vector<TrackerSampleControl> & controls, int & pnum, std::vector<int> & sampleids, std::vector<int> & wcount, bool motionflag); //6.

//8: down sample particles
__global__
void kernelDownSample(TrackerParticle * particles, int * sampleids, int * wcount, int pnum, TrackerParticle * tmpparticles); //7. pnum

//9: estimate tracker
//void hostEstimateMotionTracker(int pnum, std::vector<TrackerParticle> & particles, std::vector<Tracker> & trackers, int beamnum);
//void hostEstimateGeometryTracker(int pnum, std::vector<TrackerParticle> & particles, std::vector<Tracker> & trackers, std::vector<TrackerSampleControl> & controls, int beamnum);

//====================================================

__host__ __device__
void deviceBuildModel(TrackerParticle & particle, int beamnum);

__host__
void hostBuildModel(Tracker & tracker, int beamnum);

//====================================================




#endif // RBSSPF_CUH
