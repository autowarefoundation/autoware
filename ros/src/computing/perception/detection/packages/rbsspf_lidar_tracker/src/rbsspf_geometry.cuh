#ifndef RBSSPF_GEOMETRY_CUH
#define RBSSPF_GEOMETRY_CUH

#include"rbsspf_share.cuh"

//====================================================

//1: init control and particles
__host__
double hostInitGeometryEstimation(int trackernum, std::vector<Tracker> & trackers, std::vector<TrackerSampleControl> & controls, int & pnum, std::vector<TrackerParticle> & particles);

//2: upsample
__global__
void kernelGeometryUpSample(TrackerParticle * particles, TrackerSampleControl * controls, TrackerParticle * tmpparticles, int tmppnum, thrust::minstd_rand * rng, int beamnum, int * beamcount);

//8. estimate tracker
__host__
void hostEstimateGeometryTracker(int pnum, std::vector<TrackerParticle> & particles, std::vector<Tracker> & trackers, std::vector<TrackerSampleControl> & controls, int beamnum);

//====================================================

#endif // RBSSPF_GEOMETRY_CUH
