#ifndef RBSSPF_MOTION_CUH
#define RBSSPF_MOTION_CUH

#include"rbsspf_share.cuh"

//====================================================

//1: init control and particles
__host__
double hostInitMotionEstimation(int trackernum, std::vector<Tracker> &trackers, std::vector<TrackerSampleControl> &controls, int & pnum, std::vector<TrackerParticle> &particles);

//2: upsample
__global__
void kernelMotionUpSample(TrackerParticle * particles, TrackerSampleControl * controls, TrackerParticle * tmpparticles, TrackerParticle * tmpparticles_forward, int tmppnum, thrust::minstd_rand * rng, EgoMotion egomotion, int beamnum, int * beamcount);

//8: estimate tracker
__host__
void hostEstimateMotionTracker(int pnum, std::vector<TrackerParticle> & particles, std::vector<Tracker> & trackers, int beamnum);

//====================================================

#endif // RBSSPF_MOTION_CUH
