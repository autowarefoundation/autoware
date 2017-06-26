#ifndef TRACKER_CUH
#define TRACKER_CUH

#include"rbsspf_motion.cuh"
#include"rbsspf_geometry.cuh"

#include<chrono>

extern "C" void cudaOpenTracker();
extern "C" void cudaCloseTracker();
extern "C" void cudaSetLaserScan(LaserScan & scan);
extern "C" void cudaUpdateTracker(std::vector<Tracker> & trackers);

#endif // TRACKER_CUH

