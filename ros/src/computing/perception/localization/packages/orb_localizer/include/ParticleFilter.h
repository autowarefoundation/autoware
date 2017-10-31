/*
 * ParticleFilter.h
 *
 *  Created on: May 6, 2016
 *      Author: sujiwo
 */


/*
 * This header requires C++11 Support
 */

#ifndef _PARTICLEFILTER_H_
#define _PARTICLEFILTER_H_


#if __cplusplus < 201103L
#error "This header requires C++11"
#endif


#include <cstdlib>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
//#include <functional>



using namespace std;


namespace PF
{

inline double frandom()
{ return (double)rand() / (double)RAND_MAX; }


inline double nrand(double stdDev)
{
	return
		stdDev * sqrt(-2.0*log(
			frandom())) * cos(2.0*M_PI*frandom());
}


/*
 * Base Class for Particle Fusion
 * What you need is implement these virtual functions,
 * and add your own callback for incoming measurement
 */
template <
	class State, class Observation, class MotionCtrl
	>
class VehicleBase
{
public:

	virtual State initializeParticleState () const = 0;
	virtual State motionModel (const State &vstate, const MotionCtrl &ctrl) const = 0;
	virtual double measurementModel (const State &state, const vector<Observation> &observations) const = 0;

};


template<class State>
class Particle
{
//friend class ParticleFilter<S, class _obs, class _ctrl>;
public:
	inline Particle ():
		weight(0) {}

	inline void swapState ()
	{
		State tmp;
		tmp = previous;
		previous = current;
		current = previous;
	}

	State current;
	State previous;
	double weight;
};



template <
	class State, class Observation, class MotionCtrl
	>
class ParticleFilter
{
public:
	ParticleFilter (
		int numPart,
		VehicleBase<State, Observation, MotionCtrl> &vh
	) :
		vehicle (vh),
		numberOfParticle (numPart)
	{

		particleList.resize(numberOfParticle);
//		std::fill(particleList.begin(), particleList.end(), Particle<state>(stateInitializer));
		srand(time(0));
	}

	void initializeParticles ()
	{
		for (Particle<State> &particle: particleList) {
			particle.current = vehicle.initializeParticleState();
			particle.previous = vehicle.initializeParticleState();
		}
	}

	inline void update (const MotionCtrl &control, const vector<Observation> &observationList)
	{
		// Prediction
		for (Particle<State> &p: particleList) {
			p.current = vehicle.motionModel (p.previous, control);
			p.swapState();
		}

		if (observationList.size()==0)
			return;

		// Importance factor
		double w_all = 0;
		for (Particle<State> &p: particleList) {
			p.weight = vehicle.measurementModel (p.current, observationList);
			w_all += p.weight;
		}

		// Resampling
		double r = frandom() / (double)numberOfParticle;
		int i = 0;
		double c = particleList[0].weight / w_all;

		for (int p=0; p<numberOfParticle; p++) {
			double U = r + p/((double)numberOfParticle);
			while (U > c) {
				i += 1;
				c += particleList[i].weight / w_all;
			}
			particleList[p].current = particleList[i].previous;
		}

		for (Particle<State> &particle: particleList) {
			particle.swapState();
		}
	}


	inline vector<State> getStates ()
	{
		vector<State> states;
		states.reserve(numberOfParticle);

		for (Particle<State> &p: particleList) {
			states.push_back(p.current);
		}

		return states;
	}


	inline void getStates (vector<State*> &statePtrList)
	{
		for (int i=0; i<particleList.size(); i++) {
			Particle<State> &pt = particleList[i];
			statePtrList[i] = &(pt.current);
		}
	}


	int getNumberOfParticles () const { return numberOfParticle; }

	Particle<State> &getParticle(int num)
	{
		return particleList[num];
	}


	vector < Particle<State> > &getParticleList () const
	{ return particleList; }


//	virtual ~ParticleFilter();

protected:
	VehicleBase<State, Observation, MotionCtrl> &vehicle;
	int numberOfParticle;
	vector < Particle<State> > particleList;
};

#endif /* _PARTICLEFILTER_H_ */


}
