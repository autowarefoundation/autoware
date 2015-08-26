/*
 *  libtraj_gen.h
 *  Trajectory Generator Library
 *
 *  Created by Matthew O'Kelly on 7/17/15.
 *  Copyright (c) 2015 Matthew O'Kelly. All rights reserved.
 *  mokelly@seas.upenn.edu
 *  
*/

/*
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#ifndef TRAJECTORYGENERATOR_H
#define TRAJECTORYGENERATOR_H

// ---------DEFINE MODE---------//
//#define GEN_PLOT_FILES
//#define DEBUG_OUTPUT
//#define DEBUG
//#define ONE_ITER
//#define STANDALONE
#define USE_HEURISTIC_INIT
#define CUBIC_STABLE

// --------UNSTABLE MODES-------//
//#define QUINTIC_STABLE
//#define FIRST ORDER


// ------------BOOLEAN----------//
#define TRUE 1
#define FALSE 0

// ------------CONSTANTS----------//
// Constants for forward simulation of ego vehicle
// Maximum curvature (radians)
#define kmax (0.1900)
// Minimum curvature (radians)
#define kmin (-0.1900)
// Maximum rate of curvature (radians/second)
#define dkmax (0.1021)
// Minimum rate of curvature (radians/second)
#define dkmin (-0.1021)
// Maximum acceleration (meters/second^2)
#define dvmax (2.000)
// Maximum deceleration (meters/second^2)
#define dvmin (-6.000)
// Control latency (seconds)
#define tdelay (0.0800)
 //#define tdelay (0.03)
// Speed control logic a coefficient
#define ascl (0.1681)
// Speed control logic b coefficient
#define bscl (-0.0049)
// Speed control logic threshold (meters/second)
#define vscl (4.000)
// Max curvature for speed (radians)
#define kvmax (0.1485)
// Speed control logic safety factor
#define sf (1.000)

// ------------TERMINATION CRITERIA----------//
// User defined allowable errors for goal state approximation
// Allowable crosstrack error (meters)
#define crosstrack_e (0.001)
// Allowable inline error (meters)
#define inline_e (0.001)
// Allowable heading error (radians)
#define heading_e (0.1)
// Allowable curvature error (meters^-1)
#define curvature_e (0.005)
// General error, ill-defined heuristic (unitless)
#define general_e (0.05)

// ------------PARAMETER PERTURBATION----------//
// User defined perturbations for estimation of partial derivatives
// Perturbation for a
#define h_sx (0.001)
// Perturbation for b
#define h_sy (0.001)
// Perturbation for d
#define h_theta (0.001)
// Perturbation for s
#define h_k (0.001)
// If all parameters are perturbed equally, heuristic (unitless)
#define h_global (0.001)

// --------INTEGRATION STEP SIZE CONTROL------//
// Set time step
#define step_size (0.0001)
// Set lightweight timestep for plotting, used in genLineStrip
#define plot_step_size (0.1)

//#define step_size (0.05)

// ------------LOG FILES----------//
// Open files for data logging, define globally so all functions may access:
using namespace std;
ofstream fmm_sx;
ofstream fmm_sy;
ofstream fmm_v;
ofstream fmm_theta;
ofstream fmm_kappa;

// --------DATA STRUCTURES-------//
union State
{
    struct 
    {
        double sx;
        double sy;
        double theta; 
        double kappa;
        double v;
        double vdes;
        double timestamp;
    };

    double state_value[7];
};

union Parameters 
{
    struct 
    {
        double a;
        double b;
        double c;
        double d;
        double s;
    };

    double param_value[5];


};

union Spline
{
    struct
    {
        //double kappa_0;
        double s;
        double kappa_1;
        double kappa_2;
        //double kappa_2;
        //double kappa_3;
        
        double kappa_0;
        double kappa_3;
        bool success;
        
    };

    double spline_value[6];
};

union Command
{
    struct
    {
        double kappa;
        double v;
    };
    double cmd_index[2];
};


// ------------FUNCTION DECLARATIONS----------//

// initParams is used to generate the initial guess for the trajectory
union Spline initParams(union State veh, union State goal);

// speedControlLogic prevents the vehicle from exceeding dynamic limits
union State speedControlLogic(union State veh_next);

// responseToControlInputs computes the vehicles next state consider control delay
union State responseToControlInputs(union State veh, union State veh_next, double dt);

// getCurvatureCommand computes curvature based on the selection of trajectory parameters
double getCurvatureCommand(union Spline curvature, double dt, double v, double t);

// getVelocityCommand computes the next velocity command, very naieve right now.
double getVelocityCommand(double v_goal, double v);

// motionModel computes the vehicles next state, it calls speedControlLogic, responseToControlInputs, getCurvatureCommand and
// and getVelocityCommand
union State motionModel(union State veh, union State goal, union Spline curvature, double dt, double horizon, int flag);

// checkConvergence determines if the current final state is close enough to the goal state
bool checkConvergence(union State veh_next, union State goal);

// pDerivEstimate computes one column of the Jacobian
union State pDerivEstimate(union State veh, union State veh_next, union State goal, union Spline curvature, int p_id, double h, double dt, double horizon, int stateIndex);

// generateCorrection inverts the Jacobian and updates the spline parameters
union Spline generateCorrection(union State veh, union State veh_next, union State goal, union Spline curvature, double dt, double horizon);

// nextState is used by the robot to compute commands once an adequate set of parameters has been found
union State nextState(union State veh, union Spline curvature, double vdes, double dt, double elapsedTime);

// trajectoryGenerator is like a "main function" used to iterate through a series of goal states
union Spline trajectoryGenerator(double sx, double sy, double theta, double v, double kappa);

// plotTraj is used by rViz to compute points for line strip, it is a lighter weight version of nextState
union State genLineStrip(union State veh, union Spline curvature, double vdes, double t);


#endif // TRAJECTORYGENERATOR_H

