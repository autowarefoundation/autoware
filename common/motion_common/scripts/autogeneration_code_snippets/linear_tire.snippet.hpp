// Copyright 2019 Christopher Ho
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef COMMON__MOTION_COMMON__SCRIPTS__AUTOGENERATION_CODE_SNIPPETS__LINEAR_TIRE_SNIPPET_HPP_  // NOLINT
#define COMMON__MOTION_COMMON__SCRIPTS__AUTOGENERATION_CODE_SNIPPETS__LINEAR_TIRE_SNIPPET_HPP_  // NOLINT
// For help on getting random weird models to work, see:
// https://sourceforge.net/p/acado/discussion/general
// and
// https://github.com/acado/acado/issues

// Note: Order of variable declaration specifies order in C-arrays
//
// Variables
//

DifferentialState x, y, yaw;  // pose
DifferentialState u, v, omega;  // velocities
DifferentialState delta;  // wheel angle
DifferentialState ax;  // acceleration
Control jx, delta_dot;

// Vehicle parameters
OnlineData L_f, L_r;  // front, rear wheelbase length
OnlineData C_f, C_r;  // front, rear cornering stiffness
OnlineData m, I;  // mass, moment of inertia

//
// Differential algebraic equation
//

// Using the linear model due to LaValle:
// http://planning.cs.uiuc.edu/node695.html
// Intermediate variables
IntermediateState F_f = C_f * (((v + (L_f * omega)) / u) + delta);
IntermediateState F_r = C_r * ((v - (L_r * omega)) / u);

DifferentialEquation f;

// Easy stuff
f << dot(x) == ((u * cos(yaw)) - (v * sin(yaw)));
f << dot(y) == ((u * sin(yaw)) + (v * cos(yaw)));
f << dot(yaw) == omega;
f << dot(u) == ax;
f << dot(v) == ((-u * omega) + ((F_f + F_r) / m));
f << dot(omega) == (((L_f * F_f) - (L_r * F_r)) / I);
f << dot(delta) == delta_dot;
f << dot(ax) == jx;
#endif  // COMMON__MOTION_COMMON__SCRIPTS__AUTOGENERATION_CODE_SNIPPETS__LINEAR_TIRE_SNIPPET_HPP_  // NOLINT
