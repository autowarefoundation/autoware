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

#ifndef COMMON__MOTION_COMMON__SCRIPTS__AUTOGENERATION_CODE_SNIPPETS__KINEMATIC_BICYCLE_SNIPPET_HPP_  // NOLINT
#define COMMON__MOTION_COMMON__SCRIPTS__AUTOGENERATION_CODE_SNIPPETS__KINEMATIC_BICYCLE_SNIPPET_HPP_  // NOLINT

// For help on getting random weird models to work, see:
// https://sourceforge.net/p/acado/discussion/general
// and
// https://github.com/acado/acado/issues

// Note: Order of variable declaration specifies order in C-arrays
//
// Variables
//

DifferentialState x, y, yaw;  // pose
DifferentialState u;
Control ax;  // acceleration
Control delta;  // wheel angle

// Vehicle parameters
OnlineData L_f, L_r;  // front, rear wheelbase length
//
// Differential algebraic equation
//

// Kinematic bicycle model:
// https://borrelli.me.berkeley.edu/pdfpub/IV_KinematicMPC_jason.pdf
// Intermediate variables
IntermediateState beta = atan((L_r * tan(delta)) / (L_f + L_r));

DifferentialEquation f;

f << dot(x) == u * cos(yaw + beta);
f << dot(y) == u * sin(yaw + beta);
f << dot(yaw) == (u * sin(beta)) / L_r;
f << dot(u) == ax;
#endif  // COMMON__MOTION_COMMON__SCRIPTS__AUTOGENERATION_CODE_SNIPPETS__KINEMATIC_BICYCLE_SNIPPET_HPP_  // NOLINT
