/*
 *  Copyright (c) 2017, TierIV Inc.
 *  All rights reserved.
 *
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
*/

// ROS Includes
#include <ros/ros.h>
#include <signal.h>
#include "gmsl_interface.hpp"

gmsl_camera::GMSLCameraNode *gmsl_ptr;

void sig_int_handler(int sig)
{
	(void)sig;
	gmsl_ptr->g_runSetter(false);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gmsl_camera_interface");
	gmsl_camera::GMSLCameraNode gmsl;
	gmsl_ptr = &gmsl;	
	
	gmsl.argc_ = argc;
	gmsl.argv_ = argv;

	struct sigaction action;
	memset(&action, 0, sizeof(action));
	action.sa_handler = sig_int_handler;

	sigaction(SIGHUP, &action, NULL);  // controlling terminal closed, Ctrl-D
	sigaction(SIGINT, &action, NULL);  // Ctrl-C
	sigaction(SIGQUIT, &action, NULL); // Ctrl-\, clean quit with core dump
	sigaction(SIGABRT, &action, NULL); // abort() called.

	gmsl.run();
	return 0;
}
