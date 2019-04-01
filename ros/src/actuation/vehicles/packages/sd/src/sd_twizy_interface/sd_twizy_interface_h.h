/*
 * Copyright (C) 2019 StreetDrone Limited - All rights reserved
 * Author(s): Efimia Panagiotaki, Chris Whimpenny
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <iostream>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>
#include <can_msgs/Frame.h>
#include <ros/ros.h>
#include "autoware_msgs/VehicleCmd.h"

namespace sd{
    void SetCrc(can_msgs::Frame&, uint8_t);
    void TranslateCANData(can_msgs::Frame&, ros::Publisher, double&, double&);
    void pid_calc( double, double, double, double);
    void SteerTorqueControl(can_msgs::Frame&, double, double, double&, double&, double);
    void SetAutoCommands(can_msgs::Frame&, double*);
    void Reset(can_msgs::Frame&, double&, double&);
    bool CheckAuto(can_msgs::Frame, bool);
    bool CheckGiven(can_msgs::Frame, bool);
}

namespace speedcontroller{
    double Steer_Calc(double, double);
    double Speed_Calc(double, double, double, double);
}
