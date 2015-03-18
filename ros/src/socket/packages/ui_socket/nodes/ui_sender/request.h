/*
 *  Copyright (c) 2015, Nagoya University
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

#define ERROR_INFO_TYPE		1
#define CAN_INFO_TYPE		2
#define MODE_INFO_TYPE		3

struct error_request {
	int32_t type;
	int32_t error;

	error_request(const ui_socket::error_info& msg)
	: type(ERROR_INFO_TYPE), error(msg.error) {
	}
};

struct can_request {
	int32_t type;
	char tm[32];
	int32_t devmode;
	int32_t drvcontmode;
	int32_t drvoverridemode;
	int32_t drvservo;
	int32_t drivepedal;
	int32_t targetpedalstr;
	int32_t inputpedalstr;
	double targetveloc;
	double speed;
	int32_t driveshift;
	int32_t targetshift;
	int32_t inputshift;
	int32_t strmode;
	int32_t strcontmode;
	int32_t stroverridemode;
	int32_t strservo;
	int32_t targettorque;
	int32_t torque;
	double angle;
	double targetangle;
	int32_t bbrakepress;
	int32_t brakepedal;
	int32_t brtargetpedalstr;
	int32_t brinputpedalstr;
	double battery;
	int32_t voltage;
	double anp;
	int32_t battmaxtemparature;
	int32_t battmintemparature;
	double maxchgcurrent;
	double maxdischgcurrent;
	double sideacc;
	double accellfromp;
	double anglefromp;
	double brakepedalfromp;
	double speedfr;
	double speedfl;
	double speedrr;
	double speedrl;
	double velocfromp2;
	int32_t drvmode;
	int32_t devpedalstrfromp;
	int32_t rpm;
	double velocflfromp;
	int32_t ev_mode;
	int32_t temp;
	int32_t shiftfrmprius;
	int32_t light;
	int32_t gaslevel;
	int32_t door;
	int32_t cluise;

	can_request(const vehicle_socket::CanInfo& msg) {
		type = CAN_INFO_TYPE;
		msg.tm.copy(tm, 32, 0);
		devmode = msg.devmode;
		drvcontmode = msg.drvcontmode;
		drvoverridemode = msg.drvoverridemode;
		drvservo = msg.drvservo;
		drivepedal = msg.drivepedal;
		targetpedalstr = msg.targetpedalstr;
		inputpedalstr = msg.inputpedalstr;
		targetveloc = msg.targetveloc;
		speed = msg.speed;
		driveshift = msg.driveshift;
		targetshift = msg.targetshift;
		inputshift = msg.inputshift;
		strmode = msg.strmode;
		strcontmode = msg.strcontmode;
		stroverridemode = msg.stroverridemode;
		strservo = msg.strservo;
		targettorque = msg.targettorque;
		torque = msg.torque;
		angle = msg.angle;
		targetangle = msg.targetangle;
		bbrakepress = msg.bbrakepress;
		brakepedal = msg.brakepedal;
		brtargetpedalstr = msg.brtargetpedalstr;
		brinputpedalstr = msg.brinputpedalstr;
		battery = msg.battery;
		voltage = msg.voltage;
		anp = msg.anp;
		battmaxtemparature = msg.battmaxtemparature;
		battmintemparature = msg.battmintemparature;
		maxchgcurrent = msg.maxchgcurrent;
		maxdischgcurrent = msg.maxdischgcurrent;
		sideacc = msg.sideacc;
		accellfromp = msg.accellfromp;
		anglefromp = msg.anglefromp;
		brakepedalfromp = msg.brakepedalfromp;
		speedfr = msg.speedfr;
		speedfl = msg.speedfl;
		speedrr = msg.speedrr;
		speedrl = msg.speedrl;
		velocfromp2 = msg.velocfromp2;
		drvmode = msg.drvmode;
		devpedalstrfromp = msg.devpedalstrfromp;
		rpm = msg.rpm;
		velocflfromp = msg.velocflfromp;
		ev_mode = msg.ev_mode;
		temp = msg.temp;
		shiftfrmprius = msg.shiftfrmprius;
		light = msg.light;
		gaslevel = msg.gaslevel;
		door = msg.door;
		cluise = msg.cluise;
	}
};

struct mode_request {
	int32_t type;
	int32_t mode;

	mode_request(const ui_socket::mode_info& msg)
	: type(MODE_INFO_TYPE), mode(msg.mode) {
	}
};
