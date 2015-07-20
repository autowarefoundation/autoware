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

#ifndef _VECTOR_MAP_LOADER_H
#define _VECTOR_MAP_LOADER_H

/*
 * Graphical Primitive Class
 */

struct PointClass {
	int pid;
	double b;
	double l;
	double h;
	double bx;
	double ly;
	int ref;
	int mcode1;
	int mcode2;
	int mcode3;
};

struct VectorClass {
	int vid;
	int pid;
	double hang;
	double vang;
};

struct LineClass {
	int lid;
	int bpid;
	int fpid;
	int blid;
	int flid;
};

struct AreaClass {
	int aid;
	int slid;
	int elid;
};

struct PoleClass {
	int plid;
	int vid;
	double length;
	double dim;
};

struct BoxClass {
	int bid;
	int pid1;
	int pid2;
	int pid3;
	int pid4;
	double height;
};

/*
 * Road Data
 */

struct DTLane {
	int did;
	double dist;
	int pid;
	double dir;
	double apara;
	double r;
	double slope;
	double cant;
	double lw;
	double rw;
};

struct Node {
	int nid;
	int pid;
};

struct Lane {
	int lnid;
	int did;
	int blid;
	int flid;
	int bnid;
	int fnid;
	int jct;
	int blid2;
	int blid3;
	int blid4;
	int flid2;
	int flid3;
	int flid4;
	int clossid;
	double span;
	int lcnt;
	int lno;
};

/*
 * Object Data
 */

struct RoadEdge {
	int id;
	int lid;
	int linkid;
};

struct Gutter {
	int id;
	int aid;
	int type;
	int linkid;
};

struct Curb {
	int id;
	int lid;
	double height;
	double width;
	int dir;
	int linkid;
};

struct WhiteLine {
	int id;
	int lid;
	double width;
	char color;
	int type;
	int linkid;
};

struct StopLine {
	int id;
	int lid;
	int tlid;
	int signid;
	int linkid;
};

struct ZebraZone {
	int id;
	int aid;
	int linkid;
};

struct CrossWalk {
	int id;
	int aid;
	int type;
	int bdid;
	int linkid;
};

struct RoadMark {
	int id;
	int aid;
	int type; /* Don't use wide character */
	int linkid;
};

struct Pole {
	int id;
	int plid;
	int linkid;
};

struct RoadSign {
	int id;
	int vid;
	int plid;
	int type; /* Don't use wide character */
	int linkid;
};

struct Signal {
	int id;
	int vid;
	int plid;
	int type;
	int linkid;
};

struct StreetLight {
	int id;
	int lid;
	int plid;
	int linkid;
};

struct UtilityPole {
	int id;
	int plid;
	int linkid;
};

struct GuardRail {
	int id;
	int aid;
	int type;
	int linkid;
};

struct SideWalk {
	int id;
	int aid;
	int linkid;
};

struct CrossRoad {
	int id;
	int aid;
	int linkid;
};

#endif /* !_VECTOR_MAP_LOADER_H */
