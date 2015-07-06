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

#include <cmath>
#include <fstream>
#include <sstream>

#include <vmap_parser.h>

Point::Point(int pid, double b, double l, double h, double bx,
	     double ly, double ref, int mcode1, int mcode2,
	     int mcode3)
	: pid_(pid), b_(b), l_(l), h_(h), bx_(bx),
	  ly_(ly), ref_(ref), mcode1_(mcode1), mcode2_(mcode2),
	  mcode3_(mcode3)
{
}

int Point::pid() const
{
	return pid_;
}

double Point::b() const
{
	return b_;
}

double Point::l() const
{
	return l_;
}

double Point::h() const
{
	return h_;
}

double Point::bx() const
{
	return bx_;
}

double Point::ly() const
{
	return ly_;
}

double Point::ref() const
{
	return ref_;
}

int Point::mcode1() const
{
	return mcode1_;
}

int Point::mcode2() const
{
	return mcode2_;
}

int Point::mcode3() const
{
	return mcode3_;
}

int Point::to_lane_index(const std::vector<Node>& nodes,
			 const std::vector<Lane>& lanes) const
{
	for (const Node& node : nodes) {
		if (node.pid() != this->pid())
			continue;
		for (int i = 0; i < static_cast<int>(lanes.size()); i++) {
			if (lanes[i].bnid() != node.nid())
				continue;
			return i;
		}
	}

	return -1;
}

Node::Node(int nid, int pid)
	: nid_(nid), pid_(pid)
{
}

int Node::nid() const
{
	return nid_;
}

int Node::pid() const
{
	return pid_;
}

Lane::Lane(int lnid, int did, int blid, int flid, int bnid,
	   int fnid, int jct, int blid2, int blid3, int blid4,
	   int flid2, int flid3, int flid4, int clossid,
	   double span, int lcnt, int lno)
	: lnid_(lnid), did_(did), blid_(blid), flid_(flid), bnid_(bnid),
	  fnid_(fnid), jct_(jct), blid2_(blid2), blid3_(blid3), blid4_(blid4),
	  flid2_(flid2), flid3_(flid3), flid4_(flid4), clossid_(clossid),
	  span_(span), lcnt_(lcnt), lno_(lno)
{
}

int Lane::lnid() const
{
	return lnid_;
}

int Lane::did() const
{
	return did_;
}

int Lane::blid() const
{
	return blid_;
}

int Lane::flid() const
{
	return flid_;
}

int Lane::bnid() const
{
	return bnid_;
}

int Lane::fnid() const
{
	return fnid_;
}

int Lane::jct() const
{
	return jct_;
}

int Lane::blid2() const
{
	return blid2_;
}

int Lane::blid3() const
{
	return blid3_;
}

int Lane::blid4() const
{
	return blid4_;
}

int Lane::flid2() const
{
	return flid2_;
}

int Lane::flid3() const
{
	return flid3_;
}

int Lane::flid4() const
{
	return flid4_;
}

int Lane::clossid() const
{
	return clossid_;
}

double Lane::span() const
{
	return span_;
}

int Lane::lcnt() const
{
	return lcnt_;
}

int Lane::lno() const
{
	return lno_;
}

int Lane::to_next_lane_index(const std::vector<Lane>& lanes) const
{
	for (int i = 0; i < static_cast<int>(lanes.size()); i++) {
		if (lanes[i].lnid() != this->flid())
			continue;
		return i;
	}

	return -1;
}

int Lane::to_beginning_point_index(const std::vector<Node>& nodes,
				   const std::vector<Point>& points) const
{
	for (const Node& node : nodes) {
		if (node.nid() != this->bnid())
			continue;
		for (int i = 0; i < static_cast<int>(points.size()); i++) {
			if (points[i].pid() != node.pid())
				continue;
			return i;
		}
	}

	return -1;
}

int Lane::to_finishing_point_index(const std::vector<Node>& nodes,
				   const std::vector<Point>& points) const
{
	for (const Node& node : nodes) {
		if (node.nid() != this->fnid())
			continue;
		for (int i = 0; i < static_cast<int>(points.size()); i++) {
			if (points[i].pid() != node.pid())
				continue;
			return i;
		}
	}

	return -1;
}

Signal::Signal(int id, int vid, int plid, int type, int linkid)
	: id_(id), vid_(vid), plid_(plid), type_(type), linkid_(linkid)
{
}

int Signal::id() const
{
	return id_;
}

int Signal::vid() const
{
	return vid_;
}

int Signal::plid() const
{
	return plid_;
}

int Signal::type() const
{
	return type_;
}

int Signal::linkid() const
{
	return linkid_;
}

StopLine::StopLine(int id, int lid, int tlid, int signid, int linkid)
	: id_(id), lid_(lid), tlid_(tlid), signid_(signid), linkid_(linkid)
{
}

int StopLine::id() const
{
	return id_;
}

int StopLine::lid() const
{
	return lid_;
}

int StopLine::tlid() const
{
	return tlid_;
}

int StopLine::signid() const
{
	return signid_;
}

int StopLine::linkid() const
{
	return linkid_;
}

static Point parse_point(const std::string& line)
{
	std::istringstream ss(line);
	std::vector<std::string> columns;

	std::string column;
	while (std::getline(ss, column, ',')) {
		columns.push_back(column);
	}

	return Point(
		std::stoi(columns[0]),
		std::stod(columns[1]),
		std::stod(columns[2]),
		std::stod(columns[3]),
		std::stod(columns[4]),
		std::stod(columns[5]),
		std::stod(columns[6]),
		std::stoi(columns[7]),
		std::stoi(columns[8]),
		std::stoi(columns[9])
		);
}

static Node parse_node(const std::string& line)
{
	std::istringstream ss(line);
	std::vector<std::string> columns;

	std::string column;
	while (std::getline(ss, column, ',')) {
		columns.push_back(column);
	}

	return Node(
		std::stoi(columns[0]),
		std::stoi(columns[1])
		);
}

static Lane parse_lane(const std::string& line)
{
	std::istringstream ss(line);
	std::vector<std::string> columns;

	std::string column;
	while (std::getline(ss, column, ',')) {
		columns.push_back(column);
	}

	return Lane(
		std::stoi(columns[0]),
		std::stoi(columns[1]),
		std::stoi(columns[2]),
		std::stoi(columns[3]),
		std::stoi(columns[4]),
		std::stoi(columns[5]),
		std::stoi(columns[6]),
		std::stoi(columns[7]),
		std::stoi(columns[8]),
		std::stoi(columns[9]),
		std::stoi(columns[10]),
		std::stoi(columns[11]),
		std::stoi(columns[12]),
		std::stoi(columns[13]),
		std::stod(columns[14]),
		std::stoi(columns[15]),
		std::stoi(columns[16])
		);
}

static Signal parse_signal(const std::string& line)
{
	std::istringstream ss(line);
	std::vector<std::string> columns;

	std::string column;
	while (std::getline(ss, column, ',')) {
		columns.push_back(column);
	}

	return Signal(
		std::stoi(columns[0]),
		std::stoi(columns[1]),
		std::stoi(columns[2]),
		std::stoi(columns[3]),
		std::stoi(columns[4])
		);
}

static StopLine parse_stopline(const std::string& line)
{
	std::istringstream ss(line);
	std::vector<std::string> columns;

	std::string column;
	while (std::getline(ss, column, ',')) {
		columns.push_back(column);
	}

	return StopLine(
		std::stoi(columns[0]),
		std::stoi(columns[1]),
		std::stoi(columns[2]),
		std::stoi(columns[3]),
		std::stoi(columns[4])
		);
}

std::vector<Point> read_point(const char *filename)
{
	std::ifstream ifs(filename);
	std::string line;

	std::getline(ifs, line); // Remove first line

	std::vector<Point> points;
	while (std::getline(ifs, line)) {
		points.push_back(parse_point(line));
	}

	return points;
}

std::vector<Node> read_node(const char *filename)
{
	std::ifstream ifs(filename);
	std::string line;

	std::getline(ifs, line); // Remove first line

	std::vector<Node> nodes;
	while (std::getline(ifs, line)) {
		nodes.push_back(parse_node(line));
	}

	return nodes;
}

std::vector<Lane> read_lane(const char *filename)
{
	std::ifstream ifs(filename);
	std::string line;

	std::getline(ifs, line); // Remove first line

	std::vector<Lane> lanes;
	while (std::getline(ifs, line)) {
		lanes.push_back(parse_lane(line));
	}

	return lanes;
}

std::vector<Signal> read_signal(const char *filename)
{
	std::ifstream ifs(filename);
	std::string line;

	std::getline(ifs, line); // Remove first line

	std::vector<Signal> signals;
	while (std::getline(ifs, line)) {
		signals.push_back(parse_signal(line));
	}

	return signals;
}

std::vector<StopLine> read_stopline(const char *filename)
{
	std::ifstream ifs(filename);
	std::string line;

	std::getline(ifs, line); // Remove first line

	std::vector<StopLine> stoplines;
	while (std::getline(ifs, line)) {
		stoplines.push_back(parse_stopline(line));
	}

	return stoplines;
}

Point search_nearest(const std::vector<Point>& points, double x, double y)
{
	Point nearest_point = points[0];
	double min = hypot(x - points[0].bx(), y - points[0].ly());
	for (const Point& point : points) {
		double distance = hypot(x - point.bx(), y - point.ly());
		if (distance < min) {
			min = distance;
			nearest_point = point;
		}
	}

	return nearest_point;
}
