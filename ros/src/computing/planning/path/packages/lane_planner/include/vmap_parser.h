#ifndef _VMAP_PARSER_H
#define _VMAP_PARSER_H

#include <vector>

class Point;
class Node;
class Lane;
class Signal;

class Point {
private:
	int pid_;
	double b_;
	double l_;
	double h_;
	double bx_;
	double ly_;
	double ref_;
	int mcode1_;
	int mcode2_;
	int mcode3_;

public:
	explicit Point(int pid, double b, double l, double h, double bx,
		       double ly, double ref, int mcode1, int mcode2,
		       int mcode3);

	int pid() const;
	double b() const;
	double l() const;
	double h() const;
	double bx() const;
	double ly() const;
	double ref() const;
	int mcode1() const;
	int mcode2() const;
	int mcode3() const;

	int to_lane_index(const std::vector<Node>& nodes,
			  const std::vector<Lane>& lanes) const;
};

class Node {
private:
	int nid_;
	int pid_;

public:
	explicit Node(int nid, int pid);

	int nid() const;
	int pid() const;
};

class Lane {
private:
	int lnid_;
	int did_;
	int blid_;
	int flid_;
	int bnid_;
	int fnid_;
	int jct_;
	int blid2_;
	int blid3_;
	int blid4_;
	int flid2_;
	int flid3_;
	int flid4_;
	int clossid_;
	double span_;
	int lcnt_;
	int lno_;

public:
	explicit Lane(int lnid, int did, int blid, int flid, int bnid,
		      int fnid, int jct, int blid2, int blid3, int blid4,
		      int flid2, int flid3, int flid4, int clossid,
		      double span, int lcnt, int lno);

	int lnid() const;
	int did() const;
	int blid() const;
	int flid() const;
	int bnid() const;
	int fnid() const;
	int jct() const;
	int blid2() const;
	int blid3() const;
	int blid4() const;
	int flid2() const;
	int flid3() const;
	int flid4() const;
	int clossid() const;
	double span() const;
	int lcnt() const;
	int lno() const;

	int to_next_lane_index(const std::vector<Lane>& lanes) const;
	int to_beginning_point_index(const std::vector<Node>& nodes,
				     const std::vector<Point>& points) const;
	int to_finishing_point_index(const std::vector<Node>& nodes,
				     const std::vector<Point>& points) const;
};

class Signal {
private:
	int id_;
	int vid_;
	int plid_;
	int type_;
	int linkid_;

public:
	explicit Signal(int id, int vid, int plid, int type, int linkid);

	int id() const;
	int vid() const;
	int plid() const;
	int type() const;
	int linkid() const;
};

std::vector<Point> read_point(const char *filename);
std::vector<Node> read_node(const char *filename);
std::vector<Lane> read_lane(const char *filename);
std::vector<Signal> read_signal(const char *filename);

Point search_nearest(const std::vector<Point>& points, double x, double y);

#endif /* !_VMAP_PARSER_H */
