
#ifndef _ORB_UTILS_H
#define _ORB_UTILS_H 1

#include <tf/tf.h>
#include <opencv2/core/core.hpp>
#include <rosbag/bag.h>
#include <string>
#include <vector>
#include <exception>


using std::string;
using std::vector;
using std::exception;


void tfToCV(const tf::Transform &src, cv::Mat &position, cv::Mat &orientation);



class TfTimeTree
{
public:
	TfTimeTree (const string &bagSrcPath, const string &fromFrame, const string &toFrame, const tf::Transform &shift=tf::Transform());
	const tf::Transform& search(const double timestamp, float timeTolerance=0.1);
	const tf::Transform& search(const ros::Time &time, float timeTolerance=0.1);

	class time_not_found: public std::out_of_range {};

private:

	void insert (const tf::Transform &n, double &ts);

	struct Node {
		tf::Transform transform;
		double timevalue;
		Node *left, *right;

//		Node (tf::Transform &tsrc, double t) :
//			transform(tsrc),
//			timevalue(t),
//			left(NULL), right(NULL) {}

		Node (const tf::Transform &tsrc, double t, Node *l=NULL, Node *r=NULL) :
			transform(tsrc),
			timevalue(t),
			left(l), right(r) {}
	};
	vector<Node> allNodes;
	uint32_t curpoint;

	Node *root;
};


#endif /* _ORB_UTILS_H */
