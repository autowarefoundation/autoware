#include "utils.h"
#include <boost/foreach.hpp>
#include <rosbag/view.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>


#define foreach BOOST_FOREACH


using std::cout;
using std::endl;


void tfToCV(const tf::Transform &src, cv::Mat &position, cv::Mat &orientation)
{
	position = cv::Mat (3,1,CV_64F);
	tf::Vector3 p = src.getOrigin();
	position.at<double>(0) = p.x(),
		position.at<double>(1) = p.y(),
		position.at<double>(2) = p.z();

	orientation = cv::Mat (4,1,CV_64F);
	tf::Quaternion otn = src.getRotation();
	orientation.at<double>(0) = otn.x(),
		orientation.at<double>(1) = otn.y(),
		orientation.at<double>(2) = otn.z(),
		orientation.at<double>(3) = otn.w();
}


TfTimeTree::TfTimeTree(const string &bagSrcPath, const string &fromFrame, const string &toFrame, const tf::Transform &shift) :
	curpoint(INT_MAX),
	root (NULL)
{
	rosbag::Bag bfg(bagSrcPath);
	rosbag::View view(bfg, rosbag::TopicQuery(string("/tf")));

	foreach (rosbag::MessageInstance m, view) {
		tf::tfMessageConstPtr tfm = m.instantiate<tf::tfMessage>();

		if (tfm->transforms[0].header.frame_id != fromFrame or
			tfm->transforms[0].child_frame_id != toFrame)
			continue;

		double timeMsg = tfm->transforms[0].header.stamp.toSec();

		tf::Transform ctf;
		ctf.setOrigin(tf::Vector3(
			tfm->transforms[0].transform.translation.x,
			tfm->transforms[0].transform.translation.y,
			tfm->transforms[0].transform.translation.z
		));
		ctf.setRotation(tf::Quaternion(
			tfm->transforms[0].transform.rotation.x,
			tfm->transforms[0].transform.rotation.y,
			tfm->transforms[0].transform.rotation.z,
			tfm->transforms[0].transform.rotation.w
		));

		Node cn(ctf, timeMsg);
		allNodes.push_back(cn);
//		insert (ctf, timeMsg);
//		i ++;
	}

	cout << std::fixed << "From: " << std::setprecision(7) << allNodes[0].timevalue << " to " << allNodes.back().timevalue << endl;
	cout << "Length: " << allNodes.size() << endl;
}


const tf::Transform& TfTimeTree::search(const double timestamp, float timeTolerance)
{
//	Node *cnode = root;
//	while (cnode != NULL) {
//		if (abs(cnode->timevalue-timeTolerance)<timeTolerance)
//			return cnode->transform;
//		else if (cnode->timevalue < timestamp)
//			cnode = cnode->right;
//		else
//			cnode = cnode->left;
//	}
//	if (cnode==NULL)
//		throw std::out_of_range("Outside range");

	if (timestamp < allNodes[0].timevalue)
		throw std::out_of_range("Less than minimum");
	if (timestamp > allNodes.back().timevalue)
		throw std::out_of_range("Outside maximum");

	uint32_t start = (curpoint==INT_MAX ? 0 : curpoint);

	for (uint32_t i=start; i<allNodes.size(); i++) {
		Node &n = allNodes[i];
		if (n.timevalue > timestamp) {
			if (abs(allNodes[i-1].timevalue - timestamp) < abs(n.timevalue-timestamp)) {
				curpoint = i-1;
			}
			else {
				curpoint = i;
			}
			break;
		}
	}
	return allNodes[curpoint].transform;
}


const tf::Transform& TfTimeTree::search(const ros::Time &time, float timeTolerance)
{
	double ts = time.toSec();
	return search(ts, timeTolerance);
}


void TfTimeTree::insert (const tf::Transform &n, double &ts)
{
	Node _cnode(n, ts);
	allNodes.push_back(_cnode);
	Node *cnode = &(allNodes.back());

	if (root==NULL) {
		root = cnode;
		return;
	}

	Node *current = root;
	while (current != NULL) {

		if (cnode->timevalue < current->timevalue) {
			if (current->left==NULL) {
				current->left = cnode;
				cnode->left = NULL;
				cnode->right = NULL;
				current = NULL;
			}
			else {
				current = current->left;
			}
		}
		else {
			if (current->right==NULL) {
				current->right = cnode;
				cnode->left = NULL;
				cnode->right = NULL;
				current = NULL;
			}
			else {
				current = current->right;
			}
		}
	}
}
