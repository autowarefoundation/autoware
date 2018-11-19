/*
 * signals.cpp
 *
 *  Created on: Apr 9, 2015
 *      Author: sujiwo
 */


#include <iostream>
#include <ros/ros.h>
#include "Rate.h"
#include "libvectormap/vector_map.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <signal.h>
#include <cstdio>
#include "libvectormap/Math.h"
#include <Eigen/Eigen>
#include <autoware_msgs/Signals.h>
#include <autoware_msgs/AdjustXY.h>
#include <vector_map/vector_map.h>
#include <vector_map_server/GetSignal.h>
#include <autoware_msgs/Lane.h>

static std::string camera_id_str;

static constexpr uint32_t SUBSCRIBE_QUEUE_SIZE = 1000;

static int adjust_proj_x = 0;
static int adjust_proj_y = 0;

typedef struct
{
	double thiX;
	double thiY;
	double thiZ;
} Angle;

static VectorMap vmap;
//static Angle cameraOrientation; // camera orientation = car's orientation

static Eigen::Vector3f position;
static Eigen::Quaternionf orientation;
static float fx,
		fy,
		imageWidth,
		imageHeight,
		cx,
		cy;
static tf::StampedTransform trf;

static bool g_use_vector_map_server; // Switch flag whether vecter-map-server function will be used
static ros::ServiceClient g_ros_client;

#define SignalLampRadius 0.3

/* Define utility class to use vector map server */
namespace
{
	class VectorMapClient
	{
	private:
		geometry_msgs::PoseStamped pose_;
		autoware_msgs::Lane waypoints_;

	public:
		VectorMapClient()
		{
		}

		~VectorMapClient()
		{
		}

		geometry_msgs::PoseStamped pose() const
		{
			return pose_;
		}

		autoware_msgs::Lane waypoints() const
		{
			return waypoints_;
		}

		void set_pose(const geometry_msgs::PoseStamped &pose)
		{
			pose_ = pose;
		}

		void set_waypoints(const autoware_msgs::Lane &waypoints)
		{
			waypoints_ = waypoints;
		}
	}; // Class VectorMapClient
} // namespace
static VectorMapClient g_vector_map_client;


/* Callback function to shift projection result */
void adjust_xyCallback(const autoware_msgs::AdjustXY::ConstPtr &config_msg)
{
	adjust_proj_x = config_msg->x;
	adjust_proj_y = config_msg->y;
}

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr camInfoMsg)
{
	fx = static_cast<float>(camInfoMsg->P[0]);
	fy = static_cast<float>(camInfoMsg->P[5]);
	imageWidth = camInfoMsg->width;
	imageHeight = camInfoMsg->height;
	cx = static_cast<float>(camInfoMsg->P[2]);
	cy = static_cast<float>(camInfoMsg->P[6]);
}


/* convert degree value into 0 to 360 range */
/*static double setDegree0to360(double val)
{
	if (val < 0.0f)
	{
		return (val + 360.0f);
	} else if (360.0f < val)
	{
		return (val - 360.0f);
	}

	return val;
}*/


/*static void get_cameraRollPitchYaw(double *roll,
                                   double *pitch,
                                   double *yaw)
{
	geometry_msgs::Pose cameraPose;
	cameraPose.position.x = (double) (position.x());
	cameraPose.position.y = (double) (position.y());
	cameraPose.position.z = (double) (position.z());
	cameraPose.orientation.x = (double) (orientation.x());
	cameraPose.orientation.y = (double) (orientation.y());
	cameraPose.orientation.z = (double) (orientation.z());
	cameraPose.orientation.w = (double) (orientation.w());

	tf::Quaternion quat;

	tf::quaternionMsgToTF(cameraPose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(*roll, *pitch, *yaw);

	// convert from radian to degree
	*roll = setDegree0to360(*roll * 180.0f / M_PI);
	*pitch = setDegree0to360(*pitch * 180.0f / M_PI);
	*yaw = setDegree0to360(*yaw * 180.0f / M_PI);
}*/


/*
  check if lower < val < upper
  This function also considers circulation
*/
static bool isRange(const double lower, const double upper, const double val)
{
	if (lower <= upper)
	{
		if (lower < val && val < upper)
		{
			return true;
		}
	} else
	{
		if (val < upper || lower < val)
		{
			return true;
		}
	}

	return false;
}


void getTransform(Eigen::Quaternionf &ori, Point3 &pos)
{
	static tf::TransformListener listener;

	// target_frame    source_frame
	ros::Time now = ros::Time();
	listener.waitForTransform(camera_id_str, "map", now, ros::Duration(10.0));
	listener.lookupTransform(camera_id_str, "map", now, trf);

	tf::Vector3 &p = trf.getOrigin();
	tf::Quaternion o = trf.getRotation();
	pos.x() = p.x();
	pos.y() = p.y();
	pos.z() = p.z();
	ori.w() = o.w();
	ori.x() = o.x();
	ori.y() = o.y();
	ori.z() = o.z();
}


Point3 transform(const Point3 &psrc, tf::StampedTransform &tfsource)
{
	tf::Vector3 pt3(psrc.x(), psrc.y(), psrc.z());
	tf::Vector3 pt3s = tfsource * pt3;
	return Point3(pt3s.x(), pt3s.y(), pt3s.z());
}


/*
 * Project a point from world coordinate to image plane
 */
bool project2(const Point3 &pt, int &u, int &v, bool useOpenGLCoord = false)
{
	float nearPlane = 1.0;
	float farPlane = 200.0;
	Point3 _pt = transform(pt, trf);
	float _u = _pt.x() * fx / _pt.z() + cx;
	float _v = _pt.y() * fy / _pt.z() + cy;

	u = static_cast<int>(_u);
	v = static_cast<int>(_v);
	if (u < 0 || imageWidth < u || v < 0 || imageHeight < v || _pt.z() < nearPlane || farPlane < _pt.z())
	{
		u = -1, v = -1;
		return false;
	}

	if (useOpenGLCoord)
	{
		v = imageHeight - v;
	}

	return true;
}

double ConvertDegreeToRadian(double degree)
{
	return degree * M_PI / 180.0f;
}


double ConvertRadianToDegree(double radian)
{
	return radian * 180.0f / M_PI;
}


double GetSignalAngleInCameraSystem(double hang, double vang)
{
	// Fit the vector map format into ROS style
	double signal_pitch_in_map = ConvertDegreeToRadian(vang - 90);
	double signal_yaw_in_map = ConvertDegreeToRadian(-hang + 90);

	tf::Quaternion signal_orientation_in_map_system;
	signal_orientation_in_map_system.setRPY(0, signal_pitch_in_map, signal_yaw_in_map);

	tf::Quaternion signal_orientation_in_cam_system = trf * signal_orientation_in_map_system;
	double signal_roll_in_cam;
	double signal_pitch_in_cam;
	double signal_yaw_in_cam;
	tf::Matrix3x3(signal_orientation_in_cam_system).getRPY(signal_roll_in_cam,
	                                                       signal_pitch_in_cam,
	                                                       signal_yaw_in_cam);

	return ConvertRadianToDegree(signal_pitch_in_cam);   // holizontal angle of camera is represented by pitch
}  // double GetSignalAngleInCameraSystem()


void echoSignals2(ros::Publisher &pub, bool useOpenGLCoord = false)
{
	int countPoint = 0;
	autoware_msgs::Signals signalsInFrame;

	/* Get signals on the path if vecter_map_server is enabled */
	if (g_use_vector_map_server)
	{
		vector_map_server::GetSignal service;
		/* Set server's request */
		service.request.pose = g_vector_map_client.pose();
		service.request.waypoints = g_vector_map_client.waypoints();

		/* Get server's response*/
		if (g_ros_client.call(service))
		{
			/* Reset signal data container */
			vmap.signals.clear();

			/* Newle insert signal data on the path */
			for (const auto &response: service.response.objects.data)
			{
				if (response.id == 0)
					continue;

				Signal signal;
				signal.id = response.id;
				signal.vid = response.vid;
				signal.plid = response.plid;
				signal.type = response.type;
				signal.linkid = response.linkid;

				vmap.signals.insert(std::map<int, Signal>::value_type(signal.id, signal));
			}
			ROS_INFO("[feat_proj] VectorMapServer available. Publishing only TrafficSignals on the current lane");
		}
	}

	for (const auto& signal_map : vmap.signals)
	{
		const Signal signal = signal_map.second;
		int pid = vmap.vectors[signal.vid].pid;

		Point3 signalcenter = vmap.getPoint(pid);
		Point3 signalcenterx(signalcenter.x(), signalcenter.y(), signalcenter.z() + SignalLampRadius);

		int u, v;
		if (project2(signalcenter, u, v, useOpenGLCoord) == true)
		{
			countPoint++;
			// std::cout << u << ", " << v << ", " << std::endl;

			int radius;
			int ux, vx;
			project2(signalcenterx, ux, vx, useOpenGLCoord);
			radius = (int) distance(ux, vx, u, v);

			autoware_msgs::ExtractedPosition sign;
			sign.signalId = signal.id;

			sign.u = u + adjust_proj_x; // shift project position by configuration value from runtime manager
			sign.v = v + adjust_proj_y; // shift project position by configuration value from runtime manager

			sign.radius = radius;
			sign.x = signalcenter.x(), sign.y = signalcenter.y(), sign.z = signalcenter.z();
			sign.hang = vmap.vectors[signal.vid].hang; // hang is expressed in [0, 360] degree
			sign.type = signal.type, sign.linkId = signal.linkid;
			sign.plId = signal.plid;

			// Get holizontal angle of signal in camera corrdinate system
			double signal_angle = GetSignalAngleInCameraSystem(vmap.vectors[signal.vid].hang + 180.0f,
			                                                   vmap.vectors[signal.vid].vang + 180.0f);

			// signal_angle will be zero if signal faces to x-axis
			// Target signal should be face to -50 <= z-axis (= 90 degree) <= +50
			if (isRange(-50, 50, signal_angle - 90))
			{
				signalsInFrame.Signals.push_back(sign);
			}
		}
	}
	signalsInFrame.header.stamp = ros::Time::now();
	pub.publish(signalsInFrame);

	std::cout << "There are " << signalsInFrame.Signals.size() << " signals in range" << std::endl;
}


void interrupt(int s)
{
	ros::shutdown();
	exit(1);
}


int main(int argc, char *argv[])
{

	ros::init(argc, argv, "feat_proj", ros::init_options::NoSigintHandler);
	ros::NodeHandle rosnode;
	ros::NodeHandle private_nh("~");
	std::string cameraInfo_topic_name;
	private_nh.param<std::string>("camera_info_topic", cameraInfo_topic_name, "/camera_info");

	/* get camera ID */
	camera_id_str = cameraInfo_topic_name;
	camera_id_str.erase(camera_id_str.find("/camera_info"));
	if (camera_id_str == "/")
	{
		camera_id_str = "camera";
	}

	/* Get Flag wheter vecter_map_server function will be used  */
	private_nh.param<bool>("use_path_info", g_use_vector_map_server, false);
	ROS_INFO("[feat_proj] Use VectorMapServer: %d", g_use_vector_map_server);
	/* load vector map */
	ros::Subscriber sub_point = rosnode.subscribe("vector_map_info/point",
	                                              SUBSCRIBE_QUEUE_SIZE,
	                                              &VectorMap::load_points,
	                                              &vmap);
	ros::Subscriber sub_line = rosnode.subscribe("vector_map_info/line",
	                                             SUBSCRIBE_QUEUE_SIZE,
	                                             &VectorMap::load_lines,
	                                             &vmap);
	ros::Subscriber sub_lane = rosnode.subscribe("vector_map_info/lane",
	                                             SUBSCRIBE_QUEUE_SIZE,
	                                             &VectorMap::load_lanes,
	                                             &vmap);
	ros::Subscriber sub_vector = rosnode.subscribe("vector_map_info/vector",
	                                               SUBSCRIBE_QUEUE_SIZE,
	                                               &VectorMap::load_vectors,
	                                               &vmap);
	ros::Subscriber sub_signal = rosnode.subscribe("vector_map_info/signal",
	                                               SUBSCRIBE_QUEUE_SIZE,
	                                               &VectorMap::load_signals,
	                                               &vmap);
	ros::Subscriber sub_whiteline = rosnode.subscribe("vector_map_info/white_line",
	                                                  SUBSCRIBE_QUEUE_SIZE,
	                                                  &VectorMap::load_whitelines,
	                                                  &vmap);
	ros::Subscriber sub_dtlane = rosnode.subscribe("vector_map_info/dtlane",
	                                               SUBSCRIBE_QUEUE_SIZE,
	                                               &VectorMap::load_dtlanes,
	                                               &vmap);

	/* wait until loading all vector map is completed */
	ros::Rate wait_rate(100);
	std::cout << "Loading Vector Map. Please wait";
	while (vmap.points.empty() || vmap.lines.empty() || vmap.whitelines.empty() ||
	       vmap.lanes.empty() || vmap.dtlanes.empty() || vmap.vectors.empty() || vmap.signals.empty())
	{
		ros::spinOnce();
		std::cout << ".";
		wait_rate.sleep();
	}

	vmap.loaded = true;
	std::cout << "Loaded." << std::endl;

	ros::Subscriber cameraInfoSubscriber = rosnode.subscribe(cameraInfo_topic_name, 100, cameraInfoCallback);
	ros::Subscriber cameraImage = rosnode.subscribe(cameraInfo_topic_name, 100, cameraInfoCallback);
	ros::Subscriber adjust_xySubscriber = rosnode.subscribe("/config/adjust_xy", 100, adjust_xyCallback);
	ros::Subscriber current_pose_subscriber;
	ros::Subscriber waypoint_subscriber;
	if (g_use_vector_map_server)
	{
		/* Create subscribers which deliver informations requested by server */
		current_pose_subscriber = rosnode.subscribe("/current_pose", 1, &VectorMapClient::set_pose,
		                                            &g_vector_map_client);
		waypoint_subscriber = rosnode.subscribe("/final_waypoints", 1, &VectorMapClient::set_waypoints,
		                                        &g_vector_map_client);

		/* Create ros client to use Server-Client communication */
		g_ros_client = rosnode.serviceClient<vector_map_server::GetSignal>("vector_map_server/get_signal");
	}

	ros::Publisher signalPublisher = rosnode.advertise<autoware_msgs::Signals>("roi_signal", 100);
	signal(SIGINT, interrupt);

	Rate loop(50);
	Eigen::Vector3f prev_position(0,0,0);
	Eigen::Quaternionf prev_orientation(0,0,0,0);
	while (true)
	{
		ros::spinOnce();

		try
		{
			getTransform(orientation, position);
		}
		catch (tf::TransformException &exc)
		{
		}

		if (prev_orientation.vec() != orientation.vec()  &&
		    prev_position != position)
		{
			echoSignals2(signalPublisher, false);
		}
		prev_orientation = orientation;
		prev_position = position;
		loop.sleep();
	}

}
