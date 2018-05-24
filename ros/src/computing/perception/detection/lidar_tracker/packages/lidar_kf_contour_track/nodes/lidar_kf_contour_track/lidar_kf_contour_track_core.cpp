/*
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 * this
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
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "lidar_kf_contour_track_core.h"
#include "op_ros_helpers/op_RosHelpers.h"

namespace ContourTrackerNS {

ContourTracker::ContourTracker() {
  bNewClusters = false;
  bNewCurrentPos = false;
  ros::NodeHandle _nh;
  _nh.getParam("/kf_contour_tracker/vehicle_width", m_Params.VehicleWidth);
  _nh.getParam("/kf_contour_tracker/vehicle_length", m_Params.VehicleLength);
  _nh.getParam("/kf_contour_tracker/horizon", m_Params.DetectionRadius);
  _nh.getParam("/kf_contour_tracker/min_object_size", m_Params.MinObjSize);
  _nh.getParam("/kf_contour_tracker/max_object_size", m_Params.MaxObjSize);
  _nh.getParam("/kf_contour_tracker/polygon_quarters", m_Params.nQuarters);
  _nh.getParam("/kf_contour_tracker/polygon_resolution", m_Params.PolygonRes);
  _nh.getParam("/kf_contour_tracker/max_association_distance",
               m_ObstacleTracking.m_MAX_ASSOCIATION_DISTANCE);
  _nh.getParam("/kf_contour_tracker/max_association_size_diff",
               m_ObstacleTracking.m_MAX_ASSOCIATION_SIZE_DIFF);

  int tracking_type = 0;
  _nh.getParam("/kf_contour_tracker/tracking_type", tracking_type);
  if (tracking_type == 0)
    m_Params.trackingType = SimulationNS::ASSOCIATE_ONLY;
  else if (tracking_type == 1)
    m_Params.trackingType = SimulationNS::SIMPLE_TRACKER;
  else if (tracking_type == 2)
    m_Params.trackingType = SimulationNS::CONTOUR_TRACKER;

  _nh.getParam("/kf_contour_tracker/max_remeber_time",
               m_ObstacleTracking.m_MaxKeepTime);
  _nh.getParam("/kf_contour_tracker/trust_counter",
               m_ObstacleTracking.m_nMinTrustAppearances);
  _nh.getParam("/kf_contour_tracker/contours_circle_resolutions",
               m_ObstacleTracking.m_CirclesResolution);

  m_ObstacleTracking.m_dt = 0.12;
  m_ObstacleTracking.m_bUseCenterOnly = true;
  m_ObstacleTracking.m_Horizon = m_Params.DetectionRadius;
  m_ObstacleTracking.InitSimpleTracker();

  sub_cloud_clusters = nh.subscribe(
      "/cloud_clusters", 1, &ContourTracker::callbackGetCloudClusters, this);
  sub_current_pose = nh.subscribe(
      "/current_pose", 1, &ContourTracker::callbackGetCurrentPose, this);

  pub_AllTrackedObjects =
      nh.advertise<autoware_msgs::DetectedObjectArray>("/detected_objects", 1);
  pub_DetectedPolygonsRviz =
      nh.advertise<visualization_msgs::MarkerArray>("detected_polygons", 1);
  pub_TrackedObstaclesRviz =
      nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(
          "op_planner_tracked_boxes", 1);

  m_nDummyObjPerRep = 100;
  m_nDetectedObjRepresentations = 5;
  m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
  m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
  m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
  m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
  m_DetectedPolygonsDummy.push_back(visualization_msgs::MarkerArray());
  m_DetectedPolygonsActual = m_DetectedPolygonsDummy;
  PlannerHNS::RosHelpers::InitMarkers(
      m_nDummyObjPerRep, m_DetectedPolygonsDummy.at(0),
      m_DetectedPolygonsDummy.at(1), m_DetectedPolygonsDummy.at(2),
      m_DetectedPolygonsDummy.at(3), m_DetectedPolygonsDummy.at(4));
}

ContourTracker::~ContourTracker() {}

void ContourTracker::callbackGetCloudClusters(
    const autoware_msgs::CloudClusterArrayConstPtr &msg) {
  if (bNewCurrentPos) {
    m_OriginalClusters.clear();
    int nOriginalPoints = 0;
    int nContourPoints = 0;

    PlannerHNS::RosHelpers::ConvertFromAutowareCloudClusterObstaclesToPlannerH(
        m_CurrentPos, m_Params.VehicleWidth, m_Params.VehicleLength, *msg,
        m_OriginalClusters, m_Params.MaxObjSize, m_Params.MinObjSize,
        m_Params.DetectionRadius, m_Params.nQuarters, m_Params.PolygonRes,
        nOriginalPoints, nContourPoints);

    bNewClusters = true;
  }
}

void ContourTracker::callbackGetCurrentPose(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  m_CurrentPos = PlannerHNS::WayPoint(
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
      tf::getYaw(msg->pose.orientation));
  bNewCurrentPos = true;
}

void ContourTracker::VisualizeLocalTracking() {
  PlannerHNS::RosHelpers::ConvertTrackedObjectsMarkers(
      m_CurrentPos, m_ObstacleTracking.m_DetectedObjects,
      m_DetectedPolygonsDummy.at(0), m_DetectedPolygonsDummy.at(1),
      m_DetectedPolygonsDummy.at(2), m_DetectedPolygonsDummy.at(3),
      m_DetectedPolygonsDummy.at(4), m_DetectedPolygonsActual.at(0),
      m_DetectedPolygonsActual.at(1), m_DetectedPolygonsActual.at(2),
      m_DetectedPolygonsActual.at(3), m_DetectedPolygonsActual.at(4));

  m_DetectedPolygonsAllMarkers.markers.clear();
  m_DetectedPolygonsAllMarkers.markers.insert(
      m_DetectedPolygonsAllMarkers.markers.end(),
      m_DetectedPolygonsActual.at(0).markers.begin(),
      m_DetectedPolygonsActual.at(0).markers.end());
  m_DetectedPolygonsAllMarkers.markers.insert(
      m_DetectedPolygonsAllMarkers.markers.end(),
      m_DetectedPolygonsActual.at(1).markers.begin(),
      m_DetectedPolygonsActual.at(1).markers.end());
  m_DetectedPolygonsAllMarkers.markers.insert(
      m_DetectedPolygonsAllMarkers.markers.end(),
      m_DetectedPolygonsActual.at(2).markers.begin(),
      m_DetectedPolygonsActual.at(2).markers.end());
  m_DetectedPolygonsAllMarkers.markers.insert(
      m_DetectedPolygonsAllMarkers.markers.end(),
      m_DetectedPolygonsActual.at(3).markers.begin(),
      m_DetectedPolygonsActual.at(3).markers.end());
  m_DetectedPolygonsAllMarkers.markers.insert(
      m_DetectedPolygonsAllMarkers.markers.end(),
      m_DetectedPolygonsActual.at(4).markers.begin(),
      m_DetectedPolygonsActual.at(4).markers.end());

  visualization_msgs::MarkerArray all_circles;
  for (unsigned int i = 0; i < m_ObstacleTracking.m_InterestRegions.size();
       i++) {
    visualization_msgs::Marker circle_mkrs;
    PlannerHNS::RosHelpers::CreateCircleMarker(
        m_CurrentPos, m_ObstacleTracking.m_InterestRegions.at(i)->radius, i,
        circle_mkrs);
    all_circles.markers.push_back(circle_mkrs);
  }

  m_DetectedPolygonsAllMarkers.markers.insert(
      m_DetectedPolygonsAllMarkers.markers.end(), all_circles.markers.begin(),
      all_circles.markers.end());

  pub_DetectedPolygonsRviz.publish(m_DetectedPolygonsAllMarkers);

  jsk_recognition_msgs::BoundingBoxArray boxes_array;
  boxes_array.header.frame_id = "map";
  boxes_array.header.stamp = ros::Time();

  for (unsigned int i = 0; i < m_ObstacleTracking.m_DetectedObjects.size();
       i++) {
    jsk_recognition_msgs::BoundingBox box;
    box.header.frame_id = "map";
    box.header.stamp = ros::Time();
    box.pose.position.x =
        m_ObstacleTracking.m_DetectedObjects.at(i).center.pos.x;
    box.pose.position.y =
        m_ObstacleTracking.m_DetectedObjects.at(i).center.pos.y;
    box.pose.position.z =
        m_ObstacleTracking.m_DetectedObjects.at(i).center.pos.z;

    box.value = 0.9;

    box.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
        0, 0, UtilityHNS::UtilityH::SplitPositiveAngle(
                  m_ObstacleTracking.m_DetectedObjects.at(i).center.pos.a));
    box.dimensions.x = m_ObstacleTracking.m_DetectedObjects.at(i).w;
    box.dimensions.y = m_ObstacleTracking.m_DetectedObjects.at(i).l;
    box.dimensions.z = m_ObstacleTracking.m_DetectedObjects.at(i).h;
    boxes_array.boxes.push_back(box);
  }

  pub_TrackedObstaclesRviz.publish(boxes_array);
}

void ContourTracker::MainLoop() {
  ros::Rate loop_rate(25);

  PlannerHNS::WayPoint prevState, state_change;

  while (ros::ok()) {
    ros::spinOnce();

    if (bNewClusters) {
      m_ObstacleTracking.DoOneStep(m_CurrentPos, m_OriginalClusters,
                                   m_Params.trackingType);

      m_OutPutResults.objects.clear();
      autoware_msgs::DetectedObject obj;
      for (unsigned int i = 0; i < m_ObstacleTracking.m_DetectedObjects.size();
           i++) {
        obj.id = m_ObstacleTracking.m_DetectedObjects.at(i).id;
        obj.label = m_ObstacleTracking.m_DetectedObjects.at(i).label;
        obj.dimensions.x = m_ObstacleTracking.m_DetectedObjects.at(i).l;
        obj.dimensions.y = m_ObstacleTracking.m_DetectedObjects.at(i).w;
        obj.dimensions.z = m_ObstacleTracking.m_DetectedObjects.at(i).h;

        obj.pose.position.x =
            m_ObstacleTracking.m_DetectedObjects.at(i).center.pos.x;
        obj.pose.position.y =
            m_ObstacleTracking.m_DetectedObjects.at(i).center.pos.y;
        obj.pose.position.z =
            m_ObstacleTracking.m_DetectedObjects.at(i).center.pos.z;

        obj.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
            0, 0, UtilityHNS::UtilityH::SplitPositiveAngle(
                      m_ObstacleTracking.m_DetectedObjects.at(i).center.pos.a));

        geometry_msgs::Point32 p;
        obj.convex_hull.polygon.points.clear();

        for (unsigned int j = 0;
             j < m_ObstacleTracking.m_DetectedObjects.at(i).contour.size();
             j++) {
          p.x = m_ObstacleTracking.m_DetectedObjects.at(i).contour.at(j).x;
          p.y = m_ObstacleTracking.m_DetectedObjects.at(i).contour.at(j).y;
          p.z = m_ObstacleTracking.m_DetectedObjects.at(i).contour.at(j).z;
          obj.convex_hull.polygon.points.push_back(p);
        }

        m_OutPutResults.objects.push_back(obj);
      }

      m_OutPutResults.header.frame_id = "map";
      m_OutPutResults.header.stamp = ros::Time();

      pub_AllTrackedObjects.publish(m_OutPutResults);

      VisualizeLocalTracking();

      bNewClusters = false;
    }
    loop_rate.sleep();
  }
}
}
