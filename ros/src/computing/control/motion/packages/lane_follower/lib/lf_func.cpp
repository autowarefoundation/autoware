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

#include "lf_func.h"

/////////////////////////////////////////////////////////////////
// transform the waypoint to the vehicle plane.
/////////////////////////////////////////////////////////////////
tf::Vector3 TransformWaypoint(tf::Transform transform,geometry_msgs::Pose pose)
{

    tf::Vector3 waypoint(pose.position.x,pose.position.y,pose.position.z);
    tf::Vector3 tf_w = transform * waypoint;

    return tf_w;
}

int GetClosestWaypoint(tf::Transform transform, lane_follower::lane path, int current_closest)
{
    if (path.waypoints.size() == 0)
        return -1;

    int closest_threshold = 5;

    //interval between 2 waypoints
    tf::Vector3 v1(path.waypoints[0].pose.pose.position.x, path.waypoints[0].pose.pose.position.y, 0);

    tf::Vector3 v2(path.waypoints[1].pose.pose.position.x, path.waypoints[1].pose.pose.position.y, 0);

    for (int ratio = 1; ratio < closest_threshold; ratio++) {

        double distance_threshold = ratio * tf::tfDistance(v1, v2); //meter
        std::cout << "distance_threshold : " << distance_threshold << std::endl;

        std::vector<int> waypoint_candidates;

        for (unsigned int i = 1; i < path.waypoints.size(); i++) {

            //std::cout << waypoint << std::endl;

            // position of @waypoint.
            tf::Vector3 tf_waypoint = TransformWaypoint(transform,path.waypoints[i].pose.pose);
            tf_waypoint.setZ(0);
            //std::cout << "current path (" << path.waypoints[i].pose.pose.position.x << " " << path.waypoints[i].pose.pose.position.y << " " << path.waypoints[i].pose.pose.position.z << ")" << std::endl;

            //double dt = tf::tfDistance(v1, v2);
            double dt = tf::tfDistance(_origin_v, tf_waypoint);
            //  std::cout << i  << " "<< dt << std::endl;
            if (dt < distance_threshold) {
                //add as a candidate
                waypoint_candidates.push_back(i);
                std::cout << "waypoint = " << i << "  distance = " << dt << std::endl;
            }
        }

        if (waypoint_candidates.size() == 0) {
            continue;
        }

        std::cout << "prev closest waypoint : " << current_closest << std::endl;
        int sub_min = waypoint_candidates[0] - current_closest;
        int decided_waypoint = waypoint_candidates[0];
        for (unsigned int i = 0; i < waypoint_candidates.size(); i++) {
            int sub = waypoint_candidates[i] - current_closest;
            std::cout << "closest candidates : " << waypoint_candidates[i] << " sub : " << sub << std::endl;
            if (sub < 0)
                continue;

            if (sub < sub_min) {
                decided_waypoint = waypoint_candidates[i];
                sub_min = sub;
            }
        }
        if (decided_waypoint >= current_closest) {
            return decided_waypoint;
        }

    }
    return -1;
}

double DecelerateVelocity(double distance, double prev_velocity)
{

    double decel_ms = 1.0; // m/s
    double decel_velocity_ms = sqrt(2 * decel_ms * distance);

    std::cout << "velocity/prev_velocity :" << decel_velocity_ms << "/" << prev_velocity << std::endl;
    if (decel_velocity_ms < prev_velocity) {
        return decel_velocity_ms;
    } else {
        return prev_velocity;
    }

}
