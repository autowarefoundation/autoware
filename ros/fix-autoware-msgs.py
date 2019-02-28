#!/usr/bin/env python


# Copyright 2018-2019 Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# v1.0 Alexander Carballo 2019/02/25
#

import os
import collections
import re
import argparse
import sys
import time

# A list to define the general association between categories, nodes, and the new namespace
# according to the proposal in https://github.com/CPFL/Autoware/issues/1428
# Please run `generate_initial_dictionary.sh` in case new categories/namespaces are added in the src directory,
# and format the output in accordance to the following data.
AutowareMsgs = collections.namedtuple('AutowareMsgs', 'pattern, message, node')
replacement_list = [
    AutowareMsgs("actuation", "autoware_actuation_msgs", "ymc"),
    AutowareMsgs("actuation", "autoware_actuation_msgs", "as"),
    AutowareMsgs("detection", "autoware_detection_msgs", "range_vision_fusion"),
    AutowareMsgs("detection", "autoware_detection_msgs", "libs_dpm_ttic"),
    AutowareMsgs("detection", "autoware_detection_msgs", "lidar_apollo_cnn_seg_detect"),
    AutowareMsgs("detection", "autoware_detection_msgs", "lidar_euclidean_cluster_detect"),
    AutowareMsgs("detection", "autoware_detection_msgs", "lidar_fake_perception"),
    AutowareMsgs("detection", "autoware_detection_msgs", "lidar_naive_l_shape_detect"),
    AutowareMsgs("detection", "autoware_detection_msgs", "lidar_imm_ukf_pda_track"),
    AutowareMsgs("detection", "autoware_detection_msgs", "lidar_kf_contour_track"),
    AutowareMsgs("detection", "autoware_detection_msgs", "lidar_shape_estimation"),
    AutowareMsgs("detection", "autoware_detection_msgs", "lidar_shape_estimation_test"),
    AutowareMsgs("detection", "autoware_detection_msgs", "roi_object_filter"),
    AutowareMsgs("detection", "autoware_detection_msgs", "trafficlight_recognizer"),
    AutowareMsgs("detection", "autoware_detection_msgs", "vision_darknet_detect"),
    AutowareMsgs("detection", "autoware_detection_msgs", "vision_dpm_ttic_detect"),
    AutowareMsgs("detection", "autoware_detection_msgs", "vision_lane_detect"),
    AutowareMsgs("detection", "autoware_detection_msgs", "vision_ssd_detect"),
    AutowareMsgs("detection", "autoware_detection_msgs", "vision_beyond_track"),
    AutowareMsgs("detection", "autoware_detection_msgs", "detected_objects_visualizer"),
    AutowareMsgs("detection", "autoware_detection_msgs", "integrated_viewer"),
    AutowareMsgs("prediction", "autoware_prediction_msgs", "naive_motion_predict"),
    AutowareMsgs("semantics", "autoware_semantics_msgs", "object_map"),
    AutowareMsgs("localization", "autoware_localization_msgs", "autoware_connector"),
    AutowareMsgs("localization", "autoware_localization_msgs", "lidar_localizer"),
    AutowareMsgs("common", "autoware_common_msgs", "amathutils"),
    AutowareMsgs("common", "autoware_common_msgs", "state_machine"),
    AutowareMsgs("common", "autoware_planning_msgs", "openplanner_op_planner"),
    AutowareMsgs("common", "autoware_planning_msgs", "openplanner_op_ros_helpers"),
    AutowareMsgs("common", "autoware_planning_msgs", "openplanner_op_utility"),
    AutowareMsgs("common", "autoware_planning_msgs", "op_ros_helpers"),
    AutowareMsgs("decision", "autoware_planning_msgs", "decision_maker"),
    AutowareMsgs("mission", "autoware_planning_msgs", "freespace_planner"),
    AutowareMsgs("mission", "autoware_planning_msgs", "lane_planner"),
    AutowareMsgs("mission", "autoware_planning_msgs", "op_global_planner"),
    AutowareMsgs("mission", "autoware_planning_msgs", "way_planner"),
    AutowareMsgs("motion", "autoware_planning_msgs", "astar_planner"),
    AutowareMsgs("motion", "autoware_planning_msgs", "dp_planner"),
    AutowareMsgs("motion", "autoware_planning_msgs", "ff_waypoint_follower"),
    AutowareMsgs("motion", "autoware_planning_msgs", "lattice_planner"),
    AutowareMsgs("motion", "autoware_planning_msgs", "op_local_planner"),
    AutowareMsgs("motion", "autoware_planning_msgs", "op_simulation_package"),
    AutowareMsgs("motion", "autoware_planning_msgs", "op_utilities"),
    AutowareMsgs("motion", "autoware_planning_msgs", "waypoint_follower"),
    AutowareMsgs("motion", "autoware_planning_msgs", "waypoint_maker"),
    AutowareMsgs("map", "autoware_map_msgs", "map_file"),
    AutowareMsgs("map", "autoware_map_msgs", "obj_db"),
    AutowareMsgs("map", "autoware_map_msgs", "vector_map_server"),
    AutowareMsgs("data", "autoware_map_msgs", "pos_db"),
    AutowareMsgs("msgs", "autoware_NO_msgs", "autoware_msgs"),
    AutowareMsgs("driveworks", "autoware_driveworks_msgs", "autoware_driveworks_interface"),
    AutowareMsgs("sensing", "autoware_sensing_msgs", "points_downsampler"),
    AutowareMsgs("sensing", "autoware_sensing_msgs", "points_preprocessor"),
    AutowareMsgs("sensing", "autoware_sensing_msgs", "autoware_camera_lidar_calibrator"),
    AutowareMsgs("sensing", "autoware_sensing_msgs", "calibration_publisher"),
    AutowareMsgs("sensing", "autoware_sensing_msgs", "points2image"),
    AutowareMsgs("simulation", "autoware_simulation_msgs", "lgsvl_simulator_bridge"),
    AutowareMsgs("simulation", "autoware_simulation_msgs", "gazebo_simulator"),
    AutowareMsgs("socket", "autoware_socket_msgs", "mqtt_socket"),
    AutowareMsgs("socket", "autoware_socket_msgs", "tablet_socket"),
    AutowareMsgs("socket", "autoware_socket_msgs", "vehicle_socket"),
    AutowareMsgs("sync", "autoware_sync_msgs", "sync"),  # was system, to avoid conflict with exisint autoware_system_msgs
    AutowareMsgs("util", "autoware_util_msgs", "kitti_pkg"),
    AutowareMsgs("util", "autoware_util_msgs", "runtime_manager")]


# We allow to have manual associations of message types to a new namespace using the following 1:1 dictionary
# Example {"autoware_msgs/Lane": "autoware_map_msgs"}
# run `printAllMsgsAssociations(allMsgsToPattern(lines, filespattern))` to generate the basic associations
# based on message publishing and subscribing, manually assign namespace `autoware_unused_msgs` to those
# messages without associations, make necessary correction to suggested namespaces, and make sure each
# message type is assigned to only ONE namespace.
manual_association_list = {
    'autoware_msgs/AccelCmd': 'autoware_planning_msgs',
    'autoware_msgs/AdjustXY': 'autoware_detection_msgs',
    'autoware_msgs/BrakeCmd': 'autoware_planning_msgs',
    'autoware_msgs/CameraExtrinsic': 'autoware_sensing_msgs',
    'autoware_msgs/Centroids': 'autoware_detection_msgs',  # 'autoware_system_msgs',
    'autoware_msgs/CloudCluster': 'autoware_detection_msgs',  # 'autoware_planning_msgs',
    'autoware_msgs/CloudClusterArray': 'autoware_detection_msgs',  # 'autoware_planning_msgs', 'autoware_planning_msgs',
    'autoware_msgs/ColorSet': 'autoware_detection_msgs',
    'autoware_msgs/ControlCommand': 'autoware_planning_msgs',
    'autoware_msgs/ControlCommandStamped': 'autoware_planning_msgs',
    'autoware_msgs/DTLane': 'autoware_planning_msgs',
    'autoware_msgs/DetectedObject': 'autoware_detection_msgs',  # 'autoware_semantics_msgs', 'autoware_planning_msgs', 'autoware_prediction_msgs',
    'autoware_msgs/DetectedObjectArray': 'autoware_detection_msgs',  # 'autoware_planning_msgs', 'autoware_semantics_msgs', 'autoware_util_msgs', 'autoware_prediction_msgs',
    'autoware_msgs/ExtractedPosition': 'autoware_planning_msgs',  # 'autoware_detection_msgs'
    'autoware_msgs/GeometricRectangle': 'autoware_unused_msgs',
    'autoware_msgs/ICPStat': 'autoware_localization_msgs',
    'autoware_msgs/ImageLaneObjects': 'autoware_detection_msgs',
    'autoware_msgs/ImageObj': 'autoware_detection_msgs',  # 'autoware_system_msgs',
    'autoware_msgs/ImageObjRanged': 'autoware_detection_msgs',  # 'autoware_system_msgs',
    'autoware_msgs/ImageObjTracked': 'autoware_detection_msgs',  # 'autoware_system_msgs',
    'autoware_msgs/ImageObjects': 'autoware_unused_msgs',
    'autoware_msgs/ImageRect': 'autoware_detection_msgs',
    'autoware_msgs/ImageRectRanged': 'autoware_detection_msgs',
    # 'autoware_msgs/IndicatorCmd': 'autoware_util_msgs',  # <- wrong auto association
    'autoware_msgs/LampCmd': 'autoware_planning_msgs',  # 'autoware_planning_msgs',
    'autoware_msgs/Lane': 'autoware_planning_msgs',  # 'autoware_detection_msgs', 'autoware_map_msgs', 'autoware_planning_msgs',
    'autoware_msgs/LaneArray': 'autoware_planning_msgs',  # 'autoware_planning_msgs', 'autoware_map_msgs', 'autoware_util_msgs', 'autoware_planning_msgs',
    'autoware_msgs/NDTStat': 'autoware_localization_msgs',  # 'autoware_socket_msgs',
    'autoware_msgs/ObjLabel': 'autoware_detection_msgs',  # 'autoware_system_msgs',  #  <-  wrong auto association
    'autoware_msgs/ObjPose': 'autoware_unused_msgs',
    'autoware_msgs/PointsImage': 'autoware_sensing_msgs',  # 'autoware_detection_msgs', 'autoware_system_msgs',
    'autoware_msgs/ProjectionMatrix': 'autoware_sensing_msgs',  # 'autoware_util_msgs',
    'autoware_msgs/RemoteCmd': 'autoware_socket_msgs',  # 'autoware_planning_msgs',
    'autoware_msgs/ScanImage': 'autoware_unused_msgs',
    'autoware_msgs/Signals': 'autoware_detection_msgs',  # 'autoware_planning_msgs',
    'autoware_msgs/State': 'autoware_planning_msgs',  # 'autoware_planning_msgs',
    'autoware_msgs/StateCmd': 'autoware_unused_msgs',
    'autoware_msgs/SteerCmd': 'autoware_planning_msgs',
    'autoware_msgs/SyncTimeDiff': 'autoware_sync_msgs',  # <- manually assigned to sync, was system
    'autoware_msgs/SyncTimeMonitor': 'autoware_sync_msgs',  # <- manually assigned to sync, was system
    'autoware_msgs/TrafficLight': 'autoware_detection_msgs',  # 'autoware_planning_msgs', 'autoware_planning_msgs', 'autoware_planning_msgs',
    'autoware_msgs/TrafficLightResult': 'autoware_detection_msgs',
    'autoware_msgs/TrafficLightResultArray': 'autoware_detection_msgs',
    'autoware_msgs/TunedResult': 'autoware_detection_msgs',
    'autoware_msgs/ValueSet': 'autoware_detection_msgs',
    'autoware_msgs/VehicleCmd': 'autoware_planning_msgs',  # 'autoware_actuation_msgs', 'autoware_socket_msgs',
    'autoware_msgs/VehicleStatus': 'autoware_localization_msgs',
    'autoware_msgs/VscanTracked': 'autoware_unused_msgs',
    'autoware_msgs/VscanTrackedArray': 'autoware_unused_msgs',
    'autoware_msgs/Waypoint': 'autoware_planning_msgs',  # 'autoware_planning_msgs', 'autoware_planning_msgs',
    'autoware_msgs/WaypointState': 'autoware_planning_msgs'}


actualAutowareMsgs = ['autoware_msgs/AccelCmd', 'autoware_msgs/AdjustXY', 'autoware_msgs/BrakeCmd',
                      'autoware_msgs/CameraExtrinsic', 'autoware_msgs/Centroids', 'autoware_msgs/CloudCluster',
                      'autoware_msgs/CloudClusterArray', 'autoware_msgs/ColorSet', 'autoware_msgs/ControlCommand',
                      'autoware_msgs/ControlCommandStamped', 'autoware_msgs/DTLane', 'autoware_msgs/DetectedObject',
                      'autoware_msgs/DetectedObjectArray', 'autoware_msgs/ExtractedPosition', 'autoware_msgs/GeometricRectangle',
                      'autoware_msgs/ICPStat', 'autoware_msgs/ImageLaneObjects', 'autoware_msgs/ImageObj',
                      'autoware_msgs/ImageObjRanged', 'autoware_msgs/ImageObjTracked', 'autoware_msgs/ImageObjects',
                      'autoware_msgs/ImageRect', 'autoware_msgs/ImageRectRanged',  # 'autoware_msgs/IndicatorCmd',
                      'autoware_msgs/LampCmd', 'autoware_msgs/Lane', 'autoware_msgs/LaneArray',
                      'autoware_msgs/NDTStat', 'autoware_msgs/ObjLabel', 'autoware_msgs/ObjPose',
                      'autoware_msgs/PointsImage', 'autoware_msgs/ProjectionMatrix', 'autoware_msgs/RemoteCmd',
                      'autoware_msgs/ScanImage', 'autoware_msgs/Signals', 'autoware_msgs/State',
                      'autoware_msgs/StateCmd', 'autoware_msgs/SteerCmd', 'autoware_msgs/SyncTimeDiff',
                      'autoware_msgs/SyncTimeMonitor', 'autoware_msgs/TrafficLight', 'autoware_msgs/TrafficLightResult',
                      'autoware_msgs/TrafficLightResultArray', 'autoware_msgs/TunedResult', 'autoware_msgs/ValueSet',
                      'autoware_msgs/VehicleCmd', 'autoware_msgs/VehicleStatus', 'autoware_msgs/VscanTracked',
                      'autoware_msgs/VscanTrackedArray', 'autoware_msgs/Waypoint', 'autoware_msgs/WaypointState']


packagexml = "\
<?xml version=\"1.0\"?>\n\
<package>\n\
  <name>AUTOWARE_NEW_NAMESPACE</name>\n\
  <version>1.10.0</version>\n\
  <description>The AUTOWARE_NEW_NAMESPACE package</description>\n\
  <maintainer email=\"yusuke.fujii@tier4.jp\">Yusuke Fujii</maintainer>\n\
  <maintainer email=\"alexander@g.sp.m.is.nagoya-u.ac.jp\">Alex</maintainer>\n\
  <license>Apache 2</license>\n\
  <buildtool_depend>catkin</buildtool_depend>\n\
  <build_depend>message_generation</build_depend>\n\
  <build_depend>std_msgs</build_depend>\n\
  <build_depend>geometry_msgs</build_depend>\n\
  <build_depend>sensor_msgs</build_depend>\n\
  <!--<build_depend>jsk_recognition_msgs</build_depend>-->\n\
  <!--<build_depend>autoware_msgs</build_depend>-->\n\
  <run_depend>message_runtime</run_depend>\n\
  <run_depend>std_msgs</run_depend>\n\
  <run_depend>geometry_msgs</run_depend>\n\
  <run_depend>sensor_msgs</run_depend>\n\
  <!--<run_depend>jsk_recognition_msgs</run_depend>-->\n\
  <!--<run_depend>autoware_msgs</run_depend>-->\n\
  <export>\n\
  </export>\n\
</package>\n\
"


cmakeliststxt = "\n\
cmake_minimum_required(VERSION 2.8.3)\n\
project(AUTOWARE_NEW_NAMESPACE)\n\
\n\
find_package(catkin REQUIRED COMPONENTS\n\
        message_generation\n\
        std_msgs\n\
        geometry_msgs\n\
        sensor_msgs\n\
        #jsk_recognition_msgs\n\
        #autoware_msgs\n\
        )\n\
\n\
\n\
## Generate messages in the 'msg' folder\n\
add_message_files(\n\
        DIRECTORY msg\n\
        FILES\n\
          MESSAGE_FILENAME.msg\n\
)\n\
\n\
## Generate added messages and services with any dependencies listed here\n\
generate_messages(\n\
        DEPENDENCIES\n\
        std_msgs\n\
        geometry_msgs\n\
        sensor_msgs\n\
        #jsk_recognition_msgs\n\
        #autoware_msgs\n\
)\n\
\n\
catkin_package(\n\
        CATKIN_DEPENDS\n\
        message_runtime\n\
        std_msgs\n\
        geometry_msgs\n\
        sensor_msgs\n\
        #jsk_recognition_msgs\n\
        #autoware_msgs\n\
)\n\
"

cmakelist_autoware_msgs = "\n\
find_package(autoware_msgs REQUIRED)\n\
find_package(\n\
  catkin REQUIRED COMPONENTS\n\
  autoware_msgs\n\
)\n\
catkin_package(\n\
  CATKIN_DEPENDS\n\
  autoware_msgs\n\
)\n\
\n\
include_directories(\n\
  ${autoware_msgs_INCLUDE_DIRS}\n\
)\n\
"

def flattenList(somelist):
    ll = []
    for l in somelist:
        if type(l) is list:
            for sl in l:
                if type(sl) is list:
                    raise ValueError("too many nested levels")
                else:
                    ll.append(sl)
        else:
            ll.append(l)
    return ll


def getAllFiles(root='.'):
    files = [os.path.join(root, f) for f in os.listdir(root) if os.path.isfile(os.path.join(root, f))]
    dirs = [d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d))]
    for d in dirs:
        files_in_d = getAllFiles(os.path.join(root, d))
        if files_in_d:
            for f in files_in_d:
                files.append(f)
    return files


def isBinary(filename):
    # based on https://stackoverflow.com/questions/32184809/python-file1-why-are-the-numbers-7-8-9-10-12-13-27-and-range0x20-0x100
    textchars = bytearray({7,8,9,10,12,13,27} | set(range(0x20, 0x100)) - {0x7f})
    is_binary_string = lambda bytes: bool(bytes.translate(None, textchars))
    return is_binary_string(open(filename, 'rb').read(1024))


def findFiles(path='src', query='autoware_msgs'):
    allfiles = getAllFiles(path)
    lines = {}
    for fname in allfiles:
        # Because binary "src/driveworks/packages/autoware_driveworks_interface/lib/libautoware_driveworks.so" is part of the source code
        #if (not isBinary(fname)) and ('CHANGELOG.rst' not in fname) and ("src/msgs/autoware_msgs" not in fname):
        if (not isBinary(fname)) and ('CHANGELOG.rst' not in fname):
            with open(fname) as f:
                for line in f:
                    line = line.strip().rstrip("\n\r")
                    if query in line:
                        if fname in lines:
                            lines[fname].append(line)
                        else:
                            lines.update({fname: [line]})
    return lines


def findActualAutowareMsgs(path='src/msgs/autoware_msgs/msg/'):
    allfiles = getAllFiles(path)
    messages = []
    for f in allfiles:
        messages.append("autoware_msgs/" + os.path.basename(os.path.splitext(f)[0]))
    return messages


def plainAutowareMsgsNames(actual_msgs=actualAutowareMsgs):
    return [f.replace("autoware_msgs/", "") for f in actual_msgs]


def replaceInFile(filepath, query='autoware_msgs', replacement='autoware_msgs', takebackup=True):
    # read file contents
    changed = False
    try:
        f = open(filepath, 'r')
        contents = f.read()  # This works if the file is not several GBs in size (TODO: use mmap instead)
        f.close()
        if takebackup:
            # write backup
            f = open(filepath + '.backup', 'w')
            f.write(contents)
            f.close()
        # replace
        if type(query) is str and type(replacement) is str:  # string to string is the most common
            newdata = contents.replace(query, replacement)
        elif type(query) is list and type(replacement) is list:  # list to list
            if len(query) == len(replacement):
                newdata = contents
                for i in range(len(query)):
                    newdata = newdata.replace(query[i], replacement[i])
            else:
                raise ValueError("Query and Replacement lists must have same length")
        elif type(query) is list and type(replacement) is str:  # list to single string
            newdata = contents
            for i in range(len(query)):
                newdata = newdata.replace(query[i], replacement)
        else:  # single string to list is not possible, no idea what is the separator to use
            raise ValueError("Query {} and Replacement {} types combination not supported".format(type(query), type(replacement)))
        # write new contents
        f = open(filepath, 'w')
        f.write(newdata)
        f.close()
        if newdata != contents:
            changed = True
    except OSError:
        raise OSError
    return changed


def autowareMsgFromLine(line, query='autoware_msgs'):
    msglist = []
    delim = ",", " ", "(", ")", "&", "\"", "'"
    regexPattern = '|'.join(map(re.escape, delim))
    words = re.split(regexPattern, line)
    for w in words:
        if query in w:
            msglist.append(w)
    return msglist


def splitAutowareMsgsType(line, query='autoware_msgs'):
    msgstype = []
    delim = ",", " ", "(", ")", "&", "\"", "'", "Ptr", "ConstPtr", "include", "import", "from", "<", ">", "|", "{", "}", "*", "`", ";//", "[", "]"
    subsplit = "::", "/", ".h", ".hpp", ".msg", ".cpp", ".c"
    regexPattern = '|'.join(map(re.escape, delim))
    regexPatternSub = '|'.join(map(re.escape, subsplit))
    words = re.split(regexPattern, line)
    innexttoken = False
    for w in words:
        w = re.sub("(?:\.hpp|\.h|\:\:$|;$|:$)", "", w)
        w = re.sub("::", "/", w) # However, cases like "autoware_msgs::WaypointState::TYPE_STOPLINE" will have 2 "/"
        w = "/".join(w.split("/", 2)[:2])  # therefore, remove the second "/" and on
        if len(w) and (query + ".msg") in w:  # special case for python "from autoware_msgs.msg import xxxx"
            innexttoken = True
        if len(w) and (((query in w) and (not innexttoken)) or (innexttoken and ((query + ".msg") not in w))):
            if (innexttoken and (query not in w)):
                w = query + "/" + w
                innexttoken = False
            subwords = re.split(regexPatternSub, w)
            for sw in subwords:
                if len(sw) and query not in sw:  # we don't want 'autoware_msgs' but the type
                    msgstype.append({w: sw})
    msgstype = flattenList(msgstype)
    return msgstype


def fileMessages(lines, query='autoware_msgs'):
    fileMsgs = {}
    for k, v in lines.items():
        for l in v:
            msgs = splitAutowareMsgsType(l, query)
            for m in msgs:
                if k in fileMsgs:
                    fileMsgs[k].append(m)
                else:
                    fileMsgs.update({k: [m]})
    return fileMsgs


def files2Pattern(lines, replist=replacement_list):
    files2repl = {}
    for k, v in lines.items():
        index = 0
        found = False
        for r in replist:
            if (r.pattern in k) and (r.node in k):
                if k in files2repl:
                    # is it a better match?
                    currnode = replist[files2repl[k]].node
                    if len(r.node) > len(currnode):  # longer matching string
                        files2repl[k] = index
                        found = True
                else:
                    files2repl.update({k: index})
                    found = True
            index = index + 1
        if not found:
            print("{} not found in replacement_list".format(k))
    return files2repl


def messagesPublishersToPattern(lines, filespattern, replist=replacement_list):
    pubmsgs = {}
    for f, i in filespattern.items():
        msgstypes = []
        for l in lines[f]:
            if re.search("publish", l, re.IGNORECASE):
                msgstypes.append(splitAutowareMsgsType(l))
            if re.search("advertise", l, re.IGNORECASE):
                msgstypes.append(splitAutowareMsgsType(l))
        msgstypes = flattenList(msgstypes)
        for m in msgstypes:
            if len(m) > 1:  # if multiple matches per line
                raise ValueError("too many nested levels in {}".format(m))
            else:
                if replist[i].message in pubmsgs:
                    pubmsgs[replist[i].message].append(m)
                else:
                    pubmsgs.update({replist[i].message: [m]})
    autowaremsgs = {}
    for k, v in pubmsgs.items():
        v = flattenList(v)
        autowaremsgs.update({k: v})
    return autowaremsgs


def allMsgsToPattern(lines, filespattern, replist=replacement_list):
    allmsgs = {}
    for f, i in filespattern.items():
        msgstypes = []
        for l in lines[f]:
            msgstypes.append(splitAutowareMsgsType(l))
        msgstypes = flattenList(msgstypes)
        for m in msgstypes:
            if len(m) > 1:  # if multiple matches per line
                raise ValueError("too many nested levels in {}".format(m))
            else:
                if replist[i].message in allmsgs:
                    allmsgs[replist[i].message].append(m)
                else:
                    allmsgs.update({replist[i].message: [m]})
    autowaremsgs = {}
    for k, v in allmsgs.items():
        v = flattenList(v)
        autowaremsgs.update({k: v})
    return autowaremsgs


def publishedMsgsConversion(msgspub):
    conversion = {}
    for k, v in msgspub.items():
        vals = []
        for l in v:
            vals.append(l.keys()[0])
        values = list(set(vals))  # As a set we remove repeated values
        conversion.update({k: values})
    return conversion


def uniqueMsgsTypes(replist=replacement_list):
    uniqueMsgs = []
    for t in replist:
        uniqueMsgs.append(t.message)
    uniqueMsgs = list(set(uniqueMsgs))
    return uniqueMsgs


def msgsTypesInUse(filespattern, replist=replacement_list):
    usedmsgs = []
    for t in filespattern.items():
        usedmsgs.append(replist[t[1]].message)
    usedmsgs = list(set(usedmsgs))
    return usedmsgs


def moveMsgs(pubmsgs, allmsgs, manual_list=manual_association_list):
    pubconv = publishedMsgsConversion(pubmsgs)
    allconv = publishedMsgsConversion(allmsgs)
    movemsgs = []
    for ak, al in allconv.items():
        for m in al:
            stop = 0
            # search m in published types
            for pk, pl in pubconv.items():
                for v in pl:
                    if m == v:
                        movemsgs.append((m, pk))
                        stop = 1
                        break
                if stop:
                    break
            if not stop:
                movemsgs.append((m, ak))
    # aux = dict(movemsgs)
    aux = movemsgs
    outdict = {}
    for k, v in aux:
        if k in manual_list.keys():
            outdict.update({k: manual_list[k]})  # preserves manual assigments
        # else:  # is not on the manual list, use guessed association
        #    outdict.update({k: v})
    return outdict


def moveMsgsReverse(pubmsgs, allmsgs):
    movemsgs = moveMsgs(pubmsgs, allmsgs)
    outdict = {}
    for k, v in movemsgs.items():
        if v in outdict.keys():
            outdict[v].append(k)
        else:
            outdict.update({v: [k]})
    return outdict


def matchAllMsgs(movmsgs, actual_msgs=actualAutowareMsgs):
    existing = []
    missing = []
    errors = []  # Messages type referred but files does not exist
    for m in actual_msgs:
        if m in movmsgs.keys():
            existing.append(m)
        else:
            missing.append(m)
    for k in movmsgs.keys():
        if k not in existing and k not in missing:
            errors.append(k)
    return existing, missing, errors


def printMoveMsgs(msgs):
    for k in sorted(msgs.keys()):
        v = msgs[k]
        if type(v) is list:
            for m in v:
                if type(m) is dict:
                    print("* move `{}` to `{}`".format(m.keys()[0], k))
                else:  
                    print("* move `{}` to `{}`".format(m, k))
        else:
            print("* move `{}` to `{}`".format(k, v))


def printMoveMsgsTable(msgs):
    print("| Current message               | New namespace            |")
    print("| ----------------------------- | ------------------------ |")
    for k in sorted(msgs.keys()):
        v = msgs[k]
        if type(v) is list:
            for m in v:
                if type(m) is dict:
                    print("|`{}`|`{}`|".format(m.keys()[0], k))
                else:
                    print("|`{}`|`{}`|".format(m, k))
        else:
            print("|`{}`|`{}`|".format(k, v))


def printMoveMsgsRev(msgsrev):
    for k, v in sorted(msgsrev.items()):
        ptrstr = "`{}` -> [".format(k)
        for l in v:
            ptrstr = ptrstr + "`{}`,".format(l)
        ptrstr = ptrstr[:-1]  # get rid of last comma
        ptrstr = ptrstr + "]"
        print(ptrstr)


def printMoveMsgsRevTable(msgsrev):
    print("| New namespace                 | Associated messages      |")
    print("| ----------------------------- | ------------------------ |")
    for k, v in sorted(msgsrev.items()):
        ptrstr = "|`{}`|".format(k)
        for l in v:
            ptrstr = ptrstr + "`{}`,".format(l)
        ptrstr = ptrstr[:-1]  # get rid of last comma
        ptrstr = ptrstr + "|"
        print(ptrstr)


def printAllMsgsAssociations(allmsgs):
    for k, v in sorted(findActualMsgsInAll(allmsgs).items()):
        print("'{}': {},".format(k, v))


def findMsgsNotInUse(lines, movmsgs):
    existing, missing, errors = matchAllMsgs(movmsgs)
    missing = plainAutowareMsgsNames(missing)
    inuse = []
    for f, m in lines.items():
        for l in m:
            [inuse.append(x) for x in missing if x in l]
    notinuse = set(missing) - set(inuse)
    return list(notinuse)


def findActualMsgsInAll(allmsgs, actual_msgs=actualAutowareMsgs):
    outdict = {}
    for a in actual_msgs:
        types = set()
        for k, v in allmsgs.items():
            for l in v:
                if a in l:
                    types.add(k)
        aux = list(types)
        outdict.update({a: aux})
    return outdict


def createFolderStructure(newfolder, packagexml, cmakeliststxt):
    if not os.path.exists(newfolder):
        try:
            os.makedirs(newfolder)
        except OSError:
            raise OSError('Error creating folder {}'.format(newfolder))
    # create the msg folder
    msgfolder = newfolder + "/msg"
    if not os.path.exists(msgfolder):
        try:
            os.makedirs(msgfolder)
        except OSError:
            raise OSError('Error creating folder {}'.format(msgfolder))
    # create package.xml
    f = open(newfolder + "/package.xml", 'w')
    f.write(packagexml)
    f.close()
    # create CMAKELISTS.txt
    f = open(newfolder + "/CMakeLists.txt", 'w')
    f.write(cmakeliststxt)
    f.close()


def createNewMsgsStructure(movemsgsrev, singlenamespace='', manual_list=manual_association_list):
    # We need to add to movemsgsrev the unused messages
    movemsgsrev["autoware_unused_msgs"] = {}
    movemsgsrev.pop("autoware_unused_msgs", "")
    for k, v in manual_list.items():
        if "autoware_unused_msgs" == v:
            if v in movemsgsrev.keys():
                movemsgsrev[v].append(k)
            else:
                movemsgsrev.update({v: [k]})
    srcmsgsprefix = "src/msgs/"
    for n, m in movemsgsrev.items():
        # if single namespace was requested, make sure it matches the assigned namespace n
        if singlenamespace != '' and singlenamespace != n:
            continue
        # prepare package.xml and CMakeLists.txt files' contents
        msgsliststr = ""
        for t in m:
            msgfilename = t.replace("autoware_msgs/", "") + ".msg"
            msgsliststr = msgsliststr + "          " + msgfilename + "\n"
        packagexmlaux = packagexml.replace("AUTOWARE_NEW_NAMESPACE", n)
        cmakeliststxtaux = cmakeliststxt.replace("          MESSAGE_FILENAME.msg", msgsliststr)
        cmakeliststxtaux = cmakeliststxtaux.replace("AUTOWARE_NEW_NAMESPACE", n)
        # handle message interdependency cases statically
        if "autoware_detection_msgs" in n:
            packagexmlaux = packagexmlaux.replace("<!--<build_depend>autoware_msgs</build_depend>-->", "<build_depend>autoware_planning_msgs</build_depend>")
            packagexmlaux = packagexmlaux.replace("<!--<run_depend>autoware_msgs</run_depend>-->", "<run_depend>autoware_planning_msgs</run_depend>")
            cmakeliststxtaux = cmakeliststxtaux.replace("#autoware_msgs", "autoware_planning_msgs")
        elif "autoware_socket_msgs" in n:
            packagexmlaux = packagexmlaux.replace("<!--<build_depend>autoware_msgs</build_depend>-->", "<build_depend>autoware_planning_msgs</build_depend>")
            packagexmlaux = packagexmlaux.replace("<!--<run_depend>autoware_msgs</run_depend>-->", "<run_depend>autoware_planning_msgs</run_depend>")
            cmakeliststxtaux = cmakeliststxtaux.replace("#autoware_msgs", "autoware_planning_msgs")
        else:  # remove unnecessary comments
            packagexmlaux = packagexmlaux.replace("<!--<build_depend>autoware_msgs</build_depend>-->", "")
            packagexmlaux = packagexmlaux.replace("<!--<run_depend>autoware_msgs</run_depend>-->", "")
            cmakeliststxtaux = cmakeliststxtaux.replace("#autoware_msgs", "")
        # special case: CloudCluster depends on jsk_recognition_msgs
        if "autoware_detection_msgs" in n:
            packagexmlaux = packagexmlaux.replace("<!--<build_depend>jsk_recognition_msgs</build_depend>-->", "<build_depend>jsk_recognition_msgs</build_depend>")
            packagexmlaux = packagexmlaux.replace("<!--<run_depend>jsk_recognition_msgs</run_depend>-->", "<run_depend>jsk_recognition_msgs</run_depend>")
            cmakeliststxtaux = cmakeliststxtaux.replace("#jsk_recognition_msgs", "jsk_recognition_msgs")
        else:  # otherwise, remove these lines
            packagexmlaux = packagexmlaux.replace("<!--<build_depend>jsk_recognition_msgs</build_depend>-->\n", "")
            packagexmlaux = packagexmlaux.replace("<!--<run_depend>jsk_recognition_msgs</run_depend>-->\n", "")
            cmakeliststxtaux = cmakeliststxtaux.replace("#jsk_recognition_msgs\n", "")
        # create new namespace folder
        newfolder = srcmsgsprefix + n
        print("creating folder structure for {}".format(newfolder))
        createFolderStructure(newfolder, packagexmlaux, cmakeliststxtaux)
        # now, move the files
        for t in m:
            srcfilepath = srcmsgsprefix + t.replace("/", "/msg/") + ".msg"
            destfilepath = newfolder + "/msg/" + t.replace("autoware_msgs/", "") + ".msg"
            print("moving {} to {}".format(srcfilepath, destfilepath))
            # This way to move the files was requested (keep tracking)
            os.system("git mv {} {}".format(srcfilepath, destfilepath))
            # try:
            #     os.rename(srcfilepath, destfilepath)
            # except OSError:
            #     raise OSError('File error on move {} to {}'.format(srcfilepath, destfilepath))
    # finally, remove "src/msgs/autoware_msgs" (if singlenamespace was not requested)
    autowaremsgs = srcmsgsprefix + "autoware_msgs"
    if singlenamespace == '':
        print("deleting folder \"{}\"".format(autowaremsgs))
        # This way to remove the files was requested (keep tracking)
        os.system("git rm -rf {}".format(autowaremsgs))
        # try:
        #     os.rmdir(autowaremsgs + "/msg")  # empty by now
        #     for root, _, files in os.walk(autowaremsgs):
        #         for f in files:
        #             os.remove(os.path.join(root, f))
        #     os.rmdir(autowaremsgs)
        # except OSError:
        #     raise OSError  # in case of any error
    else:
        # have to remove from `autoware_msgs` package the types we moved to the new message package.
        with open(autowaremsgs + "/package.xml", 'r+') as fd:
            contents = fd.read()
            for t in movemsgsrev[singlenamespace]:
                contents = contents.replace(t, "")
        with open(autowaremsgs + "/package.xml", 'w') as fd:
            fd.write(contents)
        with open(autowaremsgs + "/CMakeLists.txt", 'r+') as fd:
            contents = fd.read()
            for t in movemsgsrev[singlenamespace]:
                t = t.replace("autoware_msgs/", "") + ".msg"
                print t
                contents = contents.replace(t, "")
        with open(autowaremsgs + "/CMakeLists.txt", 'w') as fd:
            fd.write(contents)
    print("operation completed")


def replaceMsgsLine(line, movemsgs, query='autoware_msgs', singlenamespace=''):
    new_l = line
    # sorts by len (descending) and alpha to avoid replacements errors,
    sortedkeys = sorted(movemsgs.keys(), key=lambda item: (-len(item), item))
    # some messages are going to be renamed
    message_nenaming = {'IndicatorCmd': 'LampCmd'}
    # for m, n in movemsgs.items():
    for i in range(len(sortedkeys)):
        m = sortedkeys[i]
        n = movemsgs[m]
        # if single namespace was requested, make sure it matches the assigned namespace n
        if singlenamespace != '' and singlenamespace != n:
            continue
        msgtype = re.split('/', m)[1]  # because all messages in movemsgs are of the form "autoware_msgs/SOMETYPE"
        newmsgtype = msgtype
        if msgtype in message_nenaming.keys():
            newmsgtype = message_nenaming[msgtype]
        replacement_dict = {"{}::{}".format(query, msgtype): "{}::{}".format(n, newmsgtype),
                            "{}/{}".format(query, msgtype): "{}/{}".format(n, newmsgtype),
                            "{}.msg import {}".format(query, msgtype): "{}.msg import {}".format(n, newmsgtype)}
        regex_split = '|'.join(map(re.escape, replacement_dict.keys()))
        p = re.compile(regex_split)
        new_l = p.sub(lambda matchobj: replacement_dict.get(matchobj.group(), matchobj.group()), new_l)
    return new_l


def genAutowareMsgsReplacement(lines, movemsgs, singlenamespace='', query='autoware_msgs'):
    replacement = {}
    for k, m in lines.items():
        for l in m:
            new_l = replaceMsgsLine(l, movemsgs, query, singlenamespace)
            if singlenamespace != '' and singlenamespace not in new_l:
                continue
            if k in replacement.keys():
                replacement[k].append((l, new_l))
            else:
                replacement.update({k: [(l, new_l)]})
    return replacement


def replaceAutowareMsgsInPlace(replacement, takebackup=True):
    modfiles = []
    for f, m in replacement.items():
        modfiles.append(f)
        # sorts by len (descending) and alpha to avoid replacements errors,
        sortedm = sorted(m, key=lambda tup: (-len(tup[0]), tup[0]))
        querylist = [l[0] for l in sortedm]
        repllist = [l[1] for l in sortedm]
        replaceInFile(f, querylist, repllist, takebackup)
    return modfiles


def revertReplacements(modifiedfiles, singlenamespace=''):
    # restore the original autoware_msgs folder
    os.system("git checkout src/msgs/autoware_msgs 2>/dev/null")
    # We have to delete the src/msgs/autoware_*_msgs directories too
    if singlenamespace == '':
        os.system("rm -rf src/msgs/autoware_actuation_msgs 2>/dev/null")
        os.system("rm -rf src/msgs/autoware_planning_msgs 2>/dev/null")
        os.system("rm -rf src/msgs/autoware_detection_msgs 2>/dev/null")
        os.system("rm -rf src/msgs/autoware_localization_msgs 2>/dev/null")
        os.system("rm -rf src/msgs/autoware_planning_msgs 2>/dev/null")
        os.system("rm -rf src/msgs/autoware_planning_msgs 2>/dev/null")
        os.system("rm -rf src/msgs/autoware_planning_msgs 2>/dev/null")
        os.system("rm -rf src/msgs/autoware_sensing_msgs 2>/dev/null")
        os.system("rm -rf src/msgs/autoware_socket_msgs 2>/dev/null")
        os.system("rm -rf src/msgs/autoware_unused_msgs 2>/dev/null")
        # os.system("rm -rf src/msgs/autoware_util_msgs 2>/dev/null")
        os.system("rm -rf src/msgs/autoware_sync_msgs 2>/dev/null")
    else:
        os.system("rm -rf src/msgs/{} 2>/dev/null".format(singlenamespace))
    # and recover each file
    for f in modifiedfiles:
        os.system("rm {}.backup 2>/dev/null".format(f))
        os.system("rm {} 2>/dev/null".format(f))
        os.system("git checkout {} 2>/dev/null".format(f))


# Find all
def findAutowareMsgsInPackage(pkgpath, movemsgsrev, singlenamespace=''):
    newmsgs = movemsgsrev.keys()
    usedmsgs = []
    # if single namespace was requested, make sure it matches the assigned namespace
    if singlenamespace != '' and singlenamespace in newmsgs:
        newmsgs = [singlenamespace]
    elif singlenamespace != '' and singlenamespace not in newmsgs:
        return []
    if os.path.isfile(pkgpath):
        pkgpath = os.path.dirname(pkgpath)
    for root, _, files in os.walk(pkgpath):
        for f in files:
            # Because binary "src/driveworks/packages/autoware_driveworks_interface/lib/libautoware_driveworks.so" is part of the source code
            fname = os.path.join(root, f)
            if (not isBinary(fname)) and ('CHANGELOG.rst' not in fname) and ("src/msgs/autoware_msgs" not in fname):
                with open(fname) as fd:
                    for line in fd:
                        line = line.strip().rstrip("\n\r")
                        for m in newmsgs:
                            if m in line:
                                usedmsgs.append(m)
    return list(set(usedmsgs))


def findPackageXMLFiles(root="src/"):
    allfiles = getAllFiles(root)
    files = [f for f in allfiles if "package.xml" in f]
    return files


def findAllAutowareMsgs(xmls, movemsgsrev, singlenamespace=''):
    allmsgs = {}
    for f in xmls:
        usedmsgs = findAutowareMsgsInPackage(f, movemsgsrev, singlenamespace)
        if usedmsgs:
            allmsgs.update({f: usedmsgs})
            if os.path.isfile(f):
                with open(f, 'r') as fd:
                    contents = fd.read()
                    if "autoware_msgs" not in contents:
                        print("Package `{}` depends on `autoware_msgs` but file `{}` does not declare the dependency".format(os.path.dirname(f), os.path.basename(f)))
            else:
                print("Problem opening file `{}`".format(f))
        else:
            if os.path.isfile(f):
                with open(f, 'r') as fd:
                    contents = fd.read()
                    if "autoware_msgs" in contents:
                        print("File `{}` depends on `autoware_msgs` but the package does not use it".format(f))
            else:
                print("Problem opening file `{}`".format(f))
    return allmsgs


def fixPackageXMLAutowareMsgs(xml2msgs, takebackup=True):
    modfiles = []
    depends = "<depend>autoware_msgs</depend>"
    builddeps = "<build_depend>autoware_msgs</build_depend>"
    rundeps = "<run_depend>autoware_msgs</run_depend>"
    buildexpdeps = "<build_export_depend>autoware_msgs</build_export_depend>"
    execdeps = "<exec_depend>autoware_msgs</exec_depend>"
    for f, m in xml2msgs.items():
        newdepends = "\n".join(["  <depend>{}</depend>".format(t) for t in m])
        newbuilddeps = "\n".join(["  <build_depend>{}</build_depend>".format(t) for t in m])  # not necessarily aligned with the rest of the xml
        newrundeps = "\n".join(["  <run_depend>{}</run_depend>".format(t) for t in m])
        newbuildexpdeps = "\n".join(["  <build_export_depend>{}</build_export_depend>".format(t) for t in m])
        newexecdeps = "\n".join(["  <exec_depend>{}</exec_depend>".format(t) for t in m])
        querylist = [depends, builddeps, rundeps, buildexpdeps, execdeps]
        repllist = [newdepends, newbuilddeps, newrundeps, newbuildexpdeps, newexecdeps]
        changed = replaceInFile(f, querylist, repllist, False)
        if not changed:  # The query messages were not on the file, have to add them
            with open(f, 'r+') as fd:
                contents = fd.read()
                newdata = contents
                pos = contents.rfind('<depend>')
                if pos > 0:
                    fd.seek(pos)
                    line = fd.readline()
                    newdata = newdata.replace(line, line + newrundeps + "\n")
                    changed = True
                pos = contents.rfind('<build_depend>')
                if pos > 0:
                    fd.seek(pos)
                    line = fd.readline()
                    newdata = newdata.replace(line, line + newbuilddeps + "\n")
                    changed = True
                pos = contents.rfind('<run_depend>')
                if pos > 0:
                    fd.seek(pos)
                    line = fd.readline()
                    newdata = newdata.replace(line, line + newrundeps + "\n")
                    changed = True
                pos = contents.rfind('<build_export_depend>')
                if pos > 0:
                    fd.seek(pos)
                    line = fd.readline()
                    newdata = newdata.replace(line, line + newbuildexpdeps + "\n")
                    changed = True
                pos = contents.rfind('<exec_depend>')
                if pos > 0:
                    fd.seek(pos)
                    line = fd.readline()
                    newdata = newdata.replace(line, line + newexecdeps + "\n")
                    changed = True
                if changed:
                    fd.seek(0)
                    fd.writelines(newdata)
        if changed:
            modfiles.append(f)
            print("Fixed file \"{}\"".format(f))
            changed = False
    return modfiles


def fixCMakeListsAutowareMsgs(xml2msgs, takebackup=True):
    modfiles = []
    multiline_querylist = ['find_package(autoware_msgs REQUIRED)', '${autoware_msgs_INCLUDE_DIRS}', 'autoware_msgs']
    p = re.compile(r"(?P<word>\w+)(?P<auto>\s+{})(?P<rest>\s*)".format("autoware_msgs"))
    for f, m in xml2msgs.items():
        changed = False
        hasautowaremsgs = False
        fname = os.path.join(os.path.dirname(f), "CMakeLists.txt")
        if os.path.isfile(fname):  # normally, there should be a CMakeLists.txt in the same directory of package.xml
            multiline1 = "\n".join(["find_package({} REQUIRED)".format(t) for t in m])
            multiline2 = "\n".join(["${%s_INCLUDE_DIRS}" %(t) for t in m])
            multiline3 = "\n".join(["{}".format(t) for t in m])
            multiline_repllist = [multiline1, multiline2, multiline3]
            singleline_replacements = " " + " ".join(["{}".format(t) for t in m]) + " "
            # Performs replacement of single-line case
            # this is for example: "CATKIN_DEPENDS std_msgs velodyne_pointcloud *autoware_msgs* autoware_config_msgs" where
            # where autoware_msgs has to be replaced by autoware_namespace1_msgs autoware_namespace2_msgs ...
            # horizontally (single line) to keep the style 
            with open(fname, 'r+') as fd:
                contents = fd.read()
                if "autoware_msgs" in contents:
                    # make a backup if requested
                    if takebackup:
                        # write backup
                        fb = open(fname + '.backup', 'w')
                        fb.write(contents)
                        fb.close()
                    hasautowaremsgs = True
                    lines = contents.splitlines()
                    newlines = [p.sub(lambda matchobj:
                                      matchobj.group(1) + singleline_replacements + matchobj.group(3) if matchobj.group(3) 
                                      else matchobj.group(1) + singleline_replacements, l)
                                for l in lines]
                    newdata = '\n'.join(newlines)
                    if singleline_replacements in newdata:
                        fd.seek(0)
                        fd.writelines(newdata)
                        changed = True
                else:
                    # if autoware_msgs was not found but the package uses it, we have to add it
                    # ex. "src/computing/planning/motion/packages/dp_planner" does not declare the dependency
                    # and yet it includes autoware_msgs in the source code
                    #
                    # The trick is to copy a template in the file and then use multiline replacement below
                    # however, finding the right possition is difficult
                    # for the time being, we report the problem and proposed solution
                    # so user can make changes by hand
                    print("File \"{}\"".format(fname) + " does not include any dependency on autoware messages but the package refers to them.\n" +
                          "Please add the following to this file where appropiate:")
                    # aux = cmakelist_autoware_msgs.replace("autoware_msgs", multiline_replacements)
                    newdata = cmakelist_autoware_msgs
                    for i in range(len(multiline_querylist)):
                        newdata = newdata.replace(multiline_querylist[i], multiline_repllist[i])
                    print(newdata)
            # file is closed
            if hasautowaremsgs:
                # Performs replacement for multiline cases
                # this includes the following:
                # find_package(autoware_msgs REQUIRED)
                # find_package(
                #   catkin REQUIRED COMPONENTS
                #   ...
                #   autoware_msgs
                #   ...
                # )
                # catkin_package(
                #   CATKIN_DEPENDS
                #   ...
                #   autoware_msgs
                #   ...
                # )
                # ${autoware_msgs_INCLUDE_DIRS}
                #
                # where autoware_msgs will be replaced by:
                #   autoware_namespace1_msgs
                #   autoware_namespace2_msgs
                #   ...
                # as multiple lines
                changed = replaceInFile(fname, multiline_querylist, multiline_repllist, False)
            # done with changes (if any)
            if changed:
                modfiles.append(fname)
                print("Fixed file \"{}\"".format(fname))
    return modfiles


def fixPackageDefFiles(movmsgsrev, root="src/", takebackup=True, singlenamespace=''):
    modfiles = []
    # Find all package.xml files
    xmls = findPackageXMLFiles(root)
    # Finds the list of new autoware msgs being used at each package
    xml2msgs = findAllAutowareMsgs(xmls, movmsgsrev, singlenamespace)
    # Fixes the contents of package.xml accordingly
    modfiles = fixPackageXMLAutowareMsgs(xml2msgs, takebackup)
    # The CMakeLists.txt should be in the same directory as the package.xml, no need to search
    modfiles = modfiles + fixCMakeListsAutowareMsgs(xml2msgs, takebackup)
    return modfiles


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='autoware_msgs refactoring.')
    parser.add_argument('--backup', action='store_true',
                        help='Keep a backup of every affected file (default: False).')
    parser.add_argument('--fix', action='store_true',
                        help='Performs autoware_msgs refactoring (default: False).')
    parser.add_argument('--recovery', action='store_true',
                        help='Performs recovery of the original state (default: False).')
    parser.add_argument('--recoveryfile', type=str, default='.autoware_msgs_refactoring_recovery',
                        help='Recovery file to store list of affected files (default: .autoware_msgs_refactoring_recovery).')
    parser.add_argument('--namespace', type=str, default='',
                        help='Single namespace refactoring (default: empty-string for all namespaces refactoring')
    opt = parser.parse_args()
    if opt.fix:
        print("Doing autoware_msgs refactoring")
        # Finds all files and lines containing "autoware_msgs"
        lines = findFiles()
        # Associates file names with replacement list
        filespattern = files2Pattern(lines)
        # Find which autoware message are published
        pubmsgs = messagesPublishersToPattern(lines, filespattern)
        # Find all the autoware messages in use
        allmsgs = allMsgsToPattern(lines, filespattern)
        # Generate the message type to new namespace associations
        movmsgs = moveMsgs(pubmsgs, allmsgs)
        # Generate the inverse (new namespace to message types) associations
        movmsgsrev = moveMsgsReverse(pubmsgs, allmsgs)
        # For each file generate the namespace replacement
        replacements = genAutowareMsgsReplacement(lines, movmsgs, opt.namespace)
        # Applies he replacements to actual files, returns the list of modified files (no backups)
        modfiles = replaceAutowareMsgsInPlace(replacements, opt.backup)
        # Fixes the contents of package.xml and CMakeLists.txt accordingly, returns the list of modified files (no backups)
        modfiles = modfiles + fixPackageDefFiles(movmsgsrev, "src/", opt.backup, opt.namespace)
        # Creates the new autoware messages structure, and deletes the old
        createNewMsgsStructure(movmsgsrev, opt.namespace)
        # Save the recovery file
        modfiles = list(set(modfiles))  # remove duplicated entries
        with open(opt.recoveryfile, 'w') as fd:
            fd.write("\n".join(modfiles))
    elif opt.recovery:
        print("Recovering altered files")
        with open(opt.recoveryfile, 'r') as fd:
            modfiles = fd.read().splitlines()
        # Revert the changed files.
        # NOTE, it is actually quicker to run `git checkout src` and then delete the "autoware_xxx_msgs" created in "src/msgs/"
        revertReplacements(modfiles, opt.namespace)
        print("Done recovering. You can check with 'git status' whether any altered files remain")
    else:
        print("Either choose --fix or --recovery to run this program")
        parser.print_help(sys.stderr)
    print("Execution completed!!")
