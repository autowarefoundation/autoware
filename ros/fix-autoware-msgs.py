#!/usr/bin/env python

import os
import collections
import re

# A list to define the general association between categories, nodes, and the new namespace
# according to the proposal in https://github.com/CPFL/Autoware/issues/1428
# Please run `generate_initial_dictionary.sh` in case new categories/namespaces are added in the src directory,
# and format the output in accordance to the following data.
AutowareMsgs = collections.namedtuple('AutowareMsgs', 'pattern, message, node')
replacement_list = [
    AutowareMsgs("actuation", "autoware_actuation_msgs", "ymc"),
    AutowareMsgs("detection", "autoware_detection_msgs", "range_vision_fusion"),
    AutowareMsgs("detection", "autoware_detection_msgs", "libs_dpm_ttic"),
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
    AutowareMsgs("common", "autoware_common_msgs", "openplanner_op_planner"),
    AutowareMsgs("common", "autoware_common_msgs", "openplanner_op_ros_helpers"),
    AutowareMsgs("common", "autoware_common_msgs", "openplanner_op_utility"),
    AutowareMsgs("decision", "autoware_decision_msgs", "decision_maker"),
    AutowareMsgs("mission", "autoware_mission_msgs", "freespace_planner"),
    AutowareMsgs("mission", "autoware_mission_msgs", "lane_planner"),
    AutowareMsgs("mission", "autoware_mission_msgs", "op_global_planner"),
    AutowareMsgs("mission", "autoware_mission_msgs", "way_planner"),
    AutowareMsgs("motion", "autoware_motion_msgs", "astar_planner"),
    AutowareMsgs("motion", "autoware_motion_msgs", "dp_planner"),
    AutowareMsgs("motion", "autoware_motion_msgs", "ff_waypoint_follower"),
    AutowareMsgs("motion", "autoware_motion_msgs", "lattice_planner"),
    AutowareMsgs("motion", "autoware_motion_msgs", "op_local_planner"),
    AutowareMsgs("motion", "autoware_motion_msgs", "op_simulation_package"),
    AutowareMsgs("motion", "autoware_motion_msgs", "op_utilities"),
    AutowareMsgs("motion", "autoware_motion_msgs", "waypoint_follower"),
    AutowareMsgs("motion", "autoware_motion_msgs", "waypoint_maker"),
    AutowareMsgs("map", "autoware_map_msgs", "map_file"),
    AutowareMsgs("map", "autoware_map_msgs", "obj_db"),
    AutowareMsgs("map", "autoware_map_msgs", "pos_db"),
    AutowareMsgs("map", "autoware_map_msgs", "vector_map_server"),
    AutowareMsgs("driveworks", "autoware_driveworks_msgs", "autoware_driveworks_interface"),
    AutowareMsgs("sensing", "autoware_sensing_msgs", "points_downsampler"),
    AutowareMsgs("sensing", "autoware_sensing_msgs", "points_preprocessor"),
    AutowareMsgs("sensing", "autoware_sensing_msgs", "autoware_camera_lidar_calibrator"),
    AutowareMsgs("sensing", "autoware_sensing_msgs", "calibration_publisher"),
    AutowareMsgs("sensing", "autoware_sensing_msgs", "points2image"),
    AutowareMsgs("simulation", "autoware_simulation_msgs", "lgsvl_simulator_bridge"),
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
    'autoware_msgs/AccelCmd': 'autoware_motion_msgs',
    'autoware_msgs/AdjustXY': 'autoware_detection_msgs',
    'autoware_msgs/BrakeCmd': 'autoware_motion_msgs',
    'autoware_msgs/CameraExtrinsic': 'autoware_sensing_msgs',
    'autoware_msgs/Centroids': 'autoware_detection_msgs',  # 'autoware_system_msgs',
    'autoware_msgs/CloudCluster': 'autoware_detection_msgs',  # 'autoware_motion_msgs',
    'autoware_msgs/CloudClusterArray': 'autoware_detection_msgs',  # 'autoware_motion_msgs', 'autoware_decision_msgs',
    'autoware_msgs/ColorSet': 'autoware_unused_msgs',
    'autoware_msgs/ControlCommand': 'autoware_motion_msgs',
    'autoware_msgs/ControlCommandStamped': 'autoware_motion_msgs',
    'autoware_msgs/DTLane': 'autoware_mission_msgs',
    'autoware_msgs/DetectedObject': 'autoware_detection_msgs',  # 'autoware_semantics_msgs', 'autoware_motion_msgs', 'autoware_prediction_msgs',
    'autoware_msgs/DetectedObjectArray': 'autoware_detection_msgs',  # 'autoware_motion_msgs', 'autoware_semantics_msgs', 'autoware_util_msgs', 'autoware_prediction_msgs',
    'autoware_msgs/ExtractedPosition': 'autoware_motion_msgs',  # 'autoware_detection_msgs'
    'autoware_msgs/GeometricRectangle': 'autoware_unused_msgs',
    'autoware_msgs/ICPStat': 'autoware_localization_msgs',
    'autoware_msgs/ImageLaneObjects': 'autoware_detection_msgs',
    'autoware_msgs/ImageObj': 'autoware_detection_msgs',  # 'autoware_system_msgs',
    'autoware_msgs/ImageObjRanged': 'autoware_detection_msgs',  # 'autoware_system_msgs',
    'autoware_msgs/ImageObjTracked': 'autoware_detection_msgs',  # 'autoware_system_msgs',
    'autoware_msgs/ImageObjects': 'autoware_unused_msgs',
    'autoware_msgs/ImageRect': 'autoware_detection_msgs',
    'autoware_msgs/ImageRectRanged': 'autoware_unused_msgs',
    'autoware_msgs/IndicatorCmd': 'autoware_util_msgs',  # <- wrong auto association
    'autoware_msgs/LampCmd': 'autoware_decision_msgs',  # 'autoware_motion_msgs',
    'autoware_msgs/Lane': 'autoware_mission_msgs',  # 'autoware_detection_msgs', 'autoware_map_msgs', 'autoware_decision_msgs', 
    'autoware_msgs/LaneArray': 'autoware_mission_msgs',  # 'autoware_motion_msgs', 'autoware_map_msgs', 'autoware_util_msgs', 'autoware_decision_msgs',
    'autoware_msgs/NDTStat': 'autoware_localization_msgs',  # 'autoware_socket_msgs',
    'autoware_msgs/ObjLabel': 'autoware_detection_msgs',  # 'autoware_system_msgs',  #  <-  wrong auto association
    'autoware_msgs/ObjPose': 'autoware_unused_msgs',
    'autoware_msgs/PointsImage': 'autoware_sensing_msgs',  # 'autoware_detection_msgs', 'autoware_system_msgs',
    'autoware_msgs/ProjectionMatrix': 'autoware_sensing_msgs',  # 'autoware_util_msgs',
    'autoware_msgs/RemoteCmd': 'autoware_socket_msgs',  # 'autoware_motion_msgs',
    'autoware_msgs/ScanImage': 'autoware_unused_msgs',
    'autoware_msgs/Signals': 'autoware_detection_msgs',  # 'autoware_motion_msgs',
    'autoware_msgs/State': 'autoware_decision_msgs',  # 'autoware_mission_msgs',
    'autoware_msgs/StateCmd': 'autoware_unused_msgs',
    'autoware_msgs/SteerCmd': 'autoware_motion_msgs',
    'autoware_msgs/SyncTimeDiff': 'autoware_sync_msgs',  # <- manually assigned to sync, was system
    'autoware_msgs/SyncTimeMonitor': 'autoware_sync_msgs',  # <- manually assigned to sync, was system
    'autoware_msgs/TrafficLight': 'autoware_detection_msgs',  # 'autoware_motion_msgs', 'autoware_decision_msgs', 'autoware_mission_msgs',
    'autoware_msgs/TrafficLightResult': 'autoware_detection_msgs',
    'autoware_msgs/TrafficLightResultArray': 'autoware_detection_msgs',
    'autoware_msgs/TunedResult': 'autoware_detection_msgs',
    'autoware_msgs/ValueSet': 'autoware_unused_msgs',
    'autoware_msgs/VehicleCmd': 'autoware_socket_msgs',  # 'autoware_actuation_msgs', 'autoware_motion_msgs', 
    'autoware_msgs/VehicleStatus': 'autoware_localization_msgs',
    'autoware_msgs/VscanTracked': 'autoware_unused_msgs',
    'autoware_msgs/VscanTrackedArray': 'autoware_unused_msgs',
    'autoware_msgs/Waypoint': 'autoware_motion_msgs',  # 'autoware_decision_msgs', 'autoware_mission_msgs',
    'autoware_msgs/WaypointState': 'autoware_decision_msgs'}


packagexml = "\
<?xml version=\"1.0\"?>\n\
<package>\n\
  <name>AUTOWARE_NEW_NAMESPACE</name>\n\
  <version>1.10.0</version>\n\
  <description>The AUTOWARE_NEW_NAMESPACE package</description>\n\
  <maintainer email=\"yusuke.fujii@tier4.jp\">Yusuke Fujii</maintainer>\n\
  <license>Apache 2</license>\n\
  <buildtool_depend>catkin</buildtool_depend>\n\
  <build_depend>message_generation</build_depend>\n\
  <build_depend>std_msgs</build_depend>\n\
  <build_depend>geometry_msgs</build_depend>\n\
  <build_depend>sensor_msgs</build_depend>\n\
  <!--<build_depend>jsk_recognition_msgs</build_depend>-->\n\
  <run_depend>message_runtime</run_depend>\n\
  <run_depend>std_msgs</run_depend>\n\
  <run_depend>geometry_msgs</run_depend>\n\
  <run_depend>sensor_msgs</run_depend>\n\
  <!--<run_depend>jsk_recognition_msgs</run_depend>-->\n\
  <export>\n\
  </export>\n\
</package>\n\
"


cmakeliststxt = "\n\
cmake_minimum_required(VERSION 2.8.3)\n\
project(autoware_msgs)\n\
\n\
find_package(catkin REQUIRED COMPONENTS\n\
        message_generation\n\
        std_msgs\n\
        geometry_msgs\n\
        sensor_msgs\n\
        #jsk_recognition_msgs\n\
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
)\n\
\n\
catkin_package(\n\
        CATKIN_DEPENDS\n\
        message_runtime\n\
        std_msgs\n\
        geometry_msgs\n\
        sensor_msgs\n\
        #jsk_recognition_msgs\n\
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
        if (not isBinary(fname)) and ('CHANGELOG.rst' not in fname) and ("src/msgs/autoware_msgs" not in fname):
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


def plainAutowareMsgsNames(actual_msgs=findActualAutowareMsgs()):
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
        for r in replist:
            if (r.pattern in k) and (r.node in k):
                if k in files2repl:
                    # is it a better match?
                    currnode = replist[files2repl[k]].node
                    if len(r.node) > len(currnode):  # longer matching string
                        files2repl[k] = index
                else:
                    files2repl.update({k: index})
            index = index + 1
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


def matchAllMsgs(movmsgs, actual_msgs=findActualAutowareMsgs()):
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


def findActualMsgsInAll(allmsgs, actual_msgs=findActualAutowareMsgs()):
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
    f = open(newfolder + "/CMAKELISTS.txt", 'w')
    f.write(cmakeliststxt)
    f.close()


def createNewMsgsStructure(movemsgsrev, manual_list=manual_association_list):
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
        # prepare package.xml and CMAKELISTS.txt files' contents
        msgsliststr = ""
        for t in m:
            msgfilename = t.replace("autoware_msgs/", "") + ".msg"
            msgsliststr = msgsliststr + "          " + msgfilename + "\n"
        packagexmlaux = packagexml.replace("AUTOWARE_NEW_NAMESPACE", n)
        cmakeliststxtaux = cmakeliststxt.replace("          MESSAGE_FILENAME.msg", msgsliststr)
        # one special case: "CloudCluster.msg" in "autoware_detection_msgs" needs jsk dependency
        if "autoware_detection_msgs" in n:
            packagexmlaux = packagexmlaux.replace("<!--<build_depend>jsk_recognition_msgs</build_depend>-->", "<build_depend>jsk_recognition_msgs</build_depend>")
            packagexmlaux = packagexmlaux.replace("<!--<run_depend>jsk_recognition_msgs</run_depend>-->", "<run_depend>jsk_recognition_msgs</run_depend>")
            cmakeliststxtaux = cmakeliststxtaux.replace("#jsk_recognition_msgs", "jsk_recognition_msgs")
        # create new namespace folder
        newfolder = srcmsgsprefix + n
        print("creating folder structure for {}".format(newfolder))
        createFolderStructure(newfolder, packagexmlaux, cmakeliststxtaux)
        # now, move the files
        for t in m:
            srcfilepath = srcmsgsprefix + t.replace("/", "/msg/") + ".msg"
            destfilepath = newfolder + "/msg/" + t.replace("autoware_msgs/", "") + ".msg"
            print("moving {} to {}".format(srcfilepath, destfilepath))
            try:
                os.rename(srcfilepath, destfilepath)
            except OSError:
                raise OSError('File error on move {} to {}'.format(srcfilepath, destfilepath))
    # finally, remove "src/msgs/autoware_msgs"
    try:
        autowaremsgs = srcmsgsprefix + "autoware_msgs"
        print("deleting folder{}".format(autowaremsgs))
        os.rmdir(autowaremsgs + "/msg")  # empty by now
        for root, _, files in os.walk(autowaremsgs):
            for f in files:
                os.remove(os.path.join(root, f))
        os.rmdir(autowaremsgs)
    except OSError:
        raise OSError  # in case of any error
    print("operation completed")


# def patternReplace(match, repl_pattern):
#     if "::" in match.group():
#         return repl_pattern[0]
#     elif "/" in match.group():
#         return repl_pattern[1]
#     elif "import" in match.group():
#         return repl_pattern[2]
#     else:
#         return ""


def replaceMsgsLine(line, movemsgs, query='autoware_msgs'):
    new_l = line
    for m, n in movemsgs.items():
        msgtype = re.split('/', m)[1]  # because all messages in movemsgs are of the form "autoware_msgs/SOMETYPE"
        replacement_dict = {"{}::{}".format(query, msgtype): "{}::{}".format(n, msgtype),
                            "{}/{}".format(query, msgtype): "{}/{}".format(n, msgtype),
                            "{}.msg import {}".format(query, msgtype): "{}.msg import {}".format(n, msgtype)}
        regex_split = '|'.join(map(re.escape, replacement_dict.keys()))
        p = re.compile(regex_split)
        new_l = p.sub(lambda matchobj: replacement_dict.get(matchobj.group(), matchobj.group()), new_l)
    return new_l


def genAutowareMsgsReplacement(lines, movemsgs, query='autoware_msgs'):
    replacement = {}
    for k, m in lines.items():
        for l in m:
            new_l = replaceMsgsLine(l, movemsgs, query)
            if k in replacement.keys():
                replacement[k].append((l, new_l))
            else:
                replacement.update({k: [(l, new_l)]})
    return replacement


def replaceAutowareMsgsInPlace(replacement, takebackup=True):
    modfiles = []
    for f, m in replacement.items():
        print("Processing file \"{}\"".format(f))
        modfiles.append(f)
        for l in m:
            replaceInFile(f, l[0], l[1], takebackup)
    return modfiles


# Use to revert all the files to their original state
def revertReplacements(files):
    for f in files:
        os.system("rm {}.backup 2>/dev/null".format(f))
        os.system("rm {} 2>/dev/null".format(f))
        os.system("git checkout {} 2>/dev/null".format(f))


# Find all
def findAutowareMsgsInPackage(pkgpath, movemsgsrev):
    newmsgs = movemsgsrev.keys()
    usedmsgs = []
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


def findAllAutowareMsgs(xmls, movemsgsrev):
    allmsgs = {}
    for f in xmls:
        usedmsgs = findAutowareMsgsInPackage(f, movemsgsrev)
        if usedmsgs:
            allmsgs.update({f: usedmsgs})
    return allmsgs


def fixPackageXMLAutowareMsgs(xml2msgs, takebackup=True):
    modfiles = []
    depends = "<depend>autoware_msgs</depend>"
    builddeps = "<build_depend>autoware_msgs</build_depend>"
    rundeps = "<run_depend>autoware_msgs</run_depend>"
    for f, m in xml2msgs.items():
        newdepends = "\n".join(["  <depend>{}</depend>".format(t) for t in m])
        newbuilddeps = "\n".join(["  <build_depend>{}</build_depend>".format(t) for t in m])  # not necessarily aligned with the rest of the xml
        newrundeps = "\n".join(["  <run_depend>{}</run_depend>".format(t) for t in m])
        querylist = [depends, builddeps, rundeps]
        repllist = [newdepends, newbuilddeps, newrundeps]
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
                if changed:
                    fd.seek(0)
                    fd.writelines(newdata)
        if changed:
            modfiles.append(f)
            changed = False
    return modfiles


def fixCMakeListsAutowareMsgs(xml2msgs, takebackup=True):
    modfiles = []
    multiline_querylist = ['find_package(autoware_msgs REQUIRED)', '${autoware_msgs_INCLUDE_DIRS}', 'autoware_msgs']
    p = re.compile(r"(?P<word>\w+)(?P<auto>\s+{})(?P<rest>\s*)".format("autoware_msgs"))
    for f, m in xml2msgs.items():
        multiline_replacements = "\n".join(["{}".format(t) for t in m])
        singleline_replacements = " " + " ".join(["{}".format(t) for t in m]) + " "
        # Performs replacement of single-line case
        with open(f, 'r+') as fd:
            contents = fd.read()
            lines = contents.splitlines()
            newlines = [p.sub(lambda matchobj: matchobj.group(1) + singleline_replacements + matchobj.group(3) if matchobj.group(3) else matchobj.group(1) + singleline_replacements, l) for l in lines]
            newdata = '\n'.join(newlines)
            if newdata != contents:
                fd.seek(0)
                fd.writelines(newdata)
                changed = True
        # Performs replacement for multiline cases
        changed = replaceInFile(f, multiline_querylist, multiline_replacements, False)
        if changed:
            modfiles.append(f)
    return modfiles


def fixPackageDefFiles(movmsgsrev, root="src/"):
    modfiles = []
    # Find all package.xml files
    xmls = findPackageXMLFiles(root)
    # Finds the list of new autoware msgs being used at each package
    xml2msgs = findAllAutowareMsgs(xmls, movmsgsrev)
    # Fixes the contents of package.xml accordingly
    modfiles = fixPackageXMLAutowareMsgs(xml2msgs, False)
    # The CMakeLists.txt should be in the same directory as the package.xml, no need to search
    # modfiles = modfiles + fixCMakeListsAutowareMsgs(xml2msgs, False)
    return modfiles


if __name__ == '__main__':
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
replacements = genAutowareMsgsReplacement(lines, movmsgs)
# Applies he replacements to actual files, returns the list of modified files
modfiles = replaceAutowareMsgsInPlace(replacements)
    # Fixes the contents of package.xml and CMakeLists.txt accordingly, returns the list of modified files
    modfiles = modfiles + fixPackageDefFiles(movmsgsrev, "src/")
    # Creates the new autoware messages structure, and deletes the old
    createNewMsgsStructure(movmsgsrev)
    #
    # Optionally, we can revert changes if needed
    # modfiles = list(set(modfiles))  # remove duplicated entries
    # revertReplacements(modfiles)
    #
