#!/usr/bin/env python3

import sys
import os
import math
import copy
from munkres import Munkres
from collections import defaultdict
from time import gmtime, strftime
try:
    from ordereddict import OrderedDict # can be installed using pip
except:
    from collections import OrderedDict # only included from python 2.7 on
from tqdm import tqdm
from shutil import copyfile
import datetime

from rect import Rect
from parse_tracklet_xml import dump_frames_text_from_tracklets

class ObjectData:
    """
        Utility class to load object data.
    """

    def __init__(self,frame=-1,obj_type="unset",truncation=-1,occlusion=-1,\
                 obs_angle=-10,x1=-1,y1=-1,x2=-1,y2=-1,w=-1,h=-1,l=-1,\
                 cx=-1000,cy=-1000,cz=-1000,yaw=-10,score=-1000,track_id=-1):
        """
            Constructor, initializes the object given the parameters.
        """

        # init object data
        self.frame      = frame
        self.track_id   = track_id
        self.obj_type   = obj_type
        self.truncation = truncation
        self.occlusion  = occlusion
        self.obs_angle  = obs_angle
        self.x1         = x1
        self.y1         = y1
        self.x2         = x2
        self.y2         = y2
        self.w          = w
        self.h          = h
        self.l          = l
        self.cx          = cx
        self.cy          = cy
        self.cz          = cz
        self.yaw        = yaw
        self.score      = score
        self.ignored    = False
        self.valid      = False
        self.tracker    = -1

    def __str__(self):
        """
            Print read data.
        """

        attrs = vars(self)
        return '\n'.join("%s: %s" % item for item in attrs.items())


class TrackingEvaluation(object):
    """ tracking statistics (CLEAR MOT, id-switches, fragments, mostly_lost/partialy_tracked/mostly_tracked, precision/recall)
             id-switches - number of id switches
             fragments   - number of fragmentations

             mostly_tracked, partialy_tracked, mostly_lost	- number of mostly tracked, partially tracked and mostly lost trajectories

             recall	        - recall = percentage of detected targets
             precision	    - precision = percentage of correctly detected targets
             FAR		    - number of false alarms per frame
             falsepositives - number of false positives (FP)
             missed         - number of missed targets (FN)
    """

    def __init__(self, data_num, gt_path="../training", min_overlap=0.99, max_truncation = 0, min_height = 25, max_occlusion = 2):
        # data and parameter
        self.n_frames          = data_num
        self.result_data       = -1

        # statistics and numbers for evaluation
        self.n_gt              = 0 # number of ground truth detections minus ignored false negatives and true positives
        self.n_igt             = 0 # number of ignored ground truth detections
        self.n_gt_trajectories = 0
        self.n_tr              = 0 # number of tracker detections minus ignored tracker detections
        self.n_itr             = 0 # number of ignored tracker detections
        self.recall            = 0
        self.precision         = 0
        self.itp               = 0 # number of ignored true positives
        self.tp                = 0 # number of true positives including ignored true positives!
        self.fn                = 0 # number of false negatives WITHOUT ignored false negatives
        self.ifn               = 0 # number of ignored false negatives
        self.fp                = 0 # number of false positives
                                   # a bit tricky, the number of ignored false negatives and ignored true positives
                                   # is subtracted, but if both tracker detection and ground truth detection
                                   # are ignored this number is added again to avoid double counting
        self.mme               = 0
        self.fragments         = 0
        self.id_switches       = 0
        self.mostly_tracked    = 0
        self.partialy_tracked  = 0
        self.mostly_lost       = 0

        self.min_overlap       = min_overlap # minimum bounding box overlap for 3rd party metrics
        self.max_truncation    = max_truncation # maximum truncation of an object for evaluation
        self.max_occlusion     = max_occlusion # maximum occlusion of an object for evaluation
        self.min_height        = min_height # minimum height of an object for evaluation

    def load_tracked_data(self, result_file_path):
        """
            Helper function to load tracker data.
        """

        try:
            if(os.path.exists(result_file_path)):
                if not self._load_data(result_file_path, loading_groundtruth=False):
                    return False
            else:
                print("Error: There is no result data file")
                raise
        except IOError:
            return False
        return True

    def load_ground_truth(self, gt_file_path):
        """
            Helper function to load ground truth.
        """

        try:
            if(os.path.exists(gt_file_path)):
                self._load_data(gt_file_path, loading_groundtruth=True)
            else:
                print("Error: There is no groundtruth file")
                raise
        except IOError:
            return False
        return True

    def _load_data(self, file_path, min_score=-1000, loading_groundtruth=False):
        """
            Generic loader for ground truth and tracking data.
            Use load_ground_truth() or load_tracked_data() to load this data.
            Loads detections in KITTI format from textfiles.
        """
        # construct objectDetections object to hold detection data
        object_data  = ObjectData()
        n_trajectories     = 0
        file           = open(file_path, "r")
        f_data         = [[] for x in range(self.n_frames)] # current set has only 1059 entries, sufficient length is checked anyway
        ids            = []
        id_frame_cache = []

        for line in file:
            # KITTI tracking benchmark data format:
            # (frame,tracklet_id,objectType,truncation,occlusion,alpha,x1,y1,x2,y2,h,w,l,cx,cy,cz,yaw)
            line   = line.strip()
            fields = line.split(" ")

            # get fields from table
            object_data.frame        = int(float(fields[0]))     # frame
            object_data.track_id     = int(float(fields[1]))     # id
            object_data.obj_type     = fields[2].lower()         # object type [car, pedestrian, cyclist, ...]
            object_data.truncation   = int(float(fields[3]))     # truncation [-1,0,1,2]
            object_data.occlusion    = int(float(fields[4]))     # occlusion  [-1,0,1,2]
            object_data.obs_angle    = float(fields[5])          # observation angle [rad]
            object_data.x1           = float(fields[6])          # left   [px]
            object_data.y1           = float(fields[7])          # top    [px]
            object_data.x2           = float(fields[8])          # right  [px]
            object_data.y2           = float(fields[9])          # bottom [px]
            object_data.h            = float(fields[10])         # height [m]
            object_data.w            = float(fields[11])         # width  [m]
            object_data.l            = float(fields[12])         # length [m]
            object_data.cx           = float(fields[13])         # cx [m]
            object_data.cy           = float(fields[14])         # cy [m]
            object_data.cz           = float(fields[15])         # cz [m]
            object_data.yaw          = float(fields[16])         # yaw angle [rad]

            # do not consider objects marked as invalid
            # do not consider objects marked as invalid
            if object_data.track_id is -1 and object_data.obj_type != "DontCare":
                continue
            idx = object_data.frame

            if object_data.frame >= self.n_frames:
                continue

            try:
                id_frame = (object_data.frame,object_data.track_id)
                if id_frame in id_frame_cache and not loading_groundtruth:
                    print("track ids are not unique for frame %d" % (object_data.frame))
                    print("track id %d occured at least twice for this frame" % object_data.track_id)
                    print("Exiting...")
                    #continue # this allows to evaluate non-unique result files
                    return False
                id_frame_cache.append(id_frame)
                f_data[object_data.frame].append(copy.copy(object_data))
            except:
                raise

            if object_data.track_id not in ids and object_data.obj_type!="DontCare":
                ids.append(object_data.track_id)
                n_trajectories +=1
        file.close()


        if not loading_groundtruth:
            self.n_tr_trajectories=n_trajectories
            self.result_data = f_data
        else:
            # split ground truth and DontCare areas
            self.dcareas     = []
            self.groundtruth = []
            s_g, s_dc = [],[]
            for f in range(len(f_data)):
                frame_gts = f_data[f]
                g,dc = [],[]
                for gg in frame_gts:
                    if gg.obj_type=="dontcare":
                        dc.append(gg)
                    else:
                        g.append(gg)
                s_g.append(g)
                s_dc.append(dc)
            self.dcareas = s_dc
            self.groundtruth = s_g
            self.n_gt_trajectories=n_trajectories
        return True

    def create_eval_dir(self, benchmark_dir):
        """
            Creates directory to store evaluation results and data for visualization.
        """

        self.eval_dir = benchmark_dir
        if not os.path.exists(self.eval_dir):
            print ("create directory:", self.eval_dir)
            os.makedirs(self.eval_dir)
            print ("done")

    def compute_3rd_party_metrics(self):
        """
            Computes the metrics defined in
                - Stiefelhagen 2008: Evaluating Multiple Object Tracking Performance: The CLEAR MOT Metrics
                  MOTA, MOTAL, MOTP
                - Nevatia 2008: Global Data Association for Multi-Object Tracking Using Network Flows
                  mostly_tracked/partialy_tracked/mostly_lost
        """
        # construct Munkres object for Hungarian Method association
        hm = Munkres()
        max_cost = 1e9

        # go through all frames and associate ground truth and tracker results
        # groundtruth and tracker contain lists for every single frame containing lists of KITTI format detections
        seq_gt                = self.groundtruth
        seq_dc                = self.dcareas # don't care areas
        seq_result_data       = self.result_data
        seq_trajectories      = defaultdict(list)
        seq_ignored           = defaultdict(list)

        last_frame_ids = [[],[]]

        for i_frame in tqdm(range(len(seq_gt))):
            frame_gts      = seq_gt[i_frame]
            frame_dcs      = seq_dc[i_frame]

            frame_results = seq_result_data[i_frame]
            # counting total number of ground truth and tracker objects
            self.n_gt += len(frame_gts)
            self.n_tr += len(frame_results)

            # use hungarian method to associate, using boxoverlap 0..1 as cost
            # build cost matrix
            cost_matrix = []
            frame_ids = [[],[]]
            # loop over ground truth objects in one frame
            for gt in frame_gts:
                # save current ids
                frame_ids[0].append(gt.track_id)
                frame_ids[1].append(-1)
                gt.tracker       = -1
                gt.id_switch     = 0
                gt.fragmentation = 0
                cost_row         = []
                # loop over tracked objects in one frame
                for result in frame_results:
                    # overlap == 1 means cost == 0
                    # Rect(cx, cy, l, w, angle)
                    r1   = Rect(gt.cx, gt.cy, gt.l, gt.w, gt.yaw)
                    r2   = Rect(result.cx, result.cy, result.l, result.w, result.yaw)
                    iou  = r1.intersection_over_union(r2)
                    cost = 1 - iou
                    # gating for boxoverlap
                    if cost<=self.min_overlap:
                        cost_row.append(cost)
                    else:
                        cost_row.append(max_cost) # = 1e9
                # return
                cost_matrix.append(cost_row)
                # all ground truth trajectories are initially not associated
                # extend groundtruth trajectories lists (merge lists)
                seq_trajectories[gt.track_id].append(-1)
                seq_ignored[gt.track_id].append(False)

            if len(frame_gts) is 0:
                cost_matrix=[[]]
            # associate
            association_matrix = hm.compute(cost_matrix)

            # tmp variables for sanity checks
            tmptp = 0
            tmpfp = 0
            tmpfn = 0

            # mapping for tracker ids and ground truth ids
            for row, col in association_matrix:
                # apply gating on boxoverlap
                c = cost_matrix[row][col]
                if c < max_cost:
                    frame_gts[row].tracker                        = frame_results[col].track_id
                    frame_ids[1][row]                             = frame_results[col].track_id
                    frame_results[col].valid                      = True
                    frame_gts[row].distance                       = c
                    seq_trajectories[frame_gts[row].track_id][-1] = frame_results[col].track_id

                    # true positives are only valid associations
                    self.tp += 1
                    tmptp   += 1
                else:
                    # wrong data association
                    frame_gts[row].tracker = -1
                    self.fn               += 1
                    tmpfn                 += 1

            # associate tracker and DontCare areas
            # ignore tracker in neighboring classes
            nignoredtracker = 0 # number of ignored tracker detections
            ignoredtrackers = dict() # will associate the track_id with -1
                                     # if it is not ignored and 1 if it is
                                     # ignored;
                                     # this is used to avoid double counting ignored
                                     # cases, see the next loop

            # check for ignored FN/TP (truncation or neighboring object class)
            nignoredfn  = 0 # the number of ignored false negatives
            nignoredtp = 0 # the number of ignored true positives

            gi = 0
            for gt in frame_gts:
                if gt.tracker < 0:
                    if gt.occlusion>self.max_occlusion or gt.truncation>self.max_truncation:
                        seq_ignored[gt.track_id][-1] = True
                        gt.ignored = True
                        nignoredfn += 1

                elif gt.tracker>=0:
                    if gt.occlusion>self.max_occlusion or gt.truncation>self.max_truncation:
                        seq_ignored[gt.track_id][-1] = True
                        gt.ignored = True
                        nignoredtp += 1

                gi += 1

            # the below might be confusion, check the comments in __init__
            # to see what the individual statistics represent

            # correct TP by number of ignored TP due to truncation
            # ignored TP are shown as tracked in visualization
            tmptp -= nignoredtp

            # count the number of ignored true positives
            self.itp += nignoredtp

            # adjust the number of ground truth objects considered
            self.n_gt -= (nignoredfn + nignoredtp)

            # count the number of ignored ground truth objects
            self.n_igt += nignoredfn + nignoredtp

            # count the number of ignored tracker objects
            self.n_itr += nignoredtracker

            # false negatives = associated gt bboxes exceding association threshold + non-associated gt bboxes
            tmpfn   += len(frame_gts)-len(association_matrix)-nignoredfn
            self.fn += len(frame_gts)-len(association_matrix)-nignoredfn
            self.ifn += nignoredfn

            # false positives = tracker bboxes - associated tracker bboxes
            tmpfp   += len(frame_results) - tmptp - nignoredtracker - nignoredtp
            self.fp += len(frame_results) - tmptp - nignoredtracker - nignoredtp

            # sanity checks
            # - the number of true positives minues ignored true positives
            #   should be greater or equal to 0
            # - the number of false negatives should be greater or equal to 0
            # - the number of false positives needs to be greater or equal to 0
            #   otherwise ignored detections might be counted double
            # - the number of counted true positives (plus ignored ones)
            #   and the number of counted false negatives (plus ignored ones)
            #   should match the total number of ground truth objects
            # - the number of counted true positives (plus ignored ones)
            #   and the number of counted false positives
            #   plus the number of ignored tracker detections should
            #   match the total number of tracker detections; note that
            #   nignoredpairs is subtracted here to avoid double counting
            #   of ignored detection sin nignoredtp and nignoredtracker
            if tmptp<0:
                print (tmptp, nignoredtp)
                raise NameError("Something went wrong! TP is negative")
            if tmpfn<0:
                print (tmpfn, len(frame_gts), len(association_matrix), nignoredfn, nignoredpairs)
                raise NameError("Something went wrong! FN is negative")
            if tmpfp<0:
                print (tmpfp, len(frame_results), tmptp, nignoredtracker, nignoredtp, nignoredpairs)
                raise NameError("Something went wrong! FP is negative")
            if tmptp + tmpfn is not len(frame_gts)-nignoredfn-nignoredtp:
                print ("seqidx", seq_idx)
                print ("frame ", f)
                print ("TP    ", tmptp)
                print ("FN    ", tmpfn)
                print ("FP    ", tmpfp)
                print ("nGT   ", len(frame_gts))
                print ("nAss  ", len(association_matrix))
                print ("ign GT", nignoredfn)
                print ("ign TP", nignoredtp)
                raise NameError("Something went wrong! nGroundtruth is not TP+FN")
            if tmptp+tmpfp+nignoredtp+nignoredtracker is not len(frame_results):
                print (seq_idx, f, len(frame_results), tmptp, tmpfp)
                print (len(association_matrix), association_matrix)
                raise NameError("Something went wrong! nTracker is not TP+FP")

            # loop over ground truth track_id
            # check for id switches or fragmentations
            for i,gt_id in enumerate(frame_ids[0]):
                if gt_id in last_frame_ids[0]:
                    idx = last_frame_ids[0].index(gt_id)
                    tid = frame_ids[1][i]
                    lid = last_frame_ids[1][idx]
                    if tid != lid and lid != -1 and tid != -1:
                        if frame_gts[i].truncation<self.max_truncation:
                            frame_gts[i].id_switch = 1
                    if tid != lid and lid != -1:
                        if frame_gts[i].truncation<self.max_truncation:
                            frame_gts[i].fragmentation = 1

            # save current index
            last_frame_ids = frame_ids

        # compute mostly_tracked/partialy_tracked/mostly_lost, fragments, idswitches for all groundtruth trajectories
        n_ignored_tr_total = 0

        if len(seq_trajectories)==0:
            print("Error: There is no trajectories data")
            return
        n_ignored_tr = 0
        for g, ign_g in zip(seq_trajectories.values(), seq_ignored.values()):
            # all frames of this gt trajectory are ignored
            if all(ign_g):
                n_ignored_tr+=1
                n_ignored_tr_total+=1
                continue
            # all frames of this gt trajectory are not assigned to any detections
            if all([this==-1 for this in g]):
                self.mostly_lost+=1
                continue
            # compute tracked frames in trajectory
            last_id = g[0]
            # first detection (necessary to be in gt_trajectories) is always tracked
            tracked = 1 if g[0]>=0 else 0
            lgt = 0 if ign_g[0] else 1
            for f in range(1,len(g)):
                if ign_g[f]:
                    last_id = -1
                    continue
                lgt+=1
                if last_id != g[f] and last_id != -1 and g[f] != -1 and g[f-1] != -1:
                    self.id_switches += 1
                if f < len(g)-1 and g[f-1] != g[f] and last_id != -1 and g[f] != -1 and g[f+1] != -1:
                    self.fragments += 1
                if g[f] != -1:
                    tracked += 1
                    last_id = g[f]
            # handle last frame; tracked state is handled in for loop (g[f]!=-1)
            if len(g)>1 and g[f-1] != g[f] and last_id != -1  and g[f] != -1 and not ign_g[f]:
                self.fragments += 1

            # compute mostly_tracked/partialy_tracked/mostly_lost
            tracking_ratio = tracked / float(len(g) - sum(ign_g))
            if tracking_ratio > 0.8:
                self.mostly_tracked += 1
            elif tracking_ratio < 0.2:
                self.mostly_lost += 1
            else: # 0.2 <= tracking_ratio <= 0.8
                self.partialy_tracked += 1

        if (self.n_gt_trajectories-n_ignored_tr_total)==0:
            self.mostly_tracked = 0.
            self.partialy_tracked = 0.
            self.mostly_lost = 0.
        else:
            self.mostly_tracked /= float(self.n_gt_trajectories-n_ignored_tr_total)
            self.partialy_tracked /= float(self.n_gt_trajectories-n_ignored_tr_total)
            self.mostly_lost /= float(self.n_gt_trajectories-n_ignored_tr_total)

        # precision/recall
        if (self.fp+self.tp)==0 or (self.tp+self.fn)==0:
            self.recall = 0.
            self.precision = 0.
        else:
            self.recall = self.tp/float(self.tp+self.fn)
            self.precision = self.tp/float(self.fp+self.tp)
        return True

    def _print_entry(self, key, val,width=(70,10)):
        """
            Pretty print an entry in a table fashion.
        """

        s_out =  key.ljust(width[0])
        if type(val)==int:
            s = "%%%dd" % width[1]
            s_out += s % val
        elif type(val)==float:
            s = "%%%df" % (width[1])
            s_out += s % val
        else:
            s_out += ("%s"%val).rjust(width[1])
        return s_out

    def create_summary(self):
        """
            Generate and mail a summary of the results.
            If mailpy.py is present, the summary is instead printed.
        """

        summary = ""

        summary += "tracking evaluation summary".center(80,"=") + "\n"
        summary += self._print_entry("Recall", self.recall) + "\n"
        summary += "\n"
        summary += self._print_entry("Mostly Tracked", self.mostly_tracked) + "\n"
        summary += self._print_entry("Partly Tracked", self.partialy_tracked) + "\n"
        summary += self._print_entry("Mostly Lost", self.mostly_lost) + "\n"
        summary += "\n"
        summary += self._print_entry("True Positives", self.tp) + "\n"
        summary += self._print_entry("Ignored True Positives", self.itp) + "\n"
        summary += self._print_entry("False Positives", self.fp) + "\n"
        summary += self._print_entry("False Negatives", self.fn) + "\n"
        summary += self._print_entry("Ignored False Negatives", self.ifn) + "\n"
        summary += self._print_entry("Missed Targets", self.fn) + "\n"
        summary += self._print_entry("ID-switches", self.id_switches) + "\n"
        summary += self._print_entry("Fragmentations", self.fragments) + "\n"
        summary += "\n"
        summary += self._print_entry("Ground Truth Objects (Total)", self.n_gt + self.n_igt) + "\n"
        summary += self._print_entry("Ignored Ground Truth Objects", self.n_igt) + "\n"
        summary += self._print_entry("Ground Truth Trajectories", self.n_gt_trajectories) + "\n"
        summary += "\n"
        summary += self._print_entry("Tracker Objects (Total)", self.n_tr) + "\n"
        summary += self._print_entry("Ignored Tracker Objects", self.n_itr) + "\n"
        summary += self._print_entry("Tracker Trajectories", self.n_tr_trajectories) + "\n"
        summary += "="*80

        return summary

    def save_to_stats(self):
        """
            Save the statistics in a whitespace separate file.
        """

        # create pretty summary
        summary = self.create_summary()

        # print the summary.
        print(summary)

        # write summary to file summary_cls.txt
        filename = os.path.join(self.eval_dir, "summary.txt" )
        dump = open(filename, "w+")
        # print>>dump, summary
        print(summary, end="", file=dump)
        dump.close()

def get_benchmark_dir_name():
    if datetime.datetime.now().minute < 10:
        minute_str = str(0) + str(datetime.datetime.now().minute)
    else:
        minute_str = str(datetime.datetime.now().minute)
    if datetime.datetime.now().second < 10:
        second_str = str(0) + str(datetime.datetime.now().second)
    else:
        second_str = str(datetime.datetime.now().second)
    time_file_name = str(datetime.datetime.now().year) + "_" + \
                    str(datetime.datetime.now().month) + \
                    str(datetime.datetime.now().day) + "_" + \
                    str(datetime.datetime.now().hour) + \
                    minute_str + second_str
    benchmark_dir_name = "benchmark_" + time_file_name
    return benchmark_dir_name

def copy_result_to_current_time_dir(base_dir, benchmark_dir_name, result_file_path):
    benchmark_dir = os.path.join(base_dir, benchmark_dir_name)
    os.makedirs(benchmark_dir)
    result_file_name = "benchmark_results.txt"
    result_file_in_benchmark_dir = os.path.join(benchmark_dir, result_file_name)
    copyfile(result_file_path, result_file_in_benchmark_dir)

def evaluate(velo_data_num, result_file_path, gt_file_path, benchmark_dir):
    """
        Entry point for evaluation, will load the data and start evaluation
    """

    # start evaluation and instanciated eval object
    e = TrackingEvaluation(velo_data_num)
    # load tracker data and check provided classes
    try:
        if not e.load_tracked_data(result_file_path):
            "failed to load tracked data"
            return
        print("Loading Results - Success")
        print("Size of result data ", len(e.result_data))
    except:
        print("Caught exception while loading result data.")
        return
    # load groundtruth data for this class
    if not e.load_ground_truth(gt_file_path):
        raise ValueError("Ground truth not found.")
    print("Loading Groundtruth - Success")
    print("Size of ground truth data ", len(e.groundtruth))
    # sanity checks
    if len(e.groundtruth) is not len(e.result_data):
        print("The uploaded data does not provide results for every sequence.")
        return False
    print("Loaded %d Sequences." % len(e.groundtruth))
    print("Start Evaluation...")
    # create needed directories, evaluate and save stats
    try:
        e.create_eval_dir(benchmark_dir)
    except:
        print("Caught exception while creating results.")
    if e.compute_3rd_party_metrics():
        print("Finished evaluation")
        e.save_to_stats()
    else:
        print("There seem to be no true positives or false positives at all in the submitted data.")

    # finish
    print("Thank you for participating in our benchmark!")
    return True

def main(argv):
    if len(argv) == 0:
        print("You have set a path to the kitti data directory")
        print("Usage: python3 evaluate_tracking.py /home/hoge/2011_09_26/2011_09_26_drive_0005_sync")
        return

    base_dir = argv[0]
    if os.path.exists(base_dir) == 0:
        print ("Error: you need to set valid path")
        return

    benchmark_dir_name = get_benchmark_dir_name()
    benchmark_dir = os.path.join(base_dir, benchmark_dir_name)
    print (benchmark_dir)

    result_file_name = "benchmark_results.txt"
    result_file_path = os.path.join(base_dir, result_file_name)
    # copy benchmark_results.txt to `benchmark_dir_name`/benchmark_results.txt
    copy_result_to_current_time_dir(base_dir, benchmark_dir_name, result_file_path)

    tracklet_file_name = "tracklet_labels.xml"
    partial_velo_path = "velodyne_points/data"
    tracklet_path    = os.path.join(base_dir, tracklet_file_name)
    gt_file_path     = os.path.join(base_dir, "gt_frame.txt")
    velo_dir         = os.path.join(base_dir, partial_velo_path)
    velo_data_num    = len(os.listdir(velo_dir))

    dump_frames_text_from_tracklets(velo_data_num, tracklet_path, gt_file_path)
    success = evaluate(velo_data_num, result_file_path, gt_file_path, benchmark_dir)

if __name__ == "__main__":
    main(sys.argv[1:])
