from sys import argv as cmdLineArgs
from xml.etree.ElementTree import ElementTree
import numpy as np
from warnings import warn
import os

STATE_UNSET = 0
STATE_INTERP = 1
STATE_LABELED = 2
STATEFROMTEXT = {'0':STATE_UNSET, '1':STATE_INTERP, '2':STATE_LABELED}

OCC_UNSET = 255  # -1 as uint8
OCC_VISIBLE = 0
OCC_PARTLY = 1
OCC_FULLY = 2
OCCFROMTEXT = {'-1':OCC_UNSET, '0':OCC_VISIBLE, '1':OCC_PARTLY, '2':OCC_FULLY}

TRUNC_UNSET = 255  # -1 as uint8, but in xml files the value '99' is used!
TRUNC_IN_IMAGE = 0
TRUNC_TRUNCATED = 1
TRUNC_OUT_IMAGE = 2
TRUNC_BEHIND_IMAGE = 3
TRUNCFROMTEXT = {'99':TRUNC_UNSET, '0':TRUNC_IN_IMAGE, '1':TRUNC_TRUNCATED, '2':TRUNC_OUT_IMAGE, '3': TRUNC_BEHIND_IMAGE}


class Tracklet(object):
    """
    Representation an annotated object track

    Tracklets are created in function parseXML and can most conveniently used as follows:

    for trackletObj in parseXML(tracklet_file):
    for translation, rotation, state, occlusion, truncation, amt_occlusion, amt_borders, absolute_frame_number in trackletObj:
      ... your code here ...
    #end: for all frames
    #end: for all tracklets

    absolute_frame_number is in range [first_frame, first_frame+n_frames[
    amt_occlusion and amt_borders could be None

    You can of course also directly access the fields objType (string), size (len-3 ndarray), first_frame/n_frames (int),
    trans/rots (n_frames x 3 float ndarrays), states/truncs (len-n_frames uint8 ndarrays), occs (n_frames x 2 uint8 ndarray),
    and for some tracklets amt_occs (n_frames x 2 float ndarray) and amt_borders (n_frames x 3 float ndarray). The last two
    can be None if the xml file did not include these fields in poses
    """

    object_type = None
    size = None  # len-3 float array: (height, width, length)
    first_frame = None
    trans = None   # n x 3 float array (x,y,z)
    rots = None    # n x 3 float array (x,y,z)
    states = None  # len-n uint8 array of states
    occs = None    # n x 2 uint8 array  (occlusion, occlusion_kf)
    truncs = None  # len-n uint8 array of truncation
    amt_occs = None    # None or (n x 2) float array  (amt_occlusion, amt_occlusion_kf)
    amt_borders = None    # None (n x 3) float array  (amt_border_l / _r / _kf)
    n_frames = None

    def __init__(self):
        """
        Creates Tracklet with no info set
        """
        self.size = np.nan*np.ones(3, dtype=float)

    def __str__(self):
        """
        Returns human-readable string representation of tracklet object

        called implicitly in
        print trackletObj
        or in
        text = str(trackletObj)
        """
        return '[Tracklet over {0} frames for {1}]'.format(self.n_frames, self.object_type)

    def __iter__(self):
        """
        Returns an iterator that yields tuple of all the available data for each frame

        called whenever code iterates over a tracklet object, e.g. in
        for translation, rotation, state, occlusion, truncation, amt_occlusion, amt_borders, absolute_frame_number in trackletObj:
          ...do something ...
        or
        trackDataIter = iter(trackletObj)
        """
        if self.amt_occs is None:
            return zip(self.trans, self.rots, self.states, self.occs, self.truncs,
                itertools.repeat(None), itertools.repeat(None), range(self.first_frame, self.first_frame+self.n_frames))
        else:
            return zip(self.trans, self.rots, self.states, self.occs, self.truncs,
                self.amt_occs, self.amt_borders, range(self.first_frame, self.first_frame+self.n_frames))
#end: class Tracklet


def parse_xml(tracklet_file):
    """
    Parses tracklet xml file and convert results to list of Tracklet objects

    :param tracklet_file: name of a tracklet xml file
    :returns: list of Tracklet objects read from xml file
    """

    # convert tracklet XML data to a tree structure
    e_tree = ElementTree()
    print('Parsing tracklet file', tracklet_file)
    with open(tracklet_file) as f:
        e_tree.parse(f)

    # now convert output to list of Tracklet objects
    tracklets_elem = e_tree.find('tracklets')
    tracklets = []
    tracklet_idx = 0
    n_tracklets = None
    for tracklet_elem in tracklets_elem:
        if tracklet_elem.tag == 'count':
            n_tracklets = int(tracklet_elem.text)
            print('File contains', n_tracklets, 'tracklets')
        elif tracklet_elem.tag == 'item_version':
            pass
        elif tracklet_elem.tag == 'item':
            new_track = Tracklet()
            is_finished = False
            has_amt = False
            frame_idx = None
            for info in tracklet_elem:
                if is_finished:
                    raise ValueError('more info on element after finished!')
                if info.tag == 'objectType':
                    new_track.object_type = info.text
                elif info.tag == 'h':
                    new_track.size[0] = float(info.text)
                elif info.tag == 'w':
                    new_track.size[1] = float(info.text)
                elif info.tag == 'l':
                    new_track.size[2] = float(info.text)
                elif info.tag == 'first_frame':
                    new_track.first_frame = int(info.text)
                elif info.tag == 'poses':
                    # this info is the possibly long list of poses
                    for pose in info:
                        if pose.tag == 'count':     # this should come before the others
                            if new_track.n_frames is not None:
                                raise ValueError('there are several pose lists for a single track!')
                            elif frame_idx is not None:
                                raise ValueError('?!')
                            new_track.n_frames = int(pose.text)
                            new_track.trans = np.nan * np.ones((new_track.n_frames, 3), dtype=float)
                            new_track.rots = np.nan * np.ones((new_track.n_frames, 3), dtype=float)
                            new_track.states = np.nan * np.ones(new_track.n_frames, dtype='uint8')
                            new_track.occs = np.nan * np.ones((new_track.n_frames, 2), dtype='uint8')
                            new_track.truncs = np.nan * np.ones(new_track.n_frames, dtype='uint8')
                            new_track.amt_occs = np.nan * np.ones((new_track.n_frames, 2), dtype=float)
                            new_track.amt_borders = np.nan * np.ones((new_track.n_frames, 3), dtype=float)
                            frame_idx = 0
                        elif pose.tag == 'item_version':
                            pass
                        elif pose.tag == 'item':
                            if frame_idx is None:
                                raise ValueError('pose item came before number of poses!')
                            for pose_info in pose:
                                if pose_info.tag == 'tx':
                                    new_track.trans[frame_idx, 0] = float(pose_info.text)
                                elif pose_info.tag == 'ty':
                                    new_track.trans[frame_idx, 1] = float(pose_info.text)
                                elif pose_info.tag == 'tz':
                                    new_track.trans[frame_idx, 2] = float(pose_info.text)
                                elif pose_info.tag == 'rx':
                                    new_track.rots[frame_idx, 0] = float(pose_info.text)
                                elif pose_info.tag == 'ry':
                                    new_track.rots[frame_idx, 1] = float(pose_info.text)
                                elif pose_info.tag == 'rz':
                                    new_track.rots[frame_idx, 2] = float(pose_info.text)
                                elif pose_info.tag == 'state':
                                    new_track.states[frame_idx] = STATEFROMTEXT[pose_info.text]
                                elif pose_info.tag == 'occlusion':
                                    new_track.occs[frame_idx, 0] = OCCFROMTEXT[pose_info.text]
                                elif pose_info.tag == 'occlusion_kf':
                                    new_track.occs[frame_idx, 1] = OCCFROMTEXT[pose_info.text]
                                elif pose_info.tag == 'truncation':
                                    new_track.truncs[frame_idx] = TRUNCFROMTEXT[pose_info.text]
                                elif pose_info.tag == 'amt_occlusion':
                                    new_track.amt_occs[frame_idx,0] = float(pose_info.text)
                                    has_amt = True
                                elif pose_info.tag == 'amt_occlusion_kf':
                                    new_track.amt_occs[frame_idx,1] = float(pose_info.text)
                                    has_amt = True
                                elif pose_info.tag == 'amt_border_l':
                                    new_track.amt_borders[frame_idx,0] = float(pose_info.text)
                                    has_amt = True
                                elif pose_info.tag == 'amt_border_r':
                                    new_track.amt_borders[frame_idx,1] = float(pose_info.text)
                                    has_amt = True
                                elif pose_info.tag == 'amt_border_kf':
                                    new_track.amt_borders[frame_idx,2] = float(pose_info.text)
                                    has_amt = True
                                else:
                                    raise ValueError('unexpected tag in poses item: {0}!'.format(pose_info.tag))
                            frame_idx += 1
                        else:
                            raise ValueError('unexpected pose info: {0}!'.format(pose.tag))
                elif info.tag == 'finished':
                    is_finished = True
                else:
                    raise ValueError('unexpected tag in tracklets: {0}!'.format(info.tag))
            #end: for all fields in current tracklet

            # some final consistency checks on new tracklet
            if not is_finished:
                warn('tracklet {0} was not finished!'.format(tracklet_idx))
            if new_track.n_frames is None:
                warn('tracklet {0} contains no information!'.format(tracklet_idx))
            elif frame_idx != new_track.n_frames:
                warn('tracklet {0} is supposed to have {1} frames, but perser found {1}!'.format(tracklet_idx, new_track.n_frames, frame_idx))
            if np.abs(new_track.rots[:,:2]).sum() > 1e-16:
                warn('track contains rotation other than yaw!')

            # if amt_occs / amt_borders are not set, set them to None
            if not has_amt:
                new_track.amt_occs = None
                new_track.amt_borders = None

            # add new tracklet to list
            tracklets.append(new_track)
            tracklet_idx += 1

        else:
            raise ValueError('unexpected tracklet info')
    #end: for tracklet list items

    print('Loaded', tracklet_idx, 'tracklets.')

    # final consistency check
    if tracklet_idx != n_tracklets:
        warn('according to xml information the file has {0} tracklets, but parser found {1}!'.format(n_tracklets, tracklet_idx))

    return tracklets
#end: function parseXML

def dump_frames_text_from_tracklets(n_frames, xml_path, gt_file_path):
    """
    Loads dataset labels also referred to as tracklets, saving them individually for each frame.

    Parameters
    ----------
    n_frames    : Number of frames in the dataset.
    xml_path    : Path to the tracklets XML.

    Returns
    -------
    Tuple of dictionaries with integer keys corresponding to absolute frame numbers and arrays as values. First array
    contains coordinates of bounding box vertices for each object in the frame, and the second array contains objects
    types as strings.
    """
    tracklets = parse_xml(xml_path)

    frame_num          = {} #1
    frame_track_id     = {} #1
    frame_obj_type     = {} #1
    frame_truncated    = {} #1
    frame_occluded     = {} #1
    frame_dimensions   = {} #3
    frame_location     = {} #3
    frame_rotation_yaw = {} #1
    for i in range(n_frames):
        frame_num[i]          = []
        frame_track_id[i]     = []
        frame_obj_type[i]     = []
        frame_truncated[i]    = []
        frame_occluded[i]     = []
        frame_dimensions[i]   = []
        frame_location[i]     = []
        frame_rotation_yaw[i] = []

    # loop over tracklets
    for i, tracklet in enumerate(tracklets):
        # this part is inspired by kitti object development kit matlab code: computeBox3D
        h, w, l = tracklet.size
        # loop over all data in tracklet
        for translation, rotation, state, occlusion, truncation, amt_occlusion, amt_borders, absolute_frame_number in tracklet:
            yaw = rotation[2]  # other rotations are supposedly 0
            assert np.abs(rotation[:2]).sum() == 0, 'object rotations other than yaw given!'

            # concate data
            frame_num[absolute_frame_number]          = frame_num[absolute_frame_number]          + [absolute_frame_number]
            frame_track_id[absolute_frame_number]     = frame_track_id[absolute_frame_number]     + [i]
            frame_obj_type[absolute_frame_number]     = frame_obj_type[absolute_frame_number]     + [tracklet.object_type]
            frame_truncated[absolute_frame_number]    = frame_truncated[absolute_frame_number]    + [truncation]
            frame_occluded[absolute_frame_number]     = frame_occluded[absolute_frame_number]     + [occlusion[0]]
            frame_dimensions[absolute_frame_number]   = frame_dimensions[absolute_frame_number]   + [np.array([l, w, h])]
            frame_location[absolute_frame_number]     = frame_location[absolute_frame_number]     + [np.array(translation)]
            frame_rotation_yaw[absolute_frame_number] = frame_rotation_yaw[absolute_frame_number] + [yaw]

    gt_text_file = os.path.join(gt_file_path)
    file = open(gt_text_file, 'w')
    for i_frame in range(n_frames):
        for i_object in range(len(frame_num[i_frame])):
            file.write(str(frame_num[i_frame][i_object]))
            file.write(' ')
            file.write(str(frame_track_id[i_frame][i_object]))
            file.write(' ')
            file.write(frame_obj_type[i_frame][i_object])
            file.write(' ')
            file.write(str(frame_truncated[i_frame][i_object]))
            file.write(' ')
            file.write(str(frame_occluded[i_frame][i_object]))
            file.write(' ')
            file.write('-10')
            file.write(' ')
            file.write('-1 -1 -1 -1')
            file.write(' ')
            file.write(str(frame_dimensions[i_frame][i_object][0]))
            file.write(' ')
            file.write(str(frame_dimensions[i_frame][i_object][1]))
            file.write(' ')
            file.write(str(frame_dimensions[i_frame][i_object][2]))
            file.write(' ')
            file.write(str(frame_location[i_frame][i_object][0]))
            file.write(' ')
            file.write(str(frame_location[i_frame][i_object][1]))
            file.write(' ')
            file.write(str(frame_location[i_frame][i_object][2]))
            file.write(' ')
            file.write(str(frame_rotation_yaw[i_frame][i_object]))
            file.write('\n')
