# Taken from
# https://sourceware.org/ml/gdb/2013-04/msg00104.html

import gdb
import cv2.cv as cv
import numpy as np


class PcvCommand(gdb.Command):
    def __init__(self):
        super(PcvCommand, self).__init__("pcv",
                                             gdb.COMMAND_DATA,
                                             gdb.COMPLETE_SYMBOL)
    def invoke(self, arg, from_tty):
        args = gdb.string_to_argv(arg)
        
        
        # generally, we type "plot someimage" in the GDB commandline
        # where "someimage" is an instance of cv::Mat
        v = gdb.parse_and_eval(args[0])
        
        # the value v is a gdb.Value object of C++
        # code's cv::Mat, we need to translate to
        # a python object under cv2.cv
        image_size =  (v['cols'],v['rows'])
        # print v
        # these two below lines do not work. I don't know why
        # channel = gdb.execute("call "+ args[0] + ".channels()", False, True)
        # channel = v.channels();
        CV_8U =0
        CV_8S =1
        CV_16U=2
        CV_16S=3
        CV_32S=4
        CV_32F=5
        CV_64F=6
        CV_USRTYPE1=7
        CV_CN_MAX = 512
        CV_CN_SHIFT = 3
        CV_MAT_CN_MASK = (CV_CN_MAX - 1) << CV_CN_SHIFT
        flags = v['flags']
        channel = (((flags) & CV_MAT_CN_MASK) >> CV_CN_SHIFT) + 1
        CV_DEPTH_MAX = (1 << CV_CN_SHIFT)
        CV_MAT_DEPTH_MASK = CV_DEPTH_MAX - 1
        depth = (flags) & CV_MAT_DEPTH_MASK
        IPL_DEPTH_SIGN = 0x80000000
        cv_elem_size = (((4<<28)|0x8442211) >> depth*4) & 15
        if (depth == CV_8S or depth == CV_16S or depth == CV_32S):
                mask = IPL_DEPTH_SIGN
        else:
                mask = 0
        ipl_depth = cv_elem_size*8 | mask     
        img = cv.CreateImageHeader(image_size, ipl_depth, channel)
        
        # conver the v['data'] type to "char*" type
        char_type = gdb.lookup_type("char")
        char_pointer_type =char_type.pointer()
        buffer = v['data'].cast(char_pointer_type)
        
        # read bytes from inferior's memory, because
        # we run the opencv-python module in GDB's own process
        # otherwise, we use memory corss processes        
        buf = v['step']['buf']
        bytes = buf[0] * v['rows'] # buf[0] is the step? Not quite sure.
        inferior = gdb.selected_inferior()
        mem = inferior.read_memory(buffer, bytes)
        
        # set the img's raw data
        cv.SetData(img, mem)
        mat = np.asarray(img[:,:])
        
        print ("Type: {}".format(mat.dtype))
        print (mat)


PcvCommand()


# I(3)
# {flags = 1124024324, dims = 2, rows = 3, cols = 3, data = 0x6087d0 "\001", refcount = 0x6087f4, datastart = 0x6087d0 "\001", dataend = 0x6087f4 "\002", datalimit = 0x6087f4 "\002", allocator = 0x0, size = {p = 0x7fffffffd688}, step = {p = 0x7fffffffd6d0, buf = {12, 4}}}
# [0, 1, 0]
# {flags = 1124057092, dims = 2, rows = 1, cols = 3, data = 0x6087dc "", refcount = 0x6087f4, datastart = 0x6087d0 "\001", dataend = 0x6087f4 "\002", datalimit = 0x6087f4 "\002", allocator = 0x0, size = {p = 0x7fffffffd6e8}, step = {p = 0x7fffffffd730, buf = {12, 4}}}