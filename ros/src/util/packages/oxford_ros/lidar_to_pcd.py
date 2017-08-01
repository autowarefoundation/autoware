#!/usr/bin/python

import numpy as np
import sys
import sdk


if __name__ == '__main__' :
    
    scanfile = sys.argv[1] 
    scan = np.fromfile(scanfile, np.double)
    scan = scan.reshape((len(scan) // 3, 3))
    sdk.create_pointcloud_file (scan, sys.argv[2])
    pass