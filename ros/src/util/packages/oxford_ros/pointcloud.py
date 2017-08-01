from sdk import Dataset
from sdk import transform
import numpy as np






def BuildPointCloud (dataset_dir):
    
    dataset = Dataset (dataset_dir)
    poseList = dataset.getIns()
    lidarTs = dataset.getTimestamp ('ldmrs', raw=True)
    scans = dataset.getMainLidar ()
    
    poses = {}
    # Build list of poses 
    for ts in lidarTs :
        timestamp = int(ts)
        



if __name__ == '__main__' :
    pass