import numpy as np


# Requires `tall' matrices
def create_pointcloud_file (pointcloud_mat, name):
#     print ('Shape: '+str(pointcloud_mat.shape))
    # Headers
    fd = file(name, 'w')
    fd.write('VERSION .7\n')
    fd.write('FIELDS x y z\n')
    if (pointcloud_mat.dtype==np.float32):
        fd.write('SIZE 4 4 4\n')
    elif (pointcloud_mat.dtype==np.double):
        fd.write('SIZE 8 8 8\n')
    fd.write('TYPE F F F\n')        
    fd.write('COUNT 1 1 1\n')
    fd.write('WIDTH '+str(pointcloud_mat.shape[0])+'\n')
    fd.write('HEIGHT 1\n')
    fd.write('VIEWPOINT 0 0 0 1 0 0 0\n')
    fd.write('POINTS '+str(pointcloud_mat.shape[0])+'\n')
    fd.write('DATA ascii\n')
    for p in range(pointcloud_mat.shape[0]):
#         print ('{} {} {}'.format(p[0], p[1], p[2]))
        s = "{} {} {}\n".format(pointcloud_mat[p,0], pointcloud_mat[p,1], pointcloud_mat[p,2])
        fd.write(s)
    fd.close()
