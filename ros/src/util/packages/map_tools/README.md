# Map Tools

A set of utility tools for map data

## PCD Grid Divider
`PCD Grid Divider` creates PCDs divided into grids from PCDs that are not divided into grids.

### How to launch
* From a sourced terminal:\
`rosrun map_tools pcd_grid_divider point_type grid_size output_directory input_pcd1 input_pcd2 ...`

``point_type``: PointXYZ | PointXYZI | PointXYZRGB

``grid_size``: integer (1,5,10,100...)

In the directory you specified, you will see PCDs that divided into grids.
The naming rule is ``*grid_size*_*lower bound of x*_*lower bound of y*.pcd``

## PCD Filter
`PCD Filter` downsamples PCDs by voxel grid filter.

### How to launch
* From a sourced termina:\
`rosrun map_tools pcd_filter point_type leaf_size input_pcd1 input_pcd2 ...`

``point_type``: PointXYZ | PointXYZI | PointXYZRGB

``leaf_size``: double (1,5,10,100...)

The downsampled files are saved in the same directory as the input pcd file.
The naming rule is ``*leaf_size*_*original_name*``
