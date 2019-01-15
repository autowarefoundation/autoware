^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grid_map_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.0 (2017-11-24)
------------------
* Fixed the constructor for the ros::Time in saveToBag to correctly use fromNSec.
* Fixed cpp-check warnings and errors.
* Removed dependency of grid_map_ros on grid_map_costmap_2d.
* Contributors: Perry Franklin, Péter Fankhauser

1.5.2 (2017-07-25)
------------------

1.5.1 (2017-07-25)
------------------
* Added backwards compatibility for costmap_2d conversion in grid_map_ros (`#111 <https://github.com/ethz-asl/grid_map/issues/111>`_).
* Contributors: Peter Fankhauser

1.5.0 (2017-07-18)
------------------
* Added new ROS converter for polygon to triangle lists.
* Move costmap_2d conversion into separate package.
* Contributors: Peter Fankhauser

1.4.2 (2017-01-24)
------------------
* Added conversion for Costmap2D from ROS Navigation (`#84 <https://github.com/ethz-asl/grid_map/issues/84>`_).
* Added Grid Map message traits to simplify code of the Grid Map RViz plugin (`#87 <https://github.com/ethz-asl/grid_map/issues/87>`_).
* Contributors: Peter Fankhauser, Stefan Kohlbrecher, Daniel Stonier

1.4.1 (2016-10-23)
------------------
* Improved transformation of images to color grid map layers.
* Contributors: Peter Fankhauser

1.4.0 (2016-08-22)
------------------

1.3.3 (2016-05-10)
------------------
* Release for ROS Kinetic.
* Contributors: Peter Fankhauser

1.3.2 (2016-05-10)
------------------

1.3.1 (2016-05-10)
------------------

1.3.0 (2016-04-26)
------------------
* Added new convenience function to change the resolution of grid maps with help of OpenCV interpolation methods (`#60 <https://github.com/ethz-asl/grid_map/issues/60>`_).
* Separated OpenCV to grid map conversions to grid_map_cv package (also fixes `#56 <https://github.com/ethz-asl/grid_map/issues/56>`_)
* Improved efficiency and generalized image to grid map conversion.
* Added image conversion for different encodings and depth resolutions.
* Fix scaling of image value to height conversion.
* Improved efficiency of the grid map to point cloud conversion by omitting invalid cells.
* Contributors: Peter Fankhauser, Daniel Stonier, Martin Wermelinger, Dominic Jud

1.2.0 (2016-03-03)
------------------
* Changed the package name from `grid_map` to `grid_map_ros` and made `grid_map` a metapackage (`#34 <https://github.com/ethz-asl/grid_map/issues/34>`_).
* Added new occupancy grid to grid map converter (`#33 <https://github.com/ethz-asl/grid_map/issues/33>`_).
* Contributors: Peter Fankhauser

1.1.3 (2016-01-11)
------------------

1.1.2 (2016-01-11)
------------------
* Should fix errors on build server regarding Eigen3 and visualization_msgs dependencies.

1.1.1 (2016-01-11)
------------------
* Changes to CMakeLists.txt to enable compatibility with Ubuntu Saucy.

1.1.0 (2016-01-08)
-------------------
* added installation instructions in CMakeLists
* new conversion from grid map to image
* general improvements and bugfixes

1.0.0 (2015-11-20)
-------------------
* release for Springer ROS Book Chapter
