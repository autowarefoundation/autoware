^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package grid_map_cv
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.0 (2017-11-24)
------------------
* Fixed compatibility issue with OpenCV 3 (`#140 <https://github.com/ethz-asl/grid_map/issues/140>`_).
* Makes the OpenCV include a bit more specific and adds in a dependency into the catkin_package call.
* Moved inpaint filter to grid_map_cv, extended filter demo, cleanups.
* Fixed problem where clamp changes were disregarded (`#115 <https://github.com/ethz-asl/grid_map/issues/115>`_ from Le-Fix/fix/toImageClamp).
* Contributors: Péter Fankhauser, Perry Franklin, Le-Fix 

1.5.2 (2017-07-25)
------------------

1.5.1 (2017-07-25)
------------------

1.5.0 (2017-07-18)
------------------
* Fixed bug for change resolution function in OpenCV processing class (including unit tests). (`#91 <https://github.com/ethz-asl/grid_map/issues/91>`_).
* Extend grid_map_cv unit test for transparent pixels/nan-values.
* Remove constructor and destructor declaration in GridMapCvConverter.
* Contributors: Peter Fankhauser, Marco Camurri, Sascha

1.4.2 (2017-01-24)
------------------
* Fixed conversion to/from images in float&double format.
* Contributors: Peter Fankhauser

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
* Updated dependency to OpenCV for compatibility with ROS Kinetic and OpenCV 2/3.
* Contributors: Peter Fankhauser

1.3.1 (2016-05-10)
------------------

1.3.0 (2016-04-26)
------------------
* Separated OpenCV to grid map conversions to grid_map_cv package. The new methods
  are more generalized, faster, and can be used without ROS message types.
* Added new convenience function to change the resolution of grid maps with help of the OpenCV interpolation methods.
* Added `initializeFromImage()` to GridMapCvConverter.
* Added unit tests for grid_map_cv. Updated documentation.
* Resolving build errors for OpenCV.
* Fixed typo and added documentation.
* Contributors: Peter Fankhauser, Dominic Jud,
