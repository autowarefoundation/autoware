^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kf
^^^^^^^^^^^^^^^^^^^^^^^^

1.6.3 (2018-03-06)
------------------

1.6.2 (2018-02-27)
------------------
* Update CHANGELOG
* Contributors: Yusuke FUJII

1.6.1 (2018-01-20)
------------------
* update CHANGELOG
* Contributors: Yusuke FUJII

1.6.0 (2017-12-11)
------------------
* Prepare release for 1.6.0
* Contributors: Yamato ANDO

1.5.1 (2017-09-25)
------------------
* Release/1.5.1 (`#816 <https://github.com/cpfl/autoware/issues/816>`_)
  * fix a build error by gcc version
  * fix build error for older indigo version
  * update changelog for v1.5.1
  * 1.5.1
* Contributors: Yusuke FUJII

1.5.0 (2017-09-21)
------------------
* Update changelog
* Contributors: Yusuke FUJII

1.4.0 (2017-08-04)
------------------
* version number must equal current release number so we can start releasing in the future
* added changelogs
* Contributors: Dejan Pangercic

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------
* fix a build issue due to autoware_msgs on the Indigo
* Contributors: Yusuke FUJII

1.2.0 (2017-06-07)
------------------
* Fixed compatibility issues with indigo
* fix circular-dependency
* ROS Kinectic Upgrade tested on Ubuntu 16.04 and OpenCV 3.2.0
  Modules not included:
  -orb_localizer
  -dpm_ocv node and lib
  Everything else working
  Added some libraries for Gazebo on src/extras
* Update for kinetic
* Contributors: Shohei Fujii, Yukihiro Saito, Yusuke FUJII, amc-nu

1.1.2 (2017-02-27 23:10)
------------------------

1.1.1 (2017-02-27 22:25)
------------------------

1.1.0 (2017-02-24)
------------------

1.0.1 (2017-01-14)
------------------

1.0.0 (2016-12-22)
------------------
* Fix to KF caused from synchro changes
* Unify how to receive /image_raw
  In order to use both rgb8 and bayer image format of /image_raw
* kf and klt modified as asked.
* Modify correct timestamps
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Integration of RCNN object detection on Autoware
  **Added a new library librcnn, which executes the object recognition using the Caffe framework, specifically the fast-rcnn branch.
  git clone --recursive https://github.com/rbgirshick/fast-rcnn.git
  -Requires CUDA for GPU support.
  To take advantage of cuDNN, at least CUDA 7.0 and a GPU with 3.5 compute capability is required.
  -Compile Caffe, located in caffe-fast-rcnn.
  Complete the requisites:http://caffe.berkeleyvision.org/install_apt.html
  -Download the pretrained models:
  http://www.cs.berkeley.edu/~rbg/fast-rcnn-data/voc12_submission.tgz
  -Modify the CMakeFiles and point them to your caffe and models directories.
  **Modified KF to use the new NMS algorithm
  **Modified Range fusion, it will not execute unnecesary fusions.
  **Added Configuration Messages to Runtime manager and RCNN node launch files
* Clipping input detections to eschew an exception
  In the case when an input detection contains invalid values, such as negative coordinates or dimensiones larger than the image, clipping is performed before acquiring the image ROI.
* Add range assign statements to kf_tracker
  [why]
  kf_tracker didn't assign object range
  into published topic correctly.
  [what]
  I added assign statements that passes subscribed range value
  to tracking state information.
  Part of this information is used as published topic value.
  (And I fixed how to copy std::vector from "=" operator
  to "copy" function)
* Fix dependency of kf
* Changed pos_downloader.cpp and pos_uploader.cpp:
  - pos_downloader.cpp: changed the type of marker put on detected car from CUBE to SPHERE
  - pos_uploader.cpp: make pos_uploader node to subscribe to obj_car/obj_label and obj_person/obj_label
* Initial commit for public release
* Contributors: AMC, Abraham, Manato Hirabayashi, Shinpei Kato, Syohei YOSHIDA, Yukihiro Saito, anhnv3991
