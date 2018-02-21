^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fusion
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.1 (2018-01-20)
------------------

1.5.1 (2017-09-25)
------------------

1.5.0 (2017-09-21)
------------------

1.4.0 (2017-08-04)
------------------

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------
* convert to autoware_msgs
* Contributors: YamatoAndo

1.2.0 (2017-06-07)
------------------
* fix circular-dependency
* Contributors: Shohei Fujii

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
* Fixed a bud. It caused by reference to out of image size
* Revert "Fixed a bud. It caused by reference to out of image size"
  This reverts commit 9d9568b650ab372883458bf42a93f13bab0be8e3.
* Fixed a bud. It caused by reference to out of image size
* Fix dependency
* Hotfix segmentation fault in range_fusion
* Add missing dependencies
* Add ROS include path to 'include_directories'
* Add new method for distance calculation of fusion
  Add new class to handle several distance calculation methods.
  We can now switch the calculation methos below by this class:
  * Shortest
  * Median
  * Mode (most common value)
  Confine scope of search_distance.h to lib/fusion
  Reconstruct search_distance.cpp
  * use function call instead of class method
  * fix typo
  * start function name with lower case
  * return 0 immediately if argument vector size is empty
  * use const reference as many as possible to avoid data copy
  Do not pass reference to std::sort
* Correct dependency name
* Delete warning cause
  * Use 'ranged-for' to reset std::vector
  * Set nested template parameter to C++11 style
* Change calculation method of ranging
  [Before] minimun distance in a detection rectangle
  [After]  median distance in a detection rectangle
* Delete image size fixing
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
* Updated point2image to set minh in the message
* fix-dependency
* Initial commit for public release
* Contributors: AMC, Hiroki Ohta, Manato Hirabayashi, Shinpei Kato, Syohei YOSHIDA, Yukihiro Saito, h_ohta
