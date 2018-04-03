^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package librcnn
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

1.2.0 (2017-06-07)
------------------

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
* Added SSD node to CV Tracker
* Fix include path and linked library setting issue on Ubuntu 15.04
  Paths of header files and libraries of libhdf5 and CUDA on Ubuntu 15.04 are
  different from Ubuntu 14.04. And those paths are set explicitly at compiling
  time on Ubuntu 15.04.
  And clean up CMake codes by using CMake and pkg-config features instead of
  absolute paths.
* Remove needless dependencies
* updated to search for libraries in caffe path instead of /usr...
* Updated to compile rcnn only if caffe and fast rcnn are installed
* Updated according to Yosh's suggestions.
  Added README
* As suggested by @syohex
  Thanks
* Removed local references
  added $ENV{HOME} as suggested.
* Added files for RCNN node
* Contributors: AMC, Syohei YOSHIDA
