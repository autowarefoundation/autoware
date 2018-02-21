^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libdpm_ocv
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.1 (2018-01-20)
------------------

1.6.0 (2017-12-11)
------------------
* merge develop and fixed slow switching map
* Merge branch 'master' into feature/decision
* Merge branch 'master' of github.com:cpfl/autoware into develop
* Merge branch 'feature/ndt_pcl_gpu' of https://github.com/CPFL/Autoware into feature/ndt_pcl_gpu
* Merge for ndt_pcl_gpu
* merge develop
* merge develop
* Contributors: Yamato ANDO, Yuki Iida, Yusuke FUJII, yukikitsukawa

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
* clean directories
* ROS Kinectic Upgrade tested on Ubuntu 16.04 and OpenCV 3.2.0
  Modules not included:
  -orb_localizer
  -dpm_ocv node and lib
  Everything else working
  Added some libraries for Gazebo on src/extras
* Update for kinetic
* Contributors: Yukihiro Saito, Yusuke FUJII, amc-nu

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
* Fix for rosjava installed platform
  Some packages don't declare package dependencies correctly.
  This makes message jar files built failure.
* Always check return value from CUDA API
* Correct wrong type name
* Add CUDA_CHECK after memory accesses
* Remove extra level of indirection from cuMemFreeHost
  cuda-memcheck complained about a failure to free, and no doubt the memory leak would soon lead to more problems.
* Need one more convStream element
  The loop starting at 493 is off-by-one without it, as are lines 568 and 569.
  Furthermore, if it so happens that the model has lots of parts but few levels (ie a small λ) such that parts ("n" here) is more than two greater than the HOG pyramid levels ("numLevels"), line 580 will overflow. This is TODO, but a higher priority IMO is to redesign the use of streams and texture memory, as texMap prevents parallelisation of part map processing.
* Fix off-by-one error
  cuda-memcheck reported an out-of-bounds read here, which would be followed by an out-of-bounds write. I think changing from <= to < is sufficient.
* Improve cmake CUDA configuration
  Use cmake CUDA feature as possible for removing absolute paths.
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Use modified OpenCV function for adjusting parameter
* Update dpm_ocv
  - support using both GPU and CPU
  - clean up code
* Contributors: KenYN, Syohei YOSHIDA
