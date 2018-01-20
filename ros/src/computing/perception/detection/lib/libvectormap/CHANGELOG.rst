^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libvectormap
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge branch 'develop' of https://github.com/CPFL/Autoware into feature/remote_monitor
* Contributors: Yamato ANDO, Yuki Iida, Yusuke FUJII, yukikitsukawa

1.5.1 (2017-09-25)
------------------

1.5.0 (2017-09-21)
------------------
* fixed cmake and package.xml for libvectormap
  moved headers into include/libvectormap since otherwise this otherwise can conflict with other files elsewhere.
* Contributors: Dejan Pangercic

1.4.0 (2017-08-04)
------------------

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
* Fix codes to use map_file messages and old vector_map_info topics
* Fix for rosjava installed platform
  Some packages don't declare package dependencies correctly.
  This makes message jar files built failure.
* Eigen3 with Fallback on ros-catkin-modules
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Initial commit for public release
* Contributors: Jit Ray Chowdhury, Shinpei Kato, Syohei YOSHIDA, syouji
