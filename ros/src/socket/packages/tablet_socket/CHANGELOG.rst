^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tablet_socket
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Runtime Manager, update tablet_sender/receiver, SIGINT SA_RESTART flag down
* Runtime Manager, fix tablet_receiver for SIGINT termination
* Runtime Manager, update tablet_sender/receiver for SIGINT termination
* Add module graph tool
* Fix for rosjava installed platform
  Some packages don't declare package dependencies correctly.
  This makes message jar files built failure.
* Synchronize button view with vehicle information
* Move initialization of points_nr
* Move definitions of variable
* Improve how to call recv
* Display warning when peer is shutdown
* deleted debug code
* switched roll and pitch
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Publish gnss_pose and gnss_stat
* Update tablet_sender node
* Initial commit for public release
* Contributors: Hiroki Ohta, Shinpei Kato, Syohei YOSHIDA, USUDA Hisashi, kondoh, pdsljp, syouji
