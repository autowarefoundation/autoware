^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vehicle_socket
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Fix build error
* refactor code
* refactor code
* refactor msg and add blinker to msg
* update vehicle sender to support twist gate
* Contributors: Akihito Ohsato, Yamato ANDO, Yuki Iida

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
* Rewrite vehicle_sender and autoware_socket for applying the topic /ctrl_cmd.
* Contributors: USUDA Hisashi

1.0.1 (2017-01-14)
------------------

1.0.0 (2016-12-22)
------------------
* Add module graph tool
* Add missing dependencies
* Add ROS include path to 'include_directories'
* Fix for rosjava installed platform
  Some packages don't declare package dependencies correctly.
  This makes message jar files built failure.
* Correct runtime manager dependencies
* Publish mode_info
* Change data transfer format from Vehicle to ROS PC
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Initial commit for public release
* Contributors: Shinpei Kato, Syohei YOSHIDA, USUDA Hisashi, syouji
