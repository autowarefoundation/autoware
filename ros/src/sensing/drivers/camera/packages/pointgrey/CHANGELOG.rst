^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_pointgrey_drivers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Fixed mirrored images on Ladybug camera (`#906 <https://github.com/cpfl/autoware/issues/906>`_)
* Runtime manager updated to new package names (`#870 <https://github.com/cpfl/autoware/issues/870>`_)
  [fix] Runtime manager updated to new pgrey package names
* [Feature] Updates to Pointgrey package/Ladybug node (`#852 <https://github.com/cpfl/autoware/issues/852>`_)
  * Added support for FindXerces on Indigo (and Cmake less than 3.1.3)
  Changed default scaling value to 20% of original image size (to improve performance)
  Changed published image format from RGB to Bayer(to reduce bag size)
  * Tested on real camera, updated
* Contributors: Abraham Monrroy, Yamato ANDO

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
* add web ui
* Fix cmake and package
* Fix build error
* Contributors: Akihito OHSATO, Yusuke FUJII, hironari.yashiro

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
* Fix ladybug driver
* Add module graph tool
* Fix typo
* Add encoding check to use bayer image format
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Add ladybug node
* Initial commit for public release
* Contributors: Manato Hirabayashi, Shinpei Kato, Syohei YOSHIDA, USUDA Hisashi
