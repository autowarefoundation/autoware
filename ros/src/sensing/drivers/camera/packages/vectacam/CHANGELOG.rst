^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vectacam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.8.0 (2018-08-31)
------------------
* [Fix] Moved C++11 flag to autoware_build_flags (`#1395 <https://github.com/CPFL/Autoware/pull/1395>`_)
* [Feature] Makes sure that all binaries have their dependencies linked (`#1385 <https://github.com/CPFL/Autoware/pull/1385>`_)
* Changed frame_id to match each camera id on tierfusion (`#1313 <https://github.com/CPFL/Autoware/pull/1313>`_)
  * Changed frame_id to match each camera id on tierfusion
  * Fix to check once the point has been transformed.
* [Fix] Extend and Update interface.yaml (`#1291 <https://github.com/CPFL/Autoware/pull/1291>`_)
* Contributors: Abraham Monrroy, Esteve Fernandez, Kenji Funaoka

1.7.0 (2018-05-18)
------------------
* update Version from 1.6.3 to 1.7.0 in package.xml and CHANGELOG.rst
* Contributors: Kosuke Murakami

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
* removing mailing list from pkg maintainer (`#834 <https://github.com/cpfl/autoware/issues/834>`_)
* Contributors: Dejan Pangercic, Yamato ANDO

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
* TierFusion Driver Updated (`#742 <https://github.com/cpfl/autoware/issues/742>`_)
  Added Support for new firmware version.
  Features:
  -IP Address changed to the same as Velodyne's
  -Automatic Device Initialization "PlugAndPlay"
  Host IP Address MUST be set 192.168.1.1
  TierFusion address is fixed to 192.168.1.200
  Velodyne address is considered as default value, 192.168.1.201
  Tested in 14.04 and 16.04
* Contributors: Abraham Monrroy, Dejan Pangercic

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------

1.2.0 (2017-06-07)
------------------
* Kf Added
  Euclidean Cluster improved
* Contributors: AMC

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
* Fixed points marked by @syohex
* Update package.xml
* fixes
* removal of unnecesary stuff
* Updated VectaCam
* VectaCam driver
* Contributors: AMC, Abraham, amc
