^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package model_publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.7.0 (2018-05-18)
------------------
* update Version from 1.6.3 to 1.7.0 in package.xml and CHANGELOG.rst
* [Feature] Collada Milee 3D model (`#982 <https://github.com/kfunaoka/Autoware/issues/982>`_)
  * Added Milee Collada Model
  * Added Launch file
* Contributors: Abraham Monrroy, Kosuke Murakami

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
* re added launch files for estima models (`#902 <https://github.com/cpfl/autoware/issues/902>`_)
  deleted by `#890 <https://github.com/cpfl/autoware/issues/890>`_
* Added 2 more color models
  Updated files and launch files
* Added Estima model
* Contributors: AMC, Abraham Monrroy, Yamato ANDO

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
* added install targets
* removed comments
* Contributors: Dejan Pangercic, Yusuke FUJII

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
* Add module graph tool
* Add Tier4 vehicle model
* Add new model publisher using robot_state_publisher package
* Modified TF tree
  Add localizer_pose
* Put vehicle model under ros/src/.config/model
  Change topic name from prius_model to vehicle_model
* Replace Prius Model.
* bug fix for model publisher
* adjusted to the center of rear tires
* Initial commit for public release
* Contributors: Hiroki Ohta, Shinpei Kato, USUDA Hisashi, Yukihiro Saito, yukikitsukawa
