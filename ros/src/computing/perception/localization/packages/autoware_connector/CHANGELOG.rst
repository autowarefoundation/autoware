^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_connector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Update CHANGELOG
* Contributors: Yusuke FUJII

1.6.1 (2018-01-20)
------------------
* update CHANGELOG
* Contributors: Yusuke FUJII

1.6.0 (2017-12-11)
------------------
* Prepare release for 1.6.0
* perfect the code. same commit with `#920 <https://github.com/cpfl/autoware/issues/920>`_.
  remove duplicate "if" assert.
  if v_info\_.is_stored == false,  tw.twist.angular.z is not set value or previous value, it is abnormal, and later in convertSteeringAngleToAngularVelocity( ) function, it also need to assert v_info\_.is_stored is true or not, duplicate assert.
* Contributors: Yamato ANDO, asimay

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
* autoware_connector deps fixed
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
* convert to autoware_msgs
* Contributors: YamatoAndo

1.2.0 (2017-06-07)
------------------
* add can2odom.launch
* Contributors: yukikitsukawa

1.1.2 (2017-02-27 23:10)
------------------------
* output log
* Contributors: h_ohta

1.1.1 (2017-02-27 22:25)
------------------------

1.1.0 (2017-02-24)
------------------
* Add autoware_connector instead of vel_pose_mux
* Contributors: h_ohta

1.0.1 (2017-01-14)
------------------

1.0.0 (2016-12-22)
------------------
