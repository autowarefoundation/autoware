^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_connector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge branch 'develop' into feature/ndt_pcl_gpu
* Merge branch 'develop' into feature/OpenPlanner
  Conflicts:
  ros/src/util/packages/runtime_manager/scripts/computing.yaml
* Merge branch 'develop' of github.com:cpfl/autoware into feature/decision
* Merge pull request `#922 <https://github.com/CPFL/Autoware/issues/922>`_ from asimay/patch-3
  perfect the code. same commit with `#920 <https://github.com/CPFL/Autoware/issues/920>`_.
* perfect the code. same commit with `#920 <https://github.com/CPFL/Autoware/issues/920>`_.
  remove duplicate "if" assert.
  if v_info\_.is_stored == false,  tw.twist.angular.z is not set value or previous value, it is abnormal, and later in convertSteeringAngleToAngularVelocity( ) function, it also need to assert v_info\_.is_stored is true or not, duplicate assert.
* merge develop and fixed slow switching map
* Merge branch 'master' into feature/decision
* Merge branch 'master' of github.com:cpfl/autoware into develop
* Merge branch 'feature/ndt_pcl_gpu' of https://github.com/CPFL/Autoware into feature/ndt_pcl_gpu
* Merge for ndt_pcl_gpu
* merge develop
* merge develop
* Merge branch 'develop' of https://github.com/CPFL/Autoware into feature/remote_monitor
* Contributors: Yamato ANDO, Yuki Iida, Yusuke FUJII, asimay, hatem-darweesh, yukikitsukawa

1.5.1 (2017-09-25)
------------------

1.5.0 (2017-09-21)
------------------
* autoware_connector deps fixed
* Contributors: Dejan Pangercic

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
