^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package icp_localizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.1 (2018-01-20)
------------------

1.6.0 (2017-12-11)
------------------
* Merge pull request `#960 <https://github.com/CPFL/Autoware/issues/960>`_ from CPFL/fix/initialpose_frame_id
  Use initialpose header.frame_id at initialposeCallback function
* Merge pull request `#965 <https://github.com/CPFL/Autoware/issues/965>`_ from CPFL/fix/icp_matching
  remove currnet_pose publisher in icp_matching
* remove currnet_pose publisher
* use header.frame_id included in initialpose topic
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
* Switch output from screen to log
* Modified file name of log for ndt_matching/icp_matching.
* Update interface.yaml of ndt_localizer, icp_localizer and points_filter
* Add measuring align_time and getFitnessScore_time.
  Fix warnings.
* Fix deprecated code
  std::basic_ios does not implement 'operator void*' in C++11 specification.
  But GCC 4.8 still supports it with '-std=c++11' option, so there is no
  problem until now. However newer GCC removes it and we should use
  'operator !' or 'operator bool' instead of 'operator void*' after C++11.
* Remove a dependency of ndt_localizer.
  Add icp_stat.msg.
* Add missing ndt_localizer dependency
* Add checkbox of icp_matching to Computing tab.
  Add ConfigICP.msg.
* Parameter tuning.
* Add icp_localizer package.
* Contributors: Syohei YOSHIDA, yukikitsukawa
