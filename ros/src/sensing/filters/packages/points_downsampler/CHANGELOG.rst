^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package points_downsampler
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.0 (2017-09-21)
------------------

1.4.0 (2017-08-04)
------------------

1.3.1 (2017-07-16)
------------------

1.3.0 (2017-07-14)
------------------
* Resolved merge conflict by new feature
* Modify measurement_range=MAX_MEASUREMENT_RANGE; (200)
* Add Error handring to removePointsByRange()
* fix initializing measurement_range
  https://github.com/CPFL/Autoware/issues/693
* https://github.com/CPFL/Autoware/issues/693
* make the commit 14f7eca unavailable.
* Localization problem patch
  https://github.com/CPFL/Autoware/issues/693
* convert to autoware_msgs
* Contributors: YamatoAndo, Yusuke FUJII, andoh104

1.2.0 (2017-06-07)
------------------
* not call removePointsByRange() when measurement_range is 200
  not compute sqrt
* add measurement_range
  refactoring
* Contributors: yukikitsukawa

1.1.2 (2017-02-27 23:10)
------------------------

1.1.1 (2017-02-27 22:25)
------------------------

1.1.0 (2017-02-24)
------------------
* add tf_mapping
  select points_topic in points_downsample.launch
* Contributors: yukikitsukawa

1.0.1 (2017-01-14)
------------------

1.0.0 (2016-12-22)
------------------
* Rename variables.
* Rename package name.
  data_filter -> filters
  points_filter -> points_downsample
* Contributors: yukikitsukawa
