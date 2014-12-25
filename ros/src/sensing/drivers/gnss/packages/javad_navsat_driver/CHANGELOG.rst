^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Change log for nmea_navsat_driver package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2014-08-03)
------------------
* Add debug logging output to the parser (PR #8, Mike Purvis)
* Add queue size arguement to publishers to fix warning on Indigo (PR #9, Mike Purvis)
* Add support for roslint and some related cleanup (PR #10, Mike Purvis)
 
0.4.0 (2014-05-04)
-------------------
* Initial release for Indigo
* Fix #5: Empty fields spam rosout with warnings. Driver now outputs sensor_msgs/NavSatFix messages that may contain NaNs in position and covariance when receiving invalid fixes from the device.

0.3.3 (2013-10-08)
-------------------
* Allow the driver to output velocity information anytime an RMC message is received

0.3.2 (2013-07-21)
-------------------
* Moved to nmea_navsat_driver package
* Removed .py extensions from new-in-Hydro scripts
* Now uses nmea_msgs/Sentence instead of custom sentence type
* nmea_topic_driver reads the `frame_id` parameter from the sentence, not from the parameter server

0.3.1 (2013-05-07)
-------------------
* Removed incorrect find_package dependencies

0.3.0 (2013-05-05)
-------------------
* Initial release for Hydro
* Converted to Catkin
* nmea_gps_driver.py is now deprecated and will be removed in I-Turtle. Replacement node is nmea_serial_driver.py .
* Refactored code into NMEA parser, common ROS driver and separate nodes for reading directly from serial or from topic.
* Bugs fixed:
  - nmea_gps_driver crashes when a sentence doesn't have a checksum * character ( http://kforge.ros.org/gpsdrivers/trac/ticket/4 )
  - Add ability for nmea_gps_driver to support reading from string topic ( https://github.com/ros-drivers/nmea_gps_driver/issues/1 ). Use the nmea_topic_driver.py node to get this support.

0.2.0 (2012-03-15)
------------------
* Initial version (released into Fuerte)
* Supports GGA or RMC+GSA sentences to generate sensor_msgs/NavSatFix messages
