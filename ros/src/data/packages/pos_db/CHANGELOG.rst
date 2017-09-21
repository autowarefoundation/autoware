^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pos_db
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* fixed conflict
* Change topic message type.
* fix circular-dependency
* Contributors: Shohei Fujii, USUDA Hisashi, Yusuke FUJII

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
* Fix typos around map_db
* Add module graph tool
* Add missing dependencies
* Fix pos_downloader time check.
  全体で最新の位置のデータしかpublishしていなかったため、
  車両毎にチェックするよう修正しました。
* Fix pos_uploader's msec bug.
  Timestamp(sec)を文字列に変換する際のmsecの指定の間違いを修正しました。
  穴があったら入りたいです…申し訳ございません。
* Fix pos_db's bugs about the time.
* Maybe fix the stopping problem of pos_db.
* Use c++11 option instead of c++0x
  We can use newer compilers which support 'c++11' option
* Fix pos_db's bad message.
* Changed pos_downloader.cpp and pos_uploader.cpp:
  - pos_downloader.cpp: changed the type of marker put on detected car from CUBE to SPHERE
  - pos_uploader.cpp: make pos_uploader node to subscribe to obj_car/obj_label and obj_person/obj_label
* Fix car_pose and Change alpha value.
* Thin color of car and pedestrian for old data.
* Initial commit for public release
* Contributors: Shinpei Kato, Syohei YOSHIDA, USUDA Hisashi, anhnv3991, syouji
