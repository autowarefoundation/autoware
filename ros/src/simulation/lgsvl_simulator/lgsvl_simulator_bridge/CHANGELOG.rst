^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lgsvl_simulator_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.0 (2019-03-21)
-------------------
* Change LGSVL simulator dir (`#2023 <https://github.com/CPFL/Autoware/issues/2023>`_)
* Contributors: Yukihiro Saito

1.10.0 (2019-01-17)
-------------------
* [fix] CMake error & warning fixes on develop (`#1808 <https://github.com/CPFL/Autoware/issues/1808>`_)
  * CMake fixes
  * CMake updated to remove unnecessary dependencies when the package is not built
  * added autoware flags
* Feature/lgsvl sim (`#1795 <https://github.com/CPFL/Autoware/issues/1795>`_)
  * add bridge package
  * add setup script
  * add getPath
  * add roslib depends
  * add launch files
  * add install in CMakeLists.txt
  * add description
  * fix .gitignore
  * remove launcher node and add launch shell script
  * add nmea2tfpose to the launch file
  * update simulation.yaml
  * update runtime manger
  * enable use runtime manager button
  * update setup script
  * add lgsvl_msgs as a submodule
  * update Dockerfile
  * update runtime manager
  * update crossbuild docker image
  * update git ignore
  * update .travis.yml
  * remove unused launch file
  * update CMakeLists.txt
  * update README.md
  * update README.md
* Contributors: Abraham Monrroy Cano, Masaya Kataoka
