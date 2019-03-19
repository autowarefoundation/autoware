# How to build and use AS node in Autoware
1. Clone the autoware repository
2. After cloning autoware get all the submodules.
```
git submodule update --init --recursive
```
3. Build the workspace
4. Ther is no GUI interface to launch this node. Hence use,
```
roslaunch as ssc_interface.launch
```
