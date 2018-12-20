# lgsvl_simulator_bridge package

This package provides lgvsl simulator and Autoware.  
[![No Image](https://img.youtube.com/vi/NgW1P75wiuA/0.jpg)](http://www.youtube.com/watch?v=NgW1P75wiuA)

## Quick Start 
### initial setup (just first time)
```
roscd lgsvl_simulator_bridge
source quick_setup_sim.bash
```

### launch simulator and bridge
Open simulation tab in runtime manager and click LGSVL Simulator button.

Please choose "SanFrancisco" map and "XE_Rigged-autoware" robot for Autoware.  

![Reference](https://github.com/lgsvl/simulator/issues/5)

![lgsvl_simulator_bridge](media/lgsvl_simulator_bridge.png) 

### launch with simulator you have built
1. set $LGSVL_SIM_DIR_PATH
```
export LGSVL_SIM_DIR_PATH=<lgsvl_sim_binary_directory_path>
```

### download pointcloud map and other datas.
1. install git lfs
```
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt update
sudo apt install git-lfs
```

1. download data from lgsvl repo
```
git clone git@github.com:lgsvl/autoware-data.git
```

2. load pointcloud data  
![pointcloud map](https://camo.qiitausercontent.com/435d9952ed982aa1fd74f4de9b399f8dd7ed5f22/68747470733a2f2f71696974612d696d6167652d73746f72652e73332e616d617a6f6e6177732e636f6d2f302f3136303334362f36643935313234612d613866342d363166632d393631362d6530363833376433353033392e706e67)  

load pointcloud data under this directory.  
```
autoware-data/data/map/pointcloud_map_sf_portion/
```

3. setup in sensing tab
enabele voxel_grid_filter

4. setup in computing tab
enable ndt_matching,vel_pose_connect,lane_rule,lane_sotp,lane_select,obstacle_void,velocity_set,pure_pursuit,twist_filter,waypoint_loader

### demonstration
[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/npTvZ09ijPA/0.jpg)](https://www.youtube.com/watch?v=npTvZ09ijPA&t=109s)