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
[Reference](https://github.com/lgsvl/simulator/issues/5)

![lgsvl_simulator_bridge](media/lgsvl_simulator_bridge.png) 

## launch with simulator you have built
1. set $LGSVL_SIM_DIR_PATH
```
export LGSVL_SIM_DIR_PATH=<lgsvl_sim_binary_directory_path>
```