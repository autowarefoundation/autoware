# autoware_carla_interface

## ROS 2/Autoware.universe bridge for CARLA simulator

Thanks to <https://github.com/gezp> for ROS 2 Humble support for CARLA Communication.
This ros package enables communication between Autoware and CARLA for autonomous driving simulation.

## Supported Environment

| ubuntu |  ros   | carla  | autoware |
| :----: | :----: | :----: | :------: |
| 22.04  | humble | 0.9.15 |   Main   |

## Setup

### Install

- [CARLA Installation](https://carla.readthedocs.io/en/latest/start_quickstart/)
- [Carla Lanelet2 Maps](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/)
- [Python Package for CARLA 0.9.15 ROS 2 Humble communication](https://github.com/gezp/carla_ros/releases/tag/carla-0.9.15-ubuntu-22.04)

  - Install the wheel using pip.
  - OR add the egg file to the `PYTHONPATH`.

1. Download maps (y-axis inverted version) to arbitrary location
2. Change names and create the map folder (example: Town01) inside `autoware_map`. (`point_cloud/Town01.pcd` -> `autoware_map/Town01/pointcloud_map.pcd`, `vector_maps/lanelet2/Town01.osm`-> `autoware_map/Town01/lanelet2_map.osm`)
3. Create `map_projector_info.yaml` on the folder and add `projector_type: local` on the first line.

### Build

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Run

1. Run carla, change map, spawn object if you need
   <!--- cspell:ignore prefernvidia -->

   ```bash
   cd CARLA
   ./CarlaUE4.sh -prefernvidia -quality-level=Low -RenderOffScreen
   ```

2. Run ros nodes

   ```bash
   ros2 launch autoware_launch e2e_simulator.launch.xml map_path:=$HOME/autoware_map/Town01 vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit simulator_type:=carla carla_map:=Town01
   ```

3. Set initial pose (Init by GNSS)
4. Set goal position
5. Wait for planning
6. Engage

## Inner-workings / Algorithms

The `InitializeInterface` class is key to setting up both the CARLA world and the ego vehicle. It fetches configuration parameters through the `autoware_carla_interface.launch.xml`.

The main simulation loop runs within the `carla_ros2_interface` class. This loop ticks simulation time inside the CARLA simulator at `fixed_delta_seconds` time, where data is received and published as ROS 2 messages at frequencies defined in `self.sensor_frequencies`.

Ego vehicle commands from Autoware are processed through the `autoware_raw_vehicle_cmd_converter`, which calibrates these commands for CARLA. The calibrated commands are then fed directly into CARLA control via `CarlaDataProvider`.

### Configurable Parameters for World Loading

All the key parameters can be configured in `autoware_carla_interface.launch.xml`.

| Name                      | Type   | Default Value                                                                     | Description                                                                                                                                                                                                         |
| ------------------------- | ------ | --------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `host`                    | string | "localhost"                                                                       | Hostname for the CARLA server                                                                                                                                                                                       |
| `port`                    | int    | "2000"                                                                            | Port number for the CARLA server                                                                                                                                                                                    |
| `timeout`                 | int    | 20                                                                                | Timeout for the CARLA client                                                                                                                                                                                        |
| `ego_vehicle_role_name`   | string | "ego_vehicle"                                                                     | Role name for the ego vehicle                                                                                                                                                                                       |
| `vehicle_type`            | string | "vehicle.toyota.prius"                                                            | Blueprint ID of the vehicle to spawn. The Blueprint ID of vehicles can be found in [CARLA Blueprint ID](https://carla.readthedocs.io/en/latest/catalogue_vehicles/)                                                 |
| `spawn_point`             | string | None                                                                              | Coordinates for spawning the ego vehicle (None is random). Format = [x, y, z, roll, pitch, yaw]                                                                                                                     |
| `carla_map`               | string | "Town01"                                                                          | Name of the map to load in CARLA                                                                                                                                                                                    |
| `sync_mode`               | bool   | True                                                                              | Boolean flag to set synchronous mode in CARLA                                                                                                                                                                       |
| `fixed_delta_seconds`     | double | 0.05                                                                              | Time step for the simulation (related to client FPS)                                                                                                                                                                |
| `objects_definition_file` | string | "$(find-pkg-share autoware_carla_interface)/objects.json"                         | Sensor parameters file that are used for spawning sensor in CARLA                                                                                                                                                   |
| `use_traffic_manager`     | bool   | True                                                                              | Boolean flag to set traffic manager in CARLA                                                                                                                                                                        |
| `max_real_delta_seconds`  | double | 0.05                                                                              | Parameter to limit the simulation speed below `fixed_delta_seconds`                                                                                                                                                 |
| `config_file`             | string | "$(find-pkg-share autoware_carla_interface)/raw_vehicle_cmd_converter.param.yaml" | Control mapping file to be used in `autoware_raw_vehicle_cmd_converter`. Current control are calibrated based on `vehicle.toyota.prius` Blueprints ID in CARLA. Changing the vehicle type may need a recalibration. |

### Configurable Parameters for Sensors

Below parameters can be configured in `carla_ros.py`.

| Name                      | Type | Default Value                                                                          | Description                                                                                                                                                                                                                       |
| ------------------------- | ---- | -------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `self.sensor_frequencies` | dict | {"top": 11, "left": 11, "right": 11, "camera": 11, "imu": 50, "status": 50, "pose": 2} | (line 67) Calculates the time interval since the last publication and checks if this interval meets the minimum required to not exceed the desired frequency. It will only affect ROS publishing frequency not CARLA sensor tick. |

- CARLA sensor parameters can be configured in `config/objects.json`.
  - For more details regarding the parameters that can be modified in CARLA are explained in [Carla Ref Sensor](https://carla.readthedocs.io/en/latest/ref_sensors/).

### World Loading

The `carla_ros.py` sets up the CARLA world:

1. **Client Connection**:

   ```python
   client = carla.Client(self.local_host, self.port)
   client.set_timeout(self.timeout)
   ```

2. **Load the Map**:

   Map loaded in CARLA world with map according to `carla_map` parameter.

   ```python
   client.load_world(self.map_name)
   self.world = client.get_world()
   ```

3. **Spawn Ego Vehicle**:

   Vehicle are spawn according to `vehicle_type`, `spawn_point`, and `agent_role_name` parameter.

   ```python
   spawn_point = carla.Transform()
   point_items = self.spawn_point.split(",")
   if len(point_items) == 6:
      spawn_point.location.x = float(point_items[0])
      spawn_point.location.y = float(point_items[1])
      spawn_point.location.z = float(point_items[2]) + 2
      spawn_point.rotation.roll = float(point_items[3])
      spawn_point.rotation.pitch = float(point_items[4])
      spawn_point.rotation.yaw = float(point_items[5])
   CarlaDataProvider.request_new_actor(self.vehicle_type, spawn_point, self.agent_role_name)
   ```

## Traffic Light Recognition

The maps provided by the Carla Simulator ([Carla Lanelet2 Maps](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/)) currently lack proper traffic light components for Autoware and have different latitude and longitude coordinates compared to the pointcloud map. To enable traffic light recognition, follow the steps below to modify the maps.

- Options to Modify the Map

  - A. Create a New Map from Scratch
  - Use the [Tier4 Vector Map Builder](https://tools.tier4.jp/feature/vector_map_builder_ll2/) to create a new map.

  - B. Modify the Existing Carla Lanelet2 Maps
  - Adjust the longitude and latitude of the [Carla Lanelet2 Maps](https://bitbucket.org/carla-simulator/autoware-contents/src/master/maps/) to align with the PCD (origin).
    - Use this [tool](https://github.com/mraditya01/offset_lanelet2/tree/main) to modify the coordinates.
    - Snap Lanelet with PCD and add the traffic lights using the [Tier4 Vector Map Builder](https://tools.tier4.jp/feature/vector_map_builder_ll2/).

- When using the Tier4 Vector Map Builder, you must convert the PCD format from `binary_compressed` to `ascii`. You can use `pcl_tools` for this conversion.
- For reference, an example of Town01 with added traffic lights at one intersection can be downloaded [here](https://drive.google.com/drive/folders/1QFU0p3C8NW71sT5wwdnCKXoZFQJzXfTG?usp=sharing).

## Tips

- Misalignment might occurs during initialization, pressing `init by gnss` button should fix it.
- Changing the `fixed_delta_seconds` can increase the simulation tick (default 0.05 s), some sensors params in `objects.json` need to be adjusted when it is changed (example: LIDAR rotation frequency have to match the FPS).

## Known Issues and Future Works

- Testing on procedural map (Adv Digital Twin).
  - Currently unable to test it due to failing in the creation of the Adv digital twin map.
- Automatic sensor configuration of the CARLA sensors from the Autoware sensor kit.
  - Sensor currently not automatically configured to have the same location as the Autoware Sensor kit. The current work around is to create a new frame of each sensors with (0, 0, 0, 0, 0, 0) coordinate relative to base_link and attach each sensor on the new frame (`autoware_carla_interface.launch.xml` Line 28). This work around is very limited and restrictive, as when the sensor_kit is changed the sensor location will be wrongly attached.
- Traffic light recognition.
  - Currently the HDmap of CARLA did not have information regarding the traffic light which is necessary for Autoware to conduct traffic light recognition.
