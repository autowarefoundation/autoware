# Vector Map Converter
This package contains converters to convert different map formats into Vector Map information used by Autoware.

## Installation
1. Install Lanelet  <br>
   follow installation instruction in https://github.com/fzi-forschungszentrum-informatik/Lanelet2
2. copy vector\_map\_converter and dependency_packages under ~/catkin\_ws/src/
3. cd ~/catkin_ws
4. catkin_make

## autowaremap2vectormap node
This node converts Autoware Map Format into vector map csv files:

### Usage
rosrun vector_map_converter autowaremap2vectormap [autoware map csv files]

### Output Files
- area.csv
- intersection.csv
- crosswalk.csv
- dtlane.csv
- lane.csv
- line.csv
- node.csv
- point.csv
- pole.csv
- roadsign.csv
- signaldata.csv
- stopline.csv
- vector.csv
- wayarea.csv

### Brief explanation about conversion
+ area.csv <br>
Converted from `autoware_map/area.csv`.
+ cross_walk.csv <br>
Converted from `autoware_map/lane_attr_relation.csv` files.
+ dtlane.csv <br>
Converted from `autoware_map/waypoint_relations.csv` and other related files.   
+ intersection.csv <br>
Converted using `autoware_map/lane_attr_relation.csv`
+ lane.csv <br>
Converted from `autoware_map/waypoint_relations.csv` and other related files.   
+ line.csv <br>
converted from `autoware_map/area`, `autoware_map_info/waypoint.csv`, and other related topics. Contains line information for areas and stop lines
+ node.csv<br>
Converted from `autoware_map/waypoints.csv`
+ pole.csv <br>
Dummy poles created for signals
+ signal.csv <br>
Converted from `autoware_map/signal.csv` files
+ stop_line.csv <br>
Converted from `autoware_map/waypoint.csv`. Creates virtual line from yaw in `waypoint_relations.csv` and also creates dummy road sign to be linked.
+ road_sign.csv <br>
Dummy road signs are created for stop line
+ vector.csv  <br>
converted from `autoware_map/signal_lights.csv` and other related files.
+ way_area.csv <br>
Converted from `autoware_map/wayarea.csv`. Not tested yet.

## lanelet2vectormap node
This node converts lanelet2 map into vector map csv files used in Autoware.
### Usage
`rosrun vector_map_converter lanelet2vectormap _map_file:=<path to lanelet map> _origin_lat:=<origin for projection>  _origin_lon:=<origin for projection>`<br>
For sample lanelet map:<br>
`rosrun vector_map_converter lanelet2vectormap _map_file:=~/catkin_ws/src/vector_map_converter/lanelet2/lanelet2_maps/res/mapping_example.osm _origin_lat:=49.00331750371  _origin_lon:=8.42403027399`

### Output Files
- area.csv
- intersection.csv
- crosswalk.csv
- dtlane.csv
- lane.csv
- line.csv
- node.csv
- point.csv
- pole.csv
- roadsign.csv
- signaldata.csv
- stopline.csv
- vector.csv
- wayarea.csv

### Notes about conversion from lanelet2.
- Any centerline information used during conversion is calculated using original function since the default centerline information in Lanelet2 did not have sufficient quality.
- coordinates in point.csv is in MGRS. (bx = northing, ly = easting)
- all dtlanes assume straight line.
- position and shape of pole.csv does not match with real environment because lanelet do not have pole information linked to traffic light. (They are placed near the related objects though)
- all white lines are converted as solid white lines. (no yellow lines, dashed lines, etc.)
- wayareas are converted from lane information since lanelet do not have such class.
- all light bulbs in traffic lights are represented as unknown color since lanelets do not contain such data(no red, green, yellow information)
- Any intersecting lanelets are interpreted as intersections (convex hull of lanelets would be intersection area)

### Limitations
Following fields in each csv files are missing(all values are 0). Most of them should be irrelevant in current Autoware (v1.11) and shouldn't have critical issue.
- crosswalk.csv: linkid
- intersection.csv: linkid
- lane.csv: roadsecid, linkwaid
- point.csv: ref
- stopline.csv: tlid

### Brief Explanation about conversion
+ area.csv<br>
Created for vectormap/crosswalk.csv, vectormap/intersection.csv, vectormap/wayarea.csv
+ crosswalk.csv<br>
Created from lanelets with type "crosswalk".
Since there are no area inforamtion for each stripes in Lanelet, stripes are created as grid within outer bound of crosswalk.
+ dtlane.csv<br>
Calculated from two consecutive points in Lanelet/Centerline.
All dtlanes are described as straight line.
Therefore, following fields are constant:
+ intersection.csv<br>
Any intersecting lanelets are interpreted as intersections.
Convex hull of intersecting lanelets would be intersection area.
+ lane.csv<br>
Calculated from two consective waypoints.
+ line.csv<br>
Created for Aisan/stoplines, Aisan/whitelines.
+ node.csv<br>
Calculated from Lanelet/Centerline.
+ point.csv<br>
bx and ly are calculated as MGRS coordinate from WGS84 coordinate in Lanelet.
+ pole.csv<br>
Virtual poles are created for all road signs and signaldata.
+ roadsign.csv <br>
Virtual Roadsigns are created for stop lines
+ signaldata.csv<br>
Calculated from lanelet's traffic light regulatory elements.
Since there are no signal orientaiton information in lanelet,  direction of related lanelet is used as direction of traffic light.
+ stopline.csv<br>
Calculated from lanelet's linestring with type "stopline".
+ vector.csv<br>
Calculated for signal lights and road signs.
+ wayarea.csv<br>
Calculated from lane and lane width.
+ whiteline.csv<br>
Calculated from rightbound and leftbound of lanelets.
