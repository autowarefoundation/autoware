# Extra Lanelet Subtypes

## Roadside Lane

The subtypes for this lanelet classify the outer lanes adjacent to the driving lane.Since the list of lanelet subtypes defined in this [link](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/LaneletAndAreaTagging.md) cannot represent the shoulder lane and pedestrian lane described below, two new subtypes are defined.When parking on the street, it is necessary to distinguish between a shoulder lane which can be used by vehicles, and a pedestrian lane which can be used by pedestrians and bicycles.If you park in a shoulder lane, you can use the entire lane for temporary parking, but if you park in a pedestrian lane, you must leave a space of at least 75cm.

### Road shoulder subtype

- refers: lanelet with subtype attribute. Subtype explains what the type of roadside it represents. If there is an area outside of this roadside lane that is open to traffic, such as a sidewalk or bike lane, select the road_shoulder subtype.

![Road shoulder](road_shoulder.svg)

Sample road shoulder in .osm format is shown below:

```xml
  <relation id="120700">
    <member type="way" role="left" ref="34577"/>
    <member type="way" role="right" ref="120694"/>
    <tag k="type" v="lanelet"/>
    <tag k="subtype" v="road_shoulder"/>
    <tag k="speed_limit" v="10"/>
    <tag k="location" v="urban"/>
    <tag k="one_way" v="yes"/>
  </relation>
```

### Pedestrian lane subtype

- refers: lanelet with subtype attribute. Subtype explains what the type of roadside it represents. If there are no passable areas outside of this roadside lane, select the pedestrian_lane subtype.

![Pedestrian lane](pedestrian_lane.svg)

Sample pedestrian lane in .osm format is shown below:

```xml
  <relation id="120700">
    <member type="way" role="left" ref="34577"/>
    <member type="way" role="right" ref="120694"/>
    <tag k="type" v="lanelet"/>
    <tag k="subtype" v="pedestrian_lane"/>
    <tag k="speed_limit" v="10"/>
    <tag k="location" v="urban"/>
    <tag k="one_way" v="yes"/>
  </relation>
```
