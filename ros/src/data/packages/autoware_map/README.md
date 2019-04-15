# autoware_map package

## Related Packages
+ *autoware_map_msgs package*: Defines the message types for Autoware Map Format
+ *map_file package*: Loads semantic map written as Autoware Map Format.

## Code API
This package provides library to access to autoware_map messages, similar to library provided by vector_map package.


### Sample Code
```
#include <autoware_map/autoware_map.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "autwoare_map_subscriber");
    ros::NodeHandle nh;

    autoware_map::AutowareMap awm;

    autoware_map::category_t awm_required_category =  autoware_map::Category::POINT |
                                                      autoware_map::Category::AREA;

    //awm.subscribe(nh, awm_required_category);
    awm.subscribe(nh, awm_required_category, ros::Duration(5));

    //find object from id
    int point_id = 1;
    autoware_map_msgs::Point point = awm.findByKey(autoware_map::Key<autoware_map_msgs::Point>(point_id));
    if(point.point_id == point_id)
    {
        std::cout << point << std::endl;
    }
    else
    {
        std::cerr << "failed to find a point with id: " << point_id << std::endl;
    }

    //find multiple object that satisfies desired conditions
    std::vector<autoware_map_msgs::Area> area_arrays;
    area_arrays = awm.findByFilter( [](const autoware_map_msgs::Area){return true;} );
    for(auto area: area_arrays)
    {
        std::cout << area << std::endl;
    }
}
```
### Code Explanation
```
    autoware_map::category_t awm_required_category =  autoware_map::Category::POINT |
                                                      autoware_map::Category::AREA;
```
Above code enables to specify topics that users would like to subscribe.   
Category is set for each autoware_map_msgs type as follwing:
+ autoware_map::Category::NONE
+ autoware_map::Category::LANE
+ autoware_map::Category::LANE_ATTR_RELATION
+ autoware_map::Category::LANE_RELATION
+ autoware_map::Category::LANE_SIGNAL_LIGHT_RELATION
+ autoware_map::Category::LANE_CHANGE_RELATION
+ autoware_map::Category::OPPOSITE_LANE_RELATION
+ autoware_map::Category::POINT
+ autoware_map::Category::AREA
+ autoware_map::Category::ROUTE
+ autoware_map::Category::SIGNAL
+ autoware_map::Category::SIGNAL_LIGHT
+ autoware_map::Category::WAYAREA
+ autoware_map::Category::WAYPOINT
+ autoware_map::Category::WAYPOINT_LANE_RELATION
+ autoware_map::Category::WAYPOINT_RELATION
+ autoware_map::Category::WAYPOINT_SIGNAL_RELATION
+ autoware_map::Category::ALL

```
//awm.subscribe(nh, awm_required_category);
awm.subscribe(nh, awm_required_category, ros::Duration(5));
```
Above code actually subscribes to specified topics.
The function in the comment blocks the process until messages are recieved from all specified categories,  
whereas the second function blocks for user specified duration.

```
awm.findByKey(autoware_map::Key<autoware_map_msgs::Point>(point_id));
awm.findByFilter( [](const autoware_map_msgs::Area){return true;});
```
The above code allows to retrieve user specified object.
