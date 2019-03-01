
cmake_minimum_required(VERSION 2.8.3)
project(autoware_planning_msgs)

find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        geometry_msgs
        sensor_msgs
                
        )


## Generate messages in the 'msg' folder
add_message_files(
        DIRECTORY msg
        FILES
          ControlCommandStamped.msg
          Waypoint.msg
          DTLane.msg
          VehicleCmd.msg
          ControlCommand.msg
          LampCmd.msg
          ExtractedPosition.msg
          BrakeCmd.msg
          WaypointState.msg
          AccelCmd.msg
          State.msg
          LaneArray.msg
          SteerCmd.msg
          Lane.msg

)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
                
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
        geometry_msgs
        sensor_msgs
                
)
