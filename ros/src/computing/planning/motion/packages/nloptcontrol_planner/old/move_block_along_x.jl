#!/usr/bin/env julia

using RobotOS
@rosimport geometry_msgs.msg: Point, Pose, Pose2D, PoseStamped, Vector3
@rosimport std_srvs.srv: Empty, SetBool
@rosimport nav_msgs.srv.GetPlan

@rosimport gazebo_msgs.msg: ModelState
@rosimport gazebo_msgs.srv: SetModelState, GetModelState, GetWorldProperties

rostypegen()
using geometry_msgs.msg
using std_srvs.srv
using nav_msgs.srv.GetPlan
using gazebo_msgs.msg
using gazebo_msgs.srv


function loop(set_state,get_state)
    loop_rate = Rate(5.0)
    modelName = "robot"

    while !is_shutdown()

        # Get the current position of the Gazebo model
        gs = GetModelStateRequest()
        gs.model_name = modelName
        gs_r = get_state(gs)

        if !gs_r.success
            error(string(" calling /gazebo/get_model_state service: ", gs_r.status_message))
        end

        # Define position to move robot
        vehPose = Pose()
        vehPose.position.x = gs_r.pose.position.x + 0.1

        # Define the robot state
        ms = ModelState()
        ms.model_name = modelName
        ms.pose = vehPose

        # Set the state of the Gazebo model
        ss = SetModelStateRequest()
        ss.model_state = ms
        println("Calling 'gazebo/set_model_state' service...")
        ss_r = set_state(ss)

        if !ss_r.success
            error(string(" calling /gazebo/set_model_state service: ", ss_r.status_message))
        end

        rossleep(loop_rate)
    end
end

function main()
    init_node("rosjl_ex")

    # Set up service to set Gazebo model state
    const set_state = ServiceProxy("/gazebo/set_model_state", SetModelState)
    println("Waiting for 'gazebo/set_model_state' service...")
    wait_for_service("gazebo/set_model_state")

    # Set up service to get Gazebo model state
    const get_state = ServiceProxy("/gazebo/get_model_state",GetModelState)
    println("Waiting for 'gazebo/get_model_state' service...")
    wait_for_service("gazebo/get_model_state")

    loop(set_state,get_state)
end

if ! isinteractive()
    main()
end


#= TODO
# wait for all services to be in some ready state, seehttp://docs.ros.org/electric/api/srs_user_tests/html/prepare__robot__manip__sim_8py_source.html
# start laucnh file in a paused state and have some sin_init file start it

 sim = get_param("/use_sim_time")
 if sim
      loginfo('Waiting until simulated robot is prepared for the task...')


      const get_world = ServiceProxy("/gazebo/get_world_properties",GetWorldProperties)
      println("Waiting for '/gazebo/get_world_properties' service...")
      wait_for_service("/gazebo/get_world_properties")

      wp_r=GetWorldPropertiesRequest()

      wp=get_world(wp_r)

wait_for_message('/sim_robot_init',EmptyMsg)
wait_for_service("/gazebo/pause_physics")

loginfo("Pausing physics")

=#
# world files with obstacles
# LiDAR data processing algorithm
# figure out elipses
