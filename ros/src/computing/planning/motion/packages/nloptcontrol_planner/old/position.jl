#!/usr/bin/env julia
using RobotOS
@rosimport geometry_msgs.msg: Point, Pose, Pose2D, PoseStamped, Vector3, Twist
@rosimport std_srvs.srv: Empty, SetBool
@rosimport nav_msgs.srv.GetPlan
@rosimport gazebo_msgs.msg: ModelState, ModelStates
@rosimport gazebo_msgs.srv: SetModelState, GetModelState, GetWorldProperties

rostypegen()
using geometry_msgs.msg
using std_srvs.srv
using nav_msgs.srv
using nav_msgs.srv.GetPlan
using gazebo_msgs.srv
using gazebo_msgs.msg

function loop(get_state, pub)
    loop_rate = Rate(100.0)
    while ! is_shutdown()
        modelName = "hmmwv"  # TODO make this a parameter

        # Get the current position of the Gazebo model
        gs = GetModelStateRequest()
        gs.model_name = modelName
        gs_r = get_state(gs)
        if !gs_r.success
            error(string(" calling /gazebo/get_model_state service: ", gs_r.status_message))
        end

        npt = gs_r.pose
        publish(pub, npt)
        rossleep(loop_rate)
    end
end

function main()
    init_node("rosjl_position")

    # Set up service to get Gazebo model state
    const get_state = ServiceProxy("/gazebo/get_model_state",GetModelState)
    println("Waiting for 'gazebo/get_model_state' service...")
    wait_for_service("gazebo/get_model_state")
    # TODO use getparam to get robot name
    pub = Publisher{Pose}("/robot/pose", queue_size=10)

    loop(get_state, pub)
end

if ! isinteractive()
    main()
end
