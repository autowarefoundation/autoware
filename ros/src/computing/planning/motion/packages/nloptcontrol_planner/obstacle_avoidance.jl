#!/usr/bin/env julia
using RobotOS
@rosimport geometry_msgs.msg: Point, Pose, Pose2D, PoseStamped, Vector3, Twist
@rosimport nloptcontrol_planner.msg: Control

rostypegen()
using geometry_msgs.msg
using nloptcontrol_planner.msg

using NLOptControl
using MAVs
using PyCall
@pyimport tf.transformations as tf

function main()
  init_node("rosjl_obstacles")

  # message for solution to optimal control problem
  const pub = Publisher{Control}("/nloptcontrol_planner/optimal_control", queue_size=10)

  # TODO pass this!
  #c=defineCase(;(:mode=>:autoGazebo)); TODO figure out why this is broken

  c = defineCase(;(:mode=>:RTPP))
  c.o = defineObs(:s1)
  setMisc!(c; N=40, solver=:Ipopt, integrationScheme=:trapezoidal)
  setSolverSettings!(c)

  n=initializeAutonomousControl(c);
  driveStraight!(n)  # NOTE probably get rid of this

  loop_rate = Rate(2.0)

  while !is_shutdown()

  n=initializeAutonomousControl(c);
  driveStraight!(n)

      for ii = 1:n.mpc.max_iter
          println("Running model for the: ",n.r.eval_num," time")
      #    setObstacleData(n.params)

          updateAutoParams!(n,c)                        # update model parameters
          status = autonomousControl!(n)                # rerun optimization
          n.mpc.t0_actual = to_sec(get_rostime())
          msg = Control()
          msg.t = n.mpc.t0_actual + n.r.t_st
          msg.x = n.r.X[:,1]
          msg.y = n.r.X[:,2]
          msg.psi = n.r.X[:,5]
          msg.sa = n.r.X[:,6]
          msg.vx = n.r.X[:,7]
          # TODO consider buffering the message here..

          publish(pub,msg)
          rossleep(loop_rate) # NOTE might not want to do this ...

          # if the vehicle is very close to the goal sometimes the optimization returns with a small final time
          # and it can even be negative (due to tolerances in NLP solver). If this is the case, the goal is slightly
          # expanded from the previous check and one final check is performed otherwise the run is failed
          if getvalue(n.tf) < 0.01 # assuming that the final time is a design variable, could check, but this module uses tf as a DV
            if ((n.r.dfs_plant[end][:x][end]-c.g.x_ref)^2 + (n.r.dfs_plant[end][:y][end]-c.g.y_ref)^2)^0.5 < 4*n.XF_tol[1]
               println("Expanded Goal Attained! \n"); n.mpc.goal_reached=true;
               break;
           else
               warn("Expanded Goal Not Attained! -> stopping simulation! \n"); break;
           end
         end

          n.mpc.t0_actual = (n.r.eval_num-1)*n.mpc.tex  # external so that it can be updated easily in PathFollowing
          simPlant!(n)  # simulating out here even if it is not :Optimal so that we can look at final solution

          if ((n.r.dfs_plant[end][:x][end]-c.g.x_ref)^2 + (n.r.dfs_plant[end][:y][end]-c.g.y_ref)^2)^0.5 < 2*n.XF_tol[1]
             println("Goal Attained! \n"); n.mpc.goal_reached=true;
             break;
          end
        end
    end
end

if !isinteractive()
    main()
end
