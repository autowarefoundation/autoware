#!/usr/bin/env julia
using RobotOS
@rosimport obstacle_detector.msg: Obstacles, CircleObstacle
rostypegen()
using obstacle_detector.msg

function setObstacleParams(msg::Obstacles)
  L = length(msg.circles)

  if L > 0
    r=zeros(L);x=r;y=r;vx=r;vy=r;
    r = (); x = (); y = (); vx = (); vy = ();
    for i in 1:L
      r = (r..., msg.circles[i].radius)
      x = (x..., msg.circles[i].center.x)
      y = (y..., msg.circles[i].center.y)
      vx = (vx..., msg.circles[i].velocity.x)
      vy = (vy..., msg.circles[i].velocity.y)
    end

    # update obstacle field parameters
    RobotOS.set_param("obstacle_radius",r)
    RobotOS.set_param("obstacle_x",x)
    RobotOS.set_param("obstacle_y",y)
    RobotOS.set_param("obstacle_vx",vx)
    RobotOS.set_param("obstacle_vy",vy)
  else
    RobotOS.set_param("obstacle_radius",NaN)
    RobotOS.set_param("obstacle_x",NaN)
    RobotOS.set_param("obstacle_y",NaN)
    RobotOS.set_param("obstacle_vx",NaN)
    RobotOS.set_param("obstacle_vy",NaN)
  end

  return nothing
end

# 2 constant static obstacles
function setConstObstacleParams(msg::Obstacles)
  L = 2
  radius=[1 1]
  center_x=[200 250]
  center_y=[50 50]
  velocity_x=[0 0]
  velocity_y=[0 0]
    r = (); x = (); y = (); vx = (); vy = ();
    for i in 1:L
      r = (r..., radius)
      x = (x..., center_x)
      y = (y..., center_y)
      vx = (vx..., velocity_x)
      vy = (vy..., velocity_y)
    end

    # update obstacle field parameters
    RobotOS.set_param("obstacle_radius",r)
    RobotOS.set_param("obstacle_x",x)
    RobotOS.set_param("obstacle_y",y)
    RobotOS.set_param("obstacle_vx",vx)
    RobotOS.set_param("obstacle_vy",vy)

<<<<<<< HEAD

  return nothing
end
=======
  return nothing
end
"""
# at the begining of the simulation assign the initial state given in the YAML file
# to the initial state that will be updated again and again
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/28/2018, Last Modified: 2/28/2018 \n
--------------------------------------------------------------------------------------\n
"""
function setInitObstacleParams(c)
  L = 2
  radius=[1 1]
  center_x=[200 250]
  center_y=[50 50]
  velocity_x=[0 0]
  velocity_y=[0 0]
    r = (); x = (); y = (); vx = (); vy = ();
    for i in 1:L
      r = (r..., radius)
      x = (x..., center_x)
      y = (y..., center_y)
      vx = (vx..., velocity_x)
      vy = (vy..., velocity_y)
    end

    # update obstacle field parameters
    RobotOS.set_param("obstacle_radius",r)
    RobotOS.set_param("obstacle_x",x)
    RobotOS.set_param("obstacle_y",y)
    RobotOS.set_param("obstacle_vx",vx)
    RobotOS.set_param("obstacle_vy",vy)

  return nothing
end

>>>>>>> beb641d592c51720bff87d7d20a594fcc17c54a8
init_node("obstacle_params")
sub = Subscriber{Obstacles}("/obstacles", setConstObstacleParams, queue_size = 10)
spin()
