#!/usr/bin/env julia

using RobotOS
@rosimport geometry_msgs.msg: Point, Pose2D
rostypegen()
using geometry_msgs.msg


function main()
    init_node("rosjl_example")
    const pub = Publisher{Point}("pts", queue_size=10)

    loop_rate = Rate(2.0)
    while ! is_shutdown()
        for i in 1:10000
            npt = Point(rand(), i, 0.0)
            publish(pub, npt)
            rossleep(loop_rate)
        end
    end

end

if ! isinteractive()
    main()
end
