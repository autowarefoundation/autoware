#!/usr/bin/python

import rospy
import math
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist, TwistStamped

class Convert:
    def __init__(self):
        self.publisher = rospy.Publisher('/sd_control/command', AckermannDriveStamped, queue_size=10)
        self.max_steering = 1
        self.min_steering = -1
        self.epsilon_steering = math.radians(0.001)

    def callbackTwist(self, dataTwist):
        steering= dataTwist.angular.z
        speed = dataTwist.linear.x
        self.Command_output(steering, speed)

    def callbackTwistStamped(self, dataTwistStamped):
        steering= dataTwistStamped.twist.angular.z
        speed = dataTwistStamped.twist.linear.x
        self.Command_output(steering, speed)

    def Command_output(self, steering, speed):
        ack_cmd = AckermannDriveStamped()
        ack_cmd.header.stamp = rospy.Time.now()

        drive = AckermannDrive()

        drive.speed = speed
        drive.steering_angle = steering

        # impose limits on commanded angle
        if drive.steering_angle > self.max_steering:
            drive.steering_angle = self.max_steering
        if drive.steering_angle < self.min_steering:
            drive.steering_angle = self.min_steering

        # clean up angle if it is very close to zero
        if math.fabs(drive.steering_angle) < self.epsilon_steering:
            drive.steering_angle = 0.0

        ack_cmd.drive = drive
        self.publisher.publish(ack_cmd)

    def listener(self):
        rospy.Subscriber("/twist_cmd", TwistStamped, self.callbackTwistStamped)
        rospy.Subscriber("/cmd_vel", Twist, self.callbackTwist)
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node("Twist_to_Ackermann", anonymous=True)
        cnv = Convert()
        cnv.listener()
    except rospy.ROSInterruptException: pass
