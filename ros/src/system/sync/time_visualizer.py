#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from synchronization.msg import time_monitor

def callback(data):
    rospy.loginfo(rospy.get_caller_id()+"I heardaa")

def visualizer():
    x = np.arange(-3,3,0.1)
    y = np.sin(x)
    plt.draw()
    time.sleep(0.05)

    plt.plot(x, y)
    plt.axis([0, 1000, 0, 1])
    plt.ion()
    plt.show()

    for i in range(1000):
        y = np.random.random()
        plt.scatter(i, y)

    rospy.init_node('time_visualizer', anonymous=False)
    rospy.Subscriber("/synchronization/obj_car/times", time_monitor, callback)
    rospy.spin()

if __name__ == '__main__':
    visualizer()
