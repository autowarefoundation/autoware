#!/usr/bin/env python

import rospy
from jsk_recognition_msgs.msg import PlotData
import csv

counter = 0
def callback(msg):
    global counter
    rospy.loginfo("writing to %s" % (filename % counter))
    with open(filename % counter, "w") as f:
        writer = csv.writer(f, delimiter=',')
        writer.writerow(["x", "y"])
        for x, y in zip(msg.xs, msg.ys):
            writer.writerow([x, y])
    rospy.loginfo("done")
    counter = counter + 1

if __name__ == "__main__":
    rospy.init_node("plot_data_to_csv")
    filename = rospy.get_param("~filename", "output_%04d.csv")
    sub = rospy.Subscriber("~input", PlotData, callback, queue_size=1)
    rospy.spin()
