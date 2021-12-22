#!/usr/bin/env python3

import rospy
import std_msgs
import time

rospy.init_node('clock_publisher', anonymous = True)
clock_pub = rospy.Publisher("clock", std_msgs.msg.Time, queue_size = 50)

r = rospy.Rate(20)
while not rospy.is_shutdown():

    clock_pub.publish(rospy.get_rostime())

    r.sleep()




