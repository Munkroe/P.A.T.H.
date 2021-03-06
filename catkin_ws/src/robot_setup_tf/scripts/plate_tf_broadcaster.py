#!/usr/bin/env python3

import rospy
import tf
import std_msgs
import math
import tf.transformations
import struct
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
rospy.init_node('plate_tf_broadcaster')

plate_broadcaster = tf.TransformBroadcaster()

r = rospy.Rate(5)

radius = 0.1739 # Placement of centerpoint of main circle of LiDAR relative to center of the robot in meters
height = 0.1135 # Placement of center height of the LiDAR relative to the top surface of bottom plate in meters
current_time = std_msgs.msg.Time()
currentAngle = 0.0
currentDir = 0.0

def callback(data):
    current_time.data = data.data

def orientCallback(data):
    #if data == "1" or data == "-1":
     #   temp_dirData = str(data)
      #  global currentDir
       # currentDir = temp_dirData[7:len(temp_dirData)-1]
        #rospy.loginfo("Data: ")
    
    global currentAngle
    currentAngle_temp = float(data.data)
    currentAngle = (math.pi/180.0)*(currentAngle_temp)
        


def broadcastData():
    clock_sub = rospy.Subscriber('clock', std_msgs.msg.Time, callback)
    orient_sub = rospy.Subscriber('orientData', std_msgs.msg.String, orientCallback)
    while not rospy.is_shutdown():
        #rospy.loginfo('currentAngle: '+str(currentAngle))
        #rospy.loginfo('X er: '+str(radius*math.cos((math.pi/180)*float(currentAngle))))
        #rospy.loginfo('Y er: '+str(radius*math.sin((math.pi/180)*float(currentAngle))))
        #rospy.loginfo('Z er: '+str(height))
        
        # first, we'll publish the transform over tf
        plate_broadcaster.sendTransform(
#            (radius * math.cos(currentAngle), radius * math.sin(currentAngle), height),
	    (0,0,height),
            tf.transformations.quaternion_from_euler(0,0,(currentAngle)),
            current_time.data,
            "top_link",
            "base_link"
            )
        r.sleep()
    
if __name__ == '__main__':
    try:
        broadcastData()
    except rospy.ROSInterruptException:
        pass
