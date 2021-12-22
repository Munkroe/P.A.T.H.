#!/usr/bin/python3
import rospy
import std_msgs.msg 
import tf
import tf.transformations
import struct
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Quaternion, Pose
from nav_msgs.msg import Odometry
import gazebo_msgs.msg
from gazebo_msgs.srv import GetLinkState
import math
import struct
import os

odomPose = None

odom_stamp = 0.0
posX = 0.
posY = 0.
rotZ = 0.

cmd_stamp = 0.
cmdVelX = 0.
cmdVelPhi = 0.

updateFreq = 1/10

def odomBedCallback(data):
    global odom_stamp, odomPose

    odom_stamp = data.header.stamp
    odomPose = data.pose.pose
    
def cmdVelCallback(data):

    global cmdVelX, cmdVelPhi
    cmd_stamp = rospy.Time.now()
    cmdVelX = data.linear.x
    cmdVelPhi = data.angular.z

    print("CMD  - Time: " + str(cmd_stamp) + ", velX: " + str(cmdVelX) + ", velPhi: " + str(cmdVelPhi))

    printOdom()
    

def calcData():
    global posX, posY, rotZ, odomPose
    if odomPose == None: return
    posX = odomPose.position.x
    posY = odomPose.position.y

    rotZ = (180/math.pi)*tf.transformations.euler_from_quaternion([odomPose.orientation.x,odomPose.orientation.y,odomPose.orientation.z,odomPose.orientation.w])[2] % 360

def printOdom():
    global odom_stamp, posX, posY, rotZ
    print("ODOM - Time: " + str(odom_stamp) + ", Pos X: " + str(posX) + ", Pos Y: " + str(posY) + ", Rot Z: " + str(rotZ))


def node_run():
    global odom_stamp, posX, posY, rotZ, odomPose

    rospy.init_node('odom_extract_node')
    rospy.loginfo("Init")

    rospy.Subscriber('cmd_vel/path', Twist, cmdVelCallback) # Modtag commands til sengen
    rospy.Subscriber('odomSim', Odometry, odomBedCallback) # Modtag odom for sengen
    
    sleeper = rospy.Rate(updateFreq)

    printOdom()

    while not rospy.is_shutdown():
    
        calcData()

        sleeper.sleep()


if __name__ == '__main__':
    try:
        node_run()
    except rospy.ROSInterruptException:
        pass