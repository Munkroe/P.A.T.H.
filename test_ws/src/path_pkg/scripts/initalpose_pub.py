#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Time

rospy.init_node('initialpose_publisher', anonymous = True)
initialpose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size = 50)

initPose = PoseWithCovarianceStamped()

initPose.header.frame_id = "map"

initPose.pose.covariance = [	0.25, 	0.0, 	0.0, 	0.0, 	0.0, 	0.0,
				0.0, 	0.25, 	0.0, 	0.0, 	0.0, 	0.0,
				0.0, 	0.0, 	0.0, 	0.0, 	0.0, 	0.0,
				0.0, 	0.0, 	0.0, 	0.0, 	0.0, 	0.0,
				0.0, 	0.0, 	0.0, 	0.0, 	0.0, 	0.0,
				0.0, 	0.0, 	0.0, 	0.0, 	0.0, 	0.06853892326654787]


initPose.pose.pose.position.x = 2.28544
initPose.pose.pose.position.y = 8.96732
initPose.pose.pose.position.z = 0

initPose.pose.pose.orientation.x = 0
initPose.pose.pose.orientation.y = 0
initPose.pose.pose.orientation.z = -0.99999
initPose.pose.pose.orientation.w = 0.002528

rospy.sleep(5.)
initialpose_pub.publish(initPose)


