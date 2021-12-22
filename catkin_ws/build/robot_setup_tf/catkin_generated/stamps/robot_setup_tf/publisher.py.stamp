#!/usr/bin/env+

import rospy
from std_msgs.msg import String

def publishMethod():
	pub = rospy.Publisher('randomTopic', String, queue_size=10)
	rospy.init_node('publisher_Node', anonymous=True)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		publishString = "Hej Sophus"
		rospy.loginfo("msg has been sent")
		pub.publish(publishString)
		rate.sleep()

if __name__ == '__main__':
	try:
		publishMethod()
	except rospy.ROSInterruptException:
		pass
	
	
