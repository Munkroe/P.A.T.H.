import tf

if __name__ == '__main__':
    try:
        node_run()
    except rospy.ROSInterruptException:
        pass

def node_run():
    init_node()
    while not rospy.is_shutdown():

def init_node():
    listener = tf.TransformListener()



