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

# Output cmdVel
duck1VelX = 0.
duck1VelPhi = 0.
duck2VelX = 0.
duck2VelPhi = 0.

# Bed odom
#odomBedPosX = 0.
#odomBedPosY = 0.
odomBedPhi = 0.

# Duck1 odom
odomDuck1PosX = 0.
odomDuck1PosY = 0.
odomDuck1PhiLocal = 0.

# Duck2 odom
odomDuck2PosX = 0.
odomDuck2PosY = 0.
odomDuck2PhiLocal = 0.

# Sengens længde
bedLength = 1.4 + 2 * 0.111 # Sengelængde plus robot offset
bedLengthHalf = bedLength/2

phiDeltaMax = 5*(math.pi/180)/2#5*(math.pi/180)/2 # 5 graders præcision
velXmax = 1

# Rotationshastigheder når retningen skal rettes
turnVelPhiMin = 0.1
turnVelPhiFactor = 0.5
minPhiDist = 1*(math.pi/180)/2 #1 grads præcision
turnVelPhiCorrectionFactor = turnVelPhiMin / minPhiDist
updateFreq = 100

bedCmdVelX = 0
bedCmdVelPhi = 0

Link_state = None
def calcDistToGoal(orientR, orientG):
    diff = orientG-orientR
    dist = diff
    if math.fabs(diff + 2*math.pi) < math.fabs(dist): dist = diff + 2*math.pi
    if math.fabs(diff - 2*math.pi) < math.fabs(dist): dist = diff - 2*math.pi
    return dist

def getRobotOdometry():
    global odomDuck1PosX, odomDuck1PosY, odomDuck1PhiLocal, odomDuck2PosX, odomDuck2PosY, odomDuck2PhiLocal

    # Hent robot odometri
    try:
        duck1_pose = Link_state('duck1', 'PATH').link_state.pose # Requester data, som er pose og twist fra linket duck1 fra referencen PATH, der er robotten.
        odomDuck1PosX = duck1_pose.position.x
        odomDuck1PosY = duck1_pose.position.y
        odomDuck1PhiLocal = tf.transformations.euler_from_quaternion([duck1_pose.orientation.x, duck1_pose.orientation.y, duck1_pose.orientation.z, duck1_pose.orientation.w])[2]
    except rospy.ServiceException as exc:
        rospy.loginfo("FAIL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        print("Link states service did not process request correctly: " + str(exc))

    try:
        duck2_pose = Link_state('duck2', 'PATH').link_state.pose # Requester data, som er linket duck2 fra referencen PATH, der er robotten.
        odomDuck2PosX = duck2_pose.position.x
        odomDuck2PosY = duck2_pose.position.y
        odomDuck2PhiLocal = tf.transformations.euler_from_quaternion([duck2_pose.orientation.x, duck2_pose.orientation.y, duck2_pose.orientation.z, duck2_pose.orientation.w])[2]
    except rospy.ServiceException as exc:
        rospy.loginfo("FAIL!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        print("Link states service did not process request correctly: " + str(exc))

def cmdVelCallback(data):

    # Hent kommandoer til sengen i form af en twist besked. 
    # Vi er kun interesserede i lineær hastighed i x-retningen og rotationel hastighed om z-aksen

    global bedCmdVelX, bedCmdVelPhi
    bedCmdVelX = data.linear.x
    bedCmdVelPhi = data.angular.z
       

def sendRobotCmd(duck1_pub, duck2_pub):

    # Publish twist instruktioner til de to topics som robotterne lytter på.

    duck1Msg = Twist()      
    duck2Msg = Twist()

    duck1Msg.linear.x = duck1VelX
    duck1Msg.angular.z = duck1VelPhi

    duck1_pub.publish(duck1Msg)

    duck2Msg.linear.x = duck2VelX
    duck2Msg.angular.z = duck2VelPhi
    duck2_pub.publish(duck2Msg)

def calcRobotCmd(bedCmdVelX, bedCmdVelPhi):

    #Udregn hvordan robotterne skal køre, afhængig af twist instruktionerne til sengen.

    global duck1VelX, duck1VelPhi, duck2VelX, duck2VelPhi, odomBedPhi, odomDuck1PhiLocal, odomDuck2PhiLocal
    #rospy.loginfo("cmdVelX: " + str(bedCmdVelX) + " cmdVelPhi: " + str(bedCmdVelPhi))

    # Korrigerer for at robotternes odometri er fastsat efter sengens, ved at lægge dens vinkel til. 
    odomDuck1Phi = odomBedPhi + odomDuck1PhiLocal
    odomDuck2Phi = odomBedPhi + odomDuck2PhiLocal

    alpha = 0

    if bedCmdVelPhi != 0:
        # Sengen skal dreje

        # Udregn radius for sengens bevægelsescirkel
        Rs = bedCmdVelX / bedCmdVelPhi

        # Udregn radius for robotternes bevægelsescirkel
        Rr = math.sqrt(Rs*Rs+bedLengthHalf*bedLengthHalf)

        # Udregn robotternes vinkel-offset ift. sengen i bevægelsescirkel
        alpha = math.acos(Rs/Rr)
        #rospy.loginfo("Rs: " + str(Rs) + ", Rr: " + str(Rr) + ", Alpha: " + str(alpha))

    # Udregn robotternes ønskede retninger
    #rospy.loginfo("BedPhi: " + str(odomBedPhi))
    cmdPhi1 = odomBedPhi + alpha
    cmdPhi2 = odomBedPhi - alpha

    # Udregn den korteste afstand og retning mellem nuværende vinkel og goal vinkel.
    duck1PhiDist = calcDistToGoal(odomDuck1Phi, cmdPhi1)
    duck2PhiDist = calcDistToGoal(odomDuck2Phi, cmdPhi2)

    
    #rospy.loginfo("duck1PhiDist: " + str(duck1PhiDist))
    #rospy.loginfo("duck2PhiDist: " + str(duck2PhiDist))

    # Roter om sig selv for at rette robotten op
    if math.fabs(duck1PhiDist) > phiDeltaMax or math.fabs(duck2PhiDist) > phiDeltaMax:
        
        if math.fabs(duck1PhiDist) <= minPhiDist:
            duck1VelPhi = 0 #duck1PhiDist * turnVelPhiCorrectionFactor
        elif math.fabs(turnVelPhiFactor * duck1PhiDist) > turnVelPhiMin:
            duck1VelPhi = turnVelPhiFactor * duck1PhiDist
        else: 
            duck1VelPhi = (turnVelPhiMin if duck1PhiDist > 0 else -turnVelPhiMin)

        if math.fabs(duck2PhiDist) <= minPhiDist:
            duck2VelPhi = 0 #duck1PhiDist * turnVelPhiCorrectionFactor
        elif math.fabs(turnVelPhiFactor * duck2PhiDist) > turnVelPhiMin:
            duck2VelPhi = turnVelPhiFactor * duck2PhiDist
        else: 
            duck2VelPhi = (turnVelPhiMin if duck2PhiDist > 0 else -turnVelPhiMin)

        duck1VelX = 0
        duck2VelX = 0
        #rospy.loginfo("Turn: " + str(duck1VelPhi) + ", " + str(duck2VelPhi))
        
        return

    # Udregn cmd translations- og rotationshastigheder for robotterne
    duck1VelPhi = bedCmdVelPhi #+ duck1PhiDist * turnVelPhiCorrectionFactor
    duck2VelPhi = bedCmdVelPhi #+ duck2PhiDist * turnVelPhiCorrectionFactor

    if (bedCmdVelPhi == 0):
        duck1VelX = bedCmdVelX
        duck2VelX = bedCmdVelX
    else:
        duck1VelX = duck1VelPhi * Rr
        duck2VelX = duck2VelPhi * Rr

    #rospy.loginfo("Duck1: velX = " + str(duck1VelX) + " velPhi = " + str(duck1VelPhi))
    #rospy.loginfo("Duck2: velX = " + str(duck2VelX) + " velPhi = " + str(duck2VelPhi))
    
    # Check om udregnede hastigheder er højere end tilladt
    if math.fabs(duck1VelX) > velXmax or math.fabs(duck2VelX) > velXmax:
        
        # Udregn skaleringsfaktor
        k = velXmax / duck1VelX

        # Udregn skalerede translations- og rotationshastigheder for robotterne
        duck1VelPhi = bedCmdVelPhi * k
        duck2VelPhi = duck1VelPhi
        duck1VelX = bedCmdVelX * k
        duck2VelX = duck1VelX
        rospy.loginfo("Scale: " + str(k))
    

def odomBedCallback(data):
    global odomBedPhi

    odomBedPhi = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])[2]

def node_run():
    rospy.init_node('robot_control_node')
    rospy.loginfo('Control node initiated')

    # Hent robot odom
    rospy.wait_for_service('gazebo/get_link_state') #Venter på at servicen er ledig før den lader en gå videre.

    global Link_state
    Link_state = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState) #Laver et objekt, der forbinder til servicen og beskedtypen GetLinkState

    rospy.Subscriber('cmd_vel/path', Twist, cmdVelCallback) # Modtag commands til sengen
    rospy.Subscriber('odomSim', Odometry, odomBedCallback) # Modtag odom for sengen
    
    duck1_pub = rospy.Publisher('cmd_vel/duck1', Twist, queue_size=50) # Send commands til duck1
    duck2_pub = rospy.Publisher('cmd_vel/duck2', Twist, queue_size=50) # Send commands til duck2
    
    sleeper = rospy.Rate(updateFreq)

    global bedCmdVelX

    while not rospy.is_shutdown():
        # Udregn robot commands fra cmdVel for sengen

        getRobotOdometry()

        calcRobotCmd(bedCmdVelX, bedCmdVelPhi)

        sendRobotCmd(duck1_pub, duck2_pub)

        sleeper.sleep()




if __name__ == '__main__':
    try:
        node_run()
    except rospy.ROSInterruptException:
        pass