#!/usr/bin/python3
from threading import current_thread
import rospy
from rospy.core import loginfo
import  std_msgs.msg
import serial
import tf
import tf.transformations
import struct
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, PoseWithCovarianceStamped, Quaternion, Twist, Vector3
import math
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_about_axis
import numpy as np

ORIENTID = 3
MOTORID = 2
ACCEL_GYROID = 4
DISTBETWHEEL = 0.370
WHEELDIA = 0.086

dirOrientation = 0.0
orientAngle = 0.0
START_DEL = 36
STOP_DEL = 64
ESCAPE_DEL = 35
posX = 0.
posY = 0.
posPhi = 0.
posX_prev = 0.
posY_prev = 0.
posPhi_prev = 0.
velX = 0.
velY = 0.
velPhi = 0.
odom_X = 0.
odom_Y = 0.
odom_Phi = 0.
current_time = std_msgs.msg.Time()
imu_broadcaster = tf.TransformBroadcaster()
accel_x = 0
accel_y = 0
accel_z = 9.82
gyro_z = 1
gyro_x = 0
gyro_y = 0

IMU_FRAME = None

input = " "
ser=serial.Serial('/dev/ttyACM0',500000, timeout=None)

def encodeStuffing(data):
    package = b''
    package += START_DEL.to_bytes(1, byteorder='big')
    i = 0
    while i < len(data):
        
        if data[i] == START_DEL or data[i] == STOP_DEL or data[i] == ESCAPE_DEL or data[i] == b'0':
            package += ESCAPE_DEL.to_bytes(1, byteorder='big')
            package += (data[i] + 2).to_bytes(1, byteorder='big')
        else:
            package += data[i].to_bytes(1, byteorder='big')

        i = i + 1
        

    package += STOP_DEL.to_bytes(1, byteorder='big')
    return package


def startUpRoutine():
    pkgToStuff = b''
    pkgToStuff += (114).to_bytes(1, byteorder='big')
    pkgToStuff += (101).to_bytes(1, byteorder='big')
    pkgToStuff += (115).to_bytes(1, byteorder='big')
    pkgToStuff += (101).to_bytes(1, byteorder='big')
    pkgToStuff += (116).to_bytes(1, byteorder='big')
    pkg_to_send = encodeStuffing(pkgToStuff)
    ser.write(pkg_to_send)




def decodeStuffing(data):
    package = b''
    i=0
    while i < len(data):
        if data[i].to_bytes(1, 'big') != b'#':
            package += data[i].to_bytes(1, 'big')
        else:
            temp = data[i + 1] - 2
            package += temp.to_bytes(1, 'big')
            i = i+1
        i = i+1
    
    return package

def decode(raw_data):
    for idx, byte in enumerate(raw_data):
        if byte == START_DEL:
            
            slice_object = slice(idx+1, len(raw_data)-1) # skærer både start og slut del af
            short_package = raw_data[slice_object]
            #her fra har vi den rene besked
            pack = decodeStuffing(short_package)
            if pack[0] == MOTORID:
                if len(pack) == 25:
                    posX_temp = struct.unpack('f', pack[1:5])
                    posY_temp = struct.unpack('f', pack[5:9])
                    posPhi_temp = struct.unpack('f', pack[9:13])
                    velX_temp = struct.unpack('f', pack[13:17])
                    velY_temp = struct.unpack('f', pack[17:21])
                    velPhi_temp = struct.unpack('f', pack[21:25])
                    global posX, posY, posPhi, velX, velY, velPhi
                    posX = float('.'.join(str(elem) for elem in posX_temp))
                    posY = float('.'.join(str(elem) for elem in posY_temp))
                    posPhi = float('.'.join(str(elem) for elem in posPhi_temp))
                    velX = float('.'.join(str(elem) for elem in velX_temp))
                    velY = float('.'.join(str(elem) for elem in velY_temp))
                    velPhi = float('.'.join(str(elem) for elem in velPhi_temp))
                    #rospy.loginfo("posx: " + str(posX) + " posy: " + str(posY) + " posPhi: " + str(posPhi) + " L: " + str(velX) + " R: " + str(velY))
                else:
                    rospy.loginfo("Flawed motor data")

		
            if pack[0] == ORIENTID:
                if len(pack) == 6:
                    global orientAngle, dirOrientation
                    orientAngle_temp = struct.unpack('f', pack[2:6]) #Naeste 4 bytes er en float, der indeholder vinklen til orientation (fra 1-360 grader)
                    dirOrientation = pack[1] #Ved ikke om det her bare virker.
                    orientAngle = float('.'.join(str(elem) for elem in orientAngle_temp)) #Pak det ud igen til en decimalvaerdi
                else:
                    rospy.loginfo("Flawed orient data")

            if pack[0] == ACCEL_GYROID:
                if len(pack) == 13:
                    accel_x_temp = struct.unpack('f', pack[1:5])
                    accel_y_temp = struct.unpack('f', pack[5:9])
                    gyro_z_temp = struct.unpack('f', pack[9:13])
                    global accel_x, accel_y, gyro_z
                    accel_x = (float('.'.join(str(elem) for elem in accel_x_temp)))*9.82
                    accel_y = (float('.'.join(str(elem) for elem in accel_y_temp)))*9.82
                    gyro_z = (float('.'.join(str(elem) for elem in gyro_z_temp)))*(math.pi/(180))
                else:
                    rospy.loginfo("Flawed IMU data")
            
            break


def callback(data):
    current_time.data = data.data


def poseCallback(data):
    global odom_X, odom_Y, odom_Phi
    odom_X = data.pose.pose.position.x
    odom_Y = data.pose.pose.position.y
    quatx = data.pose.pose.orientation.x
    quaty = data.pose.pose.orientation.y
    quatz = data.pose.pose.orientation.z
    quatw = data.pose.pose.orientation.w
    odom_Phi = tf.transformations.euler_from_quaternion([quatx, quaty, quatz, quatw])


def navToRads(data):
    motorVelX = data.linear.x
    motorVelPhi = data.angular.z
    rospy.loginfo(str(motorVelX))
    rospy.loginfo(str(motorVelPhi))

    # D_l = motorVelX/(math.cos(motorVelPhi+odom_Phi)*(1+(DISTBETWHEEL*motorVelPhi)/2))
    # D_r = DISTBETWHEEL*motorVelPhi+motorVelX/(math.cos(motorVelPhi+odom_Phi)*(1+(DISTBETWHEEL*motorVelPhi)/2))
    # speedL = (D_l/(math.pi*WHEELDIA))*2*math.pi
    # speedR = (D_r/(math.pi*WHEELDIA))*2*math.pi
    speedR = (motorVelX/(math.pi*WHEELDIA))*2*math.pi+motorVelPhi*(DISTBETWHEEL/WHEELDIA)
    speedL = (motorVelX/(math.pi*WHEELDIA))*2*math.pi-motorVelPhi*(DISTBETWHEEL/WHEELDIA)
    TransmitWheelVelocityCmd(speedR, speedL)


def TransmitWheelVelocityCmd(valueR, valueL):
    byteFloatR = bytearray(struct.pack("f", valueR)) 
    byteFloatL = bytearray(struct.pack("f", valueL)) 
    pkgToStuff = b''
    pkgToStuff += (82).to_bytes(1, byteorder='big')
    pkgToStuff += byteFloatR
    pkgToStuff += (76).to_bytes(1, byteorder='big')
    pkgToStuff += byteFloatL
    pkg_to_send = encodeStuffing(pkgToStuff)
    rospy.loginfo(str(pkg_to_send))
    ser.write(pkg_to_send)

def node_run(): #Funktion der koerer orientation odometry
    orient_odom_pub = rospy.Publisher('orientData', String, queue_size=50)
    motor_odom_pub = rospy.Publisher('odom', Odometry , queue_size=50)
    imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=50)
    rospy.init_node('orient_motor_odometry', anonymous=True)
    rospy.Subscriber('clock', std_msgs.msg.Time, callback)
    #rospy.Subscriber('robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, poseCallback)
    rospy.Subscriber('/cmd_vel', Twist, navToRads)
    rospy.loginfo("Orient monitor DUCK1 started")
    odom_broadcaster = tf.TransformBroadcaster()
    startUpRoutine()
    IMU_FRAME = rospy.get_param('~imu_frame', 'imu')
    while not rospy.is_shutdown():
        input = ser.read_until(b'@')
        
    
        if len(input) > 6:                         #Hvis længden af input er 24*2 mulige bytes + stuffed bytes + 1 ID + 2 delimiters = 51
            								   #Og minimum = 24+1+2 = 27
            decode(input)
            #orient_odom_pub.publish(str(dirOrientation))
            orient_odom_pub.publish(str(orientAngle))

            
            #DET HER ER MOTORDATA PUBLISHER KOED
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, posPhi)
            #odom_broadcaster.sendTransform(
            #    (posX, posY, 0),
            #    odom_quat,
            #    current_time.data,
            #    "base_link",
            #    "odom"
            #)
        

            odom = Odometry()
            odom.header.stamp = current_time.data
            odom.header.frame_id = "odom"
            

            #Position
            odom.pose.pose = Pose(Point(posX, posY, 0), Quaternion(*odom_quat))
            odom.pose.covariance =[ 0.001,    0.0,    0.0,    0.0,    0.0,    0.0,
                                    0.0,    0.001,    0.0,    0.0,    0.0,    0.0,
                                    0.0,    0.0,    9999.0,    0.0,    0.0,    0.0,
                                    0.0,    0.0,    0.0,    9999.0,    0.0,    0.0,
                                    0.0,    0.0,    0.0,    0.0,    9999.0,    0.0,
                                    0.0,    0.0,    0.0,    0.0,    0.0,    0.000001]

            #velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(velX, velY, 0), Vector3(0, 0, velPhi))
            odom.twist.covariance =[ 0.001,    0.0,    0.0,    0.0,    0.0,    0.0,
                                    0.0,    0.001,    0.0,    0.0,    0.0,    0.0,
                                    0.0,    0.0,    9999.0,    0.0,    0.0,    0.0,
                                    0.0,    0.0,    0.0,    9999.0,    0.0,    0.0,
                                    0.0,    0.0,    0.0,    0.0,    9999.0,    0.0,
                                    0.0,    0.0,    0.0,    0.0,    0.0,    0.000001]

            motor_odom_pub.publish(odom)



            # Calculate a quaternion representing the orientation
            # accel = accel_x, accel_y, accel_z
            # ref = np.array([0, 0, 1])
            # acceln = accel / np.linalg.norm(accel)
            # axis = np.cross(acceln, ref)
            # angle = np.arccos(np.dot(acceln, ref))
            # orientation = quaternion_about_axis(angle, axis)


            imu_msg = Imu()
            imu_msg.header.frame_id = IMU_FRAME

            # o = imu_msg.orientation
            # o.x, o.y, o.z, o.w = orientation
            imu_msg.orientation_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]

            imu_msg.linear_acceleration.x = accel_x
            imu_msg.linear_acceleration.y = accel_y
            imu_msg.linear_acceleration.z = accel_z
            imu_msg.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 9999]

            imu_msg.angular_velocity.x = gyro_x
            imu_msg.angular_velocity.y = gyro_y
            imu_msg.angular_velocity.z = gyro_z
            imu_msg.angular_velocity_covariance = [9999, 0.0, 0.0, 0.0, 9999, 0.0, 0.0, 0.0, 0.1]

            imu_msg.header.stamp = current_time.data

            imu_pub.publish(imu_msg)

            # imu_broadcaster.sendTransform(
            #     (1, 2, 0),
            #     (o.x, o.y, o.z, o.w),
            #     current_time.data,
            #     "imu2",
            #     "base_link2"
            # )
            
            input = ""
        
if __name__ == '__main__':
    try:
        node_run()
    except rospy.ROSInterruptException:
        pass
        
        
        
        
