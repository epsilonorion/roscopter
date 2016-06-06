#!/usr/bin/env python
# If looking at transform in RVIZ
# Red - x-axis (Roll About)
# Green - y-axis (Pitch About)
# Blue - z-axis (Yaw About)

import roslib; roslib.load_manifest('roscopter')
import rospy

import string
import math

from time import time
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from roscopter.msg import *
import tf
# ROSCopter Messages are in NED, so imu data must be converted to ENU
# To rotate RPY, the following may be used
#   RPY(ENU) = [0 1 0; 1 0 0; 0 0 -1] [RPY](NED)
# To convert the quaternion ENU = NED
#   w = w, x = y, y = x, z = -z
# To convert acceleration ENU = NED
#   x = y, y = x, z = -z
# To convert velocity ENU = NED
#   x = y, y = x, z = -z

grad2rad = 3.141592/180.0

# Initial IMU Message holder with generic covariances
imuMsg = Imu()
imuMsg.orientation_covariance = [999999 , 0 , 0,
0, 9999999, 0,
0, 0, 999999]
imuMsg.angular_velocity_covariance = [9999, 0 , 0,
0 , 99999, 0,
0 , 0 , 0.02]
imuMsg.linear_acceleration_covariance = [0.2 , 0 , 0,
0 , 0.2, 0,
0 , 0 , 0.2]

# Initial RPY values from Attitude Callback
roll=0
pitch=0
yaw=0
attitude_received_flag=0

pub = rospy.Publisher('imu', Imu, queue_size=10)

def attitude_callback(msg):
    global yaw, pitch, roll, attitude_received_flag

    # Convert RPY from NED to ENU
    yaw = -float(msg.yaw)
    pitch = float(msg.pitch)
    roll = float(msg.roll)

    if (attitude_received_flag==0):
        attitude_received_flag=1

def imu_callback(msg):
    # Form IMU Message

    # Convert Acceleration and Velocity from NED to ENU
    imuMsg.linear_acceleration.x = float(msg.xacc) / 1000 * 9.80665
    imuMsg.linear_acceleration.y = float(msg.yacc) / 1000 * 9.80665
    imuMsg.linear_acceleration.z = -float(msg.zacc) / 1000 * 9.60665
           
    imuMsg.angular_velocity.x = float(msg.xgyro) / 1000
    imuMsg.angular_velocity.y = float(msg.ygyro) / 1000
    imuMsg.angular_velocity.z = -float(msg.zgyro) / 1000
            
    # Convert RPY values from Attitude to Quaternion
    q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]
    imuMsg.header = Header()
    imuMsg.header.stamp= rospy.Time.now()
    imuMsg.header.frame_id = 'imu'

    # If Attitude has been received, publish IMU Message and Transform
    if (attitude_received_flag):
        pub.publish(imuMsg)

        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0),
                     q,
                     rospy.Time.now(),
                     "imu",
                     "base_link")

        #print("Transform with (%f, %f, %f)" % (roll,  pitch,  yaw))
        #print("Transform with (%f, %f, %f, %f)" % (q[0], q[1], q[2],  q[3]))

if __name__ == '__main__':
    rospy.init_node("imupub")
    rospy.Subscriber("raw_imu", Mavlink_RAW_IMU, imu_callback)
    rospy.Subscriber("attitude", Attitude, attitude_callback)
    rospy.spin()
