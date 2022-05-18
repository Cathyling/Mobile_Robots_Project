#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


#vals = [-1,0,0,0,0,0,0,0,0]

vals = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]

def callback(msg):
    msg.orientation_covariance = vals
    msg.orientation.x = 1
    pub.publish(msg)

rospy.init_node("covariance")
sub = rospy.Subscriber("/imu/data_raw", Imu, callback)
pub = rospy.Publisher("/imu/data", Imu, queue_size=2)

rospy.spin()