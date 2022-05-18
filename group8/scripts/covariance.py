#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry


vals = [0.01,0,0,0,0,0,0,0.01,0,0,0,0,0,0,0.01,0,0,0,0,0,0,0.01,0,0,0,0,0,0,0.01,0,0,0,0,0,0,0.01]

def callback(msg):
    msg.pose.covariance = vals
    msg.twist.covariance = vals
    print(msg)
    pub.publish(msg)

rospy.init_node("covariance")
sub = rospy.Subscriber("/RosAria/pose", Odometry, callback)
pub = rospy.Publisher("/pose", Odometry, queue_size=2)



rospy.spin()