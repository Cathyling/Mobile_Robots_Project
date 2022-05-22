#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import TwistWithCovariance, PoseWithCovarianceStamped, Twist, Vector3
from nav_msgs.msg import Odometry


#vals = [-1,0,0,0,0,0,0,0,0]

vals = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]

def callback(msg):
    odom = Odometry()
    odom.header = msg.header
    odom.child_frame_id = "0"
    odom.pose = msg.pose
    
    odom.twist = TwistWithCovariance()
    odom.twist.twist = Twist()
   
    pub.publish(odom)

rospy.init_node("covariance")
sub = rospy.Subscriber("/odom_wrong", PoseWithCovarianceStamped, callback)
pub = rospy.Publisher("/robot_pose_ekf/odom_combined", Odometry, queue_size=2)

rospy.spin()
