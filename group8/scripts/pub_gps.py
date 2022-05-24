#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

cov = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

#pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
pub = rospy.Publisher("gps_coord", NavSatFix, queue_size=10)
rospy.init_node('pub_gps')




rate = rospy.Rate(1)

while not rospy.is_shutdown():
    goal = NavSatFix()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "base_link"
    goal.latitude = -31.9804702
    goal.longitude = 115.8171341
    goal.altitude = 18.0857863
    goal.position_covariance=cov
    pub.publish(goal)
    rate.sleep()

rospy.spin()