#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=10)
rospy.init_node('publish_goal')

goal = PoseStamped()
goal.header.stamp = rospy.Time.now()
goal.header.frame_id = 'base_link'
# goal.pose.position.y = -31.98047015036711
goal.pose.position.x = 1.0
# goal.pose.position.x = 115.817134126657
goal.pose.orientation.w = 1.0
pub.publish(goal)

rospy.spin()