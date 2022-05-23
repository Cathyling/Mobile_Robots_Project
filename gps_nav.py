
rospy.Subscriber("odom_combined", PoseWithCovarianceStamped, update_origin)
# https://gist.github.com/clungzta/c102c88258e220442c04
import roslib
import rospy
from math import radians, cos, sin, asin, sqrt, pow, pi, atan2
import numpy as np
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

debug = False

latCur = 0.0
lonCur = 0.0

latWP = 0.0
lonWP = 0.0
altWP = 0.0
waypoints = {'home_sweet_home': {{'longtitude':115.8172200827809}, {'latitude':-31.98100166177344}, {'altitude':20.70200454005209}}}
earthRadius = 6371000.0 #Metres
currPosX = 0.0
currPosY = 0.0
currPosZ = 0.0

def haversineDistance(latCur, lonCur, latWP, lonWP): #Returns distance to waypoint in Metres
	latWP, lonWP, latCur, lonCur = map(radians, [latWP, lonWP, latCur, lonCur]) #Convert into Radians to perform math
	a = pow(sin((latWP - latCur)/2),2) + cos(latCur) * cos(latWP) * pow(sin((lonWP - lonCur)/2),2)
	return earthRadius * 2.0 * asin(sqrt(a))  #Return calculated distance to waypoint in Metres
	
def bearing(latCur, lonCur, latWP, lonWP): #Bearing to waypoint (degrees)
	latWP, lonWP, latCur, lonCur = map(radians, [latWP, lonWP, latCur, lonCur]) #Convert into Radians to perform math
	dLon = lonWP - lonCur
	return atan2(sin(dLon) * cos(latWP), cos(latCur) * sin(latWP) - (sin(latCur) * cos(latWP) * cos(dLon)))

def gpsSubscriber(gpsMsg): #GPS Coordinate recieved from ROS topic, run this function
    if gpsMsg.status.status > -1: #If there is a GPS fix (Either Augmented or Unaugmented)
        global latCur
        global lonCur
        global lastValidFixTime		
        lastValidFixTime = rospy.get_time()		
        latCur = gpsMsg.latitude
        lonCur = gpsMsg.longitude
        if debug == True:
            rospy.loginfo("GPS Fix Available, Latitude: %f, Longitude: %f", latCur, lonCur)
def robotPoseSubscriber(poseMsg): #Odometry update recieved from ROS topic, run this function
	global currPosX
	global currPosY
	global currPosZ

	currPosX = poseMsg.pose.pose.position.x
	currPosY = poseMsg.pose.pose.position.y
	currPosZ = poseMsg.pose.pose.position.z

def posePublisher(): #Convert absolute waypoint to vector relative to robot, then publish navigation goal to ROS

	global currPosX
	global currPosY
	global currPosZ
	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()
    distToWP = haversineDistance(latCur, lonCur, latWP, lonWP)
    bearingToWP = bearing(latCur, lonCur, latWP, lonWP)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = currPosX + (distToWP * cos(bearingToWP)) #Convert distance and angle to waypoint from Polar to Cartesian co-ordinates then add current position of robot odometry 
    goal.target_pose.pose.position.y = currPosY + (distToWP * sin(bearingToWP))
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    
    rospy.loginfo("Robot is heading %f metres at a bearing of %f degrees", distToWP, (bearingToWP  * 180/pi + 360) % 360)

def main():
    rospy.init_node('gps_2d_nav_goal', anonymous=True)
    rospy.loginfo("Initiating GPS 2D Nav Goal Node.")
    global latWP 
    global lonWP
    global altWP
    while not rospy.is_shutdown(): #While ros comms are running smoothly
        for wp_name, wp_point in waypoints:
            latWP= wp_point.latitude
            lonWP= wp_point.longtitude
            altWP= wp_point.altitude
        #rospy.Subscriber("waypoint", WayPoint, waypointSubscriber) #Subscribe to "pose", "fix" and "waypoint" ROS topics
            rospy.Subscriber("fix", NavSatFix, gpsSubscriber)
            rospy.Subscriber("odom_combined", PoseWithCovarianceStamped, robotPoseSubscriber)
            posePublisher()
        rospy.spin()