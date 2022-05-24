#!/usr/bin/env python3.8
# license removed for brevity

import utm
import rospy

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

waypoints = [(-31.9804702, 115.8171341, 18.0857863), (-31.980125, 115.817429, 5.7284885), (-31.9804702, 115.8171341, 18.0857863), (-31.9810017, 115.8172201, 20.7020045)]

def movebase_client():
        for point in waypoints:
            utm_coord = utm.from_latlon(point[0], point[1])
            x = utm_coord[0]
            y = utm_coord[1]
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
            client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        
        # Waits until the action server has started up and started listening for goals.
            client.wait_for_server()

        # Creates a new goal with the MoveBaseGoal constructor
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
        # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
        # No rotation of the mobile base frame w.r.t. map frame
            goal.target_pose.pose.orientation.w = 1.0

        # Sends the goal to the action server.
            client.send_goal(goal)
        # Waits for the server to finish performing the action.
            wait = client.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
            # Result of executing the action
                return client.get_result()                                              

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")