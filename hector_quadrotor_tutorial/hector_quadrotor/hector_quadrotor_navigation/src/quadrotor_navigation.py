#!/usr/bin/env python
import rospy
import time
import random
from hector_uav_msgs.srv import EnableMotors
from geometry_msgs.msg import PoseStamped
import actionlib
import hector_uav_msgs.msg
import numpy

def hector_pose_client():
     # First enable motor service
     rospy.wait_for_service("/enable_motors")
     enabler1 = rospy.ServiceProxy("/enable_motors", EnableMotors)
     resp1 = enabler1(True)

     rospy.loginfo("Creating Action Client.")
     client = actionlib.SimpleActionClient('/action/pose', hector_uav_msgs.msg.PoseAction)
     rospy.loginfo("Client created.")

     # This is where the program seems to hang even though I assumed hector would automatically run the action server
     client.wait_for_server()

     # Create the goal positions

     for x in numpy.arange(0,5 , 1):
        # print(x)
        for y in numpy.arange(0, 5, 1):
            g = hector_uav_msgs.msg.PoseGoal()
            g.target_pose.header.frame_id = 'world'
            g.target_pose.pose.position.x = x
            g.target_pose.pose.position.y = y
            g.target_pose.pose.position.z = 1

            rospy.loginfo("Sending goal")
            client.send_goal(g)

            rospy.loginfo("Waiting for result")
            time.sleep(20)
            client.wait_for_result()
            # time.sleep(20)
            goal_reached_string = "Reached goal: \n x: " + str(x) +  " y:" + str(y) + " z:1"
            rospy.loginfo(goal_reached_string)
            time.sleep(2)




     return(client.get_result())

if __name__ == "__main__":
     try:
         rospy.init_node('drone_explorer')
         result = hector_pose_client()
         rospy.loginfo("Client navigated")
     except rospy.ROSInterruptException:
         print("program interrupted before completion")