#!/usr/bin/env python
from ark_bridge.msg import SendGoal
from ark_bridge.msg import GoalStatusArray
import actionlib_msgs
import time
import rospy
tracking_goal = True
#################################################################
# This is just simple storage for the individual points by name #
#################################################################
class locations:
   def __init__(self):
       self.location_list = {}
       self.location_list[”ORIGIN”] = pose_data(”ORIGIN”, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
       self.location_list[”WORKBENCH”] = pose_data(”WORKBENCH”, 1.42520270935, -2.07659318067, 0.0, 0.0,
           0.0, 0.0, 1.0)
       self.location_list[”LOCATION 3”] = pose_data(”LOCATION 3”, 0.387670869059, -3.0498080698, 0.0, 0.0,
           0.0, 1.0, 0.0)
       self.location_list[”LOCATION 4”] = pose_data(”LOCATION4”, 3.13896623959, -1.65007802521, 0.0, 0.0,
           0.0, -0.755254064534, 0.655432145996)
##########################################################
# This is a simple class for making the end code cleaner #
##########################################################
class pose_data:
   def __init__(self, name, x, y, z, q1, q2, q3, q4):
       self.name = name
       self.x = x
       self.y = y
       self.z = z
       self.q1 = q1
       self.q2 = q2
       self.q3 = q3
       self.q4 = q4
   # Returns a ROS message to send to the ark #
   def arkPose(self):
       msg = SendGoal()
       msg.pose.position.x = self.x
       msg.pose.position.y = self.y
       msg.pose.position.z = self.z
       msg.pose.orientation.x = self.q1