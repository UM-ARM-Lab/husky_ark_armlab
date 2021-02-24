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
        self.location_list["ORIGIN"] = pose_data(
            "ORIGIN", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0
        )
        self.location_list["WORKBENCH"] = pose_data(
            "WORKBENCH", 1.42520270935, -2.07659318067, 0.0, 0.0, 0.0, 0.0, 1.0
        )
        self.location_list["LOCATION 3"] = pose_data(
            "LOCATION 3", 0.387670869059, -3.0498080698, 0.0, 0.0, 0.0, 1.0, 0.0
        )
        self.location_list["LOCATION 4"] = pose_data(
            "LOCATION4",
            3.13896623959,
            -1.65007802521,
            0.0,
            0.0,
            0.0,
            -0.755254064534,
            0.655432145996,
        )


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
        msg.pose.orientation.y = self.q2
        msg.pose.orientation.z = self.q3
        msg.pose.orientation.w = self.q4
        msg.position_tolerance = 0.1
        msg.orientation_tolerance = 0.1
        return msg


############################################################################################
# This keeps track of the ARK’s status and if it is tracking a goal or awaiting a new goal #
############################################################################################
def statusCallback(msg):
    global tracking_goal
    running_goal = False
    for s in msg.status_list:
        if s.status == 0 or s.status == 1:
            running_goal = True
    tracking_goal = running_goal


if __name__ == "__main__":
    try:
        # Start a ROS node called "PositionCommander" #
        rospy.init_node("PositionCommander", anonymous=True)
        # Get our list of locations #
        positionList = locations().location_list
        # Pull out the locations we want to use and put them in a list so we can run them in a specific order #
        positions_to_follow = []
        positions_to_follow.append(positionList["ORIGIN"])
        positions_to_follow.append(positionList["WORKBENCH"])
        positions_to_follow.append(positionList["LOCATION 4"])
        # Get a handle to the topic the ARK uses to accept goal #
        pub = rospy.Publisher("/ark_bridge/send_goal", SendGoal, queue_size=1)
        # Get a handle to the topic the ARK uses to return its status #
        rospy.Subscriber(
            "/ark_bridge/path_planner_status", GoalStatusArray, statusCallback
        )
        # Loop through the positions in our list one at a time #
        for p in positions_to_follow:
            rospy.loginfo("Moving to " + p.name)
            # Get a message for the ark and send it to the new goal topic #
            pub.publish(p.arkPose())
            # Wait for a second to make sure the ARK has time to accept the goal #
            time.sleep(1)
            # Loop until the ARK says it is done running a goal #
            while (tracking_goal) and not rospy.is_shutdown():
                time.sleep(1)

            rospy.loginfo("Done")
    except:
        print("Error running node")
