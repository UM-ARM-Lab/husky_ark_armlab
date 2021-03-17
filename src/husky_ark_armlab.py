#!/usr/bin/env python3
from ark_bridge.msg import SendGoal
from ark_bridge.msg import GoalStatusArray
import actionlib_msgs
import time
import rospy
import argparse
from .route_loader import Route
from .pose import Pose

tracking_goal = True


def path_planner_status_callback(msg: GoalStatusArray):
    """This keeps track of the ARK’s status and if it is tracking a goal or awaiting a new goal.
    The ark will maintain a list of current objectives and their status using this topic.

    Args:
        msg (GoalStatusArray): [description]
    """
    global tracking_goal
    running_goal = False
    for s in msg.status_list:
        if s.status == 0 or s.status == 1:
            running_goal = True
    tracking_goal = running_goal


def main(route_json_path):
    route = Route(route_json_path)

    # Start a ROS node called "PositionCommander"
    rospy.init_node("PositionCommander", anonymous=True)

    # Get a handle to the topic the ARK uses to accept goal
    pub = rospy.Publisher("/ark_bridge/send_goal", SendGoal, queue_size=1)

    # Get a handle to the topic the ARK uses to return its status
    rospy.Subscriber(
        "/ark_bridge/path_planner_status", GoalStatusArray, path_planner_status_callback
    )

    # Loop through the positions in our list one at a time
    while not route.is_complete():
        next_pose: Pose = route.get_next_waypoint()

        rospy.loginfo("Moving to " + next_pose.name)

        # Get a message for the ark and send it to the new goal topic
        pub.publish(next_pose.ark_pose())

        # Wait for a second to make sure the ARK has time to accept the goal
        time.sleep(1)

        # Loop until the ARK says it is done running a goal
        while (tracking_goal) and not rospy.is_shutdown():
            time.sleep(1)

        rospy.loginfo("Done")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-r", "--route", type=str, required=True, help="The path to the route JSON"
    )
    args = vars(ap.parse_args())
    main(args["route"])