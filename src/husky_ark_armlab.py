#!/usr/bin/env python
from ark_bridge.msg import SendGoal
from ark_bridge.msg import GoalStatusArray
import actionlib_msgs
import rospy
import argparse
from route_loader import Route
from pose import Pose
from ark_interface import ARK
from keyboard_input import KeyboardInput
from laser_filter_adjuster import LaserFilterAdjuster

tracking_goal = True

def path_planner_status_callback(msg):
    """This keeps track of the ARK's status and if it is tracking a goal or awaiting a new goal.
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


def pause(keyboard_input):
    while True and not rospy.is_shutdown():
        rospy.sleep(0.3)

        if keyboard_input.check_hit() and " " == keyboard_input.get_char():
            print("Resuming")
            break


def main(route_json_path):
    # Start autonomy
    while not ARK.start_autonomy() and not rospy.is_shutdown():
        rospy.loginfo("Failed to start autonomy. Retrying in 3 seconds...")
        rospy.sleep(3)

    rospy.loginfo("ARK autonomy started")

    route = Route(route_json_path)

    # Start a ROS node called "PositionCommander"
    rospy.init_node("PositionCommander", anonymous=True)

    # Get a handle to the topic the ARK uses to accept goal
    pub = rospy.Publisher("/ark_bridge/send_goal", SendGoal, queue_size=1)

    # Get a handle to the topic the ARK uses to return its status
    rospy.Subscriber(
        "/ark_bridge/path_planner_status", GoalStatusArray, path_planner_status_callback
    )

    keyboard_input = KeyboardInput()
    filter_adjuster = LaserFilterAdjuster()

    # Loop through the positions in our list one at a time
    while not route.is_complete() and not rospy.is_shutdown():
        next_pose = route.get_next_waypoint()

        rospy.loginfo("Moving to " + next_pose.name)

        # Get a message for the ark and send it to the new goal topic
        pub.publish(next_pose.ark_pose())

        # Wait for a second to make sure the ARK has time to accept the goal
        rospy.sleep(1)

        # Loop until the ARK says it is done running a goal
        while (tracking_goal) and not rospy.is_shutdown():
            if keyboard_input.check_hit():

                char = keyboard_input.get_char()

                # Skip the waypoint if n is pressed (ord == 110)
                if "n" == char:
                    print("Skipping waypoint: ", next_pose.name)
                    break

                # If the space bar is pressed, just wait until
                if " " == char:
                    print("Pausing")
                    pause(keyboard_input)

                if "w" == char:
                    filter_adjuster.increase_radius()
                    print("Increased filter radius to: ", filter_adjuster.front_upper_threshold())
                    rospy.sleep(1.0)  # sleep to allow the point clouds to update

                if "s" == char:
                    filter_adjuster.decrease_radius()
                    print("Decreased filter radius to: ", filter_adjuster.front_upper_threshold())
                    rospy.sleep(1.0)  # sleep to allow the point clouds to update

            rospy.sleep(1)

        rospy.loginfo("Done")

    # stop autonomy once the goal is reached
    ARK.stop_autonomy()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-r", "--route", type=str, required=True, help="The path to the route JSON"
    )
    args = vars(ap.parse_args(rospy.myargv()[1:]))
    main(args["route"])
