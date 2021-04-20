#!/usr/bin/env python
from ark_bridge.msg import SendGoal, Empty, GoalStatusArray
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
    ARK.pause_autonomy()
    while True and not rospy.is_shutdown():
        rospy.sleep(0.3)

        if keyboard_input.check_hit() and " " == keyboard_input.get_char():
            print("Resuming")
            ARK.resume_autonomy()
            break


def main(route_json_path, start_map, start_waypoint_index):
    # Start autonomy
    while not ARK.start_autonomy() and not rospy.is_shutdown():
        rospy.loginfo("Failed to start autonomy. Retrying in 3 seconds...")
        rospy.sleep(3)

    rospy.loginfo("ARK autonomy started")

    route = Route(
        route_json_path, start_map=start_map, start_waypoint_index=start_waypoint_index
    )

    # Start a ROS node called "PositionCommander"
    rospy.init_node("PositionCommander", anonymous=True)

    # Get a handle to the topic the ARK uses to accept goal
    pub = rospy.Publisher("/ark_bridge/send_goal", SendGoal, queue_size=1)

    # Get a handle to the topic the ARK uses to accept goal
    cancel_goal_publisher = rospy.Publisher(
        "/ark_bridge/cancel_goal", Empty, queue_size=1
    )

    # Get a handle to the topic the ARK uses to return its status
    rospy.Subscriber(
        "/ark_bridge/path_planner_status", GoalStatusArray, path_planner_status_callback
    )

    keyboard_input = KeyboardInput()
    filter_adjuster = LaserFilterAdjuster()

    # Loop through the positions in our list one at a time
    while not route.is_complete() and not rospy.is_shutdown():
        next_pose = route.get_next_waypoint()
        filter_adjuster.set_radius(next_pose.get_upper_lidar_threshold())

        rospy.loginfo("Moving to " + next_pose.name)

        # Get a message for the ark and send it to the new goal topic
        pub.publish(ARK.create_goal_message(next_pose))

        # Wait for a second to make sure the ARK has time to accept the goal
        rospy.sleep(1)

        # Loop until the ARK says it is done running a goal
        while (tracking_goal) and not rospy.is_shutdown():
            if keyboard_input.check_hit():

                char = keyboard_input.get_char()

                # Skip the waypoint if n is pressed (ord == 110)
                if "n" == char:
                    rospy.loginfo("Skipping waypoint: {}".format(next_pose.name))
                    cancel_goal_publisher.publish(Empty())

                # If the space bar is pressed, just wait until
                elif " " == char:
                    rospy.loginfo("Pausing")
                    pause(keyboard_input)

                # Set the threshold to use for the lidars using (0-9)
                elif char.isdigit():
                    filter_adjuster.set_radius(int(char))
                    rospy.sleep(1.0)  # sleep to allow the point clouds to update

                elif "w" == char:
                    filter_adjuster.increase_radius()
                    rospy.sleep(1.0)  # sleep to allow the point clouds to update

                elif "s" == char:
                    filter_adjuster.decrease_radius()
                    rospy.sleep(1.0)  # sleep to allow the point clouds to update

            rospy.sleep(1)

    rospy.loginfo("Route completed")

    # stop autonomy once the goal is reached
    ARK.stop_autonomy()
    rospy.loginfo("Autonomoy stopped")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-r", "--route", type=str, required=True, help="The path to the route JSON"
    )
    ap.add_argument("-m", "--map", type=str, default=None, help="The map to start on")
    ap.add_argument(
        "-w",
        "--waypoint_index",
        type=int,
        default=0,
        help="The waypoint index to start on",
    )
    args = vars(ap.parse_args(rospy.myargv()[1:]))

    # In order to specify NONE as a value for a ROS param,
    # we set the default value to be "NONE"
    # This will replace all entries with the value "NONE" to have
    # the value None
    args = {k: (v if v != "NONE" else None) for k, v in args.items()}

    main(args["route"], args["map"], args["waypoint_index"])
