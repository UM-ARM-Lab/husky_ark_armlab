import math

import rospy
from ark_bridge.msg import SendGoal
from ark_bridge.srv import (
    Empty_service,
    Empty_serviceRequest,
    LoadMapFromDisk_service,
    LoadMapFromDisk_serviceRequest,
    SaveMapToDisk_service,
    SaveMapToDisk_serviceRequest,
    SetPose_service,
    SetPose_serviceRequest,
    String_service,
    String_serviceRequest,
)

DEFAULT_TIMEOUT_SEC = 3


class ARK:
    @staticmethod
    def call_service(topic, service=Empty_service, request=Empty_serviceRequest()):
        """Calls an ARK service with an empty request

        Args:
            topic (str): the service topic

        Returns:
            bool: if the service call was successful
        """
        rospy.wait_for_service(topic, timeout=DEFAULT_TIMEOUT_SEC)

        try:
            service_instance = rospy.ServiceProxy(topic, service)
            response = service_instance(request)
            return not response.ark_service_timeout

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False

    @staticmethod
    def start_autonomy():
        return ARK.call_service("/ark_bridge/start_autonomy")

    @staticmethod
    def stop_autonomy():
        return ARK.call_service("/ark_bridge/stop_autonomy")

    @staticmethod
    def pause_autonomy():
        request = String_serviceRequest()
        request.req_data.data = "ark_interface"
        return ARK.call_service(
            "/ark_bridge/control_selection_autonomy_pause", String_service, request
        )

    @staticmethod
    def resume_autonomy():
        request = String_serviceRequest()
        request.req_data.data = "ark_interface"
        return ARK.call_service(
            "/ark_bridge/control_selection_autonomy_resume", String_service, request
        )

    @staticmethod
    def set_pose(pose):
        msg = SetPose_serviceRequest()

        # This just fills position and orientation
        # We are leaving covariance as all zeros
        pose.fill_message_with_pose(msg.req_data.pose)
        return ARK.call_service(
            "/ark_bridge/slam_set_initial_pose", SetPose_service, msg
        )

    @staticmethod
    def create_goal_message(pose, position_tolerance=0.3, orientation_tolerance=45):
        """Returns a ROS message to send to the ark

        Args:
            pose: an instance of the Pose class
            position_tolerance (float, optional): position tolerance in meters.
                Defaults to 0.3.
            orientation_tolerance (int, optional): orientation tolerance in degrees.
                Defaults to 45.

        Returns:
            ark_bridge.msg.goal: a Goal message
        """

        msg = SendGoal()
        pose.fill_message_with_pose(msg)

        # Set the position tolerance (in meters)
        msg.position_tolerance = position_tolerance

        # Need to convert the orientation tolerance to radians
        msg.orientation_tolerance = math.radians(orientation_tolerance)
        return msg

    @staticmethod
    def load_map(map_name):
        request = LoadMapFromDisk_serviceRequest()
        request.req_data.data = map_name

        success = False
        attempts = 0

        while not success and attempts < 4:
            success = ARK.call_service(
                "/ark_bridge/map_data_load_map_from_disk",
                LoadMapFromDisk_service,
                request,
            )
            rospy.sleep(1)
            attempts += 1

        rospy.sleep(5)
        return success

    @staticmethod
    def save_map(map_name):
        request = SaveMapToDisk_serviceRequest()
        request.req_data.map_topic = "/slam/map"
        request.req_data.filename = map_name
        return ARK.call_service(
            "/ark_bridge/map_data_save_map_to_disk", SaveMapToDisk_service, request
        )


if __name__ == "__main__":
    print("Success", ARK.stop_autonomy())
