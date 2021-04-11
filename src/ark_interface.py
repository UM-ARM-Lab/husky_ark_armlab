import rospy
from ark_bridge.srv import (
    Empty_service,
    Empty_serviceRequest,
    LoadMapFromDisk_service,
    LoadMapFromDisk_serviceRequest,
    SaveMapToDisk_service,
    SaveMapToDisk_serviceRequest,
    String_service,
    String_serviceRequest,
    SetPose_service,
    SetPose_serviceRequest
)
from ark_bridge.msg import SendGoal
import math


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
            empty_service = rospy.ServiceProxy(topic, service)
            response = empty_service(request)
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
        return ARK.call_service("/ark_bridge/control_selection_autonomy_pause", String_service, request)

    @staticmethod
    def resume_autonomy():
        request = String_serviceRequest()
        request.req_data.data = "ark_interface"
        return ARK.call_service("/ark_bridge/control_selection_autonomy_resume", String_service, request)

    @staticmethod
    def set_pose(pose):
        request.req_data
        request.req_data.data = "ark_interface"
        return ARK.call_service("/ark_bridge/slam_set_initial_pose", SetPose_service, request)

    @staticmethod
    def create_goal_message(pose, position_tolerance=0.3, orientation_tolerance=45):
        """Returns a ROS message to send to the ark

        Args:
            pose: an instance of the Pose class
            position_tolerance (float, optional): position tolerance in meters. Defaults to 0.3.
            orientation_tolerance (int, optional): orientation tolerance in degrees. Defaults to 45.

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
    def create_setpose_message(pose):
        """Creates a SetPose message. This can be used to hint to the ARK
        where the Husky currently is.

        Args:
            pose (Pose): an instance of the custom Pose class

        Returns:
            ark_bridge.srv.SetPose_serviceRequest: a set pose service request.
                It is of the folowing form with the position and orientation filled in.
                The covariance is left as all zeros:

                req_data:
                    header:
                        seq: 0
                        stamp:
                        secs: 0
                        nsecs: 0
                        frame_id: ''
                    pose:
                        pose:
                        position: {x: 0.0, y: 0.0, z: 0.0}
                        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
                        covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        """
        request = SetPose_serviceRequest()
        pose.fill_message_with_pose(request)
        return request

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
