import rospy
from ark_bridge.srv import (
    Empty_service,
    Empty_serviceRequest,
    LoadMapFromDisk_service,
    LoadMapFromDisk_serviceRequest,
    SaveMapToDisk_service,
    SaveMapToDisk_serviceRequest,
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
