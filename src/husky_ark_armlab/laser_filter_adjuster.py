import dynamic_reconfigure.client
import rospy

class LaserFilterAdjuster:
    def __init__(self):
        self.front_client = dynamic_reconfigure.client.Client("/laser_filter_front/range/")
        self.rear_client = dynamic_reconfigure.client.Client("/laser_filter_rear/range/")
        self.front_params = self.front_client.get_configuration()
        self.rear_params = self.rear_client.get_configuration()

    def front_upper_threshold(self):
        return self.front_params["upper_threshold"]

    def rear_upper_threshold(self):
        return self.front_params["upper_threshold"]

    def increase_radius(self, amount=0.5):
        self.set_radius(self.front_upper_threshold() + amount)

    def decrease_radius(self, amount=0.5):
        return self.increase_radius(-1 * amount)

    def set_radius(self, amount):
        # No need to set the radius again if no change is made
        if self.front_upper_threshold() == amount and self.rear_upper_threshold() == amount:
            return

        # Otherwise set the updated radius
        rospy.loginfo("Setting filter radius to: {}".format(amount))
        self.front_params = self.front_client.update_configuration({
            "upper_threshold": amount
        })
        self.rear_params = self.rear_client.update_configuration({
            "upper_threshold": amount
        })
