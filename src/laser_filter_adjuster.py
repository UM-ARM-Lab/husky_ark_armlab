import dynamic_reconfigure.client


class LaserFilterAdjuster:
    def __init__(self):
        self.front_client = dynamic_reconfigure.client.Client("/laser_filter_front/range/")
        self.rear_client = dynamic_reconfigure.client.Client("/laser_filter_rear/range/")
        self.front_params = self.front_client.get_configuration()
        self.rear_params = self.rear_client.get_configuration()

        print("LaserFilter front params", self.front_params)
        print("LaserFilter rear params", self.rear_params)

    def front_upper_threshold(self):
        return self.front_params["upper_threshold"]

    def rear_upper_threshold(self):
        return self.front_params["upper_threshold"]

    def increase_radius(self, amount=1.0):
        self.front_params = self.front_client.update_configuration({
            "upper_threshold": self.front_upper_threshold() + amount
        })
        self.rear_params = self.rear_client.update_configuration({
            "upper_threshold": self.rear_upper_threshold() + amount
        })

    def decrease_radius(self, amount=1.0):
        return self.increase_radius(-1 * amount)
