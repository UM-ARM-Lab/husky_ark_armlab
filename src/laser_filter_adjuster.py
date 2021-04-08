import rospy


class LaserFilterAdjuster:
    def __init__(self):
        self.FRONT_NAMESPACE = "/laser_filter_front/scan_filter_chain/"
        self.REAR_NAMESPACE = "/laser_filter_rear/scan_filter_chain/"

        """
        >>> rospy.get_param("/laser_filter_front/scan_filter_chain/")
            [
                {
                    'type': 'laser_filters/LaserScanRangeFilter',
                    'params':
                        {'upper_replacement_value': inf,
                        'use_message_range_limits': False,
                        'upper_threshold': 2.0,
                        'lower_threshold': 0.0,
                        'lower_replacement_value': -inf
                    },
                    'name': 'range'
                }
            ]

        """
        self.front_params = rospy.get_param(self.FRONT_NAMESPACE)
        self.rear_params = rospy.get_param(self.REAR_NAMESPACE)

        print("LaserFilter front params", self.front_params)
        print("LaserFilter rear params", self.rear_params)

    def front_upper_threshold(self):
        return self.front_params[0]["params"]["upper_threshold"]

    def rear_upper_threshold(self):
        return self.front_params[0]["params"]["upper_threshold"]

    def increase_radius(self, amount=0.2):
        self.front_params[0]["params"]["upper_threshold"] += amount
        self.rear_params[0]["params"]["upper_threshold"] += amount

        rospy.set_param(self.FRONT_NAMESPACE, self.front_params)
        rospy.set_param(self.REAR_NAMESPACE, self.rear_params)

    def decrease_radius(self, amount=0.2):
        return self.increase_radius(-1 * amount)
