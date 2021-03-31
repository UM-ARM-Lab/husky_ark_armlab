from euler_to_quaternion import euler_to_quaternion


class Pose:
    """This is a simple class for making the end code cleaner"""

    def __init__(
        self,
        name,
        x,
        y,
        z,
        q1,
        q2,
        q3,
        q4,
    ):
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3
        self.q4 = q4

    @classmethod
    def pose_from_xy_euler(cls, name, x, y, euler_angle):
        """Create a Pose from XY and Euler Angle (with respect to z-axis)

        Args:
            name (str): name of the Pose
            x (float): X position
            y (float): Y position
            euler_angle (float): euler angle with respect to z-axis

        Returns:
            Pose: a new Pose object
        """
        # Convert to quaternion [x, y, z, w] from Euler angles specifying axes of rotation
        rotation_quaternion = euler_to_quaternion(0, 0, euler_angle, degrees=True)

        return Pose(
            name,
            x = x,
            y = y,
            z = 0,
            q1 = rotation_quaternion[0],
            q2 = rotation_quaternion[1],
            q3 = rotation_quaternion[2],
            q4 = rotation_quaternion[3]
        )

    def ark_pose(self):
        """Returns a ROS message to send to the ark

        Returns:
            ark_bridge.msg.goal: a Goal message
        """
        from ark_bridge.msg import SendGoal

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

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "{}: X: {}, Y: {}, Quaternion: [{}, {}, {}, {}]".format(
            self.name,
            self.x,
            self.y,
            self.z,
            self.q1,
            self.q2,
            self.q3,
            self.q4
        )
