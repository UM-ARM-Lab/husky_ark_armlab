class Pose:
    """This is a simple class for making the end code cleaner"""

    def __init__(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        q1: float,
        q2: float,
        q3: float,
        q4: float,
    ):
        self.name: string = name
        self.x: float = x
        self.y: float = y
        self.z: float = z
        self.q1: float = q1
        self.q2: float = q2
        self.q3: float = q3
        self.q4: float = q4

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
