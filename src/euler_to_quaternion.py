from math import sin, cos, atan2, pi, sqrt, asin, acos

rad2deg = 180.0/pi
deg2rad = pi/180


def euler_to_quaternion(roll, pitch, yaw, degrees=True):
    """
    Euler angles euler2quat(roll, pitch, yaw, degrees=True), default is degrees, but set
    degrees False if giving radians

    This is a modified version of this:
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_Angles_Conversion

    Code comes from:
    https://github.com/MomsFriendlyRobotCompany/squaternion/blob/master/squaternion/squaternion.py

    Returns:
      (list): Quaternions of form [x, y, z, w]
    """
    if degrees:
        roll  *= deg2rad
        pitch *= deg2rad
        yaw   *= deg2rad

    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)

    w = cy * cr * cp + sy * sr * sp
    x = cy * sr * cp - sy * cr * sp
    y = cy * cr * sp + sy * sr * cp
    z = sy * cr * cp - cy * sr * sp

    return [x, y, z, w]