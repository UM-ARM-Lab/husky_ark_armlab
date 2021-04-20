from husky_ark_armlab.euler_to_quaternion import euler_to_quaternion
import pytest


def test_euler_to_quaternion():
    quaternion = euler_to_quaternion(0, 0, 5, degrees=True)
    assert pytest.approx(quaternion, [0, 0, 0.0436194, 0.9990482])
