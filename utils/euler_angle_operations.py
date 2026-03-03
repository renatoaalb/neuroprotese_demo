from scipy.spatial.transform import Rotation as R
import numpy as np

def calculate_angle_between_euler_angles(first_euler_angle, second_euler_angle):
    # tá errado!!!!
    first = R.from_euler('xyz', first_euler_angle, degrees=False)
    second = R.from_euler('xyz', second_euler_angle, degrees=False)

    relative = second.inv() * second
    angle = np.linalg.norm(relative.as_rotvec())

    return np.rad2deg(angle)