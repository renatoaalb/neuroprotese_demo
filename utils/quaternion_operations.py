from pyquaternion import Quaternion # http://kieranwynn.github.io/pyquaternion/
import math
from scipy.spatial.transform import Rotation as R

def convert_quaternion_to_unit_quaternion_if_needed(quaternion : Quaternion):
    """ Convert a quarternion in a unit quaternion if isn't.

    Args:
        quaternion: Quaternion object
    Returns:
        Unit quaternion
    """
    if(quaternion.is_unit):
        return quaternion
    else:
        return quaternion/quaternion.norm


def calculate_angle_between_quaternions(firstQuaternion : Quaternion, 
                                        secondQuaternion : Quaternion):
    """ Calculate angle between two quaternions

    Args:
        firstQuaternion: Quaternion object
        secondQuaternion: Quaternion object
    Returns:
        Angle between quaternions in degrees
    """

    # Quaternion(w, x, y, z) -> imu yei format (x, y, z, w) -> See user's manual
    #firstQuaternion = [float(q) for q in firstQuaternion if isinstance(q, (int, float))]
    #secondQuaternion = [float(q) for q in secondQuaternion if isinstance(q, (int, float))]

    firstQuaternionObject = Quaternion(firstQuaternion[1], firstQuaternion[2], firstQuaternion[3], firstQuaternion[0])
    secondQuaterionObject = Quaternion(secondQuaternion[1], secondQuaternion[2], secondQuaternion[3], secondQuaternion[0])

    firstQuaternionObject = convert_quaternion_to_unit_quaternion_if_needed(firstQuaternionObject)
    secondQuaterionObject = convert_quaternion_to_unit_quaternion_if_needed(secondQuaterionObject)

    resultantQuaternion = firstQuaternionObject.conjugate * secondQuaterionObject
    minQuat = resultantQuaternion[0]
    if abs(resultantQuaternion[0]) > 1:
        minQuat = 1 if resultantQuaternion[0] > 0 else -1
    angleDegrees = 2 * math.degrees(math.acos(minQuat))
    return angleDegrees

def get_rotation_matrix_from_quaternions(quaternions):
    """ Receiveis a quaternions and returns a rotation matrix rotated 90º around
    x axis to start in position presented in README.md.
    
    Args:
        quaternions: vector with quaternions x, y, z, w format    
    Returns: 
        rotation matrix representing imu's rotation
    """
    # create rotation object
    quaternionObject = R.from_quat(quaternions)

    # rotation_matrix = R.from_quat(quaternions).as_matrix()
    rotation_matrix = quaternionObject.as_matrix()

    # change tare position 90 degress
    rotation_matrix[[1, 2]] = rotation_matrix[[2, 1]]

    return rotation_matrix
# Check quaternion operations in isolation
# Source to check: https://www.mathworks.com/matlabcentral/answers/415936-angle-between-2-quaternions
# See for answers!!! 
if __name__ == '__main__':
    resultantQuaternion = calculate_angle_between_quaternions([ 0.968, 0.008, -0.008, 0.252], [ 0.382, 0.605,  0.413, 0.563])
    print(resultantQuaternion)

def euler_from_quaternion(quaternion):
    rotacao = R.from_quat(quaternion)

    angulo_euler = rotacao.as_euler('xyz', degrees=True)

    return angulo_euler
