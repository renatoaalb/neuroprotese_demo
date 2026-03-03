import numpy as np
import utils.serial_operations as serial_op
import utils.quaternion_operations as quaternions_op
import utils.euler_angle_operations as euler_op
import time

# position on streaming slots
QUATERNION_POSITION = 0
EULER_ANGLE_POSITION = 1
ACCEL_POSITION = 2
GYRO_POSITION = 3
COMPASS_POSITION = 4
ROTATION_MATRIX_POSITION = 5

# data strategies
data_strategies = {
    0: {
        "extract": serial_op.extract_quaternions,
        "key": "quaternions",
        "angle_function": quaternions_op.calculate_angle_between_quaternions,
        "position": QUATERNION_POSITION
    },
    1: {
        "extract": serial_op.extract_euler_angles,
        "key": "euler_vector", # Corrigido para bater com o return da função
        "angle_function": euler_op.calculate_angle_between_euler_angles,
        "position": EULER_ANGLE_POSITION
    },
    2: {
        "extract": serial_op.extract_rotation_matrix,
        "key": "rotation_matrix", # Corrigido para bater com o return da função
        "angle_function": None,
        "position": ROTATION_MATRIX_POSITION
    },
    37: {
        "extract": serial_op.extract_gyro,
        "key": "gyro",
        "angle_function": None,
        "position": GYRO_POSITION
    },
    38: {
        "extract": serial_op.extract_accel,
        "key": "accel",
        "angle_function": None,
        "position": ACCEL_POSITION
    }
}

def initialize_dongle(imu_ids):
    serial_port = serial_op.initialize_dongle(imu_ids)
    return serial_port

def configure_imu(serial_port, imu_ids, disableCompass = True, disableGyro = False, disableAccelerometer = False, gyroAutoCalib = True, 
                   filterMode = 1, tareSensor = True, show_quaternion = True, show_euler_angle = True, show_accel = False, show_gyro = False,
                   show_compass = False, show_rotation_matrix = False, tareWithQuaternion = None, buttonState = False, baudrate = 115200, timestamp = True):

    streaming_commands = [255] * 8

    if show_quaternion:
        streaming_commands[QUATERNION_POSITION] = 0

    
    if show_euler_angle:
        streaming_commands[EULER_ANGLE_POSITION] = 1

    
    if show_accel:
        streaming_commands[ACCEL_POSITION] = 39


    if show_gyro:
        streaming_commands[GYRO_POSITION] = 38


    if show_compass:
        streaming_commands[COMPASS_POSITION] = 40


    if show_rotation_matrix:
        streaming_commands[ROTATION_MATRIX_POSITION] = 2


    imu_configuration = {
        "disableCompass": disableCompass, #command 109
        "disableGyro": disableGyro, # command 107
        "disableAccelerometer": disableAccelerometer, #108
        "gyroAutoCalib": gyroAutoCalib, #command 165
        "filterMode": filterMode, #command 123
        "tareSensor": tareSensor, #command 96
        "tareWithQuaternion": tareWithQuaternion,
        "logical_ids": imu_ids,
        "streaming_commands": streaming_commands, #command 80 - ccepts a list of 8 bytes
        "baudrate": baudrate, #command 231,
        # to implement yet
        "buttonState": buttonState
    }

    serial_op.revised_initialize_imu(serial_port, imu_configuration)
    time.sleep(2)
    print()

def start_streaming(serial_port, imu_ids, frequency, timestamp, duration = 4294967295, delay =  0):
    interval = 1000000 / frequency

    if frequency == 0:
        interval = 0

    # minimum interval is 1000 or 0, thus, the maximum frequency is 1000Hz 
    streaming_configuration = {
        "interval": interval, # microseconds, how often the data will be outputed
        "duration": duration, #by standart, indefinite time
        "delay": delay,
        "timestamp": timestamp,
        "logical_ids": imu_ids
    }

    #serial_op.configure_streaming(serial_port, streaming_configuration)

    print("Starting streamnig.")
    # Start streaming
    serial_op.start_streaming(serial_port, streaming_configuration['logical_ids'])
    
    print("IMU's ready to use.")

def read_data(serial_port):
    bytes_to_read = serial_port.in_waiting

    if bytes_to_read > 0:
        data = serial_port.read(bytes_to_read)
        
        if data[0] != 0 and len(data) <=3:
            print('Corrupted data read.')
        else: 
            return data
    

def extract_data(data, type_of_data, imu_id):
    # Set parameters for IMU configuration
    # streaming commands:
    # 0: get tared orientation as quaternions
    # 1: get tared orientation as euler angles
    # 2: rotation matrix !!
    # 37: get all corrected component sensor data
    # 38: get corrected gyro rate
    # 39: get corrected accelerometer vector
    # 40: get corrected magnetometer data !!

    current_strategy = data_strategies.get(type_of_data)

    value = current_strategy["extract"](data, current_strategy["position"])

    if data[1] == imu_id:
        #timestamp = serial_op.get_timestamp(serial_port, imu_id)
        #print(timestamp)
        return value

            # Check which IMU is sending information
    
def stop_streaming(serial_port, imu_ids):
    serial_op.stop_streaming(serial_port, imu_ids)
    serial_op.manual_flush(serial_port)

# def correct_position(current_strategy["position"])
# return real_position