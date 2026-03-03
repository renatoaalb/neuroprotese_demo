import serial.tools.list_ports
import time
import numpy as np
import sys

SMALL_IMU_DONGLE_PORT = 4128
IMU_SENSOR_USB = 4144

# If permission denied error occurs in Linux try:
# sudo chmod 666 /dev/ttyACM0 -> with the correspondent COM port
def get_dongle_object():
    """ Create a serial port object to operate with imu dongle
    
    Returns: 
        serial_port: PySerial object
    """
 
    # Lists available ports and select dongle port
    ports_list = serial.tools.list_ports.comports()
    print("Ports available: ")
    for wire in ports_list:
        text = "Port: {0}\tSerial#:{1}\tDesc:{2} PID {3}".format(wire.device, 
                                                                wire.serial_number, 
                                                                wire.description,
                                                                wire.pid)
        print(text)
        if wire.pid == SMALL_IMU_DONGLE_PORT:
            portIMU = wire.device

    # Instantiate serial port object
    serial_port = serial.Serial(port=portIMU, baudrate=115200, timeout=0.01)
    manual_flush(serial_port)

    return serial_port

def get_sensor_object():
    """ Create a serial port object to operate with imu dongle
    
    Returns: 
        serial_port: PySerial object
    """
 
    # Lists available ports and select dongle port
    ports_list = serial.tools.list_ports.comports()
    print("Ports available: ")
    for wire in ports_list:
        text = "Port: {0}\tSerial#:{1}\tDesc:{2} PID {3}".format(wire.device, 
                                                                wire.serial_number, 
                                                                wire.description,
                                                                wire.pid)
        print(text)
        if wire.pid == IMU_SENSOR_USB:
            portIMU = wire.device

    serial_port = serial.Serial(port=portIMU, baudrate=115200, timeout=0.01)
    manual_flush(serial_port)
    
    return serial_port

def manual_flush(serial_port):
    """ Clean serial port buffer

    Args:
        serial_port: PySerial Object
    """
    while not serial_port.inWaiting() == 0:
        serial_port.read(serial_port.inWaiting())

def create_imu_command(logical_id, command_number, arguments = []):
    """ Create imu command string

    Args: 
        logical_id: integer represents imu sensor configure to dongle 
        (configure in sensor suit)
        command_number: integer represents a command (see all commands in
        user manual)
        arguments: list with arguments, if necessary
    
    Return:
        encoded string with the command in the correct format
    """
    # Create command
    command = ">"+str(logical_id)+","+str(command_number)
    if(len(arguments) != 0):
        arguments_string = ","
        for  argument in arguments:
            arguments_string += str(argument)
            arguments_string += ","
        arguments_string = arguments_string[:-1]
        command += arguments_string
    command += '\n'
    return command.encode()

def apply_command(serial_port, command, showResponse=False):
    """ Apply command in sensor

    Args:
        serial_port: PySerial Object
        command: encoded string with the command
        showResponse: boolean that decides if output will be displayed
    """
    serial_port.write(command)
    time.sleep(0.1)
    if(showResponse):
        while serial_port.inWaiting():
            out = '>> ' + serial_port.read(serial_port.inWaiting()).decode()
        print(out)
    time.sleep(0.1)
    # return out



def stop_streaming(serial_port, logical_ids):
    """ Apply stop streaming operation

    Args:
        serial_port: PySerial Object
        logical_ids: list of sensors that the command should be applied
    """
    for id in logical_ids:
        command = create_imu_command(id, 86)
        apply_command(serial_port, command)

def start_streaming(serial_port, logical_ids):
    """ Apply start streaming operation

    Args:
        serial_port: PySerial Object
        logical_ids: list of sensors that the command should be applied
    """
    for id in logical_ids:
        command = create_imu_command(id, 85)
        apply_command(serial_port, command)
    return serial_port

def clean_data_vector(data):
    decoded_data = data.decode()
    cleaned_data = decoded_data.replace('\r\n',' ').split(' ')
    cleaned_data = list(filter(None, cleaned_data))[0].split(",")
    cleaned_data = [float(d) for d in cleaned_data]
    return cleaned_data
def write_command_read_answer(serial_port, logical_ids, command_number, arguments=[]):
    # If it's streaming stop it
    # stop_streaming(serial_port, logical_ids)
    time.sleep(0.1)
    manual_flush(serial_port)
    
    # Apply imu command to each logical id and saves it's answers
    answer = []
    for id in logical_ids:
        time.sleep(0.1)
        command = create_imu_command(id, command_number, arguments)
        apply_command(serial_port, command)
        time.sleep(0.1)
        cleaned_data = clean_data_vector(serial_port.read(serial_port.inWaiting()))
        answer.append(cleaned_data)

    # start_streaming(serial_port, logical_ids)
    return answer

def set_streaming_slots(serial_port, logical_ids, commands):
    """ Set streaming slots

    Args:
        serial_port: PySerial Object
        logical_ids: list of sensors that the command should be applied
        commands: list of integer that commands slots should be filled
    """
    for id in logical_ids:
        command = create_imu_command(id, 80, commands)
        print(command)
        apply_command(serial_port, command, True)
    return serial_port

def configure_sensor(serial_port, configDict):
    """ Apply common sensor configuration

    Args:
        serial_port: PySerial Object
        configDict: dictionary with sensor basic configuration {
                "disableCompass": Boolean,
                "disableGyro": Boolean,
                "disableAccelerometer": Boolean,
                "gyroAutoCalib": Boolean,
                "tareSensor": Boolean,
                "tareWithQuaternion": dict {imuid : list with quaternions}
                "filterMode": Integer (see user's manual)
                "logical_ids": list of logical ids,
                "streaming_commands": list of streaming slots
                "baudrate": Integer (see user's manual)
        }
    """
    if(configDict["disableGyro"]):
        for id in configDict["logical_ids"]:
            command = create_imu_command(id, 107)
            apply_command(serial_port, command)

    if(configDict["disableAccelerometer"]):
        for id in configDict["logical_ids"]:
            command = create_imu_command(id, 108)
            apply_command(serial_port, command)

    if(configDict["disableCompass"]):
        for id in configDict["logical_ids"]:
            command = create_imu_command(id, 109)
            apply_command(serial_port, command)

    filterMode = configDict["filterMode"]
    for id in configDict["logical_ids"]:
        command = create_imu_command(id, 123, [filterMode])
        apply_command(serial_port, command)

    if configDict["gyroAutoCalib"]:
        for id in configDict["logical_ids"]:
            command = create_imu_command(id, 165)
            apply_command(serial_port, command)

    if configDict["tareSensor"]:
        for id in configDict["logical_ids"]:
            command = create_imu_command(id, 96)
            apply_command(serial_port, command)
    else:
        for id in configDict["logical_ids"]:
            command = create_imu_command(id, 97, configDict["tareWithQuaternion"]["imu"+str(id)])
        
    baudrate = configDict["baudrate"]
    if configDict["baudrate"]:
        for id in configDict["logical_ids"]:
            command = create_imu_command(id, 231,[baudrate])
            apply_command(serial_port, command)


    # Forcing axis configuration to standar
    for id in configDict["logical_ids"]:
        command = create_imu_command(id, 116, [0])
        apply_command(serial_port, command)

def configure_streaming(serial_port, configDict):
    """ Apply common sensor configuration

    Args:
        serial_port: PySerial Object
        configDict: dictionary with sensor basic configuration {
                "interval": Integer,
                "duration": Integer,
                "delay": Integer,
                "timestamp": Boolean,
                "logical_ids": list of logical ids
        }
    """
    for id in configDict["logical_ids"]:
        command = create_imu_command(id, 82, [configDict["interval"], configDict["duration"], configDict["delay"]])
        apply_command(serial_port, command)

    if configDict["timestamp"]:
        for id in configDict["logical_ids"]:
            command = create_imu_command(id, 95, [0])
            apply_command(serial_port, command)

def get_timestamp(serial_port, logical_ids):
    for id in logical_ids:
        command = create_imu_command(id, 83)
        apply_command(serial_port, command, showResponse=True)


def tare_sensor(serial_port, logical_ids):
    """ Apply tare sensor operation

    Args:
        serial_port: PySerial Object
        logical_ids: list of sensors that the command should be applied
    """
    for id in logical_ids:
        command = create_imu_command(id, 96)
        apply_command(serial_port, command)

def get_sensor_information(serial_port, logical_ids):
    """ Get some sensor current information such as filter mode and trust values

    Args:
        serial_port: PySerial Object
        logical_ids: list of sensors that the command should be applied
    """ 
    # Current filter mode
    for id in logical_ids:
        command = create_imu_command(id, 152)
        apply_command(serial_port, command)
    # Current accelerometer trust values
    for id in logical_ids:
        command = create_imu_command(id, 130)
        apply_command(serial_port, command)

def clean_list(data):
    decoded_data = data.decode()
    list_data = decoded_data.replace('\r\n',' ').split(' ')
    cleaned_list_data = list(filter(None, list_data))

    return cleaned_list_data

# Get rotation matrix streamed in first slot. 
def extract_rotation_matrix(data, position):
    """ Manipulate data to obtain rotation matrix
    
    Args:
        data: Raw data that sensor send
        position: Position of the data extracted on the streaming_commands list
    
    Returns: 
        rotation matrix dictionary
    """
   
    cleaned_list_data = clean_list(data)
    rotatation_vector = cleaned_list_data[position][3:].split(',')
    rotatation_vector = np.array(rotatation_vector, dtype=np.float64)
    rotation_matrix = rotatation_vector.reshape((3,3))
    return rotation_matrix

def extract_euler_angles(data, position):
    """ Manipulate data to obtain rotation matrix
    
    Args:
        data: Raw data that sensor send
        position: Position of the data extracted on the streaming_commands list
    
    Returns: 
        rotation matrix dictionary
    """
    cleaned_list_data = clean_list(data)

    euler_vector = cleaned_list_data[position][3:].split(',')
    euler_vector = np.array(euler_vector, dtype=np.float64)
    return euler_vector

def extract_quaternions(data, position):
    """ Manipulate data to obtain rotation matrix
    
    Args:
        data: Raw data that sensor send
        position: Position of the data extracted on the streaming_commands list
    
    Returns: 
        rotation matrix dictionary
    """
    cleaned_list_data = clean_list(data)
    
    quaternion = cleaned_list_data[position][3:].split(',')
    quaternion = np.array(quaternion, dtype=np.float64)
    return quaternion

def extract_gyro(data, position):
    """ Manipulate data to obtain rotation matrix
    
    Args:
        data: Raw data that sensor send
        position: Position of the data extracted on the streaming_commands list
    
    Returns: 
        rotation matrix dictionary
    """
    cleaned_list_data = clean_list(data)
    
    gyro = cleaned_list_data[position][2:].split(',')
    gyro = np.array(gyro, dtype=np.float64)
    return gyro

def extract_accel(data, position):
    """ Manipulate data to obtain rotation matrix
    
    Args:
        data: Raw data that sensor send
        position: Position of the data extracted on the streaming_commands list
    
    Returns: 
        rotation matrix dictionary
    """
    cleaned_list_data = clean_list(data)
    accel = cleaned_list_data[position][3:].split(',')

    accel = np.array(accel, dtype=np.float64)
    return accel


def extract_acc_quat(data):
    # old
    """ Manipulate data to obtain rotation matrix
    
    Args:
        data: Raw data that sensor send
    
    Returns: 
        rotation matrix dictionary
    """
    cleaned_list_data = clean_list(data)
    acc = cleaned_list_data[0][3:].split(',')
    quaternion = cleaned_list_data[1][:].split(',')

    acc = np.array(acc, dtype=np.float64)
    quaternion = np.array(quaternion, dtype=np.float64)


    return {'acc': acc, 'quaternions': quaternion}



def initialize_imu(configuration_dict):
    """ Initialize imu dongle and sensor

    Args:
        configDict: dictionary with sensor basic configuration {
                "disableCompass": Boolean,
                "disableGyro": Boolean,
                "disableAccelerometer": Boolean,
                "gyroAutoCalib": Boolean,
                "tareSensor": Boolean,
                "filterMode": Integer (see user's manual)
                "logical_ids": list of logical ids,
                "streaming_commands": list of streaming slots,
                "timestamp": Boolean
        }
    
    Returns:
        serial_port: PySerial Object

    """
    # Find and open serial port for the IMU dongle
    print("Getting imu object:")
    serial_port = get_dongle_object()
    print("Done.")

    # Clean outputs 
    manual_flush(serial_port)
    
    # Stop streaming
    print("Stoping streaming.")
    stop_streaming(serial_port, configuration_dict['logical_ids'])


    # Setting streaming slots, this means that while streaming sensors will send
    # this data to the dongle as in page 29 - User manual: 
    # 0 - Differential quaternions; 
    # 1 - tared orientation as euler angles; 
    # 2 - rotation matrix
    # 255 - No data
    # See user manual to get more information
    set_streaming_slots(serial_port, 
                     configuration_dict['logical_ids'], 
                     configuration_dict['streaming_commands'])

    # Set magnetometer(explain it better), calibGyro if calibGyro=True and Tare sensor
    print('Starting configuration: ')
    configure_sensor(serial_port, configuration_dict)
    print("Done.")


    print("Starting streaming.")
    # Start streaming
    start_streaming(serial_port, configuration_dict['logical_ids'])
    
    print("IMU's ready to use.")

    return serial_port

def initialize_dongle(imu_ids):
    # Find and open serial port for the IMU dongle
    print("Getting imu object:")
    serial_port = get_dongle_object()
    print("Done.")

    # Clean outputs 
    manual_flush(serial_port)

    # Stop streaming
    print("Stoping streaming.")
    stop_streaming(serial_port, imu_ids)

    return serial_port

#same, but does not include start streaming or initializing the dongle
def revised_initialize_imu(serial_port, configuration_dict):
     


    # Setting streaming slots, this means that while streaming sensors will send
    # this data to the dongle as in page 29 - User manual: 
    # 0 - Differential quaternions; 
    # 1 - tared orientation as euler angles; 
    # 2 - rotation matrix
    # 255 - No data
    # See user manual to get more information
    set_streaming_slots(serial_port, 
                     configuration_dict['logical_ids'], 
                     configuration_dict['streaming_commands'])

    # Set magnetometer(explain it better), calibGyro if calibGyro=True and Tare sensor
    print('Starting configuration: ')
    configure_sensor(serial_port, configuration_dict)
    print("Done.")
