import time
import numpy as np
import sys
import serial
import struct

class StimNode:

    command_dict = {
        'channelListModeInitialization': {
            'id': 0,
            'field_bit_sizes': {'Ident':2, 'Check':3, 'N_Factor':3, 'Channel_Stim':8, 'Channel_Lf':8, 'X': 2, 'Group_Time':5, 'Main_Time':11}
        },
        'channelListModeUpdate': {
            'id': 1,
            'field_bit_sizes': {
                'Ident':2, 'Check':5,
                'Mode1':2, 'X1':3, 'Pulse_Width1':9, 'Pulse_Current1':7,
                'Mode2':2, 'X2':3, 'Pulse_Width2':9, 'Pulse_Current2':7,
                'Mode3':2, 'X3':3, 'Pulse_Width3':9, 'Pulse_Current3':7,
                'Mode4':2, 'X4':3, 'Pulse_Width4':9, 'Pulse_Current4':7,
                'Mode5':2, 'X5':3, 'Pulse_Width5':9, 'Pulse_Current5':7,
                'Mode6':2, 'X6':3, 'Pulse_Width6':9, 'Pulse_Current6':7,
                'Mode7':2, 'X7':3, 'Pulse_Width7':9, 'Pulse_Current7':7,
                'Mode8':2, 'X8':3, 'Pulse_Width8':9, 'Pulse_Current8':7,
            }
        },
        'channelListModeStop': {
            'id': 2,
            'field_bit_sizes': {'Ident':2, 'Check':5}
        },
        'singlePulseGeneration': {
            'id': 3,
            'field_bit_sizes': {'Ident':2, 'Check':5, 'Channel_Number':3, 'X': 2, 'Pulse_Width':9, 'Pulse_Current':7}
        }
    }

    channel_stim = []

    def __init__(self, port, freq, n_factor, group_time, stim_mode, channel_lf):
        self.PORT       = port
        self.FREQ       = freq
        self.N_FACTOR   = n_factor
        self.GROUP_TIME = group_time
        self.STIM_MODE  = stim_mode
        self.CHANNEL_LF = channel_lf

        self.serial_port = serial.Serial(
            port=self.PORT,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_TWO,
            rtscts=True
        )
        print("[INFO] StimNode inicializado")

    def set_channel_byte(self, bit_list, channel_list):
        for channel in channel_list:
            bit_list |= (0b00000001 << channel - 1)
        return bit_list

    def clear_bit(self, cleared_bit_list, bit_list):
        for bit in bit_list:
            cleared_bit_list &= ~(1 << bit)
        return cleared_bit_list

    def create_command(self, cmd_name, field_values):
        command      = self.command_dict[cmd_name]
        bit_count    = 0
        command_bytes = []
        command_byte  = 0b10000000

        for field_name in field_values.keys():
            try:
                field_value    = field_values[field_name]
                field_bit_size = command['field_bit_sizes'][field_name]
            except KeyError:
                continue

            if field_name == 'X':
                bit_count += command['field_bit_sizes'][field_name]
            else:
                shift = 7 - (bit_count + field_bit_size)
                if shift >= 0:
                    command_byte |= field_value << shift
                    bit_count    += field_bit_size
                    if bit_count == 7:
                        command_bytes.append(command_byte)
                        command_byte = 0b00000000
                        bit_count    = 0
                else:
                    command_byte |= field_value >> (-shift)
                    command_bytes.append(command_byte)
                    command_byte = 0b00000000
                    bit_count    = 0
                    field_value  = self.clear_bit(field_value, range(-shift, field_bit_size))
                    command_byte |= field_value << (7 - (-shift))
                    bit_count    += (-shift)
                    if bit_count == 7:
                        command_bytes.append(command_byte)
                        command_byte = 0b00000000
                        bit_count    = 0

        return bytearray(command_bytes)

    def write_read_command(self, command_bytes):
        try:
            self.serial_port.write(command_bytes)
        except Exception:
            print('failed to write to stimulator serial port')
            raise

        while self.serial_port.in_waiting < 1:
            pass

        try:
            ack = struct.unpack('B', self.serial_port.read(1))[0]
            print(f"ack -> {ack}")
        except Exception as error:
            print(f"ack -> {error}")

        return {0: False, 1: True}[ack & 1]

    def initialize_ccl(self, channels):
        self.channel_stim = channels
        cmd_name          = 'channelListModeInitialization'
        ident             = self.command_dict[cmd_name]['id']
        channel_stim_byte = self.set_channel_byte(0, self.channel_stim)
        channel_lf_byte   = self.set_channel_byte(0, self.CHANNEL_LF)
        ts1               = round((1 / float(self.FREQ)) * 1000)
        main_time         = int(round((ts1 - 1.0) / 0.5))
        check_sum         = (self.N_FACTOR + channel_stim_byte + channel_lf_byte + self.GROUP_TIME + main_time) % 8

        field_values = {
            'Ident': ident, 'Check': check_sum, 'N_Factor': self.N_FACTOR,
            'Channel_Stim': channel_stim_byte, 'Channel_Lf': channel_lf_byte,
            'X': 0, 'Group_Time': self.GROUP_TIME, 'Main_Time': main_time,
        }
        command_bytes = self.create_command(cmd_name, field_values)
        success       = self.write_read_command(command_bytes)
        print(f"response:{success}   |")
        return success

    def update_ccl(self, pulse_width, pulse_current):
        cmd_name     = 'channelListModeUpdate'
        ident        = self.command_dict[cmd_name]['id']
        check_sum    = 0
        field_values = {'Ident': ident, 'Check': check_sum}

        for idx in range(len(self.channel_stim)):
            ch = str(self.channel_stim[idx])
            field_values['Mode' + ch]          = self.STIM_MODE
            field_values['X' + ch]             = 0
            field_values['Pulse_Width' + ch]   = pulse_width[idx]
            field_values['Pulse_Current' + ch] = pulse_current[idx]
            check_sum += self.STIM_MODE + pulse_width[idx] + pulse_current[idx]

        field_values['Check'] = check_sum % 32
        return self.write_read_command(self.create_command(cmd_name, field_values))

    def stop_ccl(self):
        cmd_name     = 'channelListModeStop'
        ident        = self.command_dict[cmd_name]['id']
        field_values = {'Ident': ident, 'Check': 0}
        return self.write_read_command(self.create_command(cmd_name, field_values))

    def ccl_callback(self, request):
        print(request)
        self.update_ccl(request.pulse_width, request.pulse_current)