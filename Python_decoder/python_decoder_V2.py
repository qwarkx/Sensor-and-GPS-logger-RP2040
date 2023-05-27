import os, re
import functools
import ctypes
from ctypes import string_at, byref, sizeof, cast, POINTER, pointer, create_string_buffer, memmove

import pandas as pd

from PIL import Image, ImageDraw

import struct
import numpy as np

from matplotlib.pyplot import plot, show
import matplotlib.pyplot as plt
import scipy.io as sp

from other_functions import *

"""
    struct sd_data_handler{
        uint32_t timestamp;
        uint32_t packet_count;
        uint32_t packet_count;

        //  Movement sensor data
        uint16_t gyiro_x;
        uint16_t gyiro_y;
        uint16_t gyiro_z;

        uint16_t acc_x;
        uint16_t acc_y;
        uint16_t acc_z;

        uint16_t mag_x;
        uint16_t mag_y;
        uint16_t mag_z;
        uint16_t imu_temp;

        // Barometric presur
        uint16_t presure_calculated;
        uint16_t temperature;
        uint16_t calculated_altitude;

        //  GPS_Data packet
        uint8_t num_satelites;
        uint8_t fix_type;

        double hight_abobe_see_level;
        double ground_speed;

        double gps_long;
        double gps_lat;
    };
"""


def export_sensor_data_mat(path, data, channel_names, channel_number = 22, compression=1):
    print("Start exporting")

    if not len(path) > 2 and not len(data) > 0:
        return 0

    export_data = []
    export_data = [data[0:, i] for i in range(0, channel_number)]
    export_data = np.array(export_data)

    try:
        if compression:
            sp.savemat(path, {'designators':channel_names, 'data': export_data}, appendmat=False, do_compression=True)
            print("Finished exporting")
            return 1
        else:
            sp.savemat(path, {'designators':channel_names, 'data': export_data}, appendmat=False)
            print("Finished exporting")
            return 1
    except Exception as e:
        print('Something happened by exporting the MAT File:  ', e)
        return 0


def main():
    file_path = "logs/"
    file_name = "DATA_014"
    file_extension = ".bin"
    file = file_path + file_name + file_extension

    with open(file, mode='rb') as log_data:
        data_bin = log_data.read()

        # packet_structure = 'II100H100H100H100H100H100H100H100H100H10H10H10H10HBBdddd'
        # packet_structure = 'II50H50H50H50H50H50H50H50H50H10H10H10H10HBBdddd'
        # packet_structure = 'II20H20H20H20H20H20H20H20H20H10H10H10H10HBBdddd'
        # packet_structure = 'IIHHHHHHHHHHHHHBBdddd'
        # packet_structure = 'IHIHHHHHHHHHHfffBBdddd'
        packet_structure =   'IIIhhhhhhhhhhfffBBdddd'
        structure_size = struct.calcsize(packet_structure)
        data_channel_number = len(packet_structure)

        # data_package_size = 1920
        data_size = len(data_bin)

        no_packets = data_size / structure_size
        full_data = []

        for i in range(0, int(no_packets)):
            data = struct.unpack(packet_structure, data_bin[structure_size * i: (i + 1) * structure_size])
            full_data.append(data)

        #  CONVERT TO NP ARRAY
        full_data = np.array(full_data)

        exporter = 1
        if exporter:
            file_export_name = file_path + file_name + '_matlab.mat'

            data_designators = ['timestamp', 'system_cpu_ticks', 'sd_write_counter',
                                'ACCx', 'ACCy', 'ACCz', 'GYx', 'GYy', 'GYz', 'MAGx', 'MAGy', 'MAGz',
                                'IMU_TEMP', 'BARO_TEMP', 'BARO_ALTITUDE',
                                'GPS_NUM_SAT', 'GPS_FIX_TYPE', 'GPS_SEE_HIGHT', 'GPS_SPEED_m/s', 'GPS_LONG', 'GPS_LAT']

            export_sensor_data_mat(file_export_name, full_data, data_designators, data_channel_number)

        longitude = []
        latitude = []
        speed = []
        see_hight = []
        timestamp = []
        packet_counter = []

        GYx = []
        GYy = []
        GYz = []
        ACCx = []
        ACCy = []
        ACCz = []

        timestamp = np.array(full_data[0:, 0], dtype='i4')
        packet_counter = np.array(full_data[0:, 1], dtype='i4')
        sample_counter = np.array(full_data[0:, 2], dtype='i4')

        time = np.diff(timestamp)

        imu_temp = np.array(full_data[0:, 12], dtype='i2') / 340 + 36.53
        baro_temp = np.array(full_data[0:, 13], dtype='f')
        baro_altitude = np.array(full_data[0:, 15], dtype='f')

        gps_num_satelit = np.array(full_data[0:, 16], dtype='i1')
        gps_fix_type = np.array(full_data[0:, 17], dtype='i1')

        see_hight = np.array(full_data[0:, -4], dtype='f')
        speed = np.array(full_data[0:, -3], dtype='d') * 0.0036  # Convert to km/h
        # speed = np.array(full_data[0:, -3], dtype='f')  # Convert to km/h

        longitude = np.array(full_data[0:, -2], dtype='c16')  # c16
        latitude = np.array(full_data[0:, -1], dtype='c16')
        gps_data = tuple(zip(latitude, longitude))


        ACC_all = removeDC(np.array([full_data[0:, 3], full_data[0:, 4], full_data[0:, 5]], dtype='i2') )  #/8
        GYRO_all = removeDC(np.array([full_data[0:, 6], full_data[0:, 7], full_data[0:, 8]], dtype='i2') )  #/2000
        MAG_all = removeDC(np.array([full_data[0:, 9], full_data[0:, 10], full_data[0:, 11]], dtype='i2'))  #/2.5

        plt.subplot(2, 4, 1)
        plt.title('GPS Plot')
        plt.scatter(longitude, latitude, zorder=1, alpha=0.2, c='b', s=10)
        plt.grid()

        GY = []
        shadow = []

        # GY, shadow = y_offsetdddd([GYx, GYy, GYz], 'offset_auto', 3, 1)
        plt.subplot(2, 4, 2)
        plt.title('GYRO')
        # plt.plot(GY)
        plt.plot(timestamp, GYRO_all[0])
        plt.plot(timestamp, GYRO_all[1] - 4)
        plt.plot(timestamp, GYRO_all[2] - 8)
        plt.legend(['GYx', 'GYz', 'GYy'], loc="best")
        plt.grid()

        plt.subplot(2, 4, 3)
        plt.title('ACC')
        plt.plot(timestamp, ACC_all[0])
        plt.plot(timestamp, ACC_all[1] - 800)
        plt.plot(timestamp, ACC_all[2] - 1600)
        plt.legend(['ACCx', 'ACCy', 'ACCz'], loc="best")
        plt.grid()

        plt.subplot(2, 4, 4)
        plt.title('MAG')
        plt.plot(timestamp, MAG_all[0])
        plt.plot(timestamp, MAG_all[1])
        plt.plot(timestamp, MAG_all[2])
        plt.legend(['MAGx', 'MAGy', 'MAGz'], loc="best")
        plt.grid()


        plt.subplot(2, 4, 5)
        plt.title('Packet counter - ' + 'Avarange time: ' + str(np.average(time))[0:6])
        plt.plot(timestamp[:-1], time)
        plt.legend(['time_parts'], loc="best")
        plt.grid()


        plt.subplot(2, 4, 6)
        plt.title('Sample counter')
        plt.plot(timestamp, packet_counter)
        plt.plot(timestamp, sample_counter)
        plt.legend(['Packet counter', 'Sample counter'], loc="best")
        plt.grid()


        plt.subplot(2, 4, 7)
        plt.title('Other Details' )
        plt.plot(timestamp, speed)
        plt.plot(timestamp, see_hight)
        plt.plot(timestamp, imu_temp)
        plt.plot(timestamp, baro_temp)
        plt.plot(timestamp, baro_altitude)
        plt.plot(timestamp, gps_num_satelit)
        plt.plot(timestamp, gps_fix_type)

        plt.legend(['Speed km/h', 'Seelevel hight', 'Imu temp', 'Baro Temp', 'Baro Altitude', 'GPS Num Satelit',
                    'GPS Fix Type'], loc="best")
        plt.grid()


        plt.subplot(2, 4, 8)
        plt.title('Sample counter')
        # plt.plot(timestamp, packet_counter)
        plt.plot(timestamp, sample_counter)
        plt.legend(['Sample counter'], loc="best")
        plt.grid()

        # SHOW the GRAPHS
        plt.subplots_adjust(top=0.95, bottom=0.05, # Topp biger than bottom
                            left=0.05, right=0.95,   # left smaller than right
                            hspace=0.2, wspace=0.2)
        # plt.margins(0,0)
        plt.show()



        # fig = plt.gcf()
        # manager = plt.ge_current_fig_manager()
        # Set the plot to full screen
        # manager.full_screen_toggle()

        plt.savefig(file_name + '.png', dpi=640)
        plt.show()

        print("Finished analisation")


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()
