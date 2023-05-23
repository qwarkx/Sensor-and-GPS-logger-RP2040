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
from scipy.io import savemat

from other_functions import *


"""
    struct sd_data_handler{
        uint32_t timestamp;
        uint32_t packet_count;
        uint32_t packet_count;
        
        //  Movement sensor data
        uint16_t gyiro_x[100];
        uint16_t gyiro_y[100];
        uint16_t gyiro_z[100];
    
        uint16_t acc_x[100];
        uint16_t acc_y[100];
        uint16_t acc_z[100];
    
        uint16_t mag_x[100];
        uint16_t mag_y[100];
        uint16_t mag_z[100];
        uint16_t imu_temp;
    
        // Barometric presur
        uint16_t presure_calculated[10];
        uint16_t temperature[10];
        uint16_t calculated_altitude[10];
    
        //  GPS_Data packet
        uint8_t num_satelites;
        uint8_t fix_type;
    
        double hight_abobe_see_level;
        double ground_speed;
        
        double gps_long;
        double gps_lat;
    };
"""

def export_sensor_data_mat(path, data, size, compression = 1):
    print("Start exporting")

    export_data = []
    channel_number = 22
    export_data = [data[0:, i] for i in range(0, channel_number)]
    export_data = np.array(export_data)

    if compression:
        savemat(path, {'data': export_data}, appendmat=False, do_compression=True)
    else:
        savemat(path, {'data': export_data}, appendmat=False)

    print("Finished exporting")

def main():
    file_path = "logs/"
    file_name = "DATA_009"
    file_extension = ".bin"
    file = file_path + file_name + file_extension

    with open(file, mode='rb') as log_data:
        data_bin = log_data.read()

        # packet_structure = 'II100H100H100H100H100H100H100H100H100H10H10H10H10HBBdddd'
        # packet_structure = 'II50H50H50H50H50H50H50H50H50H10H10H10H10HBBdddd'
        # packet_structure = 'II20H20H20H20H20H20H20H20H20H10H10H10H10HBBdddd'
        # packet_structure = 'IIHHHHHHHHHHHHHBBdddd'
        packet_structure =   'IHIHHHHHHHHHHfffBBdddd'
        structure_size = struct.calcsize(packet_structure)

        # data_package_size = 1920
        data_size = len(data_bin)

        no_packets = data_size/structure_size
        full_data = []

        for i in range(0, int(no_packets)):
            data = struct.unpack(packet_structure, data_bin[structure_size * i : ( i + 1 ) * structure_size])
            full_data.append(data)
        full_data = np.array(full_data)
        print("GPS_latitude:   " + str(full_data[0][-2]))
        print("GPS_longitude:  " + str(full_data[0][-1]))

        exporter = 1
        if exporter:
            file_export_name = file_path + file_name + '_export_compressed.mat'
            export_sensor_data_mat(file_export_name, full_data)


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

        timestamp = (full_data[0:, 0] / 1000000.0)
        packet_counter = full_data[0:, 1]
        sample_counter = full_data[0:, 2]

        see_hight = full_data[0:,-4]
        speed = full_data[0:, -3]

        longitude = full_data[0:, -2]
        latitude = full_data[0:, -1]

        time = np.diff(timestamp)

        gps_data = tuple(zip(latitude, longitude))

        ACCx = removeDC(np.array(full_data[0:, 3], dtype='i2')/8)
        ACCy = removeDC(np.array(full_data[0:, 4], dtype='i2')/8)
        ACCz = removeDC(np.array(full_data[0:, 5], dtype='i2')/8)

        GYx = removeDC(np.array(full_data[0:, 6], dtype='i2')/2000)
        GYy = removeDC(np.array(full_data[0:, 7], dtype='i2')/2000)
        GYz = removeDC(np.array(full_data[0:, 8], dtype='i2')/2000)

        MAGx = removeDC(np.array(full_data[0:, 9], dtype='i2') )
        MAGy = removeDC(np.array(full_data[0:, 10], dtype='i2') )
        MAGz = removeDC(np.array(full_data[0:, 11], dtype='i2') )

        imu_temp = np.array(full_data[0:, 12], dtype='i2')/340 + 36.53
        baro_temp = np.array(full_data[0:, 13], dtype='f')
        baro_altitude = np.array(full_data[0:, 15], dtype='f')

        gps_num_satelit = np.array(full_data[0:, 16], dtype='i1')
        gps_fix_type = np.array(full_data[0:, 17], dtype='i1')

        ACC_all = removeDC(np.array([full_data[0:, 3], full_data[0:, 4], full_data[0:, 5]], dtype='i2') / 8)
        GYRO_all = removeDC(np.array([full_data[0:, 6], full_data[0:, 7], full_data[0:, 8]], dtype='i2') / 2000)
        MAG_all = removeDC(np.array([full_data[0:, 9], full_data[0:, 10], full_data[0:, 11]], dtype='i2') )

        plt.subplot(2,3,1)
        plt.title('GPS Plot')
        plt.scatter(longitude, latitude, zorder=1, alpha=0.2, c='b', s=10)
        plt.grid()

        plt.subplot(2,3,2)
        plt.title('Other Details')
        plt.plot(timestamp, speed)
        plt.plot(timestamp, see_hight)
        plt.plot(timestamp, imu_temp)
        plt.plot(timestamp, baro_temp)
        plt.plot(timestamp, baro_altitude)
        plt.plot(timestamp, gps_num_satelit)
        plt.plot(timestamp, gps_fix_type)

        plt.legend( ['Speed km/h', 'Seelevel hight', 'Imu temp', 'Baro Temp', 'Baro Altitude', 'GPS Num Satelit', 'GPS Fix Type'], loc="best")
        plt.grid()

        plt.subplot(2,3,3)
        plt.title('Packet counter')
        plt.plot(timestamp, packet_counter)
        plt.legend(['Packet counter'], loc="best")
        plt.grid()

        plt.subplot(2, 3, 4)
        plt.title('Packet counter')
        plt.plot(timestamp[:-1], time)
        plt.legend(['time_parts'], loc="best")
        plt.grid()

        GY = []
        shadow = []

        # GY, shadow = y_offsetdddd([GYx, GYy, GYz], 'offset_auto', 3, 1)
        plt.subplot(2, 3, 5)
        plt.title('GYRO')
        # plt.plot(GY)
        plt.plot(timestamp, GYRO_all[0])
        plt.plot(timestamp, GYRO_all[1]-4)
        plt.plot(timestamp, GYRO_all[2]-8)
        plt.legend(['GYx', 'GYz', 'GYy'], loc="best")
        plt.grid()

        plt.subplot(2, 3, 6)
        plt.title('ACC')
        plt.plot(timestamp, ACC_all[0])
        plt.plot(timestamp, ACC_all[1]-800)
        plt.plot(timestamp, ACC_all[2]-1600)
        plt.legend(['ACCx', 'ACCy', 'ACCz'], loc="best")
        plt.grid()
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
