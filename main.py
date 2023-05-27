import struct

from machine import SPI, I2C, UART, Pin, Timer
import machine
# from machine import SDCard
import os
import uos
import time

import uctypes as c
#from typing import List

#from dataclasses import dataclass
import sdcard
# from machine import SDCard
from sensor_IMU import GY_86
from bmp180 import BMP180
from ublox_GPS import GPS
#from data_packet import *
import itertools as it


        
# processGPS();
# calcChecksum(unsigned char* CK, unsigned char* payload, uint8_t length);

# Initialise_SD_CARD();
# setLOG_file_name();
# checkFileExist(const char * fileNameInput);



# SD CARD Handling
SD_CARD_CS_PIN = 15

SD_CARD_CONNECTED = 6
# IMU handling
INT_PIN_IMU_sens = 22
# button handling
INT_CENTER_BUTTON = 24

#Other Defines
MAX_IMU_DATA_PACKET = 100


# OTHER VARIABLES
time_stamp = 0
packet_counter = 0
sample_counter = 0

sens_RDY_flag = 1

SD_write_data_started = 0
SD_fileName_flags = 0
SD_Write_data = 0
pin_int_sd_write = 0

pin_sd_card_connected = 0
sd_card_init_required = 0

myFile = None

def Initialise_SD_CARD( spi_interface, sd_cs_pin, sd_path = '/sd', timeout = 2):
    sd_mount_path = sd_path
    init_timeout = 0
    status = 0

    while init_timeout < timeout:
        try:
            sd_card_class = sdcard.SDCard(spi_interface, sd_cs_pin)
            uos.mount(sd_card_class, sd_mount_path)
            status = 1

            print('SD card initialised')
        except Exception as e:
            print('Not able to initialised SD card see details: ', e)
            sd_card_init_required = 1
            status = 0

        time.sleep(1)

        if status == 0:
            time.sleep(1)
            init_timeout += 1
        elif status == 1:
            return status, sd_mount_path, sd_card_class

    return status, sd_mount_path, sd_card_class

def SDtimer_interrupt_callback(timer):
    global pin_sd_card_connected
    global sd_card_init_required
    global sd_connected
    # print(sd_connected.value())
    # print(pin_int_center)
    if sd_connected.value() == 0:
        pin_sd_card_connected = 1
    else:
        pin_sd_card_connected = 0
        sd_card_init_required = 1
# SD card connected checker functionality
sd_connected = Pin(SD_CARD_CONNECTED, Pin.IN, Pin.PULL_UP)
sd_check_timer = Timer(mode=Timer.PERIODIC, period=500, callback=SDtimer_interrupt_callback)


# // SENSOR PIN INTERRUPT FUNCTION
def read_IMU_sensor(pin):
    global sens_RDY_flag
    global time_stamp
    sens_RDY_flag = 1
    time_stamp = time_stamp + 1
# Sensor Pin interrupt
imu_int = Pin(INT_PIN_IMU_sens, machine.Pin.IN)
imu_int.irq(trigger=Pin.IRQ_RISING, handler=read_IMU_sensor)


# // PIN INTERRUPT HANDLING FUNCTIONS
def start_logging_button(pin):
    global SD_write_data_started
    if SD_write_data_started == 0:
        SD_write_data_started = 1
    elif SD_write_data_started == 1:
        SD_write_data_started = 0
# Button interrupts
user_control = Pin(INT_CENTER_BUTTON, machine.Pin.IN, machine.Pin.PULL_UP)
user_control.irq(trigger=Pin.IRQ_FALLING, handler=start_logging_button)


# FILE NAME HANDLING
log_file_name = ""

def generate_log_file_name():
    global log_file_name
    log_file_name_generator = ("DATA_{:03d}.bin".format(i) for i in it.count(1))
    log_file_name = next(log_file_name_generator)
    l_name = log_file_name
    return l_name

def file_or_dir_exists(filename, path):
    try:
        if filename in uos.listdir(path):
            return True
        else:
            return False
    except OSError:
        return False


def set_log_file_name(sd_path='/sd'):
    global log_file_name

    log_file_name_generator = ("DATA_{:03d}.bin".format(i) for i in it.count(1))
    log_file_name = next(log_file_name_generator)

    while file_or_dir_exists(log_file_name, sd_path):
        log_file_name = next(log_file_name_generator)

    return log_file_name

'''
def set_log_file_name(sd_path = '/sd'):
    global log_file_name
    name = ''
    generate_log_file_name()
    print(log_file_name)
    # print(uos.listdir(sd_path))
    # print(file_or_dir_exists(log_file_name, sd_path))

    while file_or_dir_exists(log_file_name, sd_path):
        name = generate_log_file_name()
        print(log_file_name)

    print(log_file_name)
    return name
'''
# Example usage
#set_log_file_name()
#print(log_file_name)



def SD_Write_Start_Stop_control( sensor_data, sd_path = '/sd'):
    global SD_write_data_started
    global SD_fileName_flags
    global log_file_name
    global SD_fileOpen_flags
    global SD_Write_data
    global myFile
    # SD_init_status
    # Generate name of the file


    if SD_write_data_started == 0 and SD_fileName_flags == 0:
        # print('SD start failed')
        SD_Write_data = 0
        return 0
    elif SD_write_data_started == 1 and SD_fileName_flags == 0:
        set_log_file_name(sd_path)
        file_path = sd_path + '/' + log_file_name
        print(file_path)

        myFile = open(file_path, 'wb')
        print('SD data write started')
        SD_fileName_flags = 1
        SD_fileOpen_flags = 1
        SD_Write_data = 1

    # Open the file for data write
    if SD_write_data_started == 1 and SD_fileName_flags == 1 and SD_fileOpen_flags == 1:
        # print('write')
        myFile.write(sensor_data)

    elif SD_write_data_started == 0 and SD_fileName_flags == 1 and SD_fileOpen_flags == 1:
        SD_fileName_flags = 0
        SD_fileOpen_flags = 0
        # print('up')
        if myFile is not None:
            myFile.flush()
            myFile.close()
            print('SD Write finished')



#data_structure = SD_DataHandler() # possible error source

# OTHER initialisations

def main():

    # Sleep litle bit before start
    time.sleep(1)

    # Initialise Information PIN
    LED = Pin(25, machine.Pin.OUT)
    LED.value(1)

    # -----------------------------------------------------------------------------------

    print('* Initialising UART interface and GPS ...')
    # gps_speed = 19200
    gps_speed = 115200
    uart_com_gps = UART(1, baudrate = gps_speed, tx = Pin(8), rx = Pin(9))
    try:
        # GPS setup
        GPS_Serial = GPS(uart_com_gps)
        time.sleep(1)
        GPS_Serial.read_GPS()
        time.sleep(1)
    except Exception as e:
       print('ERROR with the GPS')

    # -----------------------------------------------------------------------------------

    # SD card setup
    print('* Initialising SD CARD SPI interface ...')
    SD_CARD_CS = 15
    sd_cs = Pin(SD_CARD_CS, Pin.OUT)

    global sd_card_init_required
    #global status, sd_path, other

    spi = machine.SPI(id=1,
                      baudrate=1000000,
                      polarity=0,
                      phase=0,
                      bits=8,
                      firstbit=machine.SPI.MSB,
                      sck=machine.Pin(10),
                      mosi=machine.Pin(11),
                      miso=machine.Pin(12))

    print('* Initialising SD CARD ...')
    status, sd_path, sd_cls = Initialise_SD_CARD(spi, sd_cs, '/sd', timeout=2)
    if status:
        sd_card_init_required = 0
    else:
        sd_card_init_required = 1
    print('* SD CARD Finished config')
    time.sleep(1)

    # -----------------------------------------------------------------------------------

    # SETUP Barometer tio get altitude
    try:
        print('BARO init STARTED')
        baro = BMP180(interface=0, scl=21, sda=20, frq=400000,  baseline=101325.0, oversample=3)
        print('BARO init Finished')
    except Exception as e:
        print('Something happened in the initialisation of the Barometer', e)
    # Give some Delay
    time.sleep(1)

    # -----------------------------------------------------------------------------------

    # // Start the setup of the IMU sensors
    imu = GY_86(interface=0, scl=21, sda=20, frq=400000)
    # accX, accY, accZ, Temp, gyX, gyY, gyZ, magX, magY, magZ = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    # sens_data = []
    # imu.read_sensors()

    # Imu initialisation
    if imu.GY_ACC_MAG_Init():
        print('IMU init OK')
    else:
        print('IMU init NOT OK')

    # -----------------------------------------------------------------------------------

    # OTHER initialisations
    #buffer_size = 100
    #buffer = CircularBuffer(buffer_size)

    baro_sens_data = []
    gps_sens_data = []
    
    global packet_counter
    global sample_counter
    global time_stamp
    global sd_card_init
    global pin_sd_card_connected

    # Give some delay
    time.sleep(0.5)

    # Disable LED to show Configuration ended
    LED.value(0)

    while True:

        if pin_sd_card_connected == 0 and sd_card_init_required == 1:
            sd_status, sd_path, sd_cls = Initialise_SD_CARD(spi, sd_cs, '/sd', timeout=2)
            sd_card_init_required = sd_status

        if SD_Write_data:
            LED.value(1)
        else:
            LED.value(0)

        try:
            temperature = baro.temperature
            presure = baro.pressure
            alt = baro.altitude
            baro_sens_data = (temperature, presure, alt)
        except Exception as e:
            print('Something happened while reading the BAROMETER : ', e)
            LED.value(1)

        # USE GPS to read data from the module
        if GPS_Serial.read_GPS():
            # // GPS related information
            gps_sens_data = (GPS_Serial.numSV, GPS_Serial.fixType, GPS_Serial.hMSL/1000.0, GPS_Serial.gSpeed, GPS_Serial.lon/10000000.0, GPS_Serial.lat/10000000.0)

        # FILL the BUFFER
        if sens_RDY_flag == 1:

            imu_sens_data = imu.read_sensors()
            packet_counter = time.ticks_cpu()

            # print(time_stamp, packet_counter, sample_counter, imu_sens_data, baro_sens_data)
            # buffer.add_data(time_stamp, packet_counter, imu_sens_data, baro_sens_data, gps_sens_data)
            #print(str(imu_sens_data[7]), str(imu_sens_data[8]), str(imu_sens_data[9]))

            # data = struct.pack('IIIHHHHHHHHHHfffBBdddd',
            data = struct.pack(  'IIIhhhhhhhhhhfffBBdddd',
                                time_stamp, packet_counter, sample_counter,
                                imu_sens_data[0], imu_sens_data[1], imu_sens_data[2],
                                imu_sens_data[4], imu_sens_data[5], imu_sens_data[6],
                                imu_sens_data[7], imu_sens_data[8], imu_sens_data[9],
                                imu_sens_data[3],
                                baro_sens_data[0], baro_sens_data[1], baro_sens_data[2],
                                gps_sens_data[0], gps_sens_data[1], gps_sens_data[2],
                                gps_sens_data[3], gps_sens_data[4], gps_sens_data[5])
            

            SD_Write_Start_Stop_control(data)

            # time_meaure =  time.ticks_us()
            #print(str(time.ticks_us()-time_meaure))
            
            sample_counter += 1


if __name__ == '__main__':
    main()
