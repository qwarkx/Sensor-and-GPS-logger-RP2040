

from machine import SPI, I2C, Pin, Timer
import machine

import os
import uos
import time

import uctypes as c
#from typing import List

#from dataclasses import dataclass
import sdcard
from sensor_IMU import GY_86
from bmp180 import BMP180
from data_class import *
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

SD_write_data_started_stopped = 0
pin_int_center = 0

message_updater_millis = 0
SD_Write_data = 0

pin_sd_card_connected = 0
sd_card_init_required = 0

'''
def Initialise_SD_CARD( timeout = 2, spi_interface, sd_cs_pin):
    global sd_cs
    global sd_card_init_required

    init_timeout = 0
    status = False

    while init_timeout < timeout:
        try:
            sd = sdcard.SDCard(spi_interface, sd_cs_pin)
            uos.mount(sd, '/sd2')
            # uos.listdir('/')
            print(uos.listdir('/'))

            sd_card_init_required = 0
            status = True

            print('SD card initialised')
        except Exception as e:
            print('Not able to initialised SD card')
            sd_card_init_required = 1
            status = False

        print('Storage interface Finished config')
        time.sleep(3)

        if status == False:
            time.sleep_ms(400)
            init_timeout += 1
        elif status == True:
            return True
    return False
'''
'''
def SD_Auto_Init():
	if (SD_AutoInit > 2000 and SD_init_status == 0):
		SD_AutoInit = 0
		status = Initialise_SD_CARD()
		if (status == True):
			SD_init_status = 1
		elif (status == False):
			SD_init_status = 0
'''

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
    # time_stamp = time_stamp + 1
# Sensor Pin interrupt
imu_int = Pin(INT_PIN_IMU_sens, machine.Pin.IN)
imu_int.irq(trigger=Pin.IRQ_RISING, handler=read_IMU_sensor)


# // PIN INTERRUPT HANDLING FUNCTIONS
def start_logging_button(pin):
    global pin_int_center
    if pin_int_center == 0:
        pin_int_center = 1
    elif pin_int_center == 1:
        pin_int_center = 0
# Button interrupts
user_control = Pin(INT_CENTER_BUTTON, machine.Pin.IN, machine.Pin.PULL_UP)
user_control.irq(trigger=Pin.IRQ_FALLING, handler=start_logging_button)




# FILE NAME HANDLING
log_file_name = ""

def generate_log_file_name():
    global log_file_name
    log_file_name_generator = ("DATA_{:03d}.bin".format(i) for i in itertools.count(1))
    log_file_name = next(log_file_name_generator)
    l_name = log_file_name
    return l_name

def set_log_file_name(sd_class):
    global log_file_name
    generate_log_file_name()
    while sd_class.exists(log_file_name):
        name = generate_log_file_name()
    return name

# Example usage
#set_log_file_name()
#print(log_file_name)


'''
def SD_Write_Start_Stop_control( sd_conrol_class):
	# SD_init_status
	# Generate name of the file
	if (SD_write_data_started_stopped):
		SD_write_data_started_stopped = 0
		print('SD data write stopped')
	else:
		SD_fileName_flags = setLOG_file_name()
		if SD_fileName_flags:
			SD_write_data_started_stopped = 1
			print('SD data write started')
		else:
			print('SD start failed')

	# Open the file for data write
	if (SD_write_data_started_stopped == 1):

		# try:

		# except Exception as e:

		myFile = sd_conrol_class.open(SD_fileName, 'wb')

		if myFile:
			SD_fileOpen_flags = 0
			SD_write_data_started_stopped = 0
		else:
			SD_fileOpen_flags = 1
			myFile.write((uint8_t * ) & sd_data, sizeof(sd_data))
			myFile.close()
			SD_fileOpen_flags = 0
			SD_write_data_started_stopped = 1

	if (SD_write_data_started_stopped == 1):
		myFile = SD.open(SD_fileName, FILE_WRITE)
		if (!myFile):
			SD_fileOpen_flags = 0
			SD_write_data_started_stopped = 0
		else:
			SD_fileOpen_flags = 1
		SD_write_data_started_stopped = 1

	else:
		myFile.close()
		SD_fileOpen_flags = 0
'''
#pin_int_center = 0

#data_structure = SD_DataHandler() # possible error source

# OTHER initialisations


def main():

    time.sleep(3)
    
    LED = Pin(25, machine.Pin.OUT)

    print('* Initialising GPS ...')

    # uart_com_gps = UART(1, baudrate = 115200, tx = Pin(8), rx = Pin(9))
    # GPS_Serial = GPS(uart_com_gps)
    # time.sleep(1)
    # gp.read_GPS()

    # print(gp.lat)
    # print(gp.gps_data_raw[1])

    # // posible to increase the speed of GPS to 5Hz
    # GPS_Serial.write(UBLOX_INIT, sizeof(UBLOX_INIT));
    # //-----------------------------------------------------------------------------------

    # SD card setup

    SD_CARD_CS = const(15)
    sd_cs = Pin(SD_CARD_CS, Pin.OUT)

    global sd_card_init_required

    spi = machine.SPI(id=1,
                      baudrate=1000000,
                      polarity=0,
                      phase=0,
                      bits=8,
                      firstbit=machine.SPI.MSB,
                      sck=machine.Pin(10),
                      mosi=machine.Pin(11),
                      miso=machine.Pin(12))

    try:
        sd = sdcard.SDCard(spi, sd_cs)
        uos.mount(sd, '/sd2')
        uos.listdir('/')

        sd_card_init_required = 0
        print('SD card initialised')
    except Exception as e:
        print('Not able to initialised SD card')
        sd_card_init_required = 1
    
    print('Storage interface Finished config')
    time.sleep(3)
    
    

    #print(sd)
    # Set all SD data to 0xFF
    # memset(&sd_data, 0xFF, sizeof(sd_data)) ;

    # // Try to initialize card
    '''
    if Initialise_SD_CARD():
        SD_init_status = 1
    else:
        SD_init_status = 0
    '''
    # //-----------------------------------------------------------------------------------

    # // SETUP Barometer tio get altitude
    print('BARO init STARTED')
    baro = BMP180(interface=0, scl=21, sda=20, frq=400000,  baseline=101325.0, oversample=3)
    print('BARO init END')

    time.sleep(10)
    # // Start setup the sensors to gather data

    imu = GY_86(interface=0, scl=21, sda=20, frq=400000)
    # accX, accY, accZ, Temp, gyX, gyY, gyZ, magX, magY, magZ = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    # sens_data = []
    # imu.read_sensors()

	# Imu initialisation
    if (imu.GY_ACC_MAG_Init()):
        print('IMU init OK')
    else:
        print('IMU init NOT OK')
 

    # OTHER initialisations
    buffer_size = 100
    buffer = CircularBuffer(buffer_size)


    imu_sens_data = bytearray(20)
    baro_sens_data = []
    gps_sens_data = []
    
    global packet_counter
    global sample_counter
    global time_stamp

    global sd_card_init
    global pin_sd_card_connected
    # // Desyncrhonize the beeps
    time.sleep(2)

    while True:

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
            print(e)
            LED.value(1)



        # // TRY to AUTOmaticaly initialise SD card
        #if SD_init_status == 0:
        # SD_Auto_Init()


        # USE GPS to read data from the module
        # if (message_updater_millis > 200):
        # message_updater_millis = 0
        # if GPS_Serial.read_GPS():
        # // GPS related information
        # gps_sens_data = (GPS_Serial.numSV, GPS_Serial.fixType, GPS_Serial.hMSL/1000.0f, GPS_Serial.gSpeed*0.0036f, GPS_Serial.lon/10000000.0f, GPS_Serial.lat/10000000.0f)

        # FILL the BUFFER
        if sens_RDY_flag == 1:
            
            time_stamp = time.ticks_us()
            imu_sens_data = imu.read_sensors()

            # print(time_stamp, packet_counter, sample_counter, imu_sens_data, baro_sens_data)
            #buffer.add_data(timestamp, packet_counter, imu_sens_data, baro_sens_data, gps_data)
            
            sample_counter += 1


if __name__ == '__main__':
	main()
