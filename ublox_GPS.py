import math
import machine
from machine import Pin
from machine import UART
#from micropython import const
from ustruct import pack
from array import array
import utime
import binascii
import utime
import struct



class GPS():

    def __init__(self, serial_port):
        
        self.gps_seriall = serial_port
        self.gps_data_raw = []
            

        
        self.cls = 0
        self.id = 0
        self.len = 0
        self.iTOW = 0
        self.year = 0
        self.month = 0
        self.day = 0
        self.hour = 0
        self.minute = 0
        self.second = 0
        self.valid = 0
        self.tAcc = 0
        self.nano = 0
        self.fixType = 0
        self.flags = 0
        self.flags2 = 0
        self.numSV = 0
        
        self.lon = 0
        self.lat = 0
        self.height = 0
        self.hMSL = 0
        self.hAcc = 0
        self.vAcc = 0
        
        self.velN = 0
        self.velE = 0
        self.velD = 0
        
        self.gSpeed = 0
        self.headMot = 0
        self.sAcc = 0
        self.headAcc = 0
        self.pDOP = 0
        self.flags3 = 0
        self.reserved1 = 0
        self.headVeh = 0
        self.magDec = 0
        self.magAcc = 0
        
        self.data = []
        

    def ubx_NAV_PVT(self, gps_data):
        # remove first two bytes
        payload = gps_data[2:]
        payload_cpy = []
        
        if(self.__calc_checksum(1, 7, payload)):
            
            try:
                payload_cpy = payload
                #data_struct = '=LH5BBLlB2BB4l2L5lLLH6BlhH'            
                data_struct = 'BBHLHBBBBBBLlBBBBllllLLlllllLLHhLlhH'
                
                #print('adatmeret:', int(struct.calcsize(data_struct)))
                
                self.cls, self.id, self.len, self.iTOW, self.year, self.month, self.day, self.hour, self.minute, self.second, self.valid, self.tAcc, self.nano, self.fixType, self.flags, self.flags2, self.numSV, self.lon, self.lat, self.height, self.hMSL, self.hAcc, self.vAcc, self.velN, self.velE, self.velD, self.gSpeed, self.headMot, self.sAcc, self.headAcc, self.pDOP, self.flags3, self.reserved1, self.headVeh, self.magDec, self.magAcc = struct.unpack(data_struct, payload_cpy)

                self.ubx_class = '01'
                self.ubx_id = '07'
                
                return 1
                
            except Exception as e:
                print('something happened: --- ', e)
                #Eprint("{} {}".format(sys.exc_info()[0], sys.exc_info()[1]))
        else:
            return 0
            
    def read_GPS(self):
        
        while(self.gps_seriall.any()):
            self.gps_data_raw = self.gps_seriall.read()
            
            if self.ubx_NAV_PVT(gps_data_raw):
                return 1
            else:
                return 0 
                
    def __validate_checksum(self, ubx_class, ubx_id, payload, dev):
        check1 = (ubx_class + ubx_id) % 256
        check2 = ((2*ubx_class) + ubx_id) % 256

        if self._version == 3:
            chk1 = dev.read()[0]
            chk2 = dev.read()[0]

            for i in range(0, len(payload)):
                check1 = (check1 + payload[i]) % 256
                check2 = (check1 + check2) % 256
                
        if chk1==check1 and chk2==check2:
            return True
        else:
            print("Checksum is incorrect")
            return False

    def __calc_checksum(self, ubx_class, ubx_id, payload):
        check1 = (ubx_class + ubx_id) % 256
        check2 = ((2*ubx_class) + ubx_id) % 256

        for i in range(0, len(payload)):
            check1 = (check1 + payload[i]) % 256
            check2 = (check1 + check2) % 256

        result = [check1, check2]
        print(result)
        return result



# GPS Testing

uart_com = UART(1, baudrate = 115200, tx = Pin(8), rx = Pin(9))
gp = GPS(uart_com)

while True:
    
    '''
    gps.update()
    
    print("latitude: {}\nlongitude: {}\nfix_quality: {}\nsatellites: {}\naltitude_m: {}".format(gps.latitude,
                                                                                                gps.longitude,
                                                                                                gps.fix_quality,
                                                                                                gps.satellites,
                                                                                                gps.altitude_m))
    '''
    gp.read_GPS()
    
    print(gp.lat)
    print(gp.gps_data_raw[1])
        
    utime.sleep(1)










