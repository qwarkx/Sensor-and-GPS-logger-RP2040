



import gc
from machine import Pin, I2C, PWM
import time
import micropython
from micropython import const

from ustruct import unpack


#import cfilter



'''
// ************************************************************************************************************
// I2C Compass HMC5883
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************
'''

HMC58X3_R_CONFA 			 = const(0 )
HMC58X3_R_CONFB              = const(1 )
HMC58X3_R_MODE               = const(2 )
HMC58X3_X_SELF_TEST_GAUSS    = const(1.16 )        #//!< X axis level when bias current is applied.
HMC58X3_Y_SELF_TEST_GAUSS    = const(1.16 )        #//!< Y axis level when bias current is applied.
HMC58X3_Z_SELF_TEST_GAUSS    = const(1.08 )        #//!< Y axis level when bias current is applied.
#SELF_TEST_LOW_LIMIT          = const((243.0/390.0) )  #//!< Low limit when gain is 5.
#SELF_TEST_HIGH_LIMIT         = const((575.0/390.0) )  #//!< High limit when gain is 5.
HMC_POS_BIAS                 = const(1 )
HMC_NEG_BIAS                 = const(2 )

MAG_DATA_REGISTER 			 = const(0x03 )

#// Standard data register
HMC5883L_ADDRESS             = const(0x1E)
HMC5883L_REG_CONFIG_A        = const(0x00)
HMC5883L_REG_CONFIG_B        = const(0x01)
HMC5883L_REG_MODE            = const(0x02)
HMC5883L_REG_OUT_X_M         = const(0x03)
HMC5883L_REG_OUT_X_L         = const(0x04)
HMC5883L_REG_OUT_Z_M         = const(0x05)
HMC5883L_REG_OUT_Z_L         = const(0x06)
HMC5883L_REG_OUT_Y_M         = const(0x07)
HMC5883L_REG_OUT_Y_L         = const(0x08)
HMC5883L_REG_STATUS          = const(0x09)
HMC5883L_REG_IDENT_A         = const(0x0A)
HMC5883L_REG_IDENT_B         = const(0x0B)
HMC5883L_REG_IDENT_C         = const(0x0C)

#//Samples per recording
HMC5883L_SAMPLES_8     = const(0b11)
HMC5883L_SAMPLES_4     = const(0b10)
HMC5883L_SAMPLES_2     = const(0b01)
HMC5883L_SAMPLES_1     = const(0b00)

#// Data aquisition config register
HMC5883L_DATARATE_75HZ       = const(0b110)
HMC5883L_DATARATE_30HZ       = const(0b101)
HMC5883L_DATARATE_15HZ       = const(0b100)
HMC5883L_DATARATE_7_5HZ      = const(0b011)
HMC5883L_DATARATE_3HZ        = const(0b010)
HMC5883L_DATARATE_1_5HZ      = const(0b001)
HMC5883L_DATARATE_0_75_HZ    = const(0b000)

#// Sensor ranges
HMC5883L_RANGE_8_1GA     = const(0b111)
HMC5883L_RANGE_5_6GA     = const(0b110)
HMC5883L_RANGE_4_7GA     = const(0b101)
HMC5883L_RANGE_4GA       = const(0b100)
HMC5883L_RANGE_2_5GA     = const(0b011)
HMC5883L_RANGE_1_9GA     = const(0b010)
HMC5883L_RANGE_1_3GA     = const(0b001)
HMC5883L_RANGE_0_88GA    = const(0b000)

#// other reception config registers
HMC5883L_IDLE          = const(0b10)
HMC5883L_SINGLE        = const(0b01)
HMC5883L_CONTINOUS     = const(0b00)


#CONFIG_REGISTER_A(number_samples, output_rate, bias_config) (number_samples<<5 | output_rate<<2 | bias_config)
#CONFIG_REGISTER_B(gain_settings) (gain_settings<<5)
#MODE_REGISTER(hspeed_settings, mode_select) (hspeed_settings<<7 | mode_select)


'''
// ************************************************************************************************************
// I2C Compass MPU6050
// ************************************************************************************************************
// I2C adress: 0xD0 (8bit)   0x68 (7bit)
// ************************************************************************************************************
'''

#// Used Registers
MPU6050_I2C_ADDRES        = const(0x68 )#///< MPU6050 default i2c address w/ AD0 high
MPU6050_DEVICE_ID         = const(0x68 )#///< The correct MPU6050_WHO_AM_I value

MPU6050_SELF_TEST_X       = const(0x0D )#///< Self test factory calibrated values register
MPU6050_SELF_TEST_Y       = const(0x0E )#///< Self test factory calibrated values register
MPU6050_SELF_TEST_Z       = const(0x0F )#///< Self test factory calibrated values register
MPU6050_SELF_TEST_A       = const(0x10 )#///< Self test factory calibrated values register
MPU6050_SMPLRT_DIV        = const(0x19 )#///< sample rate divisor register
MPU6050_CONFIG            = const(0x1A )#///< General configuration register
MPU6050_GYRO_CONFIG       = const(0x1B )#///< Gyro specfic configuration register
MPU6050_ACCEL_CONFIG      = const(0x1C )#///< Accelerometer specific configration register
MPU6050_FIFO_EN           = const(0x23 )

#// I2C master slave control registers
MPU6050_I2C_MST_CTRL      = const(0x24)

MPU6050_I2C_SLV0_ADDR     = const(0x25)
MPU6050_I2C_SLV0_REG      = const(0x26)
MPU6050_I2C_SLV0_CTRL     = const(0x27)

MPU6050_I2C_MST_STATUS    = const(0x36)

#//Interrupt configurtation
MPU6050_INT_PIN_CONFIG    = const(0x37 )#///< Interrupt pin configuration register
MPU6050_INT_ENABLE        = const(0x38 )#///< Interrupt enable configuration register
MPU6050_INT_STATUS        = const(0x3A )#///< Interrupt status register

#//Data Registers
MPU6050_ACCEL_XOUT_H      = const(0x3B )#///< base address for sensor data reads
MPU6050_ACCEL_XOUT_L      = const(0x3C)
MPU6050_ACCEL_YOUT_H      = const(0x3D)
MPU6050_ACCEL_YOUT_L      = const(0x3E)
MPU6050_ACCEL_ZOUT_H      = const(0x3F)
MPU6050_ACCEL_ZOUT_L      = const(0x40)
MPU6050_TEMP_OUT_H        = const(0x41 )#///< Temperature data high byte register
MPU6050_TEMP_OUT_L        = const(0x42 )#///< Temperature data low byte register
MPU6050_GYRO_XOUT_H       = const(0x43)
MPU6050_GYRO_XOUT_L       = const(0x44)
MPU6050_GYRO_YOUT_H       = const(0x45)
MPU6050_GYRO_YOUT_L       = const(0x46)
MPU6050_GYRO_ZOUT_H       = const(0x47)
MPU6050_GYRO_ZOUT_L       = const(0x48)
MPU6050_EXT_SENS_DATA_00  = const(0x49)
MPU6050_EXT_SENS_DATA_01  = const(0x4A)
MPU6050_EXT_SENS_DATA_02  = const(0x4B)
MPU6050_EXT_SENS_DATA_03  = const(0x4C)
MPU6050_EXT_SENS_DATA_04  = const(0x4D)
MPU6050_EXT_SENS_DATA_05  = const(0x4E)

MPU6050_I2C_SLV0_DO       = const(0x63)
MPU6050_I2C_SLV1_DO       = const(0x64)
MPU6050_I2C_SLV2_DO       = const(0x65)
MPU6050_I2C_SLV3_DO       = const(0x66)

MPU6050_I2C_MST_DELAY_CTRL= const(0x67)
MPU6050_SIGNAL_PATH_RESET = const(0x68 )#///< Signal path reset register

MPU6050_USER_CTRL         = const(0x6A )#///< FIFO and I2C Master control register
MPU6050_PWR_MGMT_1        = const(0x6B )#///< Primary power/sleep control register
MPU6050_PWR_MGMT_2        = const(0x6C )#///< Secondary power/sleep control register

MPU6050_FIFO_COUNTH       = const(0x72)
MPU6050_FIFO_COUNTL       = const(0x73)
MPU6050_FIFO_R_W          = const(0x74)

MPU6050_WHO_AM_I          = const(0x75 )#///< Divice ID register



MPU6050_MOT_THR           = const(0x1F )  #///< Motion detection threshold bits [7:0]
MPU6050_MOT_DUR           = const(0x20 )  #///< Duration counter threshold for motion int. 1 kHz rate, LSB = const(1 ms
MPU6050_WHO_AM_I_RESULT   = const(0x68 )
GYRO_DLPF_CFG   		  = const(0 )     #//Default settings LPF 256Hz/8000Hz sample



micropython.alloc_emergency_exception_buf(100)


default_calibration_numsamples = 200
default_calibration_accel_deadzone = 15
default_calibration_gyro_deadzone = 5

accel_range = [2, 4, 8, 16]
gyro_range = [250, 500, 1000, 2000]

# These are what the sensors ought to read at rest
# on a level surface
expected = [0, 0, 16384, None, 0, 0, 0]

class CalibrationFailure(Exception):
    pass


class GY_86(object):

    
    def __init__(self, interface = 0, scl=21, sda=20, frq=400000, mag_noMag = 0):


        #self.intr = intr if intr is not None else default_pin_intr
        #MPU_6050 Variables
        self.buffer = bytearray(16)
        self.bytebuf = memoryview(self.buffer[0:1])
        self.wordbuf = memoryview(self.buffer[0:2])
        self.sensors = bytearray(20)     

        self.calibration = [0] * 7

        #self.filter = cfilter.ComplementaryFilter()
        
        #Magneto meter variables      
        self.abs_magADC = 0
        self.raw_magneto_ADC = [0, 0, 0, 0, 0, 0]
        self.xyz_total = [0, 0, 0]
        self.magGain = [0, 0, 0]

        self.bus = I2C(interface,
                       scl = Pin(scl),
                       sda = Pin(sda),
                       freq = frq)


    def write_byte(self, address, reg, val):
        self.bytebuf[0] = val
        self.bus.writeto_mem(address, reg, self.bytebuf)

    def read_byte(self, address, reg):
        self.bus.readfrom_mem_into(address, reg, self.bytebuf)
        return self.bytebuf[0]

    def set_bitfield(self, reg, pos, length, val):
        old = self.read_byte(reg)
        shift = pos - length + 1
        mask = (2**length - 1) << shift
        new = (old & ~mask) | (val << shift)
        self.write_byte(reg, new)

    def read_word(self, address, reg):
        self.bus.readfrom_mem_into(self.address, reg, self.wordbuf)
        return unpack('>H', self.wordbuf)[0]

    def read_word2(self, address, reg):
        self.bus.readfrom_mem_into(self.address, reg, self.wordbuf)
        return unpack('>h', self.wordbuf)[0]



    def identify_MPU(self):
        print('* identifying i2c device')
        val = self.read_byte(MPU6050_I2C_ADDRES, MPU6050_WHO_AM_I)
        if val != MPU6050_WHO_AM_I_RESULT:
            raise OSError("No mpu6050 at address {}".format(MPU6050_I2C_ADDRES))
    
    def reset(self):
        
        print('* reset')
        self.write_byte(MPU6050_I2C_ADDRES, MPU6050_PWR_MGMT_1, 0x80)
        time.sleep_ms(100)

        self.write_byte(MPU6050_I2C_ADDRES, MPU6050_SIGNAL_PATH_RESET, 0x07 )
        time.sleep_ms(100)



    def get_MAG_data(self):
        sensors_mag_data = bytearray(6)  
        self.bus.readfrom_mem_into(HMC5883L_ADDRESS, MAG_DATA_REGISTER, sensors_mag_data)   
        self.raw_magneto_ADC = unpack('>hhh', sensors_mag_data)   

    def bias_collect(self, bias):
        print('* started Magnetometer bias collector')
        
        self.write_byte(HMC5883L_ADDRESS, HMC58X3_R_CONFA, bias)     #// Reg A DOR=0x010 + MS1,MS0 set to pos or negative bias
        
        
        for i in range(10):
            self.write_byte(HMC5883L_ADDRESS, HMC58X3_R_MODE, 1)
            
            time.sleep_ms(100)
            
            #Get magneto data
            #self.bus.readfrom_mem_into(HMC5883L_ADDRESS, MAG_DATA_REGISTER, self.raw_magneto_ADC)                                   #// Get the raw values in case the scales have already been changed.
            self.get_MAG_data()
                
            for axis in range(3):
                self.abs_magADC =  abs(self.raw_magneto_ADC[axis])
                self.xyz_total[axis] = self.xyz_total[axis] + self.abs_magADC                         #// Since the measurements are noisy, they should be averaged rather than taking the max.
                if ((1<<12) < self.abs_magADC):
                    return False                                     #// Detect saturation.   if false Breaks out of the for loop.  No sense in continuing if we saturated.
        return True
    

    def MAG_HMC5883L_Init(self):
        print('* Initializing Magnetometer HMC5883L')
        
        bret = True;                #// Error indicator

        #// Note that the  very first measurement after a gain change maintains the same gain as the previous setting.
        #// The new gain setting is effective from the second measurement and on.

        self.write_byte(HMC5883L_ADDRESS, HMC58X3_R_CONFB, 2 << 5);  #//Set the Gain
        self.write_byte(HMC5883L_ADDRESS, HMC58X3_R_MODE, 1);

        time.sleep_ms(100)
        
        self.get_MAG_data();  #//Get one sample, and discard it

        if not self.bias_collect(0x010 + HMC_POS_BIAS) :
            bret = False
        if not self.bias_collect(0x010 + HMC_NEG_BIAS) :
            bret = False

        if (bret): #// only if no saturation detected, compute the gain. otherwise, the default 1.0 is used 
            for axis in range(3):
                self.magGain[axis]=820.0*HMC58X3_X_SELF_TEST_GAUSS*2.0*10.0/self.xyz_total[axis];  #// note: xyz_total[axis] is always positive

        self.write_byte(HMC5883L_ADDRESS, HMC5883L_REG_CONFIG_A, 0x18);   #// sample rate = 75Hz
        self.write_byte(HMC5883L_ADDRESS, HMC5883L_REG_CONFIG_B, 0x60);   #// full scale = +/- 2.5 Gauss
        self.write_byte(HMC5883L_ADDRESS, HMC5883L_REG_MODE, 0x00);   	  #// continuous measurement mode

        time.sleep_ms(100)



    def GY_ACC_MAG_Init(self):
        
        print('* initializing mpu')
        
        try:
            # Reset the device and setup
            self.reset()

            self.write_byte(MPU6050_I2C_ADDRES, MPU6050_PWR_MGMT_1, 0x00) 			 #// exit sleep mode and set null
            self.write_byte(MPU6050_I2C_ADDRES, MPU6050_PWR_MGMT_2, 0x00)            #// set null


            # Check if the sensor is available else return
            self.identify_MPU()
            

            self.write_byte(MPU6050_I2C_ADDRES, MPU6050_PWR_MGMT_1, 0x03)              #// PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
            self.write_byte(MPU6050_I2C_ADDRES, MPU6050_SMPLRT_DIV, 0x07)              #// Semplarate  8kHz / (1+15) = 40Hz sampla rate  1k == 0x07   500== 0x0F   80== 0x50,  40==  0x28
            self.write_byte(MPU6050_I2C_ADDRES, MPU6050_CONFIG, GYRO_DLPF_CFG)         #//CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)

            # Sensors configuration
            self.write_byte(MPU6050_I2C_ADDRES, MPU6050_GYRO_CONFIG, 0x18)             #//GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
            self.write_byte(MPU6050_I2C_ADDRES, MPU6050_ACCEL_CONFIG, 0x10)            #//ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
            #// interrupt setup
            self.write_byte(MPU6050_I2C_ADDRES, MPU6050_INT_ENABLE, 0b00000001)        #// Interrupt enable


            # *** Configure magnetometer ***

            # Bypass to the magnetometer
            self.write_byte(MPU6050_I2C_ADDRES, MPU6050_USER_CTRL, 0b00000000)
            self.write_byte(MPU6050_I2C_ADDRES, MPU6050_INT_PIN_CONFIG, 0x02)          #// disable i2c master mode

            # Configure Magnetometer

            self.MAG_HMC5883L_Init()

            # Return control to MPU6050
            self.write_byte(MPU6050_I2C_ADDRES, MPU6050_INT_PIN_CONFIG, 0x00)          #// disable i2c master bypass mode
            self.write_byte(MPU6050_I2C_ADDRES, MPU6050_USER_CTRL, 0b00100000)         #// enable i2c master mode


            # Set the MAGNETO data bypass to the MPU6050
            self.write_byte(MPU6050_I2C_ADDRES, MPU6050_I2C_MST_CTRL, 0x0D)                     #//I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
            self.write_byte(MPU6050_I2C_ADDRES, MPU6050_I2C_SLV0_ADDR, 0x80|HMC5883L_ADDRESS)   #//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=HMC5883L_ADDRESS
            self.write_byte(MPU6050_I2C_ADDRES, MPU6050_I2C_SLV0_REG, MAG_DATA_REGISTER)        #//I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
            self.write_byte(MPU6050_I2C_ADDRES, MPU6050_I2C_SLV0_CTRL, 0x86)                    #//I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)
            # Delay the data interrupt by
            # i2c_writeReg(MPU6050_ADDRESS, MPU6050_I2C_MST_DELAY_CTRL, 0x86);


            #   configure the MS5611 (barometer)
            # i2c_write_register(I2C1, MS5611_ADDRESS,   0x1E, 0x00);                    // reset
            # i2c_write_register(I2C1, MS5611_ADDRESS,   0x48, 0x00);                    // start conversion of the pressure sensor

            return 1
        except Exception as e:
            return 0

    def set_gyro_range(self, fsr):
        self.gyro_range = gyro_range[fsr]
        self.set_bitfield(MPU6050_RA_GYRO_CONFIG,
                          MPU6050_GCONFIG_FS_SEL_BIT,
                          MPU6050_GCONFIG_FS_SEL_LENGTH,
                          fsr)

    def set_accel_range(self, fsr):
        self.accel_range = accel_range[fsr]
        self.set_bitfield(MPU6050_RA_ACCEL_CONFIG,
                          MPU6050_ACONFIG_AFS_SEL_BIT,
                          MPU6050_ACONFIG_AFS_SEL_LENGTH,
                          fsr)

    def read_sensors(self):
        self.bus.readfrom_mem_into(MPU6050_I2C_ADDRES,
                                   MPU6050_ACCEL_XOUT_H,
                                   self.sensors)

        data = unpack('>hhhhhhhhhh', self.sensors)

        return data


    def read_sensors_scaled(self):
        data = self.read_sensors()
        data[0:3] = [x/(65536//self.accel_range//2) for x in data[0:3]]
        data[4:7] = [x/(65536//self.gyro_range//2) for x in data[4:7]]
        return data

    def read_position(self):
        self.filter.input(self.read_sensors_scaled())
        return [
            self.filter.filter_pos,
            self.filter.accel_pos,
            self.filter.gyro_pos,
        ]

    def set_dhpf_mode(self, bandwidth):
        self.set_bitfield(MPU6050_RA_ACCEL_CONFIG,
                          MPU6050_ACONFIG_ACCEL_HPF_BIT,
                          MPU6050_ACONFIG_ACCEL_HPF_LENGTH,
                          bandwidth)


    def get_sensor_avg(self, samples, softstart=100):
        '''Return the average readings from the sensors over the
        given number of samples.  Discard the first softstart
        samples to give things time to settle.'''
        sample = self.read_sensors()
        counters = [0] * 7

        for i in range(samples + softstart):
            # the sleep here is to ensure we read a new sample
            # each time
            time.sleep_ms(2)

            sample = self.read_sensors()
            if i < softstart:
                continue

            for j, val in enumerate(sample):
                counters[j] += val

        return [x//samples for x in counters]

    stable_reading_timeout = 10
    max_gyro_variance = 5

    def wait_for_stable(self, numsamples=10):
        print('* waiting for gyros to stabilize')

        gc.collect()
        time_start = time.time()
        samples = []

        while True:
            now = time.time()
            if now - time_start > self.stable_reading_timeout:
                raise CalibrationFailure()

            # the sleep here is to ensure we read a new sample
            # each time
            time.sleep_ms(2)

            sample = self.read_sensors()
            samples.append(sample[4:7])
            if len(samples) < numsamples:
                continue

            samples = samples[-numsamples:]

            totals = [0] * 3
            for cola, colb in zip(samples, samples[1:]):
                deltas = [abs(a-b) for a,b in zip(cola, colb)]
                totals = [a+b for a,b in zip(deltas, totals)]

            avg = [a/numsamples for a in totals]

            if all(x < self.max_gyro_variance for x in avg):
                break

        now = time.time()
        print('* gyros stable after {:0.2f} seconds'.format(now-time_start))

    def calibrate(self,
                  numsamples=None,
                  accel_deadzone=None,
                  gyro_deadzone=None):

        old_calibration = self.calibration
        self.calibration = [0] * 7

        numsamples = (numsamples if numsamples is not None
                   else default_calibration_numsamples)
        accel_deadzone = (accel_deadzone if accel_deadzone is not None
                          else default_calibration_accel_deadzone)
        gyro_deadzone = (gyro_deadzone if gyro_deadzone is not None
                         else default_calibration_gyro_deadzone)

        print('* start calibration')
        
        self.set_state_calibrating()

        try:
            self.wait_for_stable()
            gc.collect()

            # calculate offsets between the expected values and
            # the average value for each sensor reading
            avg = self.get_sensor_avg(numsamples)
            off = [0 if expected[i] is None else expected[i] - avg[i]
                   for i in range(7)]

            accel_ready = False
            gyro_read = False
            for passno in range(20):
                self.calibration = off
                avg = self.get_sensor_avg(numsamples)

                check = [0 if expected[i] is None else expected[i] - avg[i]
                       for i in range(7)]
                print('- pass {}: {}'.format(passno, check))

                # check if current values are within acceptable offsets
                # from the expected values
                accel_ready = all(abs(x) < accel_deadzone
                                  for x in check[0:3])
                gyro_ready = all(abs(x) < gyro_deadzone
                                 for x in check[4:7])

                if accel_ready and gyro_ready:
                    break

                if not accel_ready:
                    off[0:3] = [off[i] + check[i]//accel_deadzone
                                for i in range(3)]

                if not gyro_ready:
                    off[4:7] = [off[i] + check[i]//gyro_deadzone
                                for i in range(4, 7)]
            else:
                raise CalibrationFailure()
        except CalibrationFailure:
            self.calibration = old_calibration
            print('! calibration failed')
            self.set_state_uncalibrated()
            return

        print('* calibrated!')
        self.set_state_calibrated()
        


'''
print('Start IMU configuration')

gy = GY_86(interface = 0, scl=21, sda=20, frq=400000)

gy.GY_ACC_MAG_Init()

interrupt_ocured = 0
accX, accY, accZ, Temp, gyX, gyY, gyZ, magX, magY, magZ = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

sens_data = []

def gy_86_INT(p):
    #global accX, accY, accZ, Temp, gyX, gyY, gyZ, magX, magY, magZ
    global sens_data
    
    interrupt_ocured = 1
    sens_data = gy.read_sensors()
    
    
    #accX, accY, accZ, Temp, gyX, gyY, gyZ, magX, magY, magZ = gy.read_sensors()
    #print('x:'+ str(x) + '  y: '+ str(y) + '  z: ' + str(z) )
    
    return


IMU_INT_PIN = 22
mag_int = Pin(IMU_INT_PIN, Pin.IN)
mag_int.irq(trigger = Pin.IRQ_RISING, handler = gy_86_INT)


while True:


    try:
        temperature = (sens_data[3] / 340.0) + 36.53
        print(sens_data )
        print('Temperature:  ' + str(temperature))
    except Exception as e:
        print(e)
    
    time.sleep_ms(1000)

'''



