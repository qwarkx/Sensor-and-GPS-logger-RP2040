
import uctypes as c
'''
timestamp: list[c.uint32_t]
packet_count: list[c.uint32_t]
sample_counter: list[c.uint32_t]

gyro_x: list[c.uint16_t]
gyro_y: list[c.uint16_t]
gyro_z: list[c.uint16_t]

acc_x: list[c.uint16_t]
acc_y: list[c.uint16_t]
acc_z: list[c.uint16_t]

mag_x: list[c.uint16_t]
mag_y: list[c.uint16_t]
mag_z: list[c.uint16_t]
temp_imu: list[c.uint16_t]

presure_calculated: list[float]
temperature: list[float]
calculated_altitude: list[float]

num_satelites: list[c.uint8_t]
fix_type: list[c.uint8_t]

hight_abobe_see_level: list[c.double]
ground_speed: list[c.double]
gps_long: list[c.double]
gps_lat: list[c.double]
'''
class DataPoint:
    timestamp: c.uint32_t
    packet_count: c.uint32_t
    sample_counter: c.uint32_t
    
    gyro_x: c.int16_t
    gyro_y: c.int16_t
    gyro_z: c.int16_t

    acc_x: c.int16_t
    acc_y: c.int16_t
    acc_z: c.int16_t

    mag_x: c.int16_t
    mag_y: c.int16_t
    mag_z: c.int16_t
    temp_imu: c.int16_t
    
    presure_calculated: float
    temperature: float
    calculated_altitude: float

    num_satelites: c.uint8_t
    fix_type: c.uint8_t

    hight_abobe_see_level: c.double
    ground_speed: c.double
    gps_long: c.double
    gps_lat: c.double

    def add_data_experimental(self, timestamp: c.uint32_t, packet_count: c.uint32_t, sample_counter: c.uint32_t,
                              imu_data: list[c.uint16_t],
                              baro_data: list[float],
                              num_satelites: c.int8_t, fix_type: c.int8_t,
                              gps_data: list[c.double]):
        self.timestamp.append(timestamp)
        self.packet_count.append(packet_count)
        self.sample_counter.append(sample_counter)

        self.gyro_x.append(imu_data[0])
        self.gyro_y.append(imu_data[1])
        self.gyro_z.append(imu_data[2])
        self.acc_x.append(imu_data[4])
        self.acc_y.append(imu_data[5])
        self.acc_z.append(imu_data[6])
        self.mag_x.append(imu_data[7])
        self.mag_y.append(imu_data[8])
        self.mag_z.append(imu_data[9])
        self.temp_imu.append(imu_data[3])

        self.presure_calculated.append(baro_data[0])
        self.temperature.append(baro_data[1])
        self.calculated_altitude.append(baro_data[2])

        self.num_satelites.append(num_satelites)
        self.fix_type.append(fix_type)

        self.hight_abobe_see_level.append(gps_data[0])
        self.ground_speed.append(gps_data[1])
        self.gps_long.append(gps_data[2])
        self.gps_lat.append(gps_data[3])
            

    def write_to_sd_card(self):
        print('write to SD card')
        # Write the data points to the SD card
        # ...


class CircularBuffer:
    def __init__(self, buffer_size: int):
        self.buffer_size = buffer_size
        self.current_index = 0
        self.buffer = [DataPoint(timestamp=[],
                                 packet_count=[],
                                 sample_counter=[],
                                 gyro_x=[],
                                 gyro_y=[],
                                 gyro_z=[],
                                 acc_x=[],
                                 acc_y=[],
                                 acc_z=[],
                                 mag_x=[],
                                 mag_y=[],
                                 mag_z=[],
                                 temp_imu=[],
                                 presure_raw=[],
                                 presure_calculated=[],
                                 temperature=[],
                                 calculated_altitude=[],
                                 num_satelites=[],
                                 fix_type=[],
                                 hight_abobe_see_level=[],
                                 ground_speed=[],
                                 gps_long=[],
                                 gps_lat=[]) for _ in range(buffer_size)]

    def add_data(self, timestamp: c.int32_t, packet_count: c.int16_t, sample_counter: c.int32_t, imu_data: list[c.uint16_t], baro_data: list[float], gps_data: []):
        data_point = self.buffer[self.current_index]
        data_point.add_data_experimental(timestamp, packet_count, sample_counter, imu_data, baro_data, gps_data)

        # Check if the buffer is full
        if len(data_point.timestamp) >= self.buffer_size:
            self.write_current_data_to_sd_card()
            self.current_index = (self.current_index + 1) % self.buffer_size

    def write_current_data_to_sd_card(self):
        data_point = self.buffer[self.current_index]
        data_point.write_to_sd_card()