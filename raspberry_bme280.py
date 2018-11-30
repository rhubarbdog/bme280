#**
# * Copyright (C) 2016 - 2017 Bosch Sensortec GmbH
# *
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions are met:
# *
# * Redistributions of source code must retain the above copyright
# * notice, this list of conditions and the following disclaimer.
# *
# * Redistributions in binary form must reproduce the above copyright
# * notice, this list of conditions and the following disclaimer in the
# * documentation and/or other materials provided with the distribution.
# *
# * Neither the name of the copyright holder nor the names of the
# * contributors may be used to endorse or promote products derived from
# * this software without specific prior written permission.
# *
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
# * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
# * OR CONTRIBUTORS BE LIABLE FOR ANY
# * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
# * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
# * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# * ANY WAY OUT OF THE USE OF THIS
# * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
# *
# * The information provided is believed to be accurate and reliable.
# * The copyright holder assumes no responsibility
# * for the consequences of use
# * of such information nor for any infringement of patents or
# * other rights of third parties which may result from its use.
# * No license is granted by implication or otherwise under any patent or
# * patent rights of the copyright holder.
# *
# * date	14 Feb 2018
# * version	3.3.4
# *

# bme280.py -
# This is a cut down version of the Bosch bme280 api, it is only intended
# to work with the associated example i.e. in FORCED MODE sampling once

# every 2 seconds
#
# Author Phil Hall - November 2018


import posix
from fcntl import ioctl
I2C_SLAVE = 0x0703 # this should be defined in fnctl
import time

class SensorError(Exception):
    pass

class BME280:

    CHIP_ID_ADDR = b'\xd0'
    RESET_REG = b'\xe0'
    TEMP_PRESS_CALIB_ADDR = b'\x88'
    HUMID_CALIB_ADDR = b'\xe1'
    TEMP_PRESS_CALIB_LENGTH = 26
    HUMID_CALIB_LENGTH = 7
    DATA_ADDR = b'\xf7'
    TPH_LENGTH = 9

    POWER_CTRL_ADDR = b'\xf4'
    HUMID_CTRL_ADDR = b'\xf2'
    MEASURE_CTRL_ADDR = b'\xf4'
    CONFIG_ADDR = b'\xf5'
    
    OVER_SAMPLE = { 0 : 0x00,
                    1 : 0x01,
                    2 : 0x02,
                    4 : 0x03,
                    8 : 0x04,
                    16 : 0x05}

    SELECT_PRESS = 1
    SELECT_TEMP = 1 << 1
    SELECT_HUMID = 1 << 2
    SELECT_FILTER = 1 << 3
    SELECT_STANDBY = 1 << 4
    SELECT_ALL= 0x1f
    SELECT_OVER_SAMPLE=0x07 
    SELECT_OTHER_SETTINGS=0x18

    FILTER_COEFFICIENT = { 0 : 0x00,
                           2 : 0x01,
                           4 : 0x02,
                           8 : 0x03,
                           16 : 0x04 }

    STANDBY_TIME_MS = {1:0x00,
                       62.5:0x01,
                       125:0x02,
                       250:0x03,
                       500:0x04,
                       1000:0x05,
                       10:0x06,
                       20:0x07}
    
    MODE_SLEEP = 0x00
    MODE_FORCED = 0x01
    MODE_NORMAL = 0x03

    MODE_MASK = 0x03
    HUMID_MASK = 0x07
    PRESS_MASK = 0x1c
    TEMP_MASK = 0xe0
    FILTER_MASK = 0x1c
    STANDBY_MASK =0xe0

    PRESS_SHIFT = 0x02
    TEMP_SHIFT = 0x05
    FILTER_SHIFT = 0x02
    STANDBY_SHIFT = 0x05

    READ_PRESS = 1
    READ_TEMP = 1 << 1
    READ_HUMID = 1 << 2
    READ_ALL = 0x07
    
    def __init__(self, i2c = 1, address=0x76):
        self._fd = posix.open("/dev/i2c-%d" % i2c, posix.O_RDWR)
        ioctl(self._fd, I2C_SLAVE, address)

        self.chip_id=None
        self.calibration=None # This is a tuple of 3 tuples
                              # temp,press,humid
        self.calib_tfine=0      # IO required on this calibration
        self.settings=None    # This is a list over sample on t,p,h 
                              # filter standby
                              
        # get the chip id to see if we are present and ready
        for _ in range(5):
            err=False
            try:
                data=self.get_registers(self.CHIP_ID_ADDR,1)
            except OSError:
                err=True
            else:
                self.chip_id=data[0]

            if not err and self.chip_id != 0x60:
                err=True
                self.chip_id=None
            elif not err:
                break

            time.sleep(0.001)

        if err:
            raise SensorError("Sensor not ready.")

        if self.chip_id != 0x60:
            raise SensorError("Invalid chip id.")

        self.reset()
        self.get_calibration_data()

        self.settings = [ self.OVER_SAMPLE[2],
                          self.OVER_SAMPLE[16],
                          self.OVER_SAMPLE[1],
                          self.FILTER_COEFFICIENT[16],
                          self.STANDBY_TIME_MS[10] ]

        select = self.SELECT_PRESS | \
                 self.SELECT_TEMP | \
                 self.SELECT_HUMID | \
                 self.SELECT_FILTER

        self.set_settings(select)
        
    def __del__(self):
        posix.close(self._fd)
        
    @staticmethod
    def unsigned_bytes(msb, lsb):
        return (msb << 8) | lsb

    @staticmethod
    def signed_bytes(msb, lsb):
        value = ((msb &  0x7F) << 8) | lsb
        if msb & 0x80:
            value=value-32768
        return value

    def get_registers(self, address, length):
        posix.write(self._fd,address)
        data=bytearray(posix.read(self._fd,length))
        return data

    def set_registers(self, addresses, data):
        # this is a direct implemetation of the Bosch bme280.c function
        # i colud re write it for sliker memory management
        length=len(data)
        temp=bytearray((2*length)-1)
        temp[0]=data[0]

        if length > 1:
           for i in range(1,length):
               temp[(i*2)-1] = addresses[i]
               temp[(i*2) ] = data[i]

        buf=bytearray(1)
        buf[0]=addresses[0]
        buf+=temp

        posix.write(self._fd, buf)
    
    def set_mode(self, mode):
        if self.get_mode() != self.MODE_SLEEP:
            self.put_to_sleep()

        data=self.get_registers(self.POWER_CTRL_ADDR, 1)
        data[0] = (data[0] & ~self.MODE_MASK) | \
                  (mode & self.MODE_MASK)

        self.set_registers(self.POWER_CTRL_ADDR, data)


    def reset(self):
        self.set_registers(self.RESET_REG, b'\xb6')
        time.sleep(0.002)

    def get_calibration_data(self):
        data=self.get_registers(self.TEMP_PRESS_CALIB_ADDR, \
                                self.TEMP_PRESS_CALIB_LENGTH)
        
        temp=(self.unsigned_bytes(data[1], data[0]),
              self.signed_bytes(data[3], data[2]),
              self.signed_bytes(data[5], data[4]))
        
        press=(self.unsigned_bytes(data[7], data[6]),
               self.signed_bytes(data[9], data[8]),
               self.signed_bytes(data[11], data[10]),
               self.signed_bytes(data[13], data[12]),
               self.signed_bytes(data[15], data[14]),
               self.signed_bytes(data[17], data[16]),
               self.signed_bytes(data[19], data[18]),
               self.signed_bytes(data[21], data[20]),
               self.signed_bytes(data[23], data[22]))
        
        humid = (data[25], ) 

        data=self.get_registers(self.HUMID_CALIB_ADDR, \
                                self.HUMID_CALIB_LENGTH)

        humid += (self.signed_bytes(data[1], data[0]),
                  data[2])

        msb = data[3] * 16
        lsb = data[4] & 0x0f
        hum_4 = msb | lsb
        if hum_4 & 0x8000:
            hum_4=(hum_4 & 0x7f00)-32768
        msb = data[5] * 16
        lsb = data[4] >> 4
        hum_5 = msb | lsb
        if hum_5 & 0x8000:
            hum_5=(hum_5 & 0x7f00)-32768
        hum_6=data[6]
        if hum_6 & 0x80:
            hum_6=(hum_6 * 0x7f)-128

        humid +=(hum_4, hum_5, hum_6)

        self.calibration=(temp, press, humid)
        
    def set_settings(self, selector):
        if self.get_mode() != self.MODE_SLEEP:
            self.put_to_sleep() 

        if self.SELECT_OVER_SAMPLE & selector:
            self.set_over_sample(selector,self.settings)

        if self.SELECT_OTHER_SETTINGS & selector:
            self.set_other_settings(selector,self.settings)    

    def get_mode(self):
        data=self.get_registers(self.POWER_CTRL_ADDR, 1)
        return (data[0] & self.MODE_MASK)

    def put_to_sleep(self):
        data=self.get_registers(self.HUMID_CTRL_ADDR, 4)
        
        humid = data[0] & self.HUMID_MASK
        press = (data[2] & self.PRESS_MASK) >> self.PRESS_SHIFT
        temp = (data[2] & self.TEMP_MASK) >> self.TEMP_SHIFT
        filter_ = (data[3] & self.FILTER_MASK) >> self.FILTER_SHIFT
        standby = (data[3] & self.STANDBY_MASK) >> self.STANDBY_SHIFT

        settings = [ temp, press, humid, filter_, standby ]

        self.reset()
        # reload device settinge
        self.set_over_sample(self.SELECT_ALL,settings)
        self.set_other_settings(self.SELECT_ALL, settings)

    def set_other_settings(self, selector, settings):
        data=self.get_registers(self.CONFIG_ADDR,1)

        if selector & self.SELECT_FILTER:
            _,_,_,filter_,_=settings
            data[0] = (data[0] & ~self.FILTER_MASK) | \
                      ((filter_ << self.FILTER_SHIFT) & \
                       self.FILTER_MASK)

        if selector & self.SELECT_STANDBY:
            _,_,_,_,standby=settings
            data[0] = (data[0] & ~self.STANDBY_MASK) | \
                      ((standby << self.STANDBY_SHIFT) & \
                       self.STANDBY_MASK)

        self.set_registers(self.CONFIG_ADDR,data)
        
    def set_over_sample(self, selector, settings):
        if selector & self.SELECT_HUMID:
            _,_,humid,_,_=settings
            data=bytearray(1)
            data[0] = humid & self.HUMID_MASK
            self.set_registers(self.HUMID_CTRL_ADDR,data)
            data=self.get_registers(self.MEASURE_CTRL_ADDR,1)
            #have to make this write to comit the changes
            self.set_registers(self.MEASURE_CTRL_ADDR,data)
            
        if selector & (self.SELECT_PRESS | self.SELECT_TEMP):
            data=self.get_registers(self.MEASURE_CTRL_ADDR,1)

            if selector & self.SELECT_PRESS:
                _,press,_,_,_=settings
                data[0] = (data[0] & ~self.PRESS_MASK) | \
                          ((press << self.PRESS_SHIFT) & \
                           self.PRESS_MASK)

            if selector & self.SELECT_TEMP:
                temp,_,_,_,_=settings
                data[0] = (data[0] & ~self.TEMP_MASK) | \
                          ((temp << self.TEMP_SHIFT) & \
                           self.TEMP_MASK)

            self.set_registers(self.MEASURE_CTRL_ADDR,data)
        
    def readSensor(self, read):
        data=self.get_registers(self.DATA_ADDR, self.TPH_LENGTH)
        uncompensated=self.parse_data(data)
        temp, press, humid=self.compensate_data(read, uncompensated , \
                                         self.calibration)
        # convert to degrees C, millibar, %
        temp/=100
        press=int(press/10000)
        humid/=1024
        return (temp, press, humid)
    
    def compensate_data(self, read, data, calibrate):
        temp=0
        press=0
        humid=0
        
        if read & (self.READ_TEMP | self.READ_PRESS | self.READ_HUMID):
            temp=self.compensate_temp(data, calibrate)
            
        if read & self.READ_PRESS:
            press=self.compensate_press(data, calibrate)

        if read & self.READ_HUMID:
            humid=self.compensate_humid(data, calibrate)
            
        return (temp, press, humid)


    def compensate_humid(self, data, calibrate):
        h_max=102400

        _,_,uncompensated=data
        _,_,calib=calibrate
        
        var1 = self.calib_tfine - 76800
        var2 = uncompensated * 16384
        var3 = calib[3] * 1048576
        var4 = calib[4] * var1
        var5 = (((var2 - var3) - var4) + 16384) // 32768
        var2 = (var1 * calib[5]) // 1024
        var3 = (var1 * calib[2]) // 2048
        var4 = ((var2 * (var3 + 32768)) // 1024)  + 2097152
        var2 = ((var4 * calib[1]) + 8192) // 16384
        var3 = var5 * var2
        var4 = ((var3 // 32768) * (var3 // 32768)) // 128
        var5 = var3 - ((var4 * calib[0]) // 16)
        if var5 < 0:
            var5=0
        elif var5 > 419430400:
            var5 = 419430400

        humid = (var5 // 4096)

        if humid > h_max:
            humid=h_max

        return humid
    
    def compensate_press(self, data, calibrate):
        p_min = 3000000
        p_max = 11000000

        _,uncompensated,_=data
        _,calib,_=calibrate
        
        var1 = self.calib_tfine - 128000
        var2 = var1 * var1 * calib[5]
        var2 = var2 + (var1 * calib[4] *131072)
        var2 = var2 + (calib[3] * 34359738368)
        var1 = ((var1 * var1 * calib[2]) // 256) + \
               (var1 * calib[1] * 4096)
        var3 = 140737488355328
        var1 = (var3 + var1) * calib[0] // 8589934592

        if var1 != 0:
            var4 = 1048576 - uncompensated
            var4 = (((var4 * 2147483648) - var2) * 3125) // var1
            var1 = (calib[8] * (var4 // 8192) * (var4 // 8192)) // 33554432
            var2 = (calib[7] * var4) // 524288
            var4 = ((var4 + var1 + var2) // 256) + (calib[6] * 16)

            press = ((var4 // 2) * 100) // 128

            if press < p_min:
                press=p_min
            elif press > p_max:
                press=p_max
        else:
            press=p_min

        return press

    def compensate_temp(self, data, calibrate):
        t_min=-4000
        t_max=8500

        uncompensated,_,_=data
        calib,_,_=calibrate
        
        var1=((uncompensated // 8) - (calib[0] * 2))
        var1=(var1 * calib[1]) // 2048

        var2=((uncompensated // 16) - calib[0])
        var2=(((var2 * var2) // 4096) * calib[2]) // 16384

        self.calib_tfine = var1 + var2
        temp = (self.calib_tfine * 5 + 128) // 256

        if temp < t_min:
            temp=t_min
        elif temp > t_max:
            temp=t_max

        return temp

    def parse_data(self, data):
        msb = data[0] << 12
        lsb = data[1] << 4
        xlsb = data[2] >> 4

        press = msb | lsb | xlsb

        msb = data[3] << 12
        lsb = data[4] << 4
        xlsb = data[5] >> 4

        temp = msb | lsb | xlsb

        lsb = data[6] << 8
        msb = data[7]

        humid = msb | lsb

        return (temp, press, humid)
