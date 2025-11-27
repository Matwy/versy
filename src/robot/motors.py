import busio
from board import SCL, SDA
from adafruit_bus_device import i2c_device

class Motors():

    M_1 = 0
    M_2 = 1
    M_3 = 2
    
    def __init__(self):
        i2c_bus = busio.I2C(SCL, SDA)
        self.arduinoi2c = i2c_device.I2CDevice(i2c_bus, 0x10)
    
    def send_motor_power(self, motor, power):
        if(power > 100):
            power = 100
        if(power < -100):
            power = -100
        
        if (power < 0):
            power = 256 - abs(power)
        
        data = [motor, power]
        try:
            self.arduinoi2c.write(bytes(data))
        except:
            print("i2c nel drifting")
        
    def set_speeds(self, m1_power, m2_power, m3_power):
        self.send_motor_power(self.M_1, m1_power)
        self.send_motor_power(self.M_2, m2_power)
        self.send_motor_power(self.M_3, m3_power)