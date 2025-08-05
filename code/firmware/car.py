"""
This file provides low-level control for the minicar.
It is to be uploaded to the SD card on the Raspberry Pi board.

The library RPi.GPIO is part of Raspbian
"""

import RPi.GPIO as io
import select
import signal
import socket
import struct
import sys
import time
import board
import busio
import adafruit_bno055

from rpi_hardware_pwm import HardwarePWM
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from adafruit_bno055 import (
    ACCGYRO_MODE,
)
"""When using the BNO085 sensor instead of the BNO055"""
# from adafruit_bno08x import (
#     BNO_REPORT_ACCELEROMETER,
#     BNO_REPORT_GYROSCOPE,
# )
# from adafruit_bno08x.i2c import BNO08X_I2C


class I2CIOComponents(object):
    """For each I2C I/O component that is connected to the Pi board"""

    def __init__(self, channel):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(self.i2c)
        self.bno = adafruit_bno055.BNO055_I2C(self.i2c)
        """When using the BNO085 sensor instead of the BNO055"""
        # self.bno = BNO08X_I2C(self.i2c)
        self.servo1 = servo.Servo(self.pca.channels[channel], actuation_range=90, min_pulse=1250, max_pulse=1750)

        self.pca.frequency = 100
        self.bno.mode = ACCGYRO_MODE
        """When using the BNO085 sensor instead of the BNO055"""
        # self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        # self.bno.enable_feature(BNO_REPORT_GYROSCOPE)

    def setservomotor(self, desired_value):
        self.servo1.angle = desired_value
    
    def runimusensor(self):
        print("---------------------------------------------------------------------------\r\n")
        time.sleep(0.5)
        print("Acceleration:\r\n")
        accel_x, accel_y, accel_z = self.bno.acceleration
        print("X: %0.2f  Y: %0.2f Z: %0.2f  m/s^2\r\n" % (accel_x, accel_y, accel_z))
        print("\r\n")

        print("Gyro:\r\n")
        gyro_x, gyro_y, gyro_z = self.bno.gyro
        print("X: %0.2f  Y: %0.2f Z: %0.2f rads/s\r\n" % (gyro_x, gyro_y, gyro_z))

    def stop(self):
        self.pca.deinit()
        
    
class Output(object):
    """Defines the Output object to send inputs to the components"""

    def __init__(self, pwm_pin, neutral_duty_cycle=0., duty_cycle_range=(0., 99.), flag='S'):
        # Local variables and starting values
        self.flag = flag
        self.neutral_duty_cycle = neutral_duty_cycle
        self.min_value = duty_cycle_range[0]
        self.max_value = duty_cycle_range[1]

        # Setup PWM pins at 100 Hz
        if (self.flag == "H1"):
            self.pwm_pin = HardwarePWM(2, 100)  
        else:     
            io.setup(pwm_pin, io.OUT)
            self.pwm_pin = io.PWM(pwm_pin, 100)
        
        self.pwm_pin.start(self.neutral_duty_cycle)

    def cleanup(self):
        self.pwm_pin.stop()

    def set(self, desired_cycle):
        """Assigns a duty cycle to the component within range"""

        # Maintain the duty cycle within the maximum range
        desired_cycle = min(self.max_value, max(desired_cycle, self.min_value))

        # Set duty cycle for pin
        if (self.flag == "H1"):
            self.pwm_pin.change_duty_cycle(desired_cycle)
        else:
            self.pwm_pin.ChangeDutyCycle(desired_cycle)


# works with L298N driver motor - not for much time, but better than the other ones
class Motor(object):
    """ 
        Defines Motor object to control Hardware GPIO pins and 
        Output object for inputs on the motor driver
    """

    def __init__(self, pwm_pin, gpio_pins, neutral_duty_cycle=0., duty_cycle_range=(0., 99.)):
        self.ENA = Output(pwm_pin, neutral_duty_cycle, duty_cycle_range, "H1")
        self.gpio_pins = gpio_pins
        io.setup(self.gpio_pins, io.OUT)

    def set(self, desired_cycle):
        """Assign output duty cycle and motor direction"""

        self.ENA.set(abs(desired_cycle))

        if desired_cycle >= 0.:
            io.output(self.gpio_pins, [io.HIGH, io.LOW])
        elif desired_cycle < 0.:
            io.output(self.gpio_pins, [io.LOW, io.HIGH])

    def stop(self):
        io.output(self.gpio_pins, io.LOW)
        self.ENA.cleanup()


def cleanup(signal=None, frame=None):
    """Default cars, clean pins and exit program"""
    print('\r\nCleaning up...\r\n')

    dcmotor.stop()
    i2ciocomps.stop()
    led1.cleanup()
    led2.cleanup()

    time.sleep(1)
    io.cleanup()
    sys.exit(0)


# Find local IP
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(('192.168.0.1', 1))
local_ip = s.getsockname()[0]

if local_ip is None:
    raise ValueError('Local IP invalid')

# Bind to UDP port
print('Binding to UDP port...\r\n')
local_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
local_socket.bind((local_ip, 6789))  # Local port = 6789
local_socket.settimeout(None)
local_socket.setblocking(False)

# Intercept Ctrl+C.
signal.signal(signal.SIGINT, cleanup)

# Initiate GPIO
io.setmode(io.BCM)

# Setup outputs
dcmotor = Motor(18, [14, 15])
i2ciocomps = I2CIOComponents(0)
led1 = Output(17)
led2 = Output(27)

# Define control variables
speed = 0.
angle = 0.
brightness = 0.
done = False

print('Listening...\r\n')

# Main loop
while not done:
    # Purge all messages
    data = None
    while select.select([local_socket], [], [], 0)[0]:
        data, _ = local_socket.recvfrom(3 * 4 + 1)
    if data is None:
        time.sleep(1. / 100.)  # The car is updated at 100Hz
        continue

    # Receive data as 3 floats and 1 boolean
    speed, angle, brightness, done = struct.unpack('fff?', data)

    # Convert desired values to duty cycles
    speed_cycle = float(speed) * 110.
    angle_cycle = float(angle) * 90. + 45.
    brightness_cycle = float(brightness) * 100.

    # Update desired values
    dcmotor.set(speed_cycle)
    led1.set(brightness_cycle)
    led2.set(brightness_cycle)
    i2ciocomps.setservomotor(angle_cycle)
    i2ciocomps.runimusensor() 


s.close()
cleanup()
