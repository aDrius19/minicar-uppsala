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

from rpi_hardware_pwm import HardwarePWM
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

#TODO modify this class to actually do proper rotations
class I2CIOComponents(object):
    """For each I2C I/O component that is connected to the Pi board"""

    def __init__(self, channel, neutral_duty_cycle=0., duty_cycle_range=(0., 99.)):
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        pca = PCA9685(i2c)
        bno = BNO08X_I2C(i2c)
        pca.frequency = 100
        servo1 = servo.Servo(pca.channels[channel], actuation_range=25)

        bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        bno.enable_feature(BNO_REPORT_GYROSCOPE)
        bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        for i in range(25):
            servo1.angle = i
            time.sleep(0.03)
        for i in range(25):
            servo1.angle = 25 - i
            time.sleep(0.03)

        servo1.angle = 0

        # You can also specify the movement fractionally.
        # fraction = 0.0
        # while fraction < 1.0:
        #     servo1.fraction = fraction
        #     fraction += 0.01
        #     time.sleep(0.03)

        pca.deinit()

        while True:
            time.sleep(0.5)
            print("Acceleration:")
            accel_x, accel_y, accel_z = bno.acceleration
            print("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
            print("")

            print("Gyro:")
            gyro_x, gyro_y, gyro_z = bno.gyro
            print("X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
            print("")

            print("Magnetometer:")
            mag_x, mag_y, mag_z = bno.magnetic
            print("X: %0.6f  Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
            print("")

            print("Rotation Vector Quaternion:")
            quat_i, quat_j, quat_k, quat_real = bno.quaternion
            print("I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real))
            print("")
        
    
class Output(object):
    """Defines the Output object to send inputs to the components"""

    def __init__(self, pwm_pin, neutral_duty_cycle=0., duty_cycle_range=(0., 99.), flag='S'):
        # Localise variables
        self.flag = flag
        self.neutral_duty_cycle = neutral_duty_cycle
        self.min_value = duty_cycle_range[0]
        self.max_value = duty_cycle_range[1]

        # Set starting values
        # self.duty_cycle = 0.
        # self.prev_time = time.time()

        # Setup PWM pins at 100 Hz
        io.setup(pwm_pin, io.OUT)
        if (self.flag == 'S'):
            self.pwm_pin = io.PWM(pwm_pin, 100)
        elif (self.flag == 'H1'):
            self.pwm_pin = HardwarePWM(0, 100)    
            # self.pwm_pin = io.PWM(pwm_pin, 100)        
        elif (self.flag == 'H2'):
            self.pwm_pin = HardwarePWM(1, 100)   
            # self.pwm_pin = io.PWM(pwm_pin, 100)         
        
        self.pwm_pin.start(self.neutral_duty_cycle)

    def __del__(self):
        self.pwm_pin.stop()

    def set(self, desired_cycle):
        """Assigns a duty cycle to the component within range"""
        # Maintain the duty cycle within the maximum range
        desired_cycle = min(self.max_value, max(desired_cycle, self.min_value))

        # Set duty cycle for pin
        if (self.flag == 'S'):
            self.pwm_pin.ChangeDutyCycle(desired_cycle)
        elif (self.flag == 'H1') | (self.flag == 'H2'):
            self.pwm_pin.change_duty_cycle(desired_cycle)
            # self.pwm_pin.ChangeDutyCycle(desired_cycle)

#TODO not working at all, investigate why there is no PWM signal received
class Motor(object):
    """ 
        Defines Motor object to control Hardware GPIO pins and 
        Output object for inputs on the motor driver
    """

    def __init__(self, gpio_pins, neutral_duty_cycle=0., duty_cycle_range=(0., 100)):
        self.gpio_pins = gpio_pins
        self.neutral_duty_cycle = neutral_duty_cycle

        self.IN1 = Output(self.gpio_pins[0], neutral_duty_cycle, duty_cycle_range, 'H1')
        self.IN2 = Output(self.gpio_pins[1], neutral_duty_cycle, duty_cycle_range, 'H2')
        self.SLEEP = self.gpio_pins[2]
        io.setup(self.SLEEP, io.OUT)

    def set(self, desired_cycle):
        """Assign output duty cycle and motor direction"""

        if desired_cycle >= 0.: # moving forward
            io.output(self.SLEEP, io.HIGH)
            self.IN1.set(abs(desired_cycle))
            self.IN2.set(self.neutral_duty_cycle)
        elif desired_cycle < 0.: # moving backwards
            io.output(self.SLEEP, io.HIGH)
            self.IN1.set(self.neutral_duty_cycle)
            self.IN2.set(abs(desired_cycle))

    def sleep(self):
        io.output(self.SLEEP, io.LOW)


def cleanup(signal=None, frame=None):
    """Default cars, clean pins and exit program"""
    print('Cleaning up...')

    del motor.IN1
    del motor.IN2
    # del servo
    del led1
    del led2

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
print('Binding to UDP port...')
local_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
local_socket.bind((local_ip, 6789))  # Local port = 6789
local_socket.settimeout(None)
local_socket.setblocking(False)

# Intercept Ctrl+C.
signal.signal(signal.SIGINT, cleanup)

# Initiate GPIO
io.setmode(io.BCM)

# Setup outputs
motor = Motor([18, 19, 23])
# servo = Output(6, 15., (12.5, 17.5))
led1 = Output(17)
led2 = Output(27)
# I2CIOComponents(0)

# Define control variables
speed = 0.
angle = 0.
brightness = 0.
done = False

print('Listening...')

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
    speed_cycle = float(speed) * 200. / 3.
    angle_cycle = float(angle) * 12 + 15
    brightness_cycle = float(brightness) * 100.

    # Update desired values
    motor.set(speed_cycle)
    # servo.set(angle_cycle)
    led1.set(brightness_cycle)
    led2.set(brightness_cycle)

s.close()
cleanup()
