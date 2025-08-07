"""
This file is the player control.
It enables the player to control one minicar in one of three modes using a controller.

The <username> and <password> on line 44 must be replaced by the username and password of the Raspberry Pi before
running this code.
"""

import argparse
import socket
import struct
import time
import os
import sys
import keyboard
from threading import Thread

from player import controllers


class Player(object):
    """ 
        Defines the player object to be controlled externally by keyboard/controller 
        to run manual, semi-autonomous or autonomous
    """

    def __init__(self, number, controller, mode, max_speed=0.55, max_angle=0.5, max_accel=1., max_angle_acc=0.1):
        # Import variables
        self.car_number = number

        # Calculate IP
        self.ip = '192.168.0.254'
        self.username = 'cpslab1'
        self.password = 'cpslab1'

        # Assign controller
        if controller == 'keyboard':
            print("Keyboard connected!")
            self.controller = controllers.Keyboard(mode, max_speed, max_angle, max_accel, max_angle_acc)
        elif controller == 'joystick':
            print("Joystick connected!")
            self.controller = controllers.Joystick(mode, max_speed, max_angle, max_accel, max_angle_acc)
        else:
            raise ValueError('Invalid controller')

        # Define variables
        self.remote_port = 6789

    def boot(self):
        """Launches car.py in the Pi"""
        os.system('putty -ssh {}@{} -pw {} -m "./player/launch.txt"'.format(self.username, self.ip, self.password))

    def copy_file(self):
        os.system('pscp -pw {} {}@{}:car-{}-log.txt ./firmware/console_prints'.format(self.password, self.username, self.ip, self.car_number))

    def ping(self):
        """Tests for response"""
        return os.system('ping -n 1 -w 200 {} | find "Reply"'.format(self.ip))

    def transfer_data(self):
        """Initiates the Pi and transfers incoming data to the Pi at 100Hz"""
        th = Thread(target=self.boot)
        th.start()
        print('Listening...')

        while self.controller.running:
            self.controller.listen()

            # Send data at 100Hz
            buffer = bytearray(
                struct.pack('fff?', self.controller.speed, self.controller.angle, self.controller.brightness,
                            self.controller.clean))
            s.sendto(buffer, (self.ip, self.remote_port))
            time.sleep(1. / 100.)
        
        time.sleep(0.85)
        print("Copying log file from the Pi board...")
        self.copy_file()


def run(car_list):
    """Starts and runs the listed cars"""

    unresponsive = []
    print('Initializing...')
    for car_number in car_list:
        car_number = int(car_number)
        players[car_number] = Player(car_number, args.controller, args.mode)

        response = players[car_number].ping()
        if response == 0:
            players_thread[car_number] = Thread(target=players[car_number].transfer_data)
            players_thread[car_number].start()
        elif response == 1:
            del players[car_number]
            print('Player {} unresponsive!'.format(car_number))
            unresponsive.append(car_number)
    
    if len(unresponsive) == 0:
        print('Initializing finished!\nSending...')
    else:
        print('Cars {} unresponsive!\nPress R to retry for unresponsive or Q to stop!'.format([*unresponsive]))

        while True:
            if keyboard.is_pressed('q'):
                print("Quitting the control!")
                break
            elif keyboard.is_pressed('r'):
                print("Retrying unresponsive cars...")
                run(unresponsive)
                break

        time.sleep(0.1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Start and manually control a car")
    parser.add_argument('-n', '--cars', nargs='+', type=int, default=None, help='Manual cars: input the ID of each one')
    parser.add_argument('-c', '--controller', type=str, default='keyboard', help='Controller type: keyboard or joystick')
    #TODO change the default to 2 when the car is fully autonomous to operate
    parser.add_argument('-m', '--mode', type=int, default='0', help='Operation mode: 0 - manual, 1 - semi-autonomous, 2 - autonomous')
    args = parser.parse_args()

    if args.cars is None:
        print('No player selected')
        sys.exit(0)

    # Setup socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    players = {}
    players_thread = {}
    run(args.cars)
