"""
This file updates the files within the SD cards.

The <username> and <password> on line 26 must be replaced by the username and password of the Raspberry Pi before
running this code.
"""

import argparse
import os
import keyboard

pi_board = 'Raspberry Pi 5'
parser = argparse.ArgumentParser(description='Updates car.py in the {} board'.format(pi_board))
parser.add_argument('-n', '--cars', nargs='+', default=[0], help='Cars to write to')
args = parser.parse_args()


def run(car_list):
    """Writes for the input list"""
    unresponsive = []
    print('Starting...')
    for car_number in car_list:
        car_number = int(car_number)
        # ip = '192.168.2.{}'.format(200 + car_number)
        ip = '192.168.0.254'
        response = os.system('ping {} -n 1 -w 200 | find "Reply"'.format(ip))
        if response == 0:
            os.system('psftp cpslab1@{} -pw cpslab1 -b ./firmware/update_firmware.txt'.format(ip))
        elif response == 1:
            unresponsive.append(car_number)

    if len(unresponsive) == 0:
        print('Copy complete')
    else:
        print('Cars {} unresponsive\nPress R to retry for unresponsive or ESC to stop'.format([*unresponsive]))
        while not keyboard.is_pressed('esc'):
            if keyboard.is_pressed('r'):
                run(unresponsive)


if __name__ == '__main__':
    run(args.cars)
