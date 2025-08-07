"""
This file updates the files within the SD cards.

The <username> and <password> on line 26 must be replaced by the username and password of the Raspberry Pi before
running this code.
"""

import argparse
import time
import os
import keyboard

ip = '192.168.0.254'
pi_board = 'Raspberry Pi 5'
parser = argparse.ArgumentParser(description='Updates car.py of the {} board'.format(pi_board))
parser.add_argument('-n', '--cars', nargs='+', default=[0], help='Cars to update')
args = parser.parse_args()

def ping():
        """Tests for response"""
        return os.system('ping -n 1 -w 200 {} | find "Reply"'.format(ip))

def run(car_list):
    """Writes for the input list"""

    unresponsive = []
    print('Starting...')
    for car_number in car_list:
        car_number = int(car_number)
        
        response = ping()
        if response == 0:
            os.system('psftp cpslab1@{} -pw cpslab1 -b ./firmware/update_firmware.txt'.format(ip))
        elif response == 1:
            print('Player {} unresponsive!'.format(car_number))
            unresponsive.append(car_number)

    if len(unresponsive) == 0:
        print('Copy complete!')
    else:
        print('Cars {} unresponsive!\nPress R to retry for unresponsive or ESC to stop!'.format([*unresponsive]))

        while True:
            if keyboard.is_pressed('esc'):
                print("Stopping the upload!")
                break
            elif keyboard.is_pressed('r'):
                print("Retrying unresponsive cars...")
                run(unresponsive)
                break

        time.sleep(0.1)


if __name__ == '__main__':
    run(args.cars)
