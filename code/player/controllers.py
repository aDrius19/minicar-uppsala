"""
This file provides the keyboard control for the player.
"""

import time
import keyboard
import pygame
import sys

from player import binds


class Keyboard(object):
    """Defines the Keyboard object to transfer keyboard inputs into player instructions"""

    def __init__(self, mode, max_speed, max_angle, max_accel, max_angle_acc):
        # Import variables
        self.max_speed = max_speed
        self.max_angle = max_angle
        self.max_accel = max_accel
        self.max_angle_acc = max_angle_acc

        # Define variables
        self.speed = 0.
        self.angle = 0.
        self.brightness = 0.

        self.control_type = mode
        self.clean = False
        self.running = True

        self.binds = binds.KeyboardBinds()
        self.file = open('./player/console_prints/keyboard-console-log.txt', 'w')

        print("Turning on the control functionality...")
        self.file.write("Turning on the control functionality...\n")

        if self.control_type == 0:
            print("Launching in mode {} - Manual Control...".format(self.control_type))
            self.file.write("Launching in mode {} - Manual Control...\r\n".format(self.control_type))
        elif self.control_type == 1:
            print("Launching in mode {} - Semi-Autonomous Control...".format(self.control_type))
            self.file.write("Launching in mode {} - Semi-Autonomous Control...\r\n".format(self.control_type))
        elif self.control_type == 2:
            print("Launching in mode {} - Autonomous Control".format(self.control_type))
            self.file.write("Launching in mode {} - Autonomous Control...\r\n".format(self.control_type))

    def break_stop(self):
        """Full breaks and stops the car"""
        if not self.file.closed:
            print("Breaks and stops the car!")
            self.file.write("Breaks and stops the car!\n")

        self.speed = -self.speed
        time.sleep(0.25)
        self.speed = 0

    def stop(self):
        """Stops the car"""
        if not self.file.closed:
            print("Stops the car!")
            self.file.write("Stops the car!\n")

        self.speed = 0

    def merge_left(self):
        """Moves the car to the left lane when possible"""
        pass

    def merge_right(self):
        """Moves the car to the right lane when possible"""
        pass

    def accelerate(self, accel):
        """Increases/decreases the speed up to the max speed"""
        self.speed = min(self.max_speed, max(self.speed + accel, -self.max_speed))
        
        print("Speed increase/decrease by {}".format(self.speed))
        self.file.write("Speed increase/decrease by {}\n".format(self.speed))

    def turn(self, turn):
        """Increases/decreases the turning angle up to the max angle"""
        self.angle = min(self.max_angle, max(self.angle + turn, -self.max_angle))
        
        print("Angle increase/decrease by {}".format(self.angle))
        self.file.write("Angle increase/decrease by {}\n".format(self.angle))

    def listen(self):
        """Interprets bound inputs"""

        # Quit
        if keyboard.is_pressed(self.binds.escape):
            print("Turning off the control functionality...")
            self.file.write("\nTurning off the control functionality...")
            self.file.close()
            
            self.running = False
            self.clean = True

        # Stop
        if keyboard.is_pressed(self.binds.stop):
            print("Car stopped...")
            self.file.write("Car stopped...\n")
            
            self.stop()

        # Set control type
        if keyboard.is_pressed(self.binds.manual):
            print('Switching to Manual Control...')
            self.file.write('Switching to Manual Control...\n')
            
            self.control_type = 0
            time.sleep(0.1)
        elif keyboard.is_pressed(self.binds.semiautonomous):
            print('Switching to Semi-Autonomous Control...')
            self.file.write('Switching to Semi-Autonomous Control...\n')
            
            self.control_type = 1
            time.sleep(0.1)
        elif keyboard.is_pressed(self.binds.autonomous):
            print('Switching to Autonomous Control...')
            self.file.write('Switching to Autonomous Control...\n')
            
            self.control_type = 2
            time.sleep(0.1)

        # Manual control
        if self.control_type == 0:
            if keyboard.is_pressed(self.binds.forwards):
                print("W - acceleration!")
                self.file.write("W - acceleration!\n")
                
                self.accelerate(self.max_speed)
            elif keyboard.is_pressed(self.binds.backwards):
                print("S - deceleration!")
                self.file.write("S - deceleration!\n")
                
                self.accelerate(-self.max_speed)
            elif keyboard.is_pressed(self.binds.turn_left):
                print("A - turn left!")
                self.file.write("A - turn left!\n")
                
                self.turn(-self.max_angle)
            elif keyboard.is_pressed(self.binds.turn_right):
                print("D - turn right!")
                self.file.write("D - turn right!\n")

                self.turn(self.max_angle)
            else:
                self.break_stop()
                self.angle = 0.

        # Semi-autonomous control
        #TODO have this made with the makers done for autonomous part
        elif self.control_type == 1:
            if keyboard.is_pressed(self.binds.accelerate):
                print("W - acceleration!")
                self.file.write("W - acceleration!\n")

                self.accelerate(self.max_speed)
            elif keyboard.is_pressed(self.binds.decelerate):
                print("S - deceleration!")
                self.file.write("S - deceleration!\n")
               
                self.accelerate(-self.max_speed)

            if keyboard.is_pressed(self.binds.merge_left):
                print("Autonomous - turn left!")
                self.file.write("Autonomous - turn left!\n")
                
                self.turn(self.max_angle)

            elif keyboard.is_pressed(self.binds.merge_right):
                print("Autonomous - turn right!")
                self.file.write("Autonomous - turn right!\n")
                
                self.turn(-self.max_angle)

            else:
                self.angle = 0.
                self.break_stop()

        # autonomous control
        #TODO integrate the ArUco markers to have the location and to know when to turn and
        # when to lower/increase the motor's speed
        elif self.control_type == 3:
            pass

        # LEDs brightness control
        if keyboard.is_pressed(self.binds.lights_off):
            print("All lights off!")
            self.file.write("All lights off!\n")

            self.brightness = 0.

        elif keyboard.is_pressed(self.binds.lights_on):
            print("All lights on!")
            self.file.write("All lights on!\n")

            self.brightness = 1.

        elif keyboard.is_pressed(self.binds.decrease_brightness):
            print("Decrease lights' brightness!")
            self.file.write("Decrease lights' brightness!\n")

            self.brightness = round(max(0., self.brightness - 0.05), 2)

        elif keyboard.is_pressed(self.binds.increase_brightness):
            print("Increase lights' brightness!")
            self.file.write("Increase lights' brightness!\n")

            self.brightness = round(min(1., self.brightness + 0.05), 2)


class Joystick(object):
    """Defines the Joystick object to transfer joystick inputs into player instructions"""

    def __init__(self, max_speed, max_angle, max_accel, max_angle_acc):
        # Import variables
        self.max_speed = max_speed
        self.max_angle = max_angle
        self.max_accel = max_accel
        self.max_angle_acc = max_angle_acc

        # Define variables
        self.speed = 0.
        self.angle = 90.
        self.brightness = 0.

        self.control_type = 0
        self.clean = False
        self.running = True
        self.joystick = None
        
        self.binds = binds.JoystickBinds()
        self.file = open('./player/console_prints/keyboard-console-log.txt', 'w')

        # Initialise joystick
        pygame.display.init()
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()

        if joystick_count == 0:
            print('No joystick connected!')
            sys.exit()

        elif joystick_count == 1:
            self.joystick = pygame.joystick.Joystick(0)

        elif joystick_count > 1:
            print('Number of joysticks: {}'.format(joystick_count))
            self.file.write('Number of joysticks: {}'.format(joystick_count))

            # Print options
            for i in range(joystick_count):
                joystick = pygame.joystick.Joystick(i)
                print('ID: {}   |   Name: {}'.format(i, joystick.get_name()))
                self.file.write('ID: {}   |   Name: {}\n'.format(i, joystick.get_name()))

            # Select options
            if joystick_count > 1:
                while self.joystick is None:
                    for i in range(joystick_count):
                        if keyboard.is_pressed(str(i)):
                            self.joystick = pygame.joystick.Joystick(i)

        self.joystick.init()
        if self.joystick.get_init():
            """
            print('Name: {}'.format(self.joystick.get_name()))
            self.file.write('Name: {}\n'.format(self.joystick.get_name()))
            print('Axes: {}'.format(self.joystick.get_numaxes()))
            self.file.write('Axes: {}\n'.format(self.joystick.get_numaxes()))
            print('Trackballs: {}'.format(self.joystick.get_numballs()))
            self.file.write('Trackballs: {}\n'.format(self.joystick.get_numballs()))
            print('Buttons: {}'.format(self.joystick.get_numbuttons()))
            self.file.write('Buttons: {}\n'.format(self.joystick.get_numbuttons()))
            print('Hats: {}'.format(self.joystick.get_numhats()))
            self.file.write('Hats: {}\n'.format(self.joystick.get_numhats()))
            """
            pass
        else:
            print('Error initialising joystick!')
            self.file.write('Error initialising joystick!')

    def __del__(self):
        try:
            self.joystick.quit()
        except (AttributeError, pygame.error):
            pass

    def check(self, bind):
        """Read data from bound location"""
        input_type = bind[0]
        input_location = bind[1]

        if input_type == 'axis':
            return round(self.joystick.get_axis(input_location), 2)
        elif input_type == 'button':
            return self.joystick.get_button(input_location)
        elif input_type == 'hat':
            return self.joystick.get_hat(input_location[0])[input_location[1]]

    def accelerate(self, accel):
        """Increases/decreases the speed up to the max speed"""
        self.speed = min(self.max_speed, max(self.speed + accel, -self.max_speed))

        print("Speed increase/decrease by {}".format(self.speed))
        self.file.write("Speed increase/decrease by {}\n".format(self.speed))

    def turn(self, turn):
        """Increases/decreases the turning angle up to the max angle"""
        self.angle = min(self.max_angle, max(self.angle + turn, -self.max_angle))

        print("Angle increase/decrease by {}".format(self.angle))
        self.file.write("Angle increase/decrease by {}\n".format(self.angle))

    def listen(self):
        """Interprets bound inputs"""
        pygame.event.pump()

        # Quit
        if self.check(self.binds.escape) == 1:
            print("Turning off the control functionality...")
            self.file.write("\nTurning off the control functionality...")
            self.file.close()

            self.running = False
            self.clean = True

        # Set control type
        if self.check(self.binds.manual) == 1:
            print('Manual control...')
            self.file.write('Manual control...\n')
            
            self.control_type = 0
            time.sleep(0.1)

        elif self.check(self.binds.semiautonomous) == 1:
            print('Semi-autonomous control...')
            self.file.write('Semi-autonomous control...\n')
            
            self.control_type = 1
            time.sleep(0.1)

        elif self.check(self.binds.autonomous) == 1:
            print('autonomous control...')
            self.file.write('autonomous control...\n')
            
            self.control_type = 2
            time.sleep(0.1)

        # Manual control
        if self.control_type == 0:
            print("Moving forward and turn...")
            self.file.write("Moving forward and turn...\n")

            self.speed = self.max_speed * -self.check(self.binds.speed)
            self.angle = self.max_angle * -self.check(self.binds.angle)

        # Semi-autonomous control
        elif self.control_type == 1:
            print("Moving forward and turn...")
            self.file.write("Moving forward and turn...\n")

            self.accelerate(self.max_speed * self.check(self.binds.accelerate) / 10.)
            self.turn(-self.max_angle * self.check(self.binds.turn) / 10.)

        # autonomous control
        elif self.control_type == 2:
            pass

        # Brightness control
        if self.check(self.binds.brightness) > 0.5:
            print("Adjust lights' brightness!")
            self.file.write("Adjust lights' brightness!\n")

            self.brightness = self.check(self.binds.brightness)

        elif self.check(self.binds.lights_on) == 1:
            print("All lights on!")
            self.file.write("All lights on!\n")

            self.brightness = 1.

        elif self.check(self.binds.lights_off) == 1:
            print("All lights off!")
            self.file.write("All lights off!\n")

            self.brightness = 0.
