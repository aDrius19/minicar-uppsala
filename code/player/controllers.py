"""class Autonomous(object):
This file provides the keyboard control for the player.
"""

import time
import keyboard
import pygame
import sys
import cv2
import cv2.aruco as aruco
import numpy as np

from player import binds

def safe_write(file, msg):
    """Displays on console and writes to a .txt file safely"""
    if file and not file.closed:
        print(msg)
        file.write(msg)
        file.flush()

#TODO adapt the class for the camera to detect the full and/or half lines in which the car needs to function 
# autonomous meaning to steer right/left itself when detecting a turn and decrease/increase the motor's speed 
# when approaching/distancing from a turn - extending to stop the car if it detects an object with a marker 
# or not within certain distance and cannot avoid it
class Autonomous(object):
    
    """ 
        Defines the autonomous functionality of the steering for the servo motor and
        the gear motor itself
    """
    # ==== Changeable Parameters ====
    # Minimum angle change required to send a new command
    ANGLE_THRESHOLD = 1
    # Distance to consider a point "to close"
    LOW_THRESHOLD = 40
    HIGH_THRESHOLD = 80
    # Minimum time between messages to a vehicle (seconds)
    SEND_INTERVAL = 0.05
    # Scale for turn intensity
    SCALE = 0.2
    WEIGHT = 0.5
    ANGLE_FAVOR = 0.7
    

    def __init__(self, include_motor):
        self.include_motor = include_motor
        self.last_sent_angles = {}
        self.last_sent_times = {}
        self.user_sockets = {}
        

    # ==== Angle and Steering ====
    def estimate_heading(corners):
        """
        Estimates the heading angle of an ArUco marker. Computes 
        the midpoint of the front (top) and back (bottom) edges
        of the marker, then calculates a vector between them. The
        angle is derived from this vector using arctangent, with
        correction for image coordinate direction.

        Parameters:
            corners (np.ndarray): Array of 4 marker corners from the
                detector, ordered as [top-left, top-right, bottom-right,
                bottom-left].

        Returns:
            float: The marker's heading angle in degrees. The result is
                in the range [0, 360), where:
                - 0° points right,
                - 90° points up,
                - 180° points left,
                - 270° points down (in image space).

        Description:
            Computes the midpoint of the front (top) and back (bottom)
            edges of the marker, then calculates a vector between them.
            The angle is derived from this vector using arctangent, with
            correction for image coordinate direction.
        """
        top_mid = (corners[0] + corners[1]) / 2
        bottom_mid = (corners[2] + corners[3]) / 2
        heading_vector = top_mid - bottom_mid
        # Calculates the Euclidean distance
        angle = np.degrees(np.arctan2(-heading_vector[1],
                    heading_vector[0])) % 360
        return angle
    
    def map_angle_to_servo(self, relative_angle, dist):
        """
        Maps a relative angle and distance to a servo angle for steering control.
        This function computes a servo command (in degrees) based on the vehicle's 
        relative angle to an lane boundary and its distance to it.
        The closer and sharper the turn, the stronger the steering correction.
        The result is clamped between 48 and 132 degrees to protect the hardware.

        Parameters:
            relative_angle (float): The angle (in degrees) between the car's heading 
                and the detected boundary. Range expected: -90 to 90.
            dist (float): The distance to the obstacle or boundary.

        Returns:
            int or None: The computed servo angle (int between 48 and 132), or 
            None if the relative angle is beyond ±90 degrees and considered invalid.
        """
        if abs(relative_angle) > 90:
            return None
        # Make sure the distance is within range
        dist = max(self.LOW_THRESHOLD, min(self.HIGH_THRESHOLD, dist))
        # Normalize the distance
        normalized_dist = (self.HIGH_THRESHOLD - dist) / (self.HIGH_THRESHOLD - self.LOW_THRESHOLD)
        # Normalize the angle
        normalized_angle = (90 - abs(relative_angle)) / 90
        weight = (normalized_angle * normalized_dist) ** self.WEIGHT
        # Calculate the final servo angle
        servo_angle = 90 - weight * 42 if relative_angle > 0 else 90 + weight * 42
        # Make sure the angle is within servo range
        return int(max(48, min(132, servo_angle)))
    
    def send_if_allowed(self, angle):
        """
        Sends a servo angle to a vehicle if enough time has passed. Sends a
        TCP message with the servo angle to the RC vehicle identified by the
        global variable marker_id. The function enforces a minimum time
        between messages (SEND_INTERVAL). If the vehicle has not received
        a command recently, the angle is sent, and the timestamp and last 
        angle are updated. If sending fails (e.g., disconnected socket), 
        the vehicle is removed from all tracking dictionaries.

        Parameters:
            angle (int): The servo angle to send. Expected range is
                        between 48 (left) and 132 (right), with 90
                        meaning straight.

        Returns:
            None
        """
        current_time = time.time()
        last_time = self.last_send_times.get(self.self.marker_id, 0)

        if current_time - last_time >= self.SEND_INTERVAL:
            try:
                #TODO have the same angle send as for the controller (keyboard/joystick)
                self.angle = angle
                print(f"[User {self.marker_id}] Sent angle: {angle}")
                self.last_sent_angles[self.marker_id] = angle
                self.last_send_times[self.marker_id] = current_time

                return self.angle, self.angle
            except Exception as e:
                print(f"Send error to user {self.marker_id}: {e}")
                self.last_sent_angles.pop(self.marker_id, None)
            self.last_send_times.pop(self.marker_id, None)
    
    def compute_point_score(self, relative_angle, dist):
        """
        Computes a weighted score for a point based on its relative angle
        and distance. This function is typically used to evaluate obstacle
        points. A lower score indicates a more desirable path (closer to 
        straight ahead and farther away). It penalizes sharp angles and 
        close distances using a weighted, non-linear scoring model.

        Parameters:
            relative_angle (float): The angle between the vehicle's heading
                and the obstacle point. Should be between -90 and 90 degrees.
            dist (float): The distance to the point being scored.

        Returns:
            float: A score value where lower is better. Returns infinity if
            the angle is outside the allowed field of view (beyond ±90°).
        """
        if abs(relative_angle) > 90:
            return float('inf')

        # Make sure the distance is within range
        dist = max(self.LOW_THRESHOLD, min(self.HIGH_THRESHOLD, dist))

        # Normalize the distance 
        normalized_dist = (dist - self.LOW_THRESHOLD) / (self.HIGH_THRESHOLD - self.LOW_THRESHOLD)
        # Normalize angle
        normalized_angle = (abs(relative_angle) / 90) ** 2.5    

        # Compute weighted score: prioritize direction over distance
        return self.ANGLE_FAVOR * normalized_dist + (1 - self.ANGLE_FAVOR) * normalized_angle
    
    def dynamic_threshold(self, relative_angle):
        """
        Computes a dynamic distance threshold based on the vehicle's steering angle.

        This function adjusts how far the system should "look ahead" depending on the 
        angle between the car's heading and a target or obstacle. Straighter paths 
        allow for farther lookahead, while sharper turns limit the useful distance.

        Parameters:
            relative_angle (float): The relative heading angle in degrees 
                                    (typically between -90 and 90).

        Returns:
            float: A distance threshold that defines how far to consider points 
                relevant for steering or scoring logic.
        """
        angle = abs(relative_angle)

        if angle < 15:
            return self.HIGH_THRESHOLD  # Straight ahead — look far
        elif angle < 30:
            return self.LOW_THRESHOLD + (self.HIGH_THRESHOLD - self.LOW_THRESHOLD) * 0.5  # Slightly reduced lookahead
        elif angle < 60:
            return self.LOW_THRESHOLD + (self.HIGH_THRESHOLD - self.LOW_THRESHOLD) * 0.25  # Conservative range
        else:
            return self.LOW_THRESHOLD  # Sharp turn — look close for quicker reaction
        
    def run(self):
        # Camera and ArUco setup
        cap = cv2.VideoCapture(0)
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()

        # HSV range for detecting green line for the circuit
        lower_green = np.array([50, 100, 100])
        upper_green = np.array([70, 255, 255]) 

        # ==== Main loop ====
        while True:
            # Read a frame from the camera
            ret, frame = cap.read()
            if not ret:
                break  # Exit if camera frame is not captured

            # Convert to grayscale for ArUco detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detector = aruco.ArucoDetector(aruco_dict, parameters)
            corners, ids, _ = detector.detectMarkers(gray)

            # Detect green lines using HSV color threshold
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            green_mask = cv2.inRange(hsv, lower_green, upper_green)
            contours, _ = cv2.findContours(
                green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(frame, contours, -1, (0, 0, 0), 2)

            # Process each detected ArUco marker
            if ids is not None:
                aruco.drawDetectedMarkers(frame, corners, ids)
                for i, marker_id in enumerate(ids.flatten()):
                    c = corners[i][0]  # Extract the 4 corners on the ArUco
                    front_point = (c[0] + c[1]) / 2  # Front edge midpoint
                    cx, cy = front_point.astype(int)
                    center = front_point
                    car_heading = self.estimate_heading(c)

                    best_point = None
                    best_angle = None
                    best_score = float('inf')
                    best_dist = None

                    # Find closest object point in front of the marker
                    for contour in contours:
                        for point in contour:
                            direction_vector = point[0] - center
                            dist = np.linalg.norm(direction_vector)
                            angle = np.degrees(np.arctan2(-direction_vector[1],
                                        direction_vector[0]))
                            relative_angle = (car_heading - angle + 360) % 360
                            if relative_angle > 180:
                                relative_angle -= 360  # Convert to [-180, 180]
                            
                            if dist < self.dynamic_threshold(relative_angle):
                                score = self.compute_point_score(relative_angle, dist)
                                if score < best_score:
                                    best_point = point[0]
                                    best_angle = relative_angle
                                    best_dist = dist
                                    best_score = score

                    # If a valid point is found, draw it and compute turn
                    if best_point is not None:
                        # Draw the best point on the shown frame
                        cv2.circle(frame, tuple(best_point), 5, (0, 0, 255), -1)
                        cv2.line(frame, (cx, cy), tuple(best_point), (0, 0, 255), 2)

                        if marker_id in self.user_sockets:
                            # Convert angle-to-point to a servo angle
                            servo_angle = self.map_angle_to_servo(best_angle, best_dist)
                            # Angle is None if closest is behind the marker 
                            if servo_angle is not None:
                                last_angle = self.last_sent_angles.get(marker_id, None)
                                if last_angle is None or (abs(servo_angle - last_angle
                                    ) >= self.ANGLE_THRESHOLD and abs(servo_angle - last_angle) < 70):
                                    # if abs(servo_angle - last_angle) < 70:
                                    #maybe it is going to be used - keeping here commented until final test
                                        return self.send_if_allowed(servo_angle)
                    # If no object found, command vehicle to go straight
                    elif self.last_sent_angles.get(marker_id, None) != 90:
                        return self.send_if_allowed(90)

            # Display the processed video frame
            cv2.imshow("Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break  # Exit on pressing 'q' 

        cap.release()
        cv2.destroyAllWindows()


class Keyboard(object):
    """Defines the Keyboard object to transfer keyboard inputs into player instructions"""

    def __init__(self, mode, max_speed, max_angle):
        # Import variables
        self.max_speed = max_speed
        self.max_angle = max_angle

        # Define variables
        self.speed = 0.
        self.angle = 0.
        self.brightness = 0.
        self.turned_on = False

        self.control_type = mode
        self.clean = False
        self.running = True

        self.binds = binds.KeyboardBinds()
        self.file = open('./player/console_prints/keyboard-console-log.txt', 'w')

        safe_write(self.file, "Turning on the control functionality...\n")

        if self.control_type == 0:
            safe_write(self.file,"Launching in mode {} - Manual Control...\r\n".format(self.control_type))
        elif self.control_type == 1:
            safe_write(self.file,"Launching in mode {} - Semi-Autonomous Control...\r\n".format(self.control_type))
        elif self.control_type == 2:
            safe_write(self.file,"Launching in mode {} - Autonomous Control...\r\n".format(self.control_type))

    def break_until_stop(self):
        """Gradually brakes until the car's motor stops"""
        brake_force = 0.1  # units to gradually break

        while abs(self.speed) > 0.:
            if self.speed > 0.:
                self.speed = max(0., self.speed - brake_force)
            else:
                self.speed = min(0., self.speed + brake_force)
            time.sleep(0.05)

    def motor_stop(self):
        """Full brakes and stops the car's motor instantly"""
        self.speed = 0.

    def neutral_steering(self):
        """Straight steering position of the car's servo"""
        self.angle = 0.

    def stop(self):
        """Shutdowns the motor, resets the servo's angle to stop the car"""
        if keyboard.is_pressed(self.binds.stop):
            self.motor_stop()
            self.neutral_steering()
            
            safe_write(self.file,"Car is stopping...\nNeutral steering position...\n")

    def accelerate(self, accel):
        """Increases/decreases the speed up to the max speed"""
        self.speed = min(self.max_speed, max(self.speed + accel, -self.max_speed))

    def turn(self, turn):
        """Increases/decreases the turning angle up to the max angle"""
        self.angle = min(self.max_angle, max(self.angle + turn, -self.max_angle))

    def listen(self):
        """Interprets bound inputs"""

        forward_pressed = keyboard.is_pressed(self.binds.forwards)
        backward_pressed = keyboard.is_pressed(self.binds.backwards)

        # Quit
        if keyboard.is_pressed(self.binds.escape):
            safe_write(self.file,"\nTurning off the control functionality...")
            self.file.close()
            
            self.running = False
            self.clean = True

            return

        # Set control type
        if keyboard.is_pressed(self.binds.manual):
            self.control_type = 0
            time.sleep(0.1)

            safe_write(self.file,'Switching to Manual Control...\n')
        elif keyboard.is_pressed(self.binds.semiautonomous):
            self.control_type = 1
            time.sleep(0.1)

            safe_write(self.file,'Switching to Semi-Autonomous Control...\n')
        elif keyboard.is_pressed(self.binds.autonomous):
            self.control_type = 2
            time.sleep(0.1)
            
            safe_write(self.file,'Switching to Autonomous Control...\n')

        # Manual control
        if self.control_type == 0:
            moved = False
            turned = False

            if forward_pressed and backward_pressed:
                self.motor_stop()
                safe_write(self.file,"W and S - breaking with {}!\n".format(self.speed))

                moved = True
            elif forward_pressed:
                self.accelerate(self.max_speed)
                safe_write(self.file,"W - acceleration with {}!\n".format(self.speed))

                moved = True
            elif backward_pressed:
                self.accelerate(-self.max_speed)
                safe_write(self.file,"S - deceleration with {}!\n".format(self.speed))
                
                moved = True

            if keyboard.is_pressed(self.binds.turn_left):
                self.turn(-self.max_angle)
                safe_write(self.file,"A - turn left with {}!\n".format(self.angle))

                turned = True
            elif keyboard.is_pressed(self.binds.turn_right):
                self.turn(self.max_angle)
                safe_write(self.file,"D - turn right with {}!\n".format(self.angle))

                turned = True
            
            if not moved and not turned:
                self.break_until_stop()
                self.neutral_steering()
            elif not moved and turned:
                self.break_until_stop()
        # Semi-autonomous control
        # TODO logic should be similar to this one - coming from the Autonomous class object
        # maybe some adjustment after the working implementation is done
        elif self.control_type == 1:
            moved = False

            if forward_pressed and backward_pressed:
                self.motor_stop()
                safe_write(self.file,"W and S - breaking with {}!\n".format(self.speed))

                moved = True
            elif forward_pressed:
                self.accelerate(self.max_speed)
                safe_write(self.file,"W - acceleration with {}!\n".format(self.speed))

                moved = True
            elif backward_pressed:
                self.accelerate(-self.max_speed)
                safe_write(self.file,"S - deceleration with {}!\n".format(self.speed))
                
                moved = True

            if not moved:
                self.break_until_stop()      

            self.angle, _ = Autonomous(0).run()
            safe_write(self.file, "Autonomous turning with {}!".format(self.angle))

            self.stop()
        # autonomous control
        # TODO logic should be similar to this one - coming from the Autonomous class object
        # maybe some adjustment after the working implementation is done
        elif self.control_type == 2:
            self.angle, self.speed = Autonomous(1).run()
            safe_write(self.file, "Autonomous speed of {} and turning with {}!".format(self.speed, self.angle))

            self.stop()

        # LEDs brightness control
        if keyboard.is_pressed(self.binds.lights_off):
            self.turned_on = False
            self.brightness = 0.
            
            safe_write(self.file,"All lights full off!\n")
        elif keyboard.is_pressed(self.binds.lights_on):
            self.turned_on = True
            self.brightness = 1.
            
            safe_write(self.file,"All lights full on!\n")

        if self.turned_on:
            if keyboard.is_pressed(self.binds.decrease_brightness) and not keyboard.is_pressed(self.binds.lights_on):
                self.brightness = round(max(0., self.brightness - 0.05), 2)
                
                if self.brightness > 0.:
                    safe_write(self.file,"Decrease lights' brightness, lights' value: {}!\n".format(self.brightness))
            elif keyboard.is_pressed(self.binds.increase_brightness) and not keyboard.is_pressed(self.binds.lights_on):
                self.brightness = round(min(1., self.brightness + 0.05), 2)
                
                if self.brightness < 1.:
                    safe_write(self.file,"Increase lights' brightness, lights' value: {}!\n".format(self.brightness))


class Joystick(object):
    """Defines the Joystick object to transfer joystick inputs into player instructions"""

    def __init__(self, max_speed, max_angle):
        # Import variables
        self.max_speed = max_speed
        self.max_angle = max_angle

        # Define variables
        self.speed = 0.
        self.angle = 90.
        self.brightness = 0.

        self.control_type = 0
        self.clean = False
        self.running = True
        self.joystick = None
        
        self.binds = binds.JoystickBinds()
        self.file = open('./player/console_prints/joystick-console-log.txt', 'w')

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
            safe_write(self.file,'Number of joysticks: {}'.format(joystick_count))

            # Print options
            for i in range(joystick_count):
                joystick = pygame.joystick.Joystick(i)
                safe_write(self.file,'ID: {}   |   Name: {}\n'.format(i, joystick.get_name()))

            # Select options
            if joystick_count > 1:
                while self.joystick is None:
                    for i in range(joystick_count):
                        if keyboard.is_pressed(str(i)):
                            self.joystick = pygame.joystick.Joystick(i)

        self.joystick.init()
        if self.joystick.get_init():
            """
            safe_write(self.file,'Name: {}\n'.format(self.joystick.get_name()))
            safe_write(self.file,'Axes: {}\n'.format(self.joystick.get_numaxes()))
            safe_write(self.file,'Trackballs: {}\n'.format(self.joystick.get_numballs()))
            safe_write(self.file,'Buttons: {}\n'.format(self.joystick.get_numbuttons()))
            safe_write(self.file,'Hats: {}\n'.format(self.joystick.get_numhats()))
            """
            pass
        else:
            safe_write(self.file,'Error initialising joystick!')

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

    def turn(self, turn):
        """Increases/decreases the turning angle up to the max angle"""
        self.angle = min(self.max_angle, max(self.angle + turn, -self.max_angle))

    def motor_stop(self):
        """Full brakes and stops the car's motor instantly"""
        self.speed = 0.

    def neutral_steering(self):
        """Straight steering position of the car's servo"""
        self.angle = 0.

    def stop(self):
        """Shutdowns the motor, resets the servo's angle to stop the car"""
        if keyboard.is_pressed(self.binds.stop):
            self.motor_stop()
            self.neutral_steering()
            
            safe_write(self.file,"Car is stopping...\nNeutral steering position...\n")

    def listen(self):
        """Interprets bound inputs"""
        pygame.event.pump()

        # Quit
        if self.check(self.binds.escape) == 1:
            safe_write(self.file,"\nTurning off the control functionality...")
            self.file.close()

            self.running = False
            self.clean = True

        # Set control type
        if keyboard.is_pressed(self.binds.manual):
            self.control_type = 0
            time.sleep(0.1)
            
            safe_write(self.file,'Switching to Manual Control...\n')
        elif keyboard.is_pressed(self.binds.semiautonomous):
            self.control_type = 1
            time.sleep(0.1)
            
            safe_write(self.file,'Switching to Semi-Autonomous Control...\n')
        elif keyboard.is_pressed(self.binds.autonomous):            
            self.control_type = 2
            time.sleep(0.1)
            
            safe_write(self.file,'Switching to Autonomous Control...\n')

        # Manual control
        if self.control_type == 0:
            self.speed = self.max_speed * -self.check(self.binds.speed)
            self.angle = self.max_angle * -self.check(self.binds.angle)
            
            safe_write(self.file,"Manual moving forward with {} and manual turn with {}!\n".format(self.speed, self.angle))

        # Semi-autonomous control
        # TODO logic should be similar to this one - coming from the Autonomous class object
        # maybe some adjustment after the working implementation is done
        elif self.control_type == 1:
            self.accelerate(self.max_speed * self.check(self.binds.accelerate) / 10.)
            safe_write(self.file,"Manual moving forward with {}!\n".format(self.speed))

            self.angle, _ = Autonomous(0).run()
            safe_write(self.file, "Autonomous turning with {}!".format(self.angle))

            self.stop()

        # autonomous control
        # TODO logic should be similar to this one - coming from the Autonomous class object
        # maybe some adjustment after the working implementation is done
        elif self.control_type == 2:
            self.angle, self.speed = Autonomous(1).run()
            safe_write(self.file, "Autonomous speed of {} and turning with {}!".format(self.speed, self.angle))

            self.stop()

        # Brightness control
        if self.check(self.binds.brightness) > 0.5:
            self.brightness = self.check(self.binds.brightness)
            safe_write(self.file,"Adjust lights' brightness, lights' value: {}!\n".format(self.brightness))
        elif self.check(self.binds.lights_on) == 1:
            self.brightness = 1.
            safe_write(self.file,"All lights full on!\n")
        elif self.check(self.binds.lights_off) == 1:
            self.brightness = 0.
            safe_write(self.file,"All lights full off!\n")
