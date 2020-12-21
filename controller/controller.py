"""Hexapod motion controller implementation."""

import sys
import time
import threading as thread
from typing import Optional

import numpy as np

from .simulator import HexapodHAL
from .cpg import OscillatorNetwork, TRIPOD_GAIT_WEIGHTS
from .const import *

from messages import Twist, Odometry, LaserScan, Pose


class HexapodController:
    """Hexapod motion controller."""

    collision: Optional[bool] = None
    odometry: Optional[Odometry] = None
    laser_scan: Optional[LaserScan] = None
    navigation_goal: Optional[Pose] = None

    def __init__(self, robot_id: int):
        self.robot = HexapodHAL(robot_id, TIME_FRAME)

        # gait parametrization
        self.v_left = 0
        self.v_right = 0
        self.stride_length = 1
        self.stride_height = 1

        # locomotion and navigation variables
        self.locomotion_stop = False
        self.locomotion_lock = thread.Lock()  # mutex for access to turn commands
        self.locomotion_status = False

        self.navigation_stop = False
        self.navigation_lock = thread.Lock()  # mutex for access to navigation coordinates
        self.navigation_status = False

    ########################################################################

    def turn_on(self):
        """Drive the robot into the default position."""
        # read out the current pose of the robot
        configuration = self.robot.get_all_servo_position()

        # interpolate to the default position
        interpolation_time = 3000  # ms
        interpolation_steps = interpolation_time // TIME_FRAME

        speed = np.zeros(18)
        for i in range(18):
            speed[i] = (SERVOS_BASE[i] - configuration[i]) / interpolation_steps

        # execute the motion
        for t in range(interpolation_steps):
            self.robot.set_all_servo_position(configuration + t * speed)

    def turn_off(self):
        """Turn off the robot."""
        self.robot.stop_simulation()

    ########################################################################
    # LOCOMOTION
    ########################################################################

    def start_locomotion(self):
        if not self.locomotion_status:
            print("Starting locomotion thread")
            try:
                self.locomotion_stop = False
                locomotion_thread = thread.Thread(target=self.locomotion)
            except:
                print("Error: unable to start locomotion thread")
                sys.exit(1)

            locomotion_thread.start()
        else:
            print("The locomotion is already running")

    def stop_locomotion(self):
        print("Stopping the locomotion thread")
        self.locomotion_stop = True

    def locomotion(self):
        self.locomotion_status = True

        # cpg network instantiation
        cpg = OscillatorNetwork(6)
        cpg.change_gait(TRIPOD_GAIT_WEIGHTS)

        coxa_angles = [0, 0, 0, 0, 0, 0]
        cycle_length = [0, 0, 0, 0, 0, 0]

        # main locomotion control loop
        while not self.locomotion_stop:
            # steering - acquire left and right steering speeds
            self.locomotion_lock.acquire()
            left = np.clip(self.v_left, -1, 1)
            right = np.clip(self.v_right, -1, 1)
            self.locomotion_lock.release()

            coxa_dir = [left, left, left, right, right, right]  # set directions for individual legs

            # next step of CPG
            cycle = cpg.oscilate_all_CPGs()
            # read the state of the network
            data = cpg.get_last_values()

            # reset coxa angles if new cycle is detected
            for i in range(6):
                cycle_length[i] += 1
                if cycle[i]:
                    coxa_angles[i] = -((cycle_length[i] - 2) / 2) * COXA_MAX
                    cycle_length[i] = 0

            angles = np.zeros(18)
            # calculate individual joint angles for each of six legs
            for i in range(6):
                femur_val = FEMUR_MAX * data[i]  # calculation of femur angle
                if femur_val < 0:
                    coxa_angles[i] -= coxa_dir[i] * COXA_MAX  # calculation of coxa angle -> stride phase
                    femur_val *= 0.025
                else:
                    coxa_angles[i] += coxa_dir[i] * COXA_MAX  # calculation of coxa angle -> stance phase

                coxa_val = coxa_angles[i]
                tibia_val = -TIBIA_MAX * data[i]  # calculation of tibia angle

                # set position of each servo
                angles[COXA_SERVOS[i] - 1] = SIGN_SERVOS[i] * coxa_val * self.stride_length + COXA_OFFSETS[i]
                angles[FEMUR_SERVOS[i] - 1] = SIGN_SERVOS[i] * femur_val * self.stride_height + FEMUR_OFFSETS[i]
                angles[TIBIA_SERVOS[i] - 1] = SIGN_SERVOS[i] * tibia_val * self.stride_height + TIBIA_OFFSETS[i]

            # set all servos simultaneously
            self.robot.set_all_servo_position(angles)

            # get data from simulator
            self.odometry = self.robot.get_robot_odometry()
            self.collision = self.robot.get_robot_collision()
            self.laser_scan = self.robot.get_laser_scan()

            time.sleep(TIME_FRAME / 100.0)

        self.locomotion_status = False

    def move(self, twist: Optional[Twist] = None):
        """Function to set the differential steering command.

        Parameters
        ----------
        twist: Twist
            Twist velocity command.
        """
        if twist is None:
            left = right = 0
            self.navigation_goal = None
        else:
            linear = np.clip(twist.linear.x, -1, 1)
            angular = np.clip(twist.angular.x, -1, 1)
            left, right = (linear - angular) / 2, (linear + angular) / 2
            # # always give a robot the full velocity at least on one side
            # if (greater := max(abs(left), abs(right))) > 0:
            #     left, right = left / greater, right / greater

        self.locomotion_lock.acquire()
        self.v_left = SPEEDUP * left
        self.v_right = SPEEDUP * right
        self.locomotion_lock.release()

    ########################################################################
    # NAVIGATION
    ########################################################################

    def start_navigation(self):
        """Start the navigation thread, that will guide the robot towards the goal set using the goto function."""
        # starting the navigation thread if it is not running
        if not self.navigation_status:
            print("Starting navigation thread")
            try:
                self.navigation_stop = False
                navigation_thread = thread.Thread(target=self.navigation)
            except:
                print("Error: unable to start navigation thread")
                sys.exit(1)

            navigation_thread.start()
        else:
            print("The navigation is already running")

    def stop_navigation(self):
        """Stop the navigation thread."""
        print("Stopping the navigation thread")
        self.navigation_stop = True
        self.locomotion_stop = True

    def goto(self, goal: Pose):
        """Open-loop navigation towards a selected goal.

        Parameters
        ----------
        goal : Position
            Location to navigate to.
        """
        self.navigation_lock.acquire()
        self.navigation_goal = goal
        self.navigation_lock.release()

    def stop(self):
        """Drops the current goal and (almost) stops the movement."""
        self.move(None)

    def navigation(self):
        self.navigation_status = True

        # start the locomotion if it is not running
        if not self.locomotion_status:
            self.start_locomotion()

        while not self.navigation_stop:
            self.make_twist()

        self.navigation_status = False

    def make_twist(self):
        """Move towards the desired goal."""
        if self.collision is None or self.odometry is None or self.navigation_goal is None:
            return

        if self.collision:
            self.stop()
            return

        pose, goal = self.odometry.pose.copy(), self.navigation_goal
        dist = pose.dist(goal)
        if dist <= DELTA_DISTANCE:
            self.stop()
            return

        # linear component
        linear = min(dist / SLOWDOWN_DISTANCE, 1)

        # angular component
        current_direction = pose.orientation.to_Euler()[0]
        desired_direction = np.arctan2(goal.position.y - pose.position.y, goal.position.x - pose.position.x)
        delta_phi = desired_direction - current_direction
        delta_phi = phi if (phi := delta_phi % (2 * np.pi)) < np.pi else phi - 2 * np.pi
        angular = TURNING_SPEED * delta_phi / np.pi

        # slow down on large turns
        linear /= max(abs(angular), 1)

        # make a move and pause for a moment
        twist = Twist()
        twist.linear.x = linear
        twist.angular.x = angular
        self.move(twist)
        time.sleep(0.1)
