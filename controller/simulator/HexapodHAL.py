# -*- coding: utf-8 -*-

import sys
from .vrep_api import vrep
import math
import time
import numpy as np

# messages
from messages import *


class HexapodHAL:
    # class for interfacing the hexapod robot in V-REP simulator

    def __init__(self, robot_id, simulation_step=1, sync_sim=False):
        self.simulation_step = simulation_step
        self.sync_sim = sync_sim
        self.sync_steps = 0

        self.robot_id = robot_id

        self.clientID = self.connect_simulator()
        self.hexapod = None
        self.ref_frame = None
        self.hexa_base = None
        self.collision = None
        self.collision_first = []
        self.laser_init = True

        self.servos = []
        self.foot_tips = []

        self.torques_first = np.ones(18, dtype=bool)
        self.joints_first = np.ones(18, dtype=bool)

        self.pos_first = True
        self.orientation_first = True
        self.on_startup()

    def __del__(self):
        self.stop_simulation()
        self.disconnect_simulator()

    #############################################################
    # helper functions for simulator interfacing
    #############################################################

    def connect_simulator(self):
        """Establish connection to the simulator on localhost
        """
        vrep.simxFinish(-1)  # just in case, close all opened connections
        IP_address = "127.0.0.1"
        port = 19997  # port on which runs the continuous remote API
        waitUntilConnected = True
        doNotReconnectOnceDisconnected = True
        timeOutInMs = 5000
        commThreadCycleInMs = 20
        new_clientID = vrep.simxStart(IP_address, port, waitUntilConnected, doNotReconnectOnceDisconnected, timeOutInMs,
                                      commThreadCycleInMs)
        if new_clientID != -1:
            print("Connected to remote API server")
        else:
            print("Connection to remote API server failed")
            sys.exit()
        return new_clientID

    def start_simulation(self):
        """Start the simulation
        """
        errorCode = vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)
        assert errorCode == vrep.simx_return_ok, "Simulation could not be started"
        return

    def stop_simulation(self):
        """Stop the simulation
        """
        errorCode = vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_oneshot)
        assert errorCode == vrep.simx_return_ok or errorCode == vrep.simx_return_novalue_flag, "Simulation could not be stopped"
        return

    def disconnect_simulator(self):
        """Disconnect from the simulator
        """
        vrep.simxFinish(self.clientID)
        return

    def get_object_handle(self, string):
        """Provides object handle for V-REP object
        Args:
            string: name of the object
        Returns:
            handle to the object
        """
        errorCode, handle = vrep.simxGetObjectHandle(self.clientID, string, vrep.simx_opmode_oneshot_wait)
        assert errorCode == vrep.simx_return_ok, "Conection to " + string + "failed"
        return handle

    def get_collision_handle(self, string):
        """Provides handle to the collision object in V-REP
        Args:
            string: name of the collision handle
        Returns:
            handle to the collision object
        """
        errorCode, handle = vrep.simxGetCollisionHandle(self.clientID, string, vrep.simx_opmode_oneshot_wait)
        assert errorCode == vrep.simx_return_ok, "Getting " + string + " handle failed"
        return handle

    def get_collision_state(self, c_handle):
        """Getting collision status of object
        Args:
            c_handle: vrep simulation collision object handle
        Returns:
            bool: True if the object is in collision state, False otherwise
        """
        if not c_handle in self.collision_first:
            errorCode, collisionState = vrep.simxReadCollision(self.clientID, c_handle, vrep.simx_opmode_streaming)
            self.collision_first.append(c_handle)
        else:
            errorCode, collisionState = vrep.simxReadCollision(self.clientID, c_handle, vrep.simx_opmode_buffer)

        if errorCode == vrep.simx_return_novalue_flag:
            collisionState = False
        else:
            assert errorCode == vrep.simx_return_ok, "Cannot read collision"

        return collisionState

    def get_servos_handles(self):
        """Retrieve servo handles
        Returns
            list (int): list of vrep servos object handles         
        """
        servos = []
        for i in range(1, 19):
            servo = self.get_object_handle("hexa_joint" + str(i) + "#" + str(self.robot_id))
            servos.append(servo)
        return servos

    def get_hexa_base_handle(self):
        """Retrieve handle of the robot base
        Returns
            int: hexapod robot base vrep handle
        """
        hexa_base = self.get_object_handle("hexa_base" + "#" + str(self.robot_id))
        return hexa_base

    def get_foot_tips_handles(self):
        """Retrieve handles of the foot tips
        Returns
            list (int): list of vrep hexapod foottip handles
        """
        foot_tips = []
        for i in range(1, 7):
            foot_tip = self.get_object_handle("hexa_footTip" + str(i) + "#" + str(self.robot_id))
            foot_tips.append(foot_tip)
        return foot_tips

    def get_hexapod_handle(self):
        """Retrieve handle for the hexapod object
        Returns
            int: hexapod robot handle
        """
        hexapod = self.get_object_handle("hexapod" + "#" + str(self.robot_id))
        return hexapod

    def on_startup(self):
        """Simulator startup routine
        """
        self.hexapod = self.get_hexapod_handle()
        self.servos = self.get_servos_handles()
        self.hexa_base = self.get_hexa_base_handle()
        self.collision = self.get_collision_handle('hexa_c' + "#" + str(self.robot_id))

        self.foot_tips = self.get_foot_tips_handles()

        # enable synchronous operation of the vrep
        if self.sync_sim:
            self.enable_sync()
            self.set_sim_step(self.simulation_step)

        # start the simulation
        self.start_simulation()
        print("Robot ready")
        return

    #############################################################
    # locomotion helper functions
    #############################################################
    def get_sim_time(self):
        """Gets the simulation time
        Returns:
            simulation time [s]
        """
        tt = vrep.simxGetLastCmdTime(self.clientID)
        return tt

    def get_servo_position(self, servoID):
        """Getting position of a single servo
        Args
            servoID: int ID of the servo to be read
        Returns
            float: current joint angle [rad]
        """
        assert servoID > 0 and servoID <= 18, "Commanding unexisting servo"
        if self.joints_first[servoID - 1]:
            self.joints_first[servoID - 1] = False
            errorCode, value = vrep.simxGetJointPosition(self.clientID, self.servos[servoID - 1],
                                                         vrep.simx_opmode_streaming)
        else:
            errorCode, value = vrep.simxGetJointPosition(self.clientID, self.servos[servoID - 1],
                                                         vrep.simx_opmode_buffer)

        assert errorCode == vrep.simx_return_novalue_flag or errorCode == vrep.simx_return_ok, "Failed to read servo position"

        if self.sync_sim:
            self.trigger()
            self.ping_time()
        else:
            time.sleep(self.simulation_step / 1000.0)

        return value

    def get_all_servo_position(self):
        """Getting position of all servos
        Returns
            list (float): joint angles of all servos [rad]
        """
        angles = np.zeros(18)
        for i in range(0, 18):
            angles[i] = self.get_servo_position(i + 1)
        return angles

    def set_servo_position(self, servoID, angle):
        """Setting position of a single servo
        Args:
            servoID: int ID of the currently commanded servo
            angle: float the desired servo angle [rad] 
        """
        assert servoID > 0 and servoID <= 18, "Commanding unexisting servo"
        errorCode = vrep.simxSetJointTargetPosition(self.clientID, self.servos[servoID - 1], angle,
                                                    vrep.simx_opmode_streaming)
        assert errorCode == vrep.simx_return_ok or errorCode == vrep.simx_return_novalue_flag, "Failed to set servo position"

        if self.sync_sim:
            self.trigger()
            self.ping_time()
        else:
            time.sleep(self.simulation_step / 10000.0)

    def set_all_servo_position(self, angles):
        """Setting position to all the servos at once
        Args:
            angles: numpy array (int)*18 angles of all the servos
        """
        assert len(angles) == 18, "wrong number of operated servos"
        for i in range(0, len(angles)):
            errorCode = vrep.simxSetJointTargetPosition(self.clientID, self.servos[i], angles[i],
                                                        vrep.simx_opmode_streaming)
            assert errorCode == vrep.simx_return_ok or errorCode == vrep.simx_return_novalue_flag, "Failed to set servo position"

        if self.sync_sim:
            self.trigger()
            self.ping_time()
        else:
            time.sleep(self.simulation_step / 10000.0)

    ##############################################################################
    ## Navigation helper functions 
    ##############################################################################
    def get_robot_odometry(self):
        """Method to get the robot odometry as position and orientation in the free space
        Returns:
            Odometry of the robot
        """
        # prepare odometry message
        odom_msg = Odometry()

        # fill in the timestamp
        timestamp = self.get_sim_time()
        if timestamp:
            odom_msg.header.timestamp = timestamp
        else:  # error in reading data from VREP - returning none
            return None

        # get the position
        if self.pos_first:
            self.pos_first = False
            errorCode, position = vrep.simxGetObjectPosition(self.clientID, self.hexapod, -1,
                                                             vrep.simx_opmode_streaming)  # start streaming
        else:
            errorCode, position = vrep.simxGetObjectPosition(self.clientID, self.hexapod, -1,
                                                             vrep.simx_opmode_buffer)  # fetch new data from stream

        if errorCode == vrep.simx_return_novalue_flag:
            return None
        else:
            assert errorCode == vrep.simx_return_ok, "Cannot get object position"

        odom_msg.pose.position.x = position[0]
        odom_msg.pose.position.y = position[1]
        odom_msg.pose.position.z = position[2]

        # get the orientation of the robot
        if self.orientation_first:
            self.orientation_first = False
            errorCode, orientation = vrep.simxGetObjectOrientation(self.clientID, self.hexapod, -1,
                                                                   vrep.simx_opmode_streaming)  # start streaming
        else:
            errorCode, orientation = vrep.simxGetObjectOrientation(self.clientID, self.hexapod, -1,
                                                                   vrep.simx_opmode_buffer)  # fetch new data from stream

        if errorCode == vrep.simx_return_novalue_flag:
            return None
        else:
            assert errorCode == vrep.simx_return_ok, "Cannot get object orientation"

        yaw = 0
        if orientation[1] < 0:
            if orientation[2] < 0:
                yaw = math.pi / 2 - orientation[1]
            else:
                yaw = 3 * math.pi / 2 + orientation[1]
        else:
            if orientation[2] < 0:
                yaw = math.pi / 2 - orientation[1]
            else:
                yaw = 3 * math.pi / 2 + orientation[1]
        odom_msg.pose.orientation.from_Euler(yaw, 0, 0)

        return odom_msg

    def get_robot_collision(self):
        """function to get the collision state of the robot
        Returns:
            bool: return if any part of the robot body is in collision with objects
        """
        state = self.get_collision_state(self.collision)
        return state

    def get_joint_torque(self, servoID):
        """Function to get the joint torques at individual joint
        Args:
            servoID: int ID of the servo
        Returns:
            torque currently exerted by the servo [N.m^-1]
        """
        if self.torques_first[servoID - 1]:
            self.torques_first[servoID - 1] = False
            errorCode, force = vrep.simxGetJointForce(self.clientID, self.servos[servoID - 1],
                                                      vrep.simx_opmode_streaming)
        else:
            errorCode, force = vrep.simxGetJointForce(self.clientID, self.servos[servoID - 1], vrep.simx_opmode_buffer)

        assert errorCode == vrep.simx_return_novalue_flag or errorCode == vrep.simx_return_ok, "Cannot get joint torque data"

        if self.sync_sim:
            self.trigger()
            self.ping_time()
        else:
            time.sleep(self.simulation_step / 10000.0)

        return force

    def get_all_joint_torques(self):
        """Function to get the joint torques of all servos
        Returns:
            list(float): joint torques
        """
        torques = np.zeros(18)
        for i in range(0, 18):
            torques[i] = self.get_joint_torque(i + 1)
        return torques

    #############################################################
    # laser scanner sensor interface
    #############################################################

    def get_laser_scan(self):
        """Function to return laser scan data of the simulated single line laser scanner
        Returns:
            LaserScan
        """
        ret = None
        # fetch the data from the laser scanner
        if self.laser_init:
            errorCode, signalVal = vrep.simxGetStringSignal(self.clientID, "MySignal" + str(self.robot_id),
                                                            vrep.simx_opmode_streaming)  # start streaming
        else:
            errorCode, signalVal = vrep.simxGetStringSignal(self.clientID, "MySignal" + str(self.robot_id),
                                                            vrep.simx_opmode_buffer)  # fetch new data from stream

        if errorCode == vrep.simx_return_novalue_flag:
            ret = None
        else:
            assert errorCode == vrep.simx_return_ok, "Cannot grab laser data"
        # clear the message queue
        vrep.simxClearStringSignal(self.clientID, "MySignal" + str(self.robot_id), vrep.simx_opmode_oneshot)

        data = vrep.simxUnpackFloats(signalVal)

        if data:
            scan_msg = LaserScan()
            scan_msg.header.timestamp = data[0]
            scan_msg.angle_max = (data[1] / 2.0) / 180.0 * math.pi
            scan_msg.angle_min = -(data[1] / 2.0) / 180.0 * math.pi
            scan_msg.range_min = 0.01
            scan_msg.range_max = 10.0
            scan_msg.angle_increment = 1.0 / data[2] / 180 * math.pi
            scan_msg.distances = data[3:]
            return scan_msg
        else:
            return None

    #############################################################
    # synchronous simulation
    #############################################################

    def enable_sync(self):
        # enable synchronous mode
        errorCode = vrep.simxSynchronous(self.clientID, True)
        assert errorCode == vrep.simx_return_ok, "Cannot enable synchronous mode"
        self.sync = True

    def disable_sync(self):
        errorCode = vrep.simxSynchronous(self.clientID, False)
        assert errorCode == vrep.simx_return_ok, "Cannot disable synchronous mode"
        self.sync = False

    def trigger(self):
        # trigger simulation step
        self.sync_steps += 1
        errorCode = vrep.simxSynchronousTrigger(self.clientID)
        assert errorCode == vrep.simx_return_ok, "Cannot trigger simulation"

    def ping_time(self):
        # get ping time
        # after this function returns, values are up-to-date
        errorCode, pingTime = vrep.simxGetPingTime(self.clientID)
        assert errorCode == vrep.simx_return_ok, "Problem occured during simulation step"
        return pingTime

    def set_sim_step(self, step_time):
        # set simulation step time in seconds
        errorCode = vrep.simxSetFloatingParameter(self.clientID, vrep.sim_floatparam_simulation_time_step, step_time,
                                                  vrep.simx_opmode_oneshot)
        # assert errorCode == vrep.simx_return_ok, "Could not set simulation step time.'"
        return errorCode

    #############################################################
    def execute_code(self, code):
        emptyBuff = bytearray()
        res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self.clientID, "hexa_base",
                                                                                     vrep.sim_scripttype_childscript,
                                                                                     'executeCode_function', [], [],
                                                                                     [code], emptyBuff,
                                                                                     vrep.simx_opmode_blocking)
        assert res == vrep.simx_return_ok, "Remote function call failed"
        return retStrings[0]
