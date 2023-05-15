#!/usr/bin/env python3

import sys
from time import sleep
import odrive
from odrive.enums import *
from typing import List

'''
Find Odrive serial number (need it in hex format) in odrivetool by running:
    hex(odrv0.serial_number).split('x')[1].upper()
'''

class OdriveController:
    def __init__(self, name:str, serial_number:str) -> None:
        self.name = name
        self.serial_number = serial_number
        print(f"Attempting to connect to ODrive {self.serial_number}")
        try:
            self.odrive = odrive.find_any(serial_number=self.serial_number)
            print(f"Connected to ODrive {self.serial_number}")
        except Exception as e:
            print(f"Failed to connect to ODrive: {e}")

        self.axis0 = self.odrive.axis0
        self.axis1 = self.odrive.axis1
        self.axes = {0: self.odrive.axis0,
                     1: self.odrive.axis1}
        self.armed = False
    
    def motor_calibration(self, axes:List[int]=[0,1], override:bool=False):
        for axis in axes:
            print(f"Motor calibration axis{axis} on {self.name}")
            self.axes[axis].requested_state = AxisState.MOTOR_CALIBRATION

    def encoder_index_search(self, axes:List[int]=[0,1], override:bool=False):
        for axis in axes:
            print(f"Encoder index search axis{axis} on {self.name}")
            self.axes[axis].requested_state = AxisState.ENCODER_INDEX_SEARCH

    def enter_closed_loop_control(self, axes:List[int]=[0,1]):
        if not self.armed:
            for axis in axes:
                print(f"Entering closed loop control mode axis{axis} on {self.name}")
                #self.axes[axis].controller.input_pos = 0
                #self.axes[axis].controller.config.input_mode = InputMode.PASSTHROUGH
                self.axes[axis].requested_state = AxisState.CLOSED_LOOP_CONTROL
        
            self.armed = True
        else:
            print(f"enter_closed_loop_control failed: Already armed. Switch to Idle first.")

    def enter_velocity_control(self, axes:List[int]=[0,1]):
        if not self.armed:
            for axis in axes:
                print(f"Arming axis{axis} on {self.name}")
                self.axes[axis].controller.input_vel = 0
                self.axes[axis].controller.config.input_mode = InputMode.VEL_RAMP
        
            self.armed = True
        else:
            print(f"enter_velocity_control failed: Already armed. Switch to Idle first.")

    def enter_position_control(self, axes:List[int]=[0,1]):
        if not self.armed:
            for axis in axes:
                print(f"Arming axis{axis} on {self.name}")
                #self.axes[axis].controller.input_pos = 0
                self.axes[axis].controller.config.input_mode = InputMode.PASSTHROUGH
                self.axes[axis].requested_state = AxisState.CLOSED_LOOP_CONTROL
            self.armed = False
        else:
            print(f"enter_position_control failed: Already armed. Switch to Idle first.")

    def enter_idle(self, axes:List[int]=[0,1]):
        for axis in axes:
            print(f"Entering idle mode axis{axis} on {self.name}")
            self.axes[axis].requested_state = AxisState.IDLE
        self.armed = False

    def enter_trapezoidal_trajectory_control(self, axes:List[int]=[0,1]):
        if not self.armed_pos:
            for axis in axes:
                print(f"Entering trapezoidal trajectory mode axis{axis} on {self.name}")
                self.axes[axis].controller.input_pos = 0
                self.axes[axis].controller.config.input_mode = InputMode.TRAP_TRAJ
                self.axes[axis].requested_state = AxisState.CLOSED_LOOP_CONTROL
            self.armed_vel = False
            self.armed_pos = True
        else:
            pass

    def command_velocity(self, axes:List[int]=[0,1], velocity:List[float] = [0.0,0.0]):
        if self.armed_vel:
            for axis in axes:
                #print(f'Commanding velocity {velocity[axis]} on axis{axis}, {self.name}')
                self.axes[axis].controller.input_vel = velocity[axis]
        else:
            print(f'Cannot process command. Velocity control is not armed on {self.name}.')

    def command_position(self, axes:List[int]=[0,1], position:List[float] = [0.0,0.0]):
        if self.armed_pos:
            for axis in axes:
                #print(f'Commanding position {position[axis]} on axis{axis}, {self.name}')
                self.axes[axis].controller.input_pos = position[axis]
        else:
            print(f'Cannot process command. Position control is not armed on {self.name}.')

    def get_velocity(self):
        return self.axis0.encoder.vel_estimate, self.axis1.encoder.vel_estimate

    def get_position(self):
        return self.axis0.encoder.pos_estimate, self.axis1.encoder.pos_estimate

    def get_errors(self, axes:List[int]=[0,1]):
        for axis in axes:
            print(f'\nGetting errors for axis{axis} on {self.name}:')
            print('Axis: ' + str(hex(self.axes[axis].error)))
            print('Motor: ' + str(hex(self.axes[axis].motor.error)))
            print('Controller :' + str(hex(self.axes[axis].controller.error)))
            print('Encoder: ' + str(hex(self.axes[axis].encoder.error)))

    def clear_errors(self, axes:List[int]=[0,1]):
        self.odrive.clear_errors()