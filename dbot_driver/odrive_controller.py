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
ODRIVE_SERIAL_NUMBER = "364D38623030"

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
        self.armed_vel = False
        self.armed_pos = False
        self.calibration_override_timer = 10
    
    def full_calibration(self, axes:List[int]=[0,1], calibration_override:bool=False):
        pass

    def encoder_offset_calibration(self, axes:List[int]=[0,1], calibration_override:bool=False):
        if not calibration_override:
            for axis in axes:
                if not self.axes[axis].encoder.is_ready:
                    print(f'Calibrating encoder on axis{axis} on {self.name}')
                    self.axes[axis].requested_state = AxisState.ENCODER_OFFSET_CALIBRATION
                else:
                    pass
            
            while len(axes):
                for axis in axes:
                    if self.axes[axis].encoder.is_ready:
                        axes.remove(axis)

        else:
            for axis in axes:
                print(f"Calibrating encoder on axis{axis} on {self.name}")
                self.axes[axis].requested_state = AxisState.ENCODER_OFFSET_CALIBRATION

                print(f'Waiting {self.calibration_override_timer} seconds for calibration to complete...')
                sleep(self.calibration_override_timer)
                print(f'Encoder calibration timed out. . .')

    def arm_velocity_control(self, axes:List[int]=[0,1]):
        for axis in axes:
            print(f"Arming axis{axis} on {self.name}")
            self.axes[axis].controller.input_vel = 0
            self.axes[axis].controller.config.input_mode = InputMode.VEL_RAMP
            self.axes[axis].requested_state = AxisState.CLOSED_LOOP_CONTROL
        
        self.armed_vel = True
        self.armed_pos = False

    def arm_position_control(self, axes:List[int]=[0,1]):
        for axis in axes:
            print(f"Arming axis{axis} on {self.name}")
            self.axes[axis].controller.input_pos = 0
            self.axes[axis].controller.config.input_mode = InputMode.POS_FILTER
            self.axes[axis].requested_state = AxisState.CLOSED_LOOP_CONTROL
    
        self.armed_vel = False
        self.armed_pos = True

    def command_velocity(self, axis:int, velocity:float):
        if self.armed_vel:
            print(f'Commanding velocity {velocity} on axis{axis}, {self.name}')
            self.axes[axis].controller.input_vel = velocity
        else:
            print(f'Cannot process command. Velocity control is not armed on {self.name}.')

    def command_position(self, axis:int, position:float):
        if self.armed_pos:
            self.axes[axis].controller.input_pos = position
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