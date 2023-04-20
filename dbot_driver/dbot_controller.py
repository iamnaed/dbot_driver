from typing import List
from odrive_controller import OdriveController


class DbotController:
    def __init__(self, odrives:List[OdriveController]) -> None:
        self.odrives = odrives
         
    def encoder_offset_calibration(self, calibration_override=False):
        for odrive in self.odrives:
            odrive.encoder_offset_calibration(calibration_override=calibration_override)

    def arm_velocity_control(self):
        for odrive in self.odrives:
            odrive.arm_velocity_control()

    def arm_position_control(self):
        for odrive in self.odrives:
            odrive.arm_position_control()

    def command_velocity(self, velocity:List[float] = [0,0,0,0,0,0]):
        # Joint 0, 1 on Odrive0
        self.odrives[0].command_velocity(axis=0, velocity=velocity[0])
        self.odrives[0].command_velocity(axis=1, velocity=velocity[1])

        # Joint 2, 3 on Odrive1
        self.odrives[1].command_velocity(axis=0, velocity=velocity[2])
        self.odrives[1].command_velocity(axis=1, velocity=velocity[3])

        # Joint 4, 5 on Odrive2
        self.odrives[2].command_velocity(axis=0, velocity=velocity[4])
        self.odrives[2].command_velocity(axis=1, velocity=velocity[5])

    def command_position(self, position:List[float]=[0,0,0,0,0,0]):

        # Joint 0, 1 on Odrive0
        self.odrives[0].command_position(axis=0, position=position[0])
        self.odrives[0].command_position(axis=1, position=position[1])

        # Joint 2, 3 on Odrive1
        self.odrives[1].command_position(axis=0, position=position[2])
        self.odrives[1].command_position(axis=1, position=position[3])

        # Joint 4, 5 on Odrive2
        self.odrives[2].command_position(axis=0, position=position[4])
        self.odrives[2].command_position(axis=1, position=position[5])

    def get_velocity(self, axis):
        return self.axes[axis].encoder.vel_estimate

    def get_position(self, axis):
        return self.axes[axis].encoder.pos_estimate

    def get_errors(self, axes=[0,1]):
        for odrive in self.odrives:
            odrive.get_errors()
