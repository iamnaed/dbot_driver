from typing import List
from .odrive_controller import OdriveController

class DbotController:
    def __init__(self, odrive_controllers:List[OdriveController]) -> None:
        self.odrive_controllers = odrive_controllers
         
    def motor_calibration(self, calibration_override = False):
        pass

    def encoder_index_search(self, calibration_override = False):
        pass
    
    def closed_loop_control():
        pass

    def encoder_offset_calibration(self, calibration_override=False):
        for odrive in self.odrive_controllers:
            odrive.encoder_offset_calibration(calibration_override=calibration_override)

    def arm_velocity_control(self):
        for odrive in self.odrive_controllers:
            odrive.enter_velocity_control()

    def arm_position_control(self):
        for odrive in self.odrive_controllers:
            odrive.enter_position_control()

    def command_velocity(self, velocity:List[float] = [0,0,0,0,0,0]):
        # Joint 0, 1 on Odrive0
        self.self.odrive_controllers[0].command_velocity(axis=0, velocity=velocity[0])
        self.self.odrive_controllers[0].command_velocity(axis=1, velocity=velocity[1])

        # Joint 2, 3 on Odrive1
        self.self.odrive_controllers[1].command_velocity(axis=0, velocity=velocity[2])
        self.self.odrive_controllers[1].command_velocity(axis=1, velocity=velocity[3])

        # Joint 4, 5 on Odrive2
        self.self.odrive_controllers[2].command_velocity(axis=0, velocity=velocity[4])
        self.self.odrive_controllers[2].command_velocity(axis=1, velocity=velocity[5])

    def command_position(self, position:List[float]=[0,0,0,0,0,0]):
        # Joint 0, 1 on Odrive0
        self.self.odrive_controllers[0].command_position(axis=0, position=position[0])
        self.self.odrive_controllers[0].command_position(axis=1, position=position[1])

        # Joint 2, 3 on Odrive1
        self.self.odrive_controllers[1].command_position(axis=0, position=position[2])
        self.self.odrive_controllers[1].command_position(axis=1, position=position[3])

        # Joint 4, 5 on Odrive2
        self.self.odrive_controllers[2].command_position(axis=0, position=position[4])
        self.self.odrive_controllers[2].command_position(axis=1, position=position[5])

    def get_joint_velocity(self):
        joint0,joint1 = self.self.odrive_controllers[0].get_position()
        joint2,joint3 = self.self.odrive_controllers[1].get_position()
        joint4,joint5 = self.self.odrive_controllers[2].get_position()
        return [joint0, joint1, joint2, joint3, joint4, joint5]

    def get_joint_position(self):
        joint0,joint1 = self.self.odrive_controllers[0].get_velocity()
        joint2,joint3 = self.self.odrive_controllers[1].get_velocity()
        joint4,joint5 = self.self.odrive_controllers[2].get_velocity()
        return [joint0, joint1, joint2, joint3, joint4, joint5]

    def get_errors(self, axes=[0,1]):
        for odrive in self.self.odrive_controllers:
            odrive.get_errors()
