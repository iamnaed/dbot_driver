from typing import List
import rclpy
from rclpy.node import Node
from odrive.enums import *
from std_msgs.msg import Float32MultiArray
from odrive_controller import OdriveController 
from dbot_controller import DbotController

class DbotNode(Node):
    def __init__(self, dbot_controller:DbotController):
        self.dbot_controller = dbot_controller
        pass

    def timer_callback(self):
        pass

    def axis_vel_callback(self, msg:Float32MultiArray):
        print(f"axis vel: {msg}")
        self.dbot_controller.command_velocity(msg.data)

    def axis_pos_callback(self, msg:Float32MultiArray):
        print(f"axis pos: {msg}")
        self.dbot_controller.command_position(msg.data)


def main(args=None):
    
    # Initialize
    rclpy.init(args=args)
    odrive0 = OdriveController("odrive0", "364D38623030")
    odrive1 = OdriveController("odrive1", "364D38623030")
    odrive2 = OdriveController("odrive2", "364D38623030")
    dbot_controller = DbotController([odrive0, odrive1, odrive2])
    odrive_node = DbotNode(dbot_controller=dbot_controller)

    # Calibrate
    dbot_controller.encoder_offset_calibration()
    dbot_controller.arm_position_control()

    # Spin
    rclpy.spin(odrive_node)

    # Cleanup
    odrive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()