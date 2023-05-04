from typing import List
import rclpy
from rclpy.node import Node
from odrive.enums import *
from std_msgs.msg import Float32MultiArray
from .dbot_controller import DbotController
from .odrive_controller import OdriveController 


class DbotNode(Node):
    def __init__(self, dbot_controller:DbotController):
        super().__init__('dbot_driver')
        self.dbot_controller = dbot_controller
        
        # Data Publishers
        self.axis_pos_publisher = self.create_publisher(Float32MultiArray, 'joint_pos_topic', 50)
        self.axis_vel_publisher = self.create_publisher(Float32MultiArray, 'joint_vel_topic', 50)

        # Create Timers for the publisher
        timer_period = 0.01  # in seconds [100 hz] [10ms]
        self.timer = self.create_timer(timer_period, self.__joint_data_callback)

        # Data Subscribers
        self.axis_pos_subscriber = self.create_subscription(Float32MultiArray, 'cmd_pos_topic', self.axis_pos_callback, 50)
        self.axis_vel_subscriber = self.create_subscription(Float32MultiArray, 'cmd_vel_topic', self.axis_vel_callback, 50)

    def __joint_data_callback(self):
        # Get Data from ODrive and publish it
        joint_pos = self.dbot_controller.get_joint_position()
        joint_vel = self.dbot_controller.get_joint_velocity()
        msg = Float32MultiArray()

        # Pos
        msg.data = joint_pos
        self.axis_pos_publisher.publish(msg)

        # Vel
        msg.data = joint_vel
        self.axis_vel_publisher.publish(msg)

    def axis_vel_callback(self, msg:Float32MultiArray):
        print(f"axis vel: {msg}")
        self.dbot_controller.command_velocity(msg.data)

    def axis_pos_callback(self, msg:Float32MultiArray):
        print(f"axis pos: {msg}")
        self.dbot_controller.command_position(msg.data)


def main(args=None):
    
    # Initialize
    rclpy.init(args=args)

    # Odrives with hard coded serial numbers
    odrive0 = OdriveController("odrive0", "2075396D4D4D") # My First odrive serial number
    odrive1 = OdriveController("odrive1", "364D38623030")
    odrive2 = OdriveController("odrive2", "364D38623030")
    dbot_controller = DbotController([odrive0, odrive1, odrive2])
    dbot_node = DbotNode(dbot_controller=dbot_controller)

    # Calibrate
    dbot_controller.encoder_index_search()
    dbot_controller.arm_position_control()

    # Spin
    rclpy.spin(dbot_node)

    # Cleanup
    dbot_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()