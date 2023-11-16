#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
import numpy as np
import copy

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(JointState, 'cmd_j_pos', 10)
        self.timer_period = 0.04
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.t = 0
        self.init_g1 = False
        self.g1_base=0
        
        self.amplitude = 20.0
        self.freq = 0.6
        
        self.subscription = self.create_subscription(
            JointState,
            'fb_j_pos',
            self.callback,
            10)
        self.reach = False
        
        rate = self.create_rate(25)

        
        
    # def timer_callback(self):
        while rclpy.ok():
            self.t += self.timer_period
            g1 = self.g1_base + self.amplitude*np.sin(self.freq * self.t)
            # g1 = self.g1_base + 90
            msg = JointState()
            msg.name = ["J1", "J2", "J3", "J4", "J5", "J6" ]
            msg.position = np.deg2rad([g1, 0, 0.0, 0.0, -90.0, 0.0])
            
            self.publisher_.publish(msg)
            rate.sleep()
    
    def callback(self, msg):
        if not self.init_g1:
            j_pos = np.array(msg.position)
            self.g1_base = np.rad2deg(j_pos[0])
            self.get_logger().info("initializing g1 to :" + str(self.g1_base) + " from: " + str(j_pos[0]))
            self.init_g1=True

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()