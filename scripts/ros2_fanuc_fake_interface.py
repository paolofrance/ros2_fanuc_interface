#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

import numpy as np

from sensor_msgs.msg import JointState

class FanucRosInterface(Node):

    def __init__(self):
        super().__init__('ros_fanuc_fake_inteface')
        
        self.declare_parameter("buffer_length_param", 1)
        self.declare_parameter("control_time", 0.04)

        self.buffer_length = self.get_parameter("buffer_length_param").value
        self.control_time = self.get_parameter("control_time").value

        self.timer = self.create_timer(self.control_time, self.fb_joint_pose_callback)
        
        self.publisher_ = self.create_publisher(JointState, 'fb_j_pos', 10)
        self.subscription = self.create_subscription(JointState, 'cmd_j_pos', self.cmd_joint_pos_callback, 10)
        
        self.command = np.zeros(6).astype(np.float32)
        self.command[4]=-1.5707
        self.jp_prev = self.command

    def cmd_joint_pos_callback(self, msg):
        self.command = np.array(msg.position)
        
        
    def fb_joint_pose_callback(self):
        msg = JointState()
        
        jp_cur = self.command
        
        self.get_logger().info("jp_cur: "+str(jp_cur))
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["J1", "J2", "J3", "J4", "J5", "J6" ]
        msg.position = jp_cur.tolist()
        msg.velocity = ( (jp_cur-self.jp_prev)/self.control_time ).tolist()
        # msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publisher_.publish(msg)
        
        self.jp_prev = jp_cur


def main(args=None):
    rclpy.init(args=args)
    
    minimal_subscriber = FanucRosInterface()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





