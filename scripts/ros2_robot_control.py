#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

import numpy as np

from robot_control import robot
from sensor_msgs.msg import JointState

class FanucRosInterface(Node):

    def __init__(self):
        super().__init__('fanuc_ros_inteface')
        
        self.declare_parameter("buffer_length_param", 8)
        self.declare_parameter("control_time", 0.04)
        self.declare_parameter("robot_ip", "10.11.31.111")

        self.buffer_length = self.get_parameter("buffer_length_param").value
        self.control_time = self.get_parameter("control_time").value
        robot_ip = self.get_parameter("robot_ip").value
        self.r = robot(robot_ip)

        self.pr_number=0
        self.timer = self.create_timer(self.control_time, self.fb_joint_pose_callback)
        
        self.r.allow_motion(False)
        self.set_current_pose_to_target()
        self.get_logger().info(str(self.r.get_current_joint_pos()))
        
        self.r.allow_motion(True)
        self.get_logger().info("register is now: " + str(self.r.read_register_n(1)))
        
        while self.r.read_register_n(1) is not 1:
            self.get_logger().info("waiting for start register to be set to one")
            self.r.allow_motion(True)
            time.sleep(0.5)
        
        self.get_logger().info("starting motion allowed")

        self.publisher_ = self.create_publisher(JointState, 'fb_j_pos', 10)
        self.subscription = self.create_subscription(
            JointState,
            'cmd_j_pos',
            self.cmd_joint_pos_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.jp_prev = self.r.get_current_joint_pos()

    def cmd_joint_pos_callback(self, msg):
        self.pr_number += 1
        
        j_pos = np.array(msg.position)
        # self.r.PRNumber = (self.pr_number % self.buffer_length) + 1
        # self.r.write_joint_pose(np.rad2deg(j_pos))
        # self.get_logger().info("callback writing into register " + str(self.r.PRNumber) + " target j pose to : " + str(j_pos))

        # for i in range(3):
            # self.r.PRNumber = i + 1
            # self.r.write_joint_pose(np.rad2deg(j_pos))

        self.r.PRNumber = 1
        self.r.write_joint_pose(np.rad2deg(j_pos))
    
    def fb_joint_pose_callback(self):
        msg = JointState()
        
        jp_cur = np.deg2rad(self.r.get_current_joint_pos())
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["J1", "J2", "J3", "J4", "J5", "J6" ]
        msg.position = jp_cur
        msg.velocity = (jp_cur-self.jp_prev)/self.control_time
        self.publisher_.publish(msg)
        
        self.jp_prev = jp_cur
        
    def set_current_pose_to_target(self):
        cp = self.r.get_current_joint_pos()
        for i in range(self.buffer_length):
            self.r.PRNumber = i + 1
            self.r.write_joint_pose(cp)
            self.get_logger().info("writing into register " + str(self.r.PRNumber) + " current pose as target : " + str(cp))

            
    def destroy_node(self):
        super().destroy_node()
        self.r.allow_motion(False)


def main(args=None):
    rclpy.init(args=args)
    
    minimal_subscriber = FanucRosInterface()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





