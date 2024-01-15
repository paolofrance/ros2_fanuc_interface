#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

import numpy as np

from robot_control import robot
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16

class FanucRosInterface(Node):

    def __init__(self):
        super().__init__('fanuc_ros_inteface')
        
        self.declare_parameter("buffer_length_param", 8)
        self.declare_parameter("control_time", 0.04)
        self.declare_parameter("robot_ip", "10.11.31.111")
        self.declare_parameter("read_only", False)

        self.buffer_length = self.get_parameter("buffer_length_param").value
        self.control_time = self.get_parameter("control_time").value
        robot_ip = self.get_parameter("robot_ip").value
        self.read_only = self.get_parameter("read_only").value
        
        self.get_logger().info("read_only: " + str(self.read_only))
        self.get_logger().info("robot_ip: " + str(robot_ip))
        
        self.r = robot(robot_ip)
        
        
        self.r.write_digital_input(4)
        
        

        self.pr_number=0
        self.timer = self.create_timer(self.control_time, self.fb_joint_pose_callback)
        
        self.j23_factor = 1
        
        
        
        self.r.allow_motion(False)
        self.set_current_pose_to_target()
        
        cp = self.r.get_current_joint_pos()

        self.j2_offset = np.deg2rad(cp[1])
        
        self.get_logger().info(str(self.r.get_current_joint_pos()))
        
        self.r.allow_motion(True)
        self.get_logger().info("register is now: " + str(self.r.read_register_n(1)))
        
        self.r.set_speed(100)
        
        if not self.read_only:
            while self.r.read_register_n(1) != 1:
                self.get_logger().info("waiting for start register to be set to one")
                self.r.allow_motion(True)
                time.sleep(0.5)
        
        self.get_logger().info("starting motion allowed")

        self.publisher_ = self.create_publisher(JointState, 'fb_j_pos', 1)
        self.subscription = self.create_subscription(
            JointState,
            'cmd_j_pos',
            self.cmd_joint_pos_callback,
            1)
        self.subscription  # prevent unused variable warning
        
        self.speed_ovr_subscription = self.create_subscription(
            Int16,
            '/speed_ovr',
            self.speed_ovr_callback,
            1)
        
        self.jp_prev = self.r.get_current_joint_pos()

        
        
        

    def cmd_joint_pos_callback(self, msg):
        
        if not self.read_only:
        
            self.pr_number += 1
            
            cmd_j_pos = np.array(msg.position) # rad
            
            self.adjust_j3_pos(j_pos=cmd_j_pos, j23_factor=-self.j23_factor)

            self.r.PRNumber = (self.pr_number % self.buffer_length) + 1
            self.r.write_joint_pose(np.rad2deg(cmd_j_pos))

            self.get_logger().debug("writing into register " + str(self.r.PRNumber) + " target pose : " + str(cmd_j_pos))
        
    def fb_joint_pose_callback(self):
        msg = JointState()
        
        fb_j_pos = np.deg2rad(self.r.get_current_joint_pos()) # rad

        self.adjust_j3_pos(j_pos=fb_j_pos, j23_factor=self.j23_factor)
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["J1", "J2", "J3", "J4", "J5", "J6" ]
        msg.position = fb_j_pos.tolist()
        msg.velocity = ( (fb_j_pos-self.jp_prev)/self.control_time ).tolist()
        self.publisher_.publish(msg)
        
        self.jp_prev = fb_j_pos
        
    
    def speed_ovr_callback(self, msg):
        # TODO: in general, should set the velocity scalinig at robot controller level. Requires Local setup no copmatible?
        self.r.set_speed(msg.data)
        # self.r.set_bin_speed_registers(msg.data)

        
    def set_current_pose_to_target(self):
        if not self.read_only:
            
            cp = self.r.get_current_joint_pos()
            for i in range(self.buffer_length):
                self.r.PRNumber = i + 1
                self.r.write_joint_pose(cp)
                self.get_logger().debug("writing into register " + str(self.r.PRNumber) + " current pose as target : " + str(cp))

    def adjust_j3_pos(self,j_pos,j23_factor):
        j_pos[2] += j23_factor * ( j_pos[1])
        

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





