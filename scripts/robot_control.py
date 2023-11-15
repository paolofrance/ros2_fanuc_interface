
# Imports
import sys
sys.path.append('./pycomm3/pycomm3')
import math

import fanuc_ros2_driver.FANUCethernetipDriver as FANUCethernetipDriver
## The mode of operation; 
FANUCethernetipDriver.DEBUG = False

## Robot Class
# @param self, robotIP
class robot:
    def __init__(self, robotIP):
        """! Initializes the robot.
        @param robotIP      IP address of robot
        """
        self.robot_IP = robotIP
        self.CurJointPosList = FANUCethernetipDriver.returnJointCurrentPosition(self.robot_IP)
        self.CurCartesianPosList = FANUCethernetipDriver.returnCartesianCurrentPostion(self.robot_IP)
        self.PRNumber = 1 # This is the position register for holding coordinates
        self.start_register = 1
        self.sync_register = 2
        self.sync_value = 1
        self.speed_register = 5

    # Joint movement functions
    
    def get_current_joint_pos(self):
        self.read_current_joint_position()
        return self.CurJointPosList[2:8]
     
    def read_current_joint_position(self):
        """! Updates the robots joint position list.
        """
        self.CurJointPosList = FANUCethernetipDriver.returnJointCurrentPosition(self.robot_IP)

    def read_register_n(self, n):
        """! reads register number n
        """
        return FANUCethernetipDriver.readR_Register(self.robot_IP, n)

    # read PR[1] Joint Coordinates
    def read_joint_position_register(self):
        """! Reads joint position register(PR1) and prints the value and prints list.
        """
        PR_1_Value = FANUCethernetipDriver.readJointPositionRegister(self.robot_IP, self.PRNumber)
        print("PR[%d]"% self.PRNumber)
        print("list=", PR_1_Value)

    # write PR[1] offset
    def write_joint_offset(self, joint, value):
        """! Offsets current joint position by value given in degrees.
        @param joint        which joint to move
        @param value        degrees you want to move, negative or positive direction
        """
        print("***********************************************")
        print(f" Write Joint Offset Value:[{value}] to Joint:[{joint}] ")
        print("***********************************************")
        joint += 1

        newPosition = self.CurJointPosList[joint] + value
        print(f"New Position value: {newPosition}\n")

        self.CurJointPosList[joint] = newPosition
        print(self.CurJointPosList[joint])

        myList = self.CurJointPosList

        FANUCethernetipDriver.writeJointPositionRegister(self.robot_IP, self.PRNumber, myList)

    # write PR[1] Joint value
    def write_joint_position(self, joint, value):
        """! Sets joint to specific angle based on value
        @param joint        which joint to move
        @param value        angle to set joint to from -180 to 180
        """
        print("--------------------------------")
        print("| write PR[1] Joint Coordinate |")
        print("--------------------------------")
        joint = joint + 1

        newPosition = value

        self.CurJointPosList[joint] = newPosition

        myList = self.CurJointPosList

        FANUCethernetipDriver.writeJointPositionRegister(self.robot_IP, self.PRNumber, myList)

    # DEPRACTED FUNCTION
    # WILL BE REMOVED IN NEXT UPDATE
    # USE write_joint_pose
    def set_pose(self, joint_position_array):
        """! Set a pose(all joint positions) for robot
        @param joint_position_array         a list of joint angles 
        """
        joint_number = 1
        for joint in joint_position_array:
            self.CurJointPosList[joint_number + 1] = joint_position_array[joint_number - 1]
            joint_number += 1

        myList = self.CurJointPosList

        FANUCethernetipDriver.writeJointPositionRegister(self.robot_IP, self.PRNumber, myList)
        print("FUNCTION DEPRACTED, USE: write_joint_pose()")

    # Set pose of robot by passing an array of joint positions
    def write_joint_pose(self, joint_position_array):
        """! Set a pose(all joint positions) for robot
        @param joint_position_array         a list of joint angles 
        """
        joint_number = 1
        for joint in joint_position_array:
            self.CurJointPosList[joint_number + 1] = joint_position_array[joint_number - 1]
            joint_number += 1

        myList = self.CurJointPosList

        FANUCethernetipDriver.writeJointPositionRegister(self.robot_IP, self.PRNumber, myList)
        

    # Put robot in home position
    def set_joints_to_home_position(self):
        """! This used to set a CRX10 into a 'home' position. Not useful for other machines probably
        """
        print("*************************************************")
        print("* Setting Joint Positions to Home Configuration *")
        print("*************************************************")

        # Set positions DO NOT USE 0
        # joint coordinates start at list item 2, ie: Joint2 = CurPosList[3]
        self.CurJointPosList[2] = 1.0 # J1
        self.CurJointPosList[3] = 1.0 # J2
        self.CurJointPosList[4] = 1.0 # J3
        self.CurJointPosList[5] = 1.0 # J4
        self.CurJointPosList[6] = 1.0 # J5 PB50IB does not like this join, OK on CRX10
        self.CurJointPosList[7] = 1.0 # J6

        myList = self.CurJointPosList

        FANUCethernetipDriver.writeJointPositionRegister(self.robot_IP, self.PRNumber, myList)

    # Cartesian Movement Functions

    # read current cartesian position from Robot
    def get_coords(self):
        """! Print current cartesian coordinates from robot.
        """
        print("--------------------------")
        print("| read CURPOS from Robot |")
        print("--------------------------")
        CurPosList = FANUCethernetipDriver.returnCartesianCurrentPostion(self.robot_IP)

        print("CURPOS=", CurPosList)

    # read PR[1] Cartesian Coordinates
    def read_cartesian_position_register(self):
        """! Reads the cartesian position register and prints a list
        """
        print("-----------------------------------")
        print("| read PR[1] Cartesian Coordinate |")
        print("-----------------------------------")

        PR_1_Value = FANUCethernetipDriver.readCartesianPositionRegister(self.robot_IP, self.PRNumber)
        print("PR[%d]"% self.PRNumber)
        print("list=", PR_1_Value)

    # DEPRACTED FUNCTION
    # WILL BE REMOVED IN NEXT UPDATE
    # USE write_cartesian_position()
    def send_coords(self, X, Y, Z, W=None, P=None, R=None):
        """! Send cartesian coordinates to robot using X, Y, Z, W, P, R system. 
        These coordinates usually correlate to the tool end effectors position.
        @param X            X cartesian coordinate
        @param Y            Y cartesian coordinate
        @param Z            Z cartesian coordinate
        @param W            Yaw
        @param P            Pitch
        @param R            Roll
        """
        self.CurCartesianPosList[2] = X
        self.CurCartesianPosList[3] = Y
        self.CurCartesianPosList[4] = Z
        self.CurCartesianPosList[5] = W if W is not None else self.CurCartesianPosList[5]
        self.CurCartesianPosList[6] = P if P is not None else self.CurCartesianPosList[6]
        self.CurCartesianPosList[7] = R if R is not None else self.CurCartesianPosList[7]

        newPositionList = self.CurCartesianPosList

        FANUCethernetipDriver.writeCartesianPositionRegister(self.robot_IP, self.PRNumber, newPositionList)
        print("FUNCTION DEPRACTED, USE: write_cartesian_position()")

    # write PR[1] Cartesian Coordinates
    # Takes x, y, z, w, p, r coords.
    # WPR are the orientation of the end effector, DEFAULT to current orientation
    def write_cartesian_position(self, X, Y, Z, W=None, P=None, R=None):
        """! Send cartesian coordinates to robot using X, Y, Z, W, P, R system. 
        These coordinates usually correlate to the tool end effectors position.
        @param X            X cartesian coordinate
        @param Y            Y cartesian coordinate
        @param Z            Z cartesian coordinate
        @param W            Yaw
        @param P            Pitch
        @param R            Roll
        """
        self.CurCartesianPosList[2] = X
        self.CurCartesianPosList[3] = Y
        self.CurCartesianPosList[4] = Z
        self.CurCartesianPosList[5] = W if W is not None else self.CurCartesianPosList[5]
        self.CurCartesianPosList[6] = P if P is not None else self.CurCartesianPosList[6]
        self.CurCartesianPosList[7] = R if R is not None else self.CurCartesianPosList[7]

        newPositionList = self.CurCartesianPosList

        FANUCethernetipDriver.writeCartesianPositionRegister(self.robot_IP, self.PRNumber, newPositionList)

    # Utility Functions

    # write R[5] to set Speed in mm/sec
    def set_speed(self, value):
        """! Set movement speed of robot in mm/s
        @param value        speed in mm/s
        """
        # print("------------------------------")
        # print(f"| Speed set to {value}mm/sec |")
        # print("------------------------------")
        FANUCethernetipDriver.writeR_Register(self.robot_IP, self.speed_register, value)

    # get current speed
    def get_speed(self):
        """! Returns current set speed of robot
        @return             speed in mm/s
        """
        return FANUCethernetipDriver.readR_Register(self.robot_IP, self.speed_register)
    
    def allow_motion(self, start=True):
        reg = 0
        if start:
            reg = 1
        FANUCethernetipDriver.writeR_Register(self.robot_IP, self.start_register, reg)
    
    # Starts robot movement and checks to see when it has completed
    # Default to blocking 
    # Function will block until move action is complete
    def start_robot(self, blocking=True):  
        """! starts robot movement by setting the sync register to 1 on the TP program executing commands.
        @param blocking     True/False program will wait to continue till move is finished. Default=True
        """
        # Write to start register to begin movement
        FANUCethernetipDriver.writeR_Register(self.robot_IP, self.start_register, 1)

        # Wait till robot is done moving
        if blocking == True:
            moving = self.read_robot_start_register()
            while(moving):
                moving = self.read_robot_start_register()

            # Update position List
            self.read_current_joint_position()

            # Signal end of move action
            print("********************************************")
            print("* Moving Joint(s) to Position(s): COMPLETE *")
            print("********************************************")
        elif blocking == False:
            pass

    # Detect if the robot is moving
    def is_moving(self):
        """! checks to see if robot is moving based on the value of the sync register 1=moving 0=not moving
        """
        start_register = self.read_robot_start_register()
        if start_register == 1:
            return 1

        elif start_register == 0:
            return 0

        else:
            return -1


    # Put CRX10 in a mount position to change end tooling
    # !!! -- THIS CAN BE MOVED INTO OWN FILE -- !!!
    # !!! -- THIS JOIN CONFIGURATION IS FOR THE CRX10 ROBOT -- !!!
    def set_joints_to_mount_position(self):
        """! set the joints to be in a 'mount' position for tooling. 
        This is for CRX10's and will most likely be removed from this API
        """
        print("**************************************************")
        print("* Setting Joint Positions to Mount Configuration *")
        print("**************************************************")

        # Set positions DO NOT USE 0
        # joint coordinates start at list item 2, ie: Joint2 = CurPosList[3]
        self.CurJointPosList[2] = 1.0 # J1
        self.CurJointPosList[3] = 58.0 # J2
        self.CurJointPosList[4] = -12.0 # J3
        self.CurJointPosList[5] = -2.0 # J4
        self.CurJointPosList[6] = 11.0 # J5 PB50IB does not like this join, OK on CRX10
        self.CurJointPosList[7] = -6.0 # J6

        myList = self.CurJointPosList

        FANUCethernetipDriver.writeJointPositionRegister(self.robot_IP, self.PRNumber, myList)

    # This function reads register 1(sync bit for position register)
    def read_robot_start_register(self):
        """! returns value of start register
        @return             value of start register
        """
        start_register = FANUCethernetipDriver.readR_Register(self.robot_IP, self.start_register)
        return start_register



