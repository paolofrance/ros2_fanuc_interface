# ros2_fanuc_interface

This repository implements a ros2 hardware interface for the CRX family Fanuc robots. 
The code was tested on real hardware on Fanuc CRX-10iaL and CRX-20iaL with R30iB Mini Plus controller.

**OS: Ubuntu 22.04**

**ROS: Humble** (see the branch _rolling_ for the rolling distro)

<img title="Fanuc CRX robots (credits: Fanuc)" alt="Alt text" src="/doc/crx_robots.jpg">


# Outline


1. [Prerequisites](#prerequisites)
2. [Content](#content)
3. [Installation on the remote PC](#installation)
4. [Setting up your CRX robot](#setting-up-robot)
5. [Usage](#usage)
6. [DPM](#dpm)
7. [Known issues](#issues)

## Prerequisites <a name="prerequisites"></a>

The communication between ROS and the robot happens via EthernetIP.
**The EtherNet/IP Adapter option must be loaded on your robot controller to use this package.**

To check is the EthernetIP module is loaded on your robot, open the FanucTP app and check that the EthernetIP options shows up as in the image below.

<img title="EthernetIP loaded on FANUC controller" alt="Alt text" src="/doc/ethip.jpg" width="200">


## Content <a name="content"></a>



| Function | Description |
| ------------- | ------------- |
| fanuc_eth_ip  | Implementation of the communication via Ethernet/IP between the remote PC and the Fanuc controller. |
| fanuc_rmi     | Implementation of the communication via Remote Motion Interface (RMI) between the remote PC and the Fanuc controller. RMI is used as an alternative communication channel. From our experience, EthernetIP communication gives the best results in terms of real-time control of the robot. |
| ros2_fanuc_interface | ROS2 Hardware Interface. Allows for controlling the robot via Ethernet/IP and RMI. |
| fanuc_srvs | Services to call TP programs and allows for interaction with the Fanuc controller.
Most of the services require TP programs to be written and running as Background Logics, that read Registers to start an action.
More details will be added in future releases. |

## Installation on the remote PC <a name="installation"></a>


### Install EIPScanner

The communication between the external pc and the Fanuc controller happens by means of the [Ethernet/IP](https://en.wikipedia.org/wiki/EtherNet/IP) protocol. 
In particular, this package relies on the implementation provided in [EIPScanner](https://eipscanner.readthedocs.io/en/latest/).  
Please, download it from [here](https://github.com/nimbuscontrols/EIPScanner), and follow the instructions [here](https://eipscanner.readthedocs.io/en/latest/getting_started.html#installing) to install it.

NOTE: a previous version of this driver used a python driver for the Ethernet/IP communication. Please refer to the [python-driver](https://github.com/paolofrance/ros2_fanuc_interface/tree/python-driver) branch.

### Package installation and dependencies

Install additional [ros2_control](https://control.ros.org/master/index.html) and [Moveit!](https://moveit.picknik.ai/main/index.html)

```console
sudo apt update
sudo apt upgrade
sudo apt install ros-<distro>-moveit
sudo apt install ros-<distro>-moveit-planners-chomp
sudo apt install ros-<distro>-ros2-control
sudo apt install ros-<distro>-ros2-controllers
```

Install robot description adnd moveit_config packages:
```console
git clone https://github.com/paolofrance/crx20_moveit_config
git clone https://github.com/paolofrance/crx_description
cd ..
colcon build --symlink-install
```

Install the current package:
```console
cd <to-your-src>
git clone https://github.com/paolofrance/ros2_fanuc_interface
cd ..
colcon build --symlink-install
```

## Setting up your CRX robot <a name="setting-up-robot"></a>

**The EtherNet/IP Adapter option must be loaded on your robot controller to use this package.**

### Setting up the robot network

The robot controller and the external PC must be under the same network:
1. Set your PC's IP address as static
2. On the robot controller, in [MENU]->[SETUP]->[HOSTCOMM]->[TCP/IP], make sure the configuration looks like the image below, where Port#1 IP address must be under the same network of your PC.
3. Make sure you can ping the robot from your PC.

<img title="TCPIP configuration" alt="Alt text" src="/doc/tcpip_config.jpg" width="200">


### TP program installation

To actually move the robot, you need a teach-pendant (TP) program running on the robot controller. 
1. Copy the TP programs from [this folder](https://github.com/paolofrance/ros2_fanuc_interface/tree/main/ros2_fanuc_interface/TP_programs) to your robot controller.
2. Set the TP to AUTO mode.
3. Load and run the program ROS2.TP.

### DPM input setup (optional)

To use the DPM (Dynamic Path Modification) module provided by the Fanuc robots, it is necessary to configure 6 Group Inputs and map them to the DPM settings. 

## Usage <a name="usage"></a>

#### Real robot

1. Make sure the program ROS2.TP is running on the robot controller.
2. Make sure the TP is in AUTO mode.
3. Launch:
	```console
	ros2 launch ros2_fanuc_interface robot_bringup.launch.py robot_ip:=your.robot.ip.address robot_type:=crx10ia_l
	```

Available parameters:  
- "robot_type" [crx5ia, crx10ia, crx10ia_l, crx20ia_l, crx25ia_l]: the CRX robot model
- "use_mock_hardware" [true/false]: if the real robot is controlled or a virtual one   
- "controllers_file" ["file location"]: where the list of available ros2_controllers is  
- "robot_ip" [string]: the IP of the actual robot  
- "read_only" [true/false]: useful for recording a trajectory under manual guidance (see below)
- "use_rmi" [true/false]: allows to select the communication method (RMI or EthernetIP). If true, RMI is used. Default: false.


#### Real robot - read only

It is sometimes necessary to move the robot from the Teach Pendant, or via manual guidance, but still required to read the joint states (e.g. kinestetic teaching). This feature requires no programs running on the TP. 
To use it open a terminal and do the following command:

```console
ros2 launch ros2_fanuc_interface robot_bringup.launch.py robot_ip:=your.robot.ip.address robot_type:=crx10ia_l read_only:=true
```

**NOTE**: with some version of the robot controller software we had some issues related to loss of communication. The reason is still unclear. 

#### Simulated robot

To test this with mock components just add the "use_mock_hardware:=true" param to your launch command
```console
ros2 launch ros2_fanuc_interface robot_bringup.launch.py robot_type:=crx10ia_l use_mock_hardware:=true
```

#### Trajectory execution velocity scaling

It is also possible to control the robot allowing dynamic velocity scaling of the trajectory under execution.
This feature is unrelated to this package but is a useful tool that can be used for example to dynamically scale the velocity according to the distance between human and robot.

To test the velocity scaling controller you have to download the [scaled_fjt_controller](https://github.com/paolofrance/scaled_fjt_controller). 

If you want to use the velocity scaler controller, pass the controller configuration to the launcher as follows:

```console
ros2 launch ros2_fanuc_interface robot_bringup.launch.py robot_type:=crx10ia_l controllers_file:=scaled_velocity_controller.yaml
```

You should now see the "/speed_ovr" topic, where you can publish the desired velocity (as a percentage of the maximum velocity). See the documentation [here](https://github.com/paolofrance/scaled_fjt_controller).

## DPM (WiP) <a name="dpm"></a>

This package allows the use of the Dynamic Path Modification (DPM) mode from Fanuc.

The DPM communication is implemented in a ROS node.
The DPM node subscribes a topic with message type [std_msgs/msg/Int64MultiArray.msg](https://docs.ros2.org/foxy/api/std_msgs/msg/Int64MultiArray.html). The message required is in the form [x, y, z,rx, ry, rz].  

**NOTE**: The values are integers because the DPM allows integers only as inputs. 
In the proposed case, the integers are mapped as 0.01 mm, and a conversion factor is implemented on the robot controller side. See the DPM manual for more info.


To run the node

```console
ros2 launch ros2_fanuc_interface dpm_example.launch.py
```

Then the DPM can be activated via a service [std_srvs/SetBool.srv](https://docs.ros.org/en/noetic/api/std_srvs/html/srv/SetBool.html)

To activate the dpm, after running the node
```console
ros2 service call /activate_dpm std_srvs/srv/SetBool {"data: true"}
```

To move the robot from command line
```console
 ros2 topic pub /dpm_move std_msgs/msg/Int64MultiArray {"data:[-10,0,0,0,0,0]"}
```
This command will move the robot in negative x direction.  

to stop the DPM
```console
ros2 service call /activate_dpm std_srvs/srv/SetBool {"data: false"}
```

In the [dpm_params.yaml](https://github.com/paolofrance/ros2_fanuc_interface/blob/main/config/dpm_params.yaml) it is possible to set some parameters.

**NOTE**: The DPM is not directly integrated into the Ros2 control framework since it requires Cartesian relative commands. If you want to contribute, please let us know. 

## Known issues <a name="issues"></a>
1. execution delay of about 0.2 seconds with Ethernet/IP - reduced compared to the previous version with python
2. execution/feedback delay of about 0.6 seconds with RMI - To be tested

## TODO
list of known todos and desiderata:  
1. integration of the Fanuc DPM into the ros control framework

## Contacts
Hardware Interface & Ethernet/IP driver: paolo.franceschi@supsi.ch  
RMI driver: matteo.lavit@stiima.cnr.it  
TP fanuc and services: stefano.baraldo@supsi.ch, andrea.bussolan@supsi.ch  
Tester and user: vincenzo.pomponi@supsi.ch  

## Acknowledgements <a name="ack"></a>
This package is developed by the [ARM (Automation Robotics and Machines Laboratory)](https://sites.supsi.ch/isteps_en/Laboratories/gruppo1.html) at SUPSI, Lugano, CH. 
This package also uses components developed by CNR-SIIMA, Lecco.   
The EU project [Fluently](https://www.fluently-horizonproject.eu/) partially funded the development of this package.

