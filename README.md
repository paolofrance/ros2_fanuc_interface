# ros2_fanuc_interface

This package implements ros2 hardware interface for the CRX family Fanuc robots. 



## Installation guidelines

To install the current package just
```console
$ cd <to-your-src>
$ git clone https://github.com/paolofrance/ros2_fanuc_interface
```
you should be ready to test the basic ros2-fanuc interface by publishing topics.

To use it with the [ros2_control](https://control.ros.org/master/index.html) framework, additional Moveit! and description packages are required. 
Download the packages
```console
$ git clone https://github.com/paolofrance/crx20_moveit_config
$ git clone https://github.com/paolofrance/crx_description
$ cd ..
$ colcon build --symlink-install
```
To complete the setup the [Moveit!2](https://moveit.picknik.ai/main/index.html) and the [ros2_control](https://control.ros.org/master/index.html) are expected to be running.
If some error appears, please check their installation and install them.
```console
$ sudo apt update
$ sudo apt upgrade
$ sudo apt install ros-<distro>-moveit
$ sudo apt install ros-<distro>-ros2-control
$ sudo apt install ros-<distro>-ros2-controllers
```

## usage

To use it open a terminal and do the following command:

```console
$ ros2 launch ros2_fanuc_interface robot_bringup.launch.py 
```




To test this with mock components just add the "use_mock_hardware:=true" param to your launch command
```console
$ ros2 launch ros2_fanuc_interface moveit_test.launch.py use_mock_hardware:=false
```


## Known issues
1. execution delay of about 0.2 seconds - reduced compared to the previous version with python

## TODO
list of known todos and desiderata:  
1. implementation of the Fanuc DPM to allow faster control for contact tasks
2. implementation of the Fanuc RMI to remove the need for TP programs

## Contacts
paolo.franceschi@supsi.ch  
vincenzo.pomponi@supsi.ch  
stefano.baraldo@supsi.ch  


