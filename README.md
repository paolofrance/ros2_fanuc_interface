# ros2_fanuc_interface

This package implements a custom interface between ros2 and Fanuc robots. 
This implementation builds upon the [fanuc_ros2_driver](https://github.com/UofI-CDACS/fanuc_ros2_drivers) repository.

To use it opena a termianl and do the following command:


```console
$ ros2 launch ros2_fanuc_interface ros2_fanuc_interface.launch.py
```
to test with a simple sinusoidal trajectory on the first joint, on a new terminal 

```console
$ ros2 run ros2_fanuc_interface joint_state_pub.py
```
