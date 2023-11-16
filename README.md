# ros2_fanuc_interface

This package implements a custom interface between ros2 and Fanuc robots. 
This implementation builds upon the [fanuc_ros2_driver](https://github.com/UofI-CDACS/fanuc_ros2_drivers) repository.

This package interfaces ros2 and the fanuc ros controller by means of ros topics.

i) cmd_j_pos (type: [sensor_msgs/msg/JointState.msg](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html)) - listened topic in the node that takes command joint positions (possible TODO: commanded velocities?) and writes on the fanuc controller a Position Register(PR). Note: it is possible to write more than one PR, setting this number in the  config/params.yaml file.z

ii) fb_j_pos (type: [sensor_msgs/msg/JointState.msg](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html)) - published topic with feedback information about joint positions and velocities.



to test ros_control pipelines

```console
$ ros2 launch ros2_fanuc_interface test.launch.py
```

then to test joint commands
```console
$ ros2 topic pub /position_commands std_msgs/msg/Float64MultiArray "{data: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],layout: {dim:[], data_offset: 1"}}
```



To use it opena a termianl and do the following command:


```console
$ ros2 launch ros2_fanuc_interface ros2_fanuc_interface.launch.py
```
to test with a simple sinusoidal trajectory on the first joint, on a new terminal 

```console
$ ros2 run ros2_fanuc_interface joint_state_pub.py
```


