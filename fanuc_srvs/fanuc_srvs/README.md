# moveit_srv


A simple package implementing Moveit! services to be called e.g., via Python clients


Two services are available:  
i) /plan_trg_pose  
ii) /plan_trg_jnt

The first allows sending a [geometry_msgs::msg::Pose](https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html) message.

The second allows sending a [std_msgs::msg::Float32[]](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) message.


A brief example can be found [here](https://gitlab-core.supsi.ch/dti-isteps/armlab/fluently/dexterity/-/blob/main/fluently_app/scripts/moveit_client_test.py?ref_type=heads)